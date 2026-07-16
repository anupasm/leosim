#!/usr/bin/env python3
# Copyright (c) 2024 Anupa De Silva
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation;
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

"""
Generate satellite positions from TLE (Two-Line Element) files for ns-3 mobility.

This script reads TLE data and generates satellite positions (x, y, z coordinates)
at specified time intervals. The output can be directly used with ns-3 mobility models.

Dependencies:
    pip install skyfield numpy
"""

import argparse
import concurrent.futures
import csv
import os
import re
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path
import numpy as np

try:
    from skyfield.api import load, EarthSatellite, wgs84
    from skyfield.timelib import Time
except ImportError:
    print("ERROR: skyfield library not found. Install it with: pip install skyfield")
    sys.exit(1)

START = datetime(2025, 9, 14, 9, 0)


_WORKER_TIMES = None
_WORKER_TS = None
_WORKER_TIMESTEP = None
_WORKER_COORDINATE_SYSTEM = None


def _build_times(ts, start_time, duration, timestep):
    """Build the Skyfield time vector once per process."""
    num_steps = int(duration / timestep) + 1
    datetimes = [start_time + timedelta(seconds=i * timestep) for i in range(num_steps)]
    return ts.utc(
        [t.year for t in datetimes],
        [t.month for t in datetimes],
        [t.day for t in datetimes],
        [t.hour for t in datetimes],
        [t.minute for t in datetimes],
        [t.second + t.microsecond / 1_000_000 for t in datetimes],
    )


def _position_records_from_geocentric(geocentric, timestep, coordinate_system):
    if coordinate_system == 'cartesian':
        positions_km = geocentric.position.km
        return [
            {
                'timestep': idx,
                'time': idx * timestep,
                'x': positions_km[0][idx] * 1000,
                'y': positions_km[1][idx] * 1000,
                'z': positions_km[2][idx] * 1000,
            }
            for idx in range(positions_km.shape[1])
        ]

    if coordinate_system == 'geodetic':
        subpoints = wgs84.subpoint(geocentric)
        return [
            {
                'timestep': idx,
                'time': idx * timestep,
                'latitude': subpoints.latitude.degrees[idx],
                'longitude': subpoints.longitude.degrees[idx],
                'altitude': subpoints.elevation.m[idx],
            }
            for idx in range(len(subpoints.latitude.degrees))
        ]

    raise ValueError(f"Unsupported coordinate system: {coordinate_system}")


def _generate_satellite_positions(sat_info, times, timestep, coordinate_system):
    satellite = EarthSatellite(
        sat_info['line1'],
        sat_info['line2'],
        sat_info['name'],
        _WORKER_TS if _WORKER_TS is not None else load.timescale(),
    )
    positions = _position_records_from_geocentric(
        satellite.at(times),
        timestep,
        coordinate_system,
    )
    return sat_info['id'], {
        'name': sat_info['name'],
        'operator': sat_info['operator'],
        'source_file': sat_info['source_file'],
        'positions': positions,
    }


def _init_position_worker(start_time, duration, timestep, coordinate_system):
    global _WORKER_TIMES, _WORKER_TS, _WORKER_TIMESTEP, _WORKER_COORDINATE_SYSTEM
    _WORKER_TS = load.timescale()
    _WORKER_TIMES = _build_times(_WORKER_TS, start_time, duration, timestep)
    _WORKER_TIMESTEP = timestep
    _WORKER_COORDINATE_SYSTEM = coordinate_system


def _generate_satellite_positions_worker(sat_info):
    return _generate_satellite_positions(
        sat_info,
        _WORKER_TIMES,
        _WORKER_TIMESTEP,
        _WORKER_COORDINATE_SYSTEM,
    )


class TLEPositionGenerator:
    """Generate satellite positions from TLE data."""

    def __init__(self, tle_file, start_time=START, duration=3600, timestep=1.0, workers=1):
        """
        Initialize the TLE position generator.

        Args:
            tle_file (str): Path to TLE file
            start_time (datetime): Start time for simulation (default: now)
            duration (float): Duration in seconds
            timestep (float): Time step in seconds
            workers (int): Number of worker processes for position generation
        """
        self.tle_file = Path(tle_file)
        self.start_time = start_time if start_time else START
        self.duration = duration
        self.timestep = timestep
        self.workers = workers
        self.satellites = []
        self.ts = load.timescale()

    def _input_files(self):
        """Return TLE files to process in deterministic order."""
        if self.tle_file.is_dir():
            return sorted(
                path
                for pattern in ('*.txt', '*.csv')
                for path in self.tle_file.glob(pattern)
            )
        return [self.tle_file]

    @staticmethod
    def _tle_checksum(line):
        """Return the TLE checksum digit for a line without its checksum."""
        total = 0
        for char in line:
            if char.isdigit():
                total += int(char)
            elif char == '-':
                total += 1
        return str(total % 10)

    @staticmethod
    def _format_tle_epoch(value):
        epoch = datetime.fromisoformat(value.replace('Z', '+00:00'))
        if epoch.tzinfo is not None:
            epoch = epoch.astimezone(timezone.utc).replace(tzinfo=None)

        start_of_year = datetime(epoch.year, 1, 1)
        day_of_year = (epoch - start_of_year).days + 1
        seconds = (
            epoch.hour * 3600
            + epoch.minute * 60
            + epoch.second
            + epoch.microsecond / 1_000_000
        )
        return f"{epoch.year % 100:02d}{day_of_year + seconds / 86400:012.8f}"

    @staticmethod
    def _format_tle_decimal(value):
        text = f"{float(value): .8f}"
        return text.replace(" 0.", " .").replace("-0.", "-.")

    @staticmethod
    def _format_tle_exponential(value):
        value = float(value)
        if value == 0.0:
            return " 00000+0"

        sign = '-' if value < 0 else ' '
        value = abs(value)
        exponent = 0
        while value >= 1.0:
            value /= 10.0
            exponent += 1
        while value < 0.1:
            value *= 10.0
            exponent -= 1

        mantissa = round(value * 100000)
        if mantissa == 100000:
            mantissa = 10000
            exponent += 1

        return f"{sign}{mantissa:05d}{exponent:+2d}"

    @staticmethod
    def _format_international_designator(value):
        value = value.strip()
        if not value:
            return "        "

        match = re.match(r'^(\d{4})-(\d{3})([A-Z]{0,3})$', value, re.IGNORECASE)
        if not match:
            return value[:8].ljust(8)

        launch_year = int(match.group(1)) % 100
        launch_number = int(match.group(2))
        launch_piece = match.group(3).upper()[:3]
        return f"{launch_year:02d}{launch_number:03d}{launch_piece:<3s}"

    def _csv_row_to_tle(self, row):
        """Convert a CelesTrak GP CSV/OMM row into classic TLE lines."""
        name = row['OBJECT_NAME'].strip()
        satellite_number = int(row['NORAD_CAT_ID'])
        classification = row.get('CLASSIFICATION_TYPE', 'U').strip() or 'U'
        designator = self._format_international_designator(row.get('OBJECT_ID', ''))
        epoch = self._format_tle_epoch(row['EPOCH'])
        mean_motion_dot = self._format_tle_decimal(row.get('MEAN_MOTION_DOT', 0.0))
        mean_motion_ddot = self._format_tle_exponential(row.get('MEAN_MOTION_DDOT', 0.0))
        bstar = self._format_tle_exponential(row.get('BSTAR', 0.0))
        ephemeris_type = int(float(row.get('EPHEMERIS_TYPE', 0)))
        element_set_number = int(float(row.get('ELEMENT_SET_NO', 999)))

        eccentricity = f"{round(float(row['ECCENTRICITY']) * 10_000_000):07d}"[-7:]
        inclination = float(row['INCLINATION'])
        raan = float(row['RA_OF_ASC_NODE'])
        argument_of_perigee = float(row['ARG_OF_PERICENTER'])
        mean_anomaly = float(row['MEAN_ANOMALY'])
        mean_motion = float(row['MEAN_MOTION'])
        revolution_number = int(float(row.get('REV_AT_EPOCH', 0)))

        line1 = (
            f"1 {satellite_number:05d}{classification[0]} {designator:<8s} {epoch} "
            f"{mean_motion_dot:>10s} {mean_motion_ddot:>8s} {bstar:>8s} "
            f"{ephemeris_type:1d} {element_set_number:4d}"
        )
        line2 = (
            f"2 {satellite_number:05d} {inclination:8.4f} {raan:8.4f} "
            f"{eccentricity:>7s} {argument_of_perigee:8.4f} {mean_anomaly:8.4f} "
            f"{mean_motion:11.8f}{revolution_number:5d}"
        )

        return name, line1 + self._tle_checksum(line1), line2 + self._tle_checksum(line2)

    def _csv_rows(self, tle_path):
        """Yield normalized rows from a CelesTrak GP CSV file."""
        fieldnames = [
            'OBJECT_NAME', 'OBJECT_ID', 'EPOCH', 'MEAN_MOTION', 'ECCENTRICITY',
            'INCLINATION', 'RA_OF_ASC_NODE', 'ARG_OF_PERICENTER',
            'MEAN_ANOMALY', 'EPHEMERIS_TYPE', 'CLASSIFICATION_TYPE',
            'NORAD_CAT_ID', 'ELEMENT_SET_NO', 'REV_AT_EPOCH', 'BSTAR',
            'MEAN_MOTION_DOT', 'MEAN_MOTION_DDOT',
        ]

        with open(tle_path, newline='') as f:
            sample = f.read(1024)
            f.seek(0)
            sample_lines = sample.splitlines()
            if not sample_lines:
                return
            first_row = next(csv.reader([sample_lines[0]]), [])
            has_header = first_row and first_row[0].strip().upper() == 'OBJECT_NAME'
            reader = csv.DictReader(f) if has_header else csv.DictReader(f, fieldnames=fieldnames)
            for row in reader:
                if not row or not row.get('OBJECT_NAME') or row['OBJECT_NAME'].startswith('#'):
                    continue
                yield {key: (value or '').strip() for key, value in row.items()}

    def _add_satellite(self, name, line1, line2, operator, tle_path):
        try:
            satellite = EarthSatellite(line1, line2, name, self.ts)
            self.satellites.append({
                'name': name,
                'line1': line1,
                'line2': line2,
                'satellite': satellite,
                'id': len(self.satellites),
                'operator': operator,
                'source_file': str(tle_path)
            })
            loaded_count = len(self.satellites)
            if loaded_count <= 100:
                print(f"    Loaded: {name} (ID: {loaded_count - 1})")
            elif loaded_count % 1000 == 0:
                print(f"    Loaded {loaded_count} satellites...")
        except Exception as e:
            print(f"    Warning: Failed to parse TLE for {name}: {e}")

    def load_tles(self):
        """Load TLE data from file."""
        input_files = self._input_files()
        print(f"Loading TLEs from {self.tle_file}...")

        for tle_path in input_files:
            operator = tle_path.stem
            print(f"  File: {tle_path} (operator: {operator})")

            if tle_path.suffix.lower() == '.csv':
                for row in self._csv_rows(tle_path):
                    try:
                        name, line1, line2 = self._csv_row_to_tle(row)
                    except Exception as e:
                        print(f"    Warning: Failed to parse CSV row for {row.get('OBJECT_NAME', 'unknown')}: {e}")
                        continue
                    self._add_satellite(name, line1, line2, operator, tle_path)
                continue

            with open(tle_path, 'r') as f:
                lines = f.readlines()

            # Parse TLEs (format: name, line1, line2)
            i = 0
            while i < len(lines):
                # Skip empty lines and comments
                if not lines[i].strip() or lines[i].lstrip().startswith('#'):
                    i += 1
                    continue

                # Check if we have at least 3 lines (name + 2 TLE lines)
                if i + 2 >= len(lines):
                    break

                name = lines[i].strip()
                line1 = lines[i + 1].strip()
                line2 = lines[i + 2].strip()

                # Validate TLE format (lines should start with '1' and '2')
                if line1.startswith('1 ') and line2.startswith('2 '):
                    self._add_satellite(name, line1, line2, operator, tle_path)
                    i += 3
                else:
                    i += 1
        
        print(f"Successfully loaded {len(self.satellites)} satellites\n")
        return len(self.satellites)

    def _effective_worker_count(self):
        if self.workers is None or self.workers <= 0:
            return max(1, os.cpu_count() or 1)
        return self.workers

    def generate_positions(self, coordinate_system='cartesian', reference='geocentric', workers=None):
        """
        Generate satellite positions over time.

        Args:
            coordinate_system (str): 'cartesian' (x,y,z) or 'geodetic' (lat,lon,alt)
            reference (str): 'geocentric' (Earth-centered) or 'topocentric' (ground station)
            workers (int): Optional override for number of worker processes

        Returns:
            dict: Satellite positions organized by satellite ID
        """
        if not self.satellites:
            raise ValueError("No satellites loaded. Call load_tles() first.")
        if coordinate_system not in ('cartesian', 'geodetic'):
            raise ValueError(f"Unsupported coordinate system: {coordinate_system}")

        # Generate time array
        num_steps = int(self.duration / self.timestep) + 1
        times = _build_times(self.ts, self.start_time, self.duration, self.timestep)
        worker_count = self.workers if workers is None else workers
        worker_count = max(1, worker_count if worker_count and worker_count > 0 else (os.cpu_count() or 1))
        worker_count = min(worker_count, len(self.satellites))

        print(f"Generating positions for {len(self.satellites)} satellites")
        print(f"Time range: {self.start_time} to {self.start_time + timedelta(seconds=self.duration)}")
        print(f"Number of timesteps: {num_steps}")
        print(f"Timestep: {self.timestep} seconds")
        print(f"Workers: {worker_count}\n")

        all_positions = {}

        if worker_count > 1:
            sat_infos = [
                {
                    'id': sat_info['id'],
                    'name': sat_info['name'],
                    'line1': sat_info['line1'],
                    'line2': sat_info['line2'],
                    'operator': sat_info['operator'],
                    'source_file': sat_info['source_file'],
                }
                for sat_info in self.satellites
            ]
            chunksize = max(1, len(sat_infos) // (worker_count * 8))

            with concurrent.futures.ProcessPoolExecutor(
                max_workers=worker_count,
                initializer=_init_position_worker,
                initargs=(self.start_time, self.duration, self.timestep, coordinate_system),
            ) as executor:
                for completed, (sat_id, sat_data) in enumerate(
                    executor.map(
                        _generate_satellite_positions_worker,
                        sat_infos,
                        chunksize=chunksize,
                    ),
                    start=1,
                ):
                    all_positions[sat_id] = sat_data
                    if completed == len(sat_infos) or completed % 1000 == 0:
                        print(f"  Generated positions for {completed}/{len(sat_infos)} satellites")

            return all_positions

        for sat_info in self.satellites:
            sat_id = sat_info['id']
            sat_name = sat_info['name']
            satellite = sat_info['satellite']

            positions = _position_records_from_geocentric(
                satellite.at(times),
                self.timestep,
                coordinate_system,
            )
            
            all_positions[sat_id] = {
                'name': sat_name,
                'operator': sat_info['operator'],
                'source_file': sat_info['source_file'],
                'positions': positions
            }
            
            if len(self.satellites) <= 100:
                print(f"  Generated {len(positions)} positions for {sat_name} (ID: {sat_id})")
            elif (sat_id + 1) % 1000 == 0 or sat_id + 1 == len(self.satellites):
                print(f"  Generated positions for {sat_id + 1}/{len(self.satellites)} satellites")
        
        return all_positions

    def save_to_csv(self, positions, output_file, coordinate_system='cartesian'):
        """
        Save positions to CSV file.

        Args:
            positions (dict): Satellite positions from generate_positions()
            output_file (str): Output CSV file path
            coordinate_system (str): 'cartesian' or 'geodetic'
        """
        output_path = Path(output_file)
        if output_path.suffix == '':
            output_path = output_path / 'satellite_positions.csv'
        output_path.parent.mkdir(parents=True, exist_ok=True)

        print(f"\nSaving positions to {output_path}...")

        with open(output_path, 'w') as f:
            # Write header
            if coordinate_system == 'cartesian':
                f.write("sat_id,sat_name,timestep,time_s,x_m,y_m,z_m\n")
            else:
                f.write("sat_id,sat_name,timestep,time_s,latitude_deg,longitude_deg,altitude_m\n")
            
            # Write data for each satellite
            for sat_id, sat_data in sorted(positions.items()):
                sat_name = sat_data['name']
                for pos in sat_data['positions']:
                    if coordinate_system == 'cartesian':
                        f.write(f"{sat_id},{sat_name},{pos['timestep']},{pos['time']:.3f},"
                               f"{pos['x']:.3f},{pos['y']:.3f},{pos['z']:.3f}\n")
                    else:
                        f.write(f"{sat_id},{sat_name},{pos['timestep']},{pos['time']:.3f},"
                               f"{pos['latitude']:.6f},{pos['longitude']:.6f},{pos['altitude']:.3f}\n")
        
        print(f"Successfully saved positions to {output_path}")

    def _position_results(self, coordinate_system='cartesian', workers=None):
        if not self.satellites:
            raise ValueError("No satellites loaded. Call load_tles() first.")
        if coordinate_system not in ('cartesian', 'geodetic'):
            raise ValueError(f"Unsupported coordinate system: {coordinate_system}")

        worker_count = self.workers if workers is None else workers
        worker_count = max(1, worker_count if worker_count and worker_count > 0 else (os.cpu_count() or 1))
        worker_count = min(worker_count, len(self.satellites))

        if worker_count > 1:
            sat_infos = [
                {
                    'id': sat_info['id'],
                    'name': sat_info['name'],
                    'line1': sat_info['line1'],
                    'line2': sat_info['line2'],
                    'operator': sat_info['operator'],
                    'source_file': sat_info['source_file'],
                }
                for sat_info in self.satellites
            ]
            chunksize = max(1, len(sat_infos) // (worker_count * 8))
            with concurrent.futures.ProcessPoolExecutor(
                max_workers=worker_count,
                initializer=_init_position_worker,
                initargs=(self.start_time, self.duration, self.timestep, coordinate_system),
            ) as executor:
                yield from executor.map(
                    _generate_satellite_positions_worker,
                    sat_infos,
                    chunksize=chunksize,
                )
            return

        times = _build_times(self.ts, self.start_time, self.duration, self.timestep)
        for sat_info in self.satellites:
            positions = _position_records_from_geocentric(
                sat_info['satellite'].at(times),
                self.timestep,
                coordinate_system,
            )
            yield sat_info['id'], {
                'name': sat_info['name'],
                'operator': sat_info['operator'],
                'source_file': sat_info['source_file'],
                'positions': positions,
            }

    def generate_positions_csv(self, output_file, coordinate_system='cartesian', workers=None):
        """Generate positions and stream them directly to a CSV file."""
        output_path = Path(output_file)
        if output_path.suffix == '':
            output_path = output_path / 'satellite_positions.csv'
        output_path.parent.mkdir(parents=True, exist_ok=True)

        num_steps = int(self.duration / self.timestep) + 1
        worker_count = self.workers if workers is None else workers
        worker_count = max(1, worker_count if worker_count and worker_count > 0 else (os.cpu_count() or 1))
        worker_count = min(worker_count, len(self.satellites))

        print(f"Generating positions for {len(self.satellites)} satellites")
        print(f"Time range: {self.start_time} to {self.start_time + timedelta(seconds=self.duration)}")
        print(f"Number of timesteps: {num_steps}")
        print(f"Timestep: {self.timestep} seconds")
        print(f"Workers: {worker_count}")
        print(f"\nSaving positions to {output_path}...")

        with open(output_path, 'w') as f:
            if coordinate_system == 'cartesian':
                f.write("sat_id,sat_name,timestep,time_s,x_m,y_m,z_m\n")
            else:
                f.write("sat_id,sat_name,timestep,time_s,latitude_deg,longitude_deg,altitude_m\n")

            for completed, (sat_id, sat_data) in enumerate(
                self._position_results(coordinate_system, worker_count),
                start=1,
            ):
                sat_name = sat_data['name']
                for pos in sat_data['positions']:
                    if coordinate_system == 'cartesian':
                        f.write(f"{sat_id},{sat_name},{pos['timestep']},{pos['time']:.3f},"
                                f"{pos['x']:.3f},{pos['y']:.3f},{pos['z']:.3f}\n")
                    else:
                        f.write(f"{sat_id},{sat_name},{pos['timestep']},{pos['time']:.3f},"
                                f"{pos['latitude']:.6f},{pos['longitude']:.6f},{pos['altitude']:.3f}\n")

                if completed == len(self.satellites) or completed % 1000 == 0:
                    print(f"  Wrote positions for {completed}/{len(self.satellites)} satellites")

        print(f"Successfully saved positions to {output_path}")

    def save_to_ns3_format(self, positions, output_file):
        """
        Save positions in ns-3 mobility trace format.
        
        Format: $ns_ at <time> "$node_(<id>) set X_ <x>"
                $ns_ at <time> "$node_(<id>) set Y_ <y>"
                $ns_ at <time> "$node_(<id>) set Z_ <z>"
        """
        output_path = Path(output_file)
        if output_path.suffix == '':
            output_path = output_path / 'satellite_mobility.tcl'
        output_path.parent.mkdir(parents=True, exist_ok=True)

        print(f"\nSaving positions to ns-3 format: {output_path}...")

        with open(output_path, 'w') as f:
            for sat_id, sat_data in sorted(positions.items()):
                for pos in sat_data['positions']:
                    time = pos['time']
                    f.write(f"$ns_ at {time:.3f} \"$node_({sat_id}) set X_ {pos['x']:.3f}\"\n")
                    f.write(f"$ns_ at {time:.3f} \"$node_({sat_id}) set Y_ {pos['y']:.3f}\"\n")
                    f.write(f"$ns_ at {time:.3f} \"$node_({sat_id}) set Z_ {pos['z']:.3f}\"\n")

        print(f"Successfully saved ns-3 mobility trace to {output_path}")

        # Also save satellite names for trace consumers
        names_file = f"{output_path}.names.csv"
        with open(names_file, 'w') as nf:
            nf.write("sat_id,sat_name,operator,source_file\n")
            for sat_id, sat_data in sorted(positions.items()):
                nf.write(f"{sat_id},{sat_data['name']},{sat_data['operator']},{sat_data['source_file']}\n")

        print(f"Saved satellite names to {names_file}")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Generate satellite positions from TLE files for ns-3 mobility',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate positions from TLE file
  %(prog)s -i satellites.tle -o positions.csv
  
  # Generate positions with custom duration and timestep
  %(prog)s -i satellites.tle -o positions.csv -d 7200 -t 10

  # Generate a large constellation using all available CPU cores
  %(prog)s -i celestrak.csv -o positions.csv -d 300 -t 1 --workers 0
  
  # Generate geodetic coordinates
  %(prog)s -i satellites.tle -o positions.csv --geodetic
  
  # Generate ns-3 mobility trace format
  %(prog)s -i satellites.tle -o mobility.tcl --ns3-format
        """
    )
    
    parser.add_argument('-i', '--input', default='contrib/leosim/data/tles',
                       help='Input TLE file path')
    parser.add_argument('-o', '--output', default='contrib/leosim/data/prepro',
                       help='Output file path')
    parser.add_argument('-d', '--duration', type=float, default=3600,
                       help='Simulation duration in seconds (default: 3600)')
    parser.add_argument('-t', '--timestep', type=float, default=1.0,
                       help='Time step in seconds (default: 1.0)')
    parser.add_argument('-w', '--workers', type=int, default=10,
                       help='Worker processes for position generation; use 0 for all CPU cores (default: 1)')
    parser.add_argument('--geodetic', action='store_true',
                       help='Output geodetic coordinates (lat, lon, alt) instead of Cartesian (x, y, z)')
    parser.add_argument('--ns3-format', action='store_true',
                       help='Output in ns-3 mobility trace format')
    parser.add_argument('--start-time', type=str,
                       help='Start time in ISO format (default: current time), e.g., 2025-09-14T12:00:00')
    
    args = parser.parse_args()

    if args.duration < 0:
        print("ERROR: duration must be non-negative")
        sys.exit(1)
    if args.timestep <= 0:
        print("ERROR: timestep must be greater than zero")
        sys.exit(1)
    if args.workers < 0:
        print("ERROR: workers must be zero or greater")
        sys.exit(1)
    
    # Parse start time if provided
    start_time = START
    if args.start_time:
        try:
            start_time = datetime.fromisoformat(args.start_time)
        except ValueError:
            print(f"ERROR: Invalid start time format: {args.start_time}")
            print("Use ISO format: YYYY-MM-DDTHH:MM:SS")
            sys.exit(1)
    
    # Create generator
    generator = TLEPositionGenerator(
        tle_file=args.input,
        start_time=start_time,
        duration=args.duration,
        timestep=args.timestep,
        workers=args.workers
    )
    
    # Load TLEs
    if generator.load_tles() == 0:
        print("ERROR: No valid TLEs found in input file")
        sys.exit(1)
    
    coordinate_system = 'geodetic' if args.geodetic else 'cartesian'
    
    # Save output
    if args.ns3_format:
        if args.geodetic:
            print("WARNING: ns-3 format requires Cartesian coordinates. Converting...")
        positions = generator.generate_positions(coordinate_system='cartesian')
        generator.save_to_ns3_format(positions, args.output)
    else:
        generator.generate_positions_csv(args.output, coordinate_system=coordinate_system)
    
    print("\nDone!")


if __name__ == '__main__':
    main()
