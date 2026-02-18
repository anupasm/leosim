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
import sys
from datetime import datetime, timedelta
import numpy as np

try:
    from skyfield.api import load, EarthSatellite, wgs84
    from skyfield.timelib import Time
except ImportError:
    print("ERROR: skyfield library not found. Install it with: pip install skyfield")
    sys.exit(1)


class TLEPositionGenerator:
    """Generate satellite positions from TLE data."""

    def __init__(self, tle_file, start_time=None, duration=3600, timestep=1.0):
        """
        Initialize the TLE position generator.

        Args:
            tle_file (str): Path to TLE file
            start_time (datetime): Start time for simulation (default: now)
            duration (float): Duration in seconds
            timestep (float): Time step in seconds
        """
        self.tle_file = tle_file
        self.start_time = start_time if start_time else datetime.utcnow()
        self.duration = duration
        self.timestep = timestep
        self.satellites = []
        self.ts = load.timescale()

    def load_tles(self):
        """Load TLE data from file."""
        print(f"Loading TLEs from {self.tle_file}...")
        
        with open(self.tle_file, 'r') as f:
            lines = f.readlines()
        
        # Parse TLEs (format: name, line1, line2)
        i = 0
        while i < len(lines):
            # Skip empty lines
            if not lines[i].strip():
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
                try:
                    satellite = EarthSatellite(line1, line2, name, self.ts)
                    self.satellites.append({
                        'name': name,
                        'satellite': satellite,
                        'id': len(self.satellites)
                    })
                    print(f"  Loaded: {name} (ID: {len(self.satellites) - 1})")
                except Exception as e:
                    print(f"  Warning: Failed to parse TLE for {name}: {e}")
                
                i += 3
            else:
                i += 1
        
        print(f"Successfully loaded {len(self.satellites)} satellites\n")
        return len(self.satellites)

    def generate_positions(self, coordinate_system='cartesian', reference='geocentric'):
        """
        Generate satellite positions over time.

        Args:
            coordinate_system (str): 'cartesian' (x,y,z) or 'geodetic' (lat,lon,alt)
            reference (str): 'geocentric' (Earth-centered) or 'topocentric' (ground station)

        Returns:
            dict: Satellite positions organized by satellite ID
        """
        if not self.satellites:
            raise ValueError("No satellites loaded. Call load_tles() first.")

        # Generate time array
        num_steps = int(self.duration / self.timestep) + 1
        times = []
        
        for i in range(num_steps):
            t = self.start_time + timedelta(seconds=i * self.timestep)
            times.append(self.ts.utc(t.year, t.month, t.day, t.hour, t.minute, t.second))

        print(f"Generating positions for {len(self.satellites)} satellites")
        print(f"Time range: {self.start_time} to {self.start_time + timedelta(seconds=self.duration)}")
        print(f"Number of timesteps: {num_steps}")
        print(f"Timestep: {self.timestep} seconds\n")

        # Generate positions for each satellite
        all_positions = {}
        
        for sat_info in self.satellites:
            sat_id = sat_info['id']
            sat_name = sat_info['name']
            satellite = sat_info['satellite']
            
            positions = []
            
            for idx, t in enumerate(times):
                geocentric = satellite.at(t)
                
                if coordinate_system == 'cartesian':
                    # Get position in kilometers (geocentric)
                    pos = geocentric.position.km
                    x, y, z = pos[0] * 1000, pos[1] * 1000, pos[2] * 1000  # Convert to meters
                    
                    positions.append({
                        'timestep': idx,
                        'time': idx * self.timestep,
                        'x': x,
                        'y': y,
                        'z': z
                    })
                
                elif coordinate_system == 'geodetic':
                    # Get geodetic coordinates (lat, lon, altitude)
                    subpoint = wgs84.subpoint(geocentric)
                    
                    positions.append({
                        'timestep': idx,
                        'time': idx * self.timestep,
                        'latitude': subpoint.latitude.degrees,
                        'longitude': subpoint.longitude.degrees,
                        'altitude': subpoint.elevation.m
                    })
            
            all_positions[sat_id] = {
                'name': sat_name,
                'positions': positions
            }
            
            print(f"  Generated {len(positions)} positions for {sat_name} (ID: {sat_id})")
        
        return all_positions

    def save_to_csv(self, positions, output_file, coordinate_system='cartesian'):
        """
        Save positions to CSV file.

        Args:
            positions (dict): Satellite positions from generate_positions()
            output_file (str): Output CSV file path
            coordinate_system (str): 'cartesian' or 'geodetic'
        """
        print(f"\nSaving positions to {output_file}...")
        
        with open(output_file, 'w') as f:
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
        
        print(f"Successfully saved positions to {output_file}")

    def save_to_ns3_format(self, positions, output_file):
        """
        Save positions in ns-3 mobility trace format.
        
        Format: $ns_ at <time> "$node_(<id>) set X_ <x>"
                $ns_ at <time> "$node_(<id>) set Y_ <y>"
                $ns_ at <time> "$node_(<id>) set Z_ <z>"
        """
        print(f"\nSaving positions to ns-3 format: {output_file}...")
        
        with open(output_file, 'w') as f:
            for sat_id, sat_data in sorted(positions.items()):
                for pos in sat_data['positions']:
                    time = pos['time']
                    f.write(f"$ns_ at {time:.3f} \"$node_({sat_id}) set X_ {pos['x']:.3f}\"\n")
                    f.write(f"$ns_ at {time:.3f} \"$node_({sat_id}) set Y_ {pos['y']:.3f}\"\n")
                    f.write(f"$ns_ at {time:.3f} \"$node_({sat_id}) set Z_ {pos['z']:.3f}\"\n")
        
        print(f"Successfully saved ns-3 mobility trace to {output_file}")

        # Also save satellite names for trace consumers
        names_file = f"{output_file}.names.csv"
        with open(names_file, 'w') as nf:
            nf.write("sat_id,sat_name\n")
            for sat_id, sat_data in sorted(positions.items()):
                nf.write(f"{sat_id},{sat_data['name']}\n")

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
  
  # Generate geodetic coordinates
  %(prog)s -i satellites.tle -o positions.csv --geodetic
  
  # Generate ns-3 mobility trace format
  %(prog)s -i satellites.tle -o mobility.tcl --ns3-format
        """
    )
    
    parser.add_argument('-i', '--input', required=True,
                       help='Input TLE file path')
    parser.add_argument('-o', '--output', required=True,
                       help='Output file path')
    parser.add_argument('-d', '--duration', type=float, default=3600,
                       help='Simulation duration in seconds (default: 3600)')
    parser.add_argument('-t', '--timestep', type=float, default=1.0,
                       help='Time step in seconds (default: 1.0)')
    parser.add_argument('--geodetic', action='store_true',
                       help='Output geodetic coordinates (lat, lon, alt) instead of Cartesian (x, y, z)')
    parser.add_argument('--ns3-format', action='store_true',
                       help='Output in ns-3 mobility trace format')
    parser.add_argument('--start-time', type=str,
                       help='Start time in ISO format (default: current time), e.g., 2025-09-14T12:00:00')
    
    args = parser.parse_args()
    
    # Parse start time if provided
    start_time = None
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
        timestep=args.timestep
    )
    
    # Load TLEs
    if generator.load_tles() == 0:
        print("ERROR: No valid TLEs found in input file")
        sys.exit(1)
    
    # Generate positions
    coordinate_system = 'geodetic' if args.geodetic else 'cartesian'
    positions = generator.generate_positions(coordinate_system=coordinate_system)
    
    # Save output
    if args.ns3_format:
        if args.geodetic:
            print("WARNING: ns-3 format requires Cartesian coordinates. Converting...")
            positions = generator.generate_positions(coordinate_system='cartesian')
        generator.save_to_ns3_format(positions, args.output)
    else:
        generator.save_to_csv(positions, args.output, coordinate_system=coordinate_system)
    
    print("\nDone!")


if __name__ == '__main__':
    main()
