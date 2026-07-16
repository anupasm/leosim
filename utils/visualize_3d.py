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
3D Visualization of LEO Satellite Network on Earth Globe

This script visualizes satellite, server, and UE positions from ns-3 simulation
data on an interactive 3D Earth globe with animated trajectories.

Usage:
    python3 visualize_3d.py --position_file leosim_positions.csv [options]

Requirements:
    pip install plotly numpy
"""

import argparse
import csv
import sys
import numpy as np
try:
    import plotly.graph_objects as go
except ModuleNotFoundError as e:
    raise SystemExit(
        "Missing dependency: plotly. Install it with: pip install plotly"
    ) from e

# Earth radius in meters for globe rendering
EARTH_RADIUS = 6378137.0

# WGS84 ellipsoid constants (match ns-3 loader conversion)
WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014
SERVING_STATES = {'CONNECTED', 'EXECUTING', 'SERVING'}
PARTICIPATING_LINK_STATES = SERVING_STATES | {'ACTIVE', 'ESTABLISHED'}
DEFAULT_MAX_STALENESS_S = 1.5
GROUND_MARKER_LIFT_M = 120000.0
UE_MARKER_LIFT_M = 80000.0
SERVER_MARKER_COLOR = 'orange'
SERVER_MARKER_LINE_COLOR = 'darkorange'
UE_MARKER_COLOR = 'blue'
UE_MARKER_LINE_COLOR = 'darkblue'


def geodetic_to_cartesian(lat_deg, lon_deg, altitude_m=0.0):
    """Convert geodetic coordinates (degrees, meters) to WGS84 ECEF cartesian."""
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)

    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    cos_lon = np.cos(lon)
    sin_lon = np.sin(lon)

    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (n + altitude_m) * cos_lat * cos_lon
    y = (n + altitude_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + altitude_m) * sin_lat
    return np.array([x, y, z])


def cartesian_to_geodetic(pos_xyz, radius=EARTH_RADIUS):
    """Convert WGS84 ECEF cartesian coordinates to geodetic lat/lon/alt."""
    x, y, z = pos_xyz

    if x == 0.0 and y == 0.0 and z == 0.0:
        return 0.0, 0.0, -radius

    lon = np.arctan2(y, x)
    p = np.sqrt(x * x + y * y)

    # Initialize latitude then refine iteratively.
    lat = np.arctan2(z, p * (1.0 - WGS84_E2))
    for _ in range(10):
        sin_lat = np.sin(lat)
        n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat_next = np.arctan2(z + WGS84_E2 * n * sin_lat, p)
        if np.abs(lat_next - lat) < 1e-12:
            lat = lat_next
            break
        lat = lat_next

    sin_lat = np.sin(lat)
    n = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / np.cos(lat) - n

    lat = np.degrees(lat)
    lon = np.degrees(lon)

    # Clamp tiny negative altitudes from numerical precision.
    if abs(alt) < 1.0:
        alt = 0.0
    return lat, lon, alt


def beam_footprint_circle(lat_deg, lon_deg, radius_km, num_points=36):
    """Generate 3D cartesian points for a beam footprint circle on Earth surface."""
    if radius_km <= 0:
        return None

    lat1 = np.radians(lat_deg)
    lon1 = np.radians(lon_deg)
    angular_distance = (radius_km * 1000.0) / EARTH_RADIUS

    xs, ys, zs = [], [], []
    for i in range(num_points + 1):
        bearing = 2.0 * np.pi * (i / num_points)

        lat2 = np.arcsin(
            np.sin(lat1) * np.cos(angular_distance)
            + np.cos(lat1) * np.sin(angular_distance) * np.cos(bearing)
        )
        lon2 = lon1 + np.arctan2(
            np.sin(bearing) * np.sin(angular_distance) * np.cos(lat1),
            np.cos(angular_distance) - np.sin(lat1) * np.sin(lat2),
        )

        p = geodetic_to_cartesian(np.degrees(lat2), np.degrees(lon2))
        xs.append(p[0])
        ys.append(p[1])
        zs.append(p[2])

    return xs, ys, zs


def lifted_marker_position(pos_xyz, lift_m=90000.0):
    """Lift a marker radially away from Earth so event markers are not hidden."""
    pos = np.array(pos_xyz, dtype=float)
    norm = np.linalg.norm(pos)
    if norm < 1.0:
        return pos
    return pos + (pos / norm) * lift_m


def load_position_data(filename):
    """Load position data from CSV file."""
    data = {
        'SATELLITE': {},
        'SERVER': {},
        'UE': {}
    }
    
    try:
        with open(filename, 'r') as f:
            # Skip header
            next(f)
            
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 7:
                    continue
                
                time = float(parts[0])
                node_type = parts[1]
                node_id = int(parts[2])
                node_name = parts[3]
                x = float(parts[4])
                y = float(parts[5])
                z = float(parts[6])
                
                if node_type not in data:
                    continue
                
                if node_id not in data[node_type]:
                    data[node_type][node_id] = {
                        'name': node_name,
                        'times': [],
                        'positions': []
                    }
                
                data[node_type][node_id]['times'].append(time)
                data[node_type][node_id]['positions'].append([x, y, z])
        
        # Convert lists to numpy arrays
        for node_type in data:
            for node_id in data[node_type]:
                data[node_type][node_id]['times'] = \
                    np.array(data[node_type][node_id]['times'])
                data[node_type][node_id]['positions'] = \
                    np.array(data[node_type][node_id]['positions'])
        
        return data
    
    except FileNotFoundError:
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading file: {e}")
        sys.exit(1)

def load_link_data(filename):
    """Load channel-visible links from legacy or unified link-state CSV files."""
    links = {}  # time_key -> list of dicts

    try:
        with open(filename, 'r') as f:
            for raw in csv.DictReader(f):
                row = _norm_row(raw)
                time = _safe_float(_first(row, ['time']))
                link_type = _first(row, ['link_type'], '').upper()
                sat_id = _node_id(_first(row, ['sat_id', 'satellite_id']))
                peer_id = _node_id(_first(row, ['ground_id', 'peer_id', 'node2_id']))
                peer_type = _first(row, ['ground_type', 'peer_type', 'link_type'], 'UNKNOWN').upper()
                beam_id = _node_id(_first(row, ['beam_id']), -1)

                if link_type == 'ISL':
                    sat_id = _node_id(_first(row, ['node1_id', 'sat_id', 'satellite_id']))
                    peer_id = _node_id(_first(row, ['node2_id', 'peer_id']))
                    peer_type = 'ISL'

                if np.isnan(time) or sat_id < 0 or peer_id < 0:
                    continue

                time_key = f"{time:.3f}"
                links.setdefault(time_key, []).append({
                    'link_type': link_type,
                    'sat_id': sat_id,
                    'peer_id': peer_id,
                    'peer_type': peer_type,
                    'beam_id': beam_id,
                    'link_state': _first(row, ['channel_state', 'link_state', 'state'], 'UNKNOWN').upper(),
                })

        return links

    except FileNotFoundError:
        print(f"Link file '{filename}' not found, skipping link visualization.")
        return {}
    except Exception as e:
        print(f"Error loading link file: {e}")
        return {}


def _get_col_index(header, aliases):
    """Return first matching index from aliases, or None."""
    for name in aliases:
        if name in header:
            return header.index(name)
    return None


def _norm_row(row):
    """Normalize DictReader keys and trim cell whitespace."""
    return {
        (key or '').strip().lower(): (value.strip() if isinstance(value, str) else value)
        for key, value in row.items()
    }


def _first(row, aliases, default=''):
    """Read the first present CSV field from aliases."""
    for name in aliases:
        if name in row and row[name] not in (None, ''):
            return row[name]
    return default


def _safe_float(value, default=float('nan')):
    try:
        if value in (None, ''):
            return default
        return float(value)
    except (TypeError, ValueError):
        return default


def _safe_int(value, default=-1):
    try:
        if value in (None, ''):
            return default
        return int(value)
    except (TypeError, ValueError):
        return default


def _node_id(value, default=-1):
    """Parse node/beam ids and map UINT32_MAX invalid sentinels to default."""
    parsed = _safe_int(value, default)
    return default if parsed >= 4294967295 else parsed


def _parse_csv_bool(value, default=True):
    """Parse bool-ish CSV values such as 1/0, true/false, yes/no."""
    if value is None:
        return default

    token = str(value).strip().lower()
    if token in ('1', 'true', 't', 'yes', 'y', 'on'):
        return True
    if token in ('0', 'false', 'f', 'no', 'n', 'off'):
        return False
    return default


def _latest_time_key_at_or_before(time_keys_sorted, current_time):
    """Return latest key <= current_time from a sorted list of string time keys."""
    if not time_keys_sorted:
        return None

    current = float(current_time)
    last_key = None
    for key in time_keys_sorted:
        t = float(key)
        if t <= current:
            last_key = key
        else:
            break
    return last_key


def _records_at_time(series, time_keys_sorted, current_time, max_staleness_s=DEFAULT_MAX_STALENESS_S):
    """Return records at the latest sample <= current_time, bounded by staleness."""
    key = _latest_time_key_at_or_before(time_keys_sorted, current_time)
    if key is None:
        return None, []
    if max_staleness_s is not None and float(current_time) - float(key) > max_staleness_s:
        return None, []
    return key, series.get(key, [])


def load_coverage_data(filename):
    """Load satellite ground coverage data from CSV file.

        Supported formats:
            Legacy link-based:
                time,satellite_id,ground_id,ground_type,distance_m,elevation_deg,snr_db,link_state
            Beam-footprint based:
                time,satellite_id,beam_id,cell_id,color_group,center_lat,center_lon,radius_km,active_in_slot
    Returns:
      Dict[time_key -> List[dict]]
    """
    coverage = {}
    try:
        with open(filename, 'r') as f:
            header = next(f, '').strip().lower().split(',')
            if not header:
                return {}

            time_idx = _get_col_index(header, ['time'])
            sat_idx = _get_col_index(header, ['satellite_id', 'sat_id'])

            # Beam-footprint format columns
            beam_idx = _get_col_index(header, ['beam_id'])
            cell_idx = _get_col_index(header, ['cell_id'])
            colr_idx = _get_col_index(header, ['color_group'])
            blat_idx = _get_col_index(header, ['center_lat', 'beam_center_lat'])
            blon_idx = _get_col_index(header, ['center_lon', 'beam_center_lon'])
            brad_idx = _get_col_index(header, ['radius_km', 'beam_radius_km'])
            bact_idx = _get_col_index(header, ['active_in_slot', 'beam_active'])

            is_beam_format = (
                beam_idx is not None and blat_idx is not None and blon_idx is not None and brad_idx is not None
            )

            if is_beam_format:
                required = [time_idx, sat_idx, beam_idx, blat_idx, blon_idx, brad_idx]
                if any(v is None for v in required):
                    print(f"Coverage file '{filename}' has unexpected beam header, skipping coverage visualization.")
                    return {}

                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) <= max(required):
                        continue
                    try:
                        t = float(parts[time_idx])
                        sat_id = int(parts[sat_idx])
                        beam_id = int(parts[beam_idx])
                        center_lat = float(parts[blat_idx])
                        center_lon = float(parts[blon_idx])
                        radius_km = float(parts[brad_idx])
                    except ValueError:
                        continue

                    entry = {
                        'coverage_mode': 'beam',
                        'time': t,
                        'satellite_id': sat_id,
                        'beam_id': beam_id,
                        'cell_id': int(parts[cell_idx]) if cell_idx is not None and cell_idx < len(parts) else -1,
                        'color_group': int(parts[colr_idx]) if colr_idx is not None and colr_idx < len(parts) else -1,
                        'center_lat': center_lat,
                        'center_lon': center_lon,
                        'radius_km': radius_km,
                        'active_in_slot': _parse_csv_bool(parts[bact_idx], True) if bact_idx is not None and bact_idx < len(parts) else True,
                    }

                    time_key = f"{t:.3f}"
                    coverage.setdefault(time_key, []).append(entry)

                return coverage

            # Legacy link-based format columns
            grd_idx = _get_col_index(header, ['ground_id', 'node_id'])
            gtp_idx = _get_col_index(header, ['ground_type', 'node_type'])
            dis_idx = _get_col_index(header, ['distance_m', 'distance'])
            elv_idx = _get_col_index(header, ['elevation_deg', 'elevation'])
            snr_idx = _get_col_index(header, ['snr_db', 'snr'])
            stt_idx = _get_col_index(header, ['link_state', 'state'])

            required = [time_idx, sat_idx, grd_idx, gtp_idx]
            if any(v is None for v in required):
                print(f"Coverage file '{filename}' has unexpected header, skipping coverage visualization.")
                return {}

            for line in f:
                parts = line.strip().split(',')
                if len(parts) <= max(required):
                    continue
                try:
                    t = float(parts[time_idx])
                    sat_id = int(parts[sat_idx])
                    ground_id = int(parts[grd_idx])
                except ValueError:
                    continue

                entry = {
                    'coverage_mode': 'link',
                    'time': t,
                    'satellite_id': sat_id,
                    'ground_id': ground_id,
                    'ground_type': parts[gtp_idx].upper(),
                    'distance_m': float(parts[dis_idx]) if dis_idx is not None and dis_idx < len(parts) else np.nan,
                    'elevation_deg': float(parts[elv_idx]) if elv_idx is not None and elv_idx < len(parts) else np.nan,
                    'snr_db': float(parts[snr_idx]) if snr_idx is not None and snr_idx < len(parts) else np.nan,
                    'link_state': parts[stt_idx].upper() if stt_idx is not None and stt_idx < len(parts) else 'UNKNOWN',
                }

                time_key = f"{t:.3f}"
                coverage.setdefault(time_key, []).append(entry)

        return coverage

    except FileNotFoundError:
        print(f"Coverage file '{filename}' not found, skipping coverage visualization.")
        return {}
    except Exception as e:
        print(f"Error loading coverage file: {e}")
        return {}


def load_link_quality_data(filename):
    """Load out-of-threshold link quality data from CSV file.

    Expected columns include:
      time,node1_id,node2_id,link_type,snr_db,distance_m,elevation_deg,path_loss_db,
      signal_strength_dbm,link_state,degradation_reason
    Returns:
      Dict[time_key -> List[dict]]
    """
    quality = {}
    try:
        with open(filename, 'r') as f:
            header = next(f, '').strip().lower().split(',')
            if not header:
                return {}

            time_idx = _get_col_index(header, ['time'])
            n1_idx = _get_col_index(header, ['node1_id', 'src_id'])
            n2_idx = _get_col_index(header, ['node2_id', 'dst_id'])
            ltp_idx = _get_col_index(header, ['link_type', 'type'])
            snr_idx = _get_col_index(header, ['snr_db', 'snr'])
            dis_idx = _get_col_index(header, ['distance_m', 'distance'])
            elv_idx = _get_col_index(header, ['elevation_deg', 'elevation'])
            pls_idx = _get_col_index(header, ['path_loss_db', 'path_loss'])
            sgn_idx = _get_col_index(header, ['signal_strength_dbm', 'signal_dbm'])
            stt_idx = _get_col_index(header, ['link_state', 'state'])
            rsn_idx = _get_col_index(header, ['degradation_reason', 'reason'])

            required = [time_idx, n1_idx, n2_idx]
            if any(v is None for v in required):
                print(f"Link quality file '{filename}' has unexpected header, skipping quality visualization.")
                return {}

            for line in f:
                parts = line.strip().split(',')
                if len(parts) <= max(required):
                    continue
                try:
                    t = float(parts[time_idx])
                    node1_id = int(parts[n1_idx])
                    node2_id = int(parts[n2_idx])
                except ValueError:
                    continue

                entry = {
                    'time': t,
                    'node1_id': node1_id,
                    'node2_id': node2_id,
                    'link_type': parts[ltp_idx].upper() if ltp_idx is not None and ltp_idx < len(parts) else 'UNKNOWN',
                    'snr_db': float(parts[snr_idx]) if snr_idx is not None and snr_idx < len(parts) else np.nan,
                    'distance_m': float(parts[dis_idx]) if dis_idx is not None and dis_idx < len(parts) else np.nan,
                    'elevation_deg': float(parts[elv_idx]) if elv_idx is not None and elv_idx < len(parts) else np.nan,
                    'path_loss_db': float(parts[pls_idx]) if pls_idx is not None and pls_idx < len(parts) else np.nan,
                    'signal_strength_dbm': float(parts[sgn_idx]) if sgn_idx is not None and sgn_idx < len(parts) else np.nan,
                    'link_state': parts[stt_idx].upper() if stt_idx is not None and stt_idx < len(parts) else 'UNKNOWN',
                    'degradation_reason': parts[rsn_idx] if rsn_idx is not None and rsn_idx < len(parts) else 'UNKNOWN',
                }

                time_key = f"{t:.3f}"
                quality.setdefault(time_key, []).append(entry)

        return quality

    except FileNotFoundError:
        print(f"Link quality file '{filename}' not found, skipping quality visualization.")
        return {}
    except Exception as e:
        print(f"Error loading link quality file: {e}")
        return {}

def load_packet_data(filename):
    """Load packet data from CSV file."""
    packets = {}  # time_key -> list of events

    try:
        with open(filename, 'r') as f:
            next(f)

            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 7:
                    continue

                time = float(parts[0])
                event = parts[1]
                node_id = int(parts[2])
                device_id = int(parts[3])

                if len(parts) >= 9:
                    peer_node_id = int(parts[4])
                    link_type = parts[5]
                    size_bytes = int(parts[6])
                    snr_db = float(parts[7])
                    doppler_hz = float(parts[8])
                else:
                    peer_node_id = -1
                    link_type = "UNKNOWN"
                    size_bytes = int(parts[4])
                    snr_db = float(parts[5])
                    doppler_hz = float(parts[6])

                time_key = f"{time:.3f}"
                packets.setdefault(time_key, []).append({
                    'time': time,
                    'event': event,
                    'node_id': node_id,
                    'device_id': device_id,
                    'peer_node_id': peer_node_id,
                    'link_type': link_type,
                    'size_bytes': size_bytes,
                    'snr_db': snr_db,
                    'doppler_hz': doppler_hz,
                })

        return packets

    except FileNotFoundError:
        print(f"Packet file '{filename}' not found, skipping packet visualization.")
        return {}


def load_beam_data(filename):
    """Load beam state data from CSV file.

    Supported headers:
      New: time,ground_id,ground_type,sat_id,beam_id,cell_id,color_group,rsrp_dbm,sinr_db,
           intra_ici_dbm,inter_ici_dbm,elevation_deg,tte_sec,beam_load,beam_util,beam_active,
           topsis_score,state
      Old: time,ue_id,sat_id,...
    Returns: Dict[time_key -> List[dict]] where each dict includes all available columns.
    """
    beams = {}
    try:
        with open(filename, 'r') as f:
            header = next(f, '').strip().lower().split(',')
            if not header:
                return {}

            time_idx = _get_col_index(header, ['time'])
            gid_idx = _get_col_index(header, ['ground_id', 'ue_id'])
            gtp_idx = _get_col_index(header, ['ground_type'])
            sat_idx = _get_col_index(header, ['sat_id', 'satellite_id'])
            beam_idx = _get_col_index(header, ['beam_id'])
            cell_idx = _get_col_index(header, ['cell_id'])
            colr_idx = _get_col_index(header, ['color_group'])
            rsrp_idx = _get_col_index(header, ['rsrp_dbm', 'rsrp'])
            sinr_idx = _get_col_index(header, ['sinr_db', 'sinr'])
            intra_idx = _get_col_index(header, ['intra_ici_dbm', 'intra_ici'])
            inter_idx = _get_col_index(header, ['inter_ici_dbm', 'inter_ici'])
            elv_idx  = _get_col_index(header, ['elevation_deg', 'elevation'])
            tte_idx  = _get_col_index(header, ['tte_sec', 'tte'])
            load_idx = _get_col_index(header, ['beam_load'])
            util_idx = _get_col_index(header, ['beam_util'])
            bact_idx = _get_col_index(header, ['beam_active'])
            tops_idx = _get_col_index(header, ['topsis_score'])
            stt_idx  = _get_col_index(header, ['state'])

            def _f(parts, idx):
                """Safe float parse from parts list at idx, returns nan on failure."""
                if idx is not None and idx < len(parts) and parts[idx] != '':
                    try:
                        return float(parts[idx])
                    except ValueError:
                        pass
                return float('nan')

            required = [time_idx, gid_idx, sat_idx]
            if any(v is None for v in required):
                print(f"Beam file '{filename}' has unexpected header, skipping beam visualization.")
                return {}

            for line in f:
                parts = line.strip().split(',')
                if len(parts) <= max(v for v in required if v is not None):
                    continue
                try:
                    t = float(parts[time_idx])
                    ground_id = int(parts[gid_idx])
                    sat_id = int(parts[sat_idx])
                except ValueError:
                    continue

                time_key = f"{t:.3f}"
                ground_type = 'UE'
                if gtp_idx is not None and gtp_idx < len(parts):
                    ground_type = parts[gtp_idx].strip().upper() or 'UE'

                entry = {
                    'ground_id': ground_id,
                    'ground_type': ground_type,
                    'sat_id': _node_id(sat_id),
                    'beam_id': _node_id(parts[beam_idx]) if beam_idx is not None and beam_idx < len(parts) and parts[beam_idx] != '' else -1,
                    'cell_id': _node_id(parts[cell_idx]) if cell_idx is not None and cell_idx < len(parts) and parts[cell_idx] != '' else -1,
                    'color_group': int(parts[colr_idx]) if colr_idx is not None and colr_idx < len(parts) and parts[colr_idx] != '' else -1,
                    'rsrp_dbm': _f(parts, rsrp_idx),
                    'sinr_db': _f(parts, sinr_idx),
                    'intra_ici_dbm': _f(parts, intra_idx),
                    'inter_ici_dbm': _f(parts, inter_idx),
                    'elevation_deg': _f(parts, elv_idx),
                    'tte_sec': _f(parts, tte_idx),
                    'beam_load': _f(parts, load_idx),
                    'beam_util': _f(parts, util_idx),
                    'beam_active': _parse_csv_bool(parts[bact_idx], True) if bact_idx is not None and bact_idx < len(parts) else True,
                    'topsis_score': _f(parts, tops_idx),
                    'state': parts[stt_idx].strip().upper() if stt_idx is not None and stt_idx < len(parts) else '',
                }

                beams.setdefault(time_key, []).append(entry)

        return beams

    except FileNotFoundError:
        print(f"Beam file '{filename}' not found, skipping beam visualization.")
        return {}
    except Exception as e:
        print(f"Error loading beam file: {e}")
        return {}


def load_handover_data(filename):
    """Load handover events from CSV file.

    Expected header:
      time_ms,ue_id,src_sat,tgt_sat,src_beam,src_cell,tgt_beam,tgt_cell,
      mode,type,trigger,latency_ms,buff_pkts,drop_pkts,success,route_change,
      sinr_before,sinr_after
    Returns: List[dict] with time_s float and event metadata.
    """
    events = []
    try:
        with open(filename, 'r') as f:
            header = next(f, '').strip().lower().split(',')
            if not header:
                return []

            def idx(name):
                try:
                    return header.index(name)
                except ValueError:
                    return None

            time_idx    = idx('time_ms')
            ue_idx      = idx('ue_id')
            src_idx     = idx('src_sat')
            tgt_idx     = idx('tgt_sat')
            sbeam_idx   = idx('src_beam')
            scell_idx   = idx('src_cell')
            tbeam_idx   = idx('tgt_beam')
            tcell_idx   = idx('tgt_cell')
            mode_idx    = idx('mode')
            type_idx    = idx('type')
            trig_idx    = idx('trigger')
            lat_idx     = idx('latency_ms')
            buf_idx     = idx('buff_pkts')
            drp_idx     = idx('drop_pkts')
            succ_idx    = idx('success')
            rchg_idx    = idx('route_change')
            sinrb_idx   = idx('sinr_before')
            sinra_idx   = idx('sinr_after')

            required = [time_idx, ue_idx, src_idx, tgt_idx]
            if any(v is None for v in required):
                print(f"Handover file '{filename}' has unexpected header, skipping handover visualization.")
                return []

            def _safe_str(parts, i):
                return parts[i] if i is not None and i < len(parts) else ''

            def _safe_float(parts, i):
                if i is not None and i < len(parts) and parts[i] != '':
                    try:
                        return float(parts[i])
                    except ValueError:
                        pass
                return float('nan')

            def _safe_int(parts, i):
                if i is not None and i < len(parts) and parts[i] != '':
                    try:
                        return int(parts[i])
                    except ValueError:
                        pass
                return -1

            for line in f:
                parts = line.strip().split(',')
                if len(parts) <= max(v for v in required if v is not None):
                    continue
                try:
                    time_ms = float(parts[time_idx])
                    time_s = time_ms / 1000.0
                    ue_id = int(parts[ue_idx])
                    src_sat = int(parts[src_idx])
                    tgt_sat = int(parts[tgt_idx])
                except ValueError:
                    continue

                evt = {
                    'time_s': time_s,
                    'ue_id': ue_id,
                    'src_sat': src_sat,
                    'tgt_sat': tgt_sat,
                    'src_beam': _safe_int(parts, sbeam_idx),
                    'src_cell': _safe_int(parts, scell_idx),
                    'tgt_beam': _safe_int(parts, tbeam_idx),
                    'tgt_cell': _safe_int(parts, tcell_idx),
                    'mode': _safe_str(parts, mode_idx),
                    'type': _safe_str(parts, type_idx),
                    'trigger': _safe_str(parts, trig_idx),
                    'latency_ms': _safe_float(parts, lat_idx),
                    'buff_pkts': _safe_int(parts, buf_idx),
                    'drop_pkts': _safe_int(parts, drp_idx),
                    'success': _safe_str(parts, succ_idx),
                    'route_change': _safe_str(parts, rchg_idx),
                    'sinr_before': _safe_float(parts, sinrb_idx),
                    'sinr_after': _safe_float(parts, sinra_idx),
                }
                events.append(evt)

        return events

    except FileNotFoundError:
        print(f"Handover file '{filename}' not found, skipping handover visualization.")
        return []
    except Exception as e:
        print(f"Error loading handover file: {e}")
        return []

def build_packet_flows(packets, node_index, time_window=0.5):
    """
    Build packet flow paths by matching TX and RX events.
    Creates arrows showing packet transmission direction.
    
    Args:
        packets: Dict of time -> packet events
        node_index: Node information indexed by node_id
        time_window: Time window (seconds) to match TX/RX pairs
    
    Returns:
        Dict of time -> list of (src_pos, dst_pos, packet_info)
    """
    flows = {}  # time_key -> list of flow arrows
    
    # Sort all events by time
    all_events = []
    for time_key, events in packets.items():
        all_events.extend(events)
    all_events.sort(key=lambda e: e['time'])
    
    # Match TX events with subsequent RX events
    tx_buffer = []  # Pending TX events
    matched_count = 0
    unmatched_rx = 0
    
    for event in all_events:
        time_key = f"{event['time']:.3f}"
        
        if event['event'] == 'TX':
            tx_buffer.append(event)
            # Clean old TX events outside time window
            tx_buffer = [tx for tx in tx_buffer 
                        if event['time'] - tx['time'] <= time_window]
        
        elif event['event'] == 'RX':
            # Try to match with a recent TX using peer_node_id and link_type
            matched_tx = None
            
            # First priority: match using peer_node_id if available
            if event.get('peer_node_id', -1) >= 0:
                for tx in tx_buffer:
                    # Match if:
                    # 1. TX node is RX's peer (TX from the peer)
                    # 2. Same size
                    # 3. Same link type
                    if (tx['node_id'] == event['peer_node_id'] and 
                        tx['size_bytes'] == event['size_bytes'] and
                        tx.get('link_type') == event.get('link_type')):
                        matched_tx = tx
                        break
            
            # Fallback: match by size and link type only
            if not matched_tx:
                for tx in tx_buffer:
                    if (tx['node_id'] != event['node_id'] and 
                        tx['size_bytes'] == event['size_bytes'] and
                        tx.get('link_type') == event.get('link_type')):
                        matched_tx = tx
                        break
            
            if matched_tx:
                # Get positions
                src_node = node_index.get(matched_tx['node_id'])
                dst_node = node_index.get(event['node_id'])
                
                if src_node and dst_node:
                    src_pos = get_position_at_time(src_node, matched_tx['time'])
                    dst_pos = get_position_at_time(dst_node, event['time'])
                    if src_pos is not None and dst_pos is not None:
                        # Use actual TX time (floor to nearest second) for animation frame
                        # This ensures flows appear in the correct frame
                        tx_time = int(matched_tx['time'])
                        if tx_time not in flows:
                            flows[tx_time] = []
                        
                        flows[tx_time].append({
                            'src_pos': src_pos,
                            'dst_pos': dst_pos,
                            'src_node': matched_tx['node_id'],
                            'dst_node': event['node_id'],
                            'src_type': src_node['type'],
                            'dst_type': dst_node['type'],
                            'size': matched_tx['size_bytes'],
                            'tx_time': matched_tx['time'],
                            'rx_time': event['time'],
                            'snr': event['snr_db'],
                            'link_type': event.get('link_type', matched_tx.get('link_type', 'UNKNOWN'))
                        })
                        matched_count += 1
                
                # Remove matched TX
                tx_buffer.remove(matched_tx)
            else:
                unmatched_rx += 1
    
    print(f"  Matched {matched_count} TX/RX pairs out of {len(all_events)} events")
    print(f"  Unmatched RX events: {unmatched_rx}")
    print(f"  Flow frames: {sorted(flows.keys())}")
    for time_key in sorted(flows.keys())[:5]:
        print(f"    Time {time_key}s: {len(flows[time_key])} flows")
        for flow in flows[time_key][:3]:
            print(f"      {flow['src_type']} {flow['src_node']} -> {flow['dst_type']} {flow['dst_node']} ({flow['link_type']})")
    return flows

def create_earth_sphere(radius=EARTH_RADIUS):
    """Create a sphere representing Earth."""
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)
    x = radius * np.outer(np.cos(u), np.sin(v))
    y = radius * np.outer(np.sin(u), np.sin(v))
    z = radius * np.outer(np.ones(np.size(u)), np.cos(v))
    
    return go.Surface(
        x=x, y=y, z=z,
        colorscale=[[0, 'lightblue'], [1, 'blue']],
        showscale=False,
        opacity=0.3,
        name='Earth',
        hoverinfo='none'
    )

def build_node_index(data):
    """Build node index by NodeList id."""
    node_index = {}
    for node_type in ['SATELLITE', 'SERVER', 'UE']:
        for node_id, node_data in data[node_type].items():
            node_index[node_id] = {
                'type': node_type,
                'name': node_data['name'],
                'times': node_data['times'],
                'positions': node_data['positions']
            }
    return node_index


def filter_participating_nodes(data, links, packets, beams, handovers):
    """Return position data containing only nodes that truly participate in activity.

    Coverage and reachability-only records are deliberately excluded so the
    visualization focuses on nodes involved in packets, serving beams,
    handovers, or active/established links.
    """
    participating = {node_type: set() for node_type in data}
    node_index = build_node_index(data)

    def add_global_node(node_id):
        entry = node_index.get(node_id)
        if entry is not None:
            participating[entry['type']].add(node_id)

    for records in packets.values():
        for record in records:
            add_global_node(record.get('node_id', -1))
            add_global_node(record.get('peer_node_id', -1))

    for records in beams.values():
        for record in records:
            if not record.get('beam_active', True):
                continue
            state = str(record.get('state', '')).upper()
            if state not in SERVING_STATES:
                continue
            sat_id = record.get('sat_id', -1)
            ground_id = record.get('ground_id', -1)
            ground_type = str(record.get('ground_type', 'UE')).upper()
            if sat_id in data['SATELLITE']:
                participating['SATELLITE'].add(sat_id)
            if ground_type in ('SERVER', 'UE') and ground_id in data[ground_type]:
                participating[ground_type].add(ground_id)

    for event in handovers:
        ue_id = event.get('ue_id', -1)
        if ue_id in data['UE']:
            participating['UE'].add(ue_id)
        for field in ('src_sat', 'tgt_sat'):
            sat_id = event.get(field, -1)
            if sat_id in data['SATELLITE']:
                participating['SATELLITE'].add(sat_id)

    for records in links.values():
        for record in records:
            state = str(record.get('link_state', '')).upper()
            if state not in PARTICIPATING_LINK_STATES:
                continue
            sat_id = record.get('sat_id', -1)
            peer_id = record.get('peer_id', -1)
            peer_type = str(record.get('peer_type', '')).upper()
            if sat_id in data['SATELLITE']:
                participating['SATELLITE'].add(sat_id)
            if peer_type in ('ISL', 'SATELLITE') and peer_id in data['SATELLITE']:
                participating['SATELLITE'].add(peer_id)
            elif peer_type in ('SERVER', 'UE') and peer_id in data[peer_type]:
                participating[peer_type].add(peer_id)

    if not any(participating.values()):
        return {node_type: {} for node_type in data}

    filtered = {
        node_type: {
            node_id: node_data
            for node_id, node_data in nodes.items()
            if node_id in participating[node_type]
        }
        for node_type, nodes in data.items()
    }
    return filtered

def get_position_at_time(node_entry, current_time):
    """Get node position at or before current time."""
    times = node_entry['times']
    positions = node_entry['positions']
    if len(times) == 0:
        return None

    idx = np.searchsorted(times, current_time)
    if idx >= len(positions):
        idx = len(positions) - 1
    return positions[idx]

def visualize_animated(
    data,
    links,
    packets,
    beams,
    handovers,
    coverage,
    link_quality,
    max_frames=None,
    output_file=None,
    show_flows=True,
    show_channel_links=False,
    show_isl_links=True,
    show_beams=True,
    show_handovers=True,
    show_coverage=True,
    show_link_quality=True,
):
    """Create animated 3D visualization with interactive globe and channel links."""
    # Get all unique timestamps
    all_times = set()
    for sat_data in data['SATELLITE'].values():
        all_times.update(sat_data['times'])
    
    times = sorted(list(all_times))
    if max_frames and len(times) > max_frames:
        times = times[:max_frames]
    
    print(f"Creating animation with {len(times)} frames...")
    
    # Debug: Print what servers and UEs we have
    print(f"Servers: {sorted(data['SERVER'].keys())}")
    print(f"UEs: {sorted(data['UE'].keys())}")
    if links:
        print(f"Links available for {len(links)} time steps")
    if beams:
        print(f"Beams available for {len(beams)} time steps")
    if handovers:
        print(f"Handovers loaded: {len(handovers)} events")
    if coverage:
        print(f"Coverage loaded for {len(coverage)} time steps")
    if link_quality:
        print(f"Link quality loaded for {len(link_quality)} time steps")
    
    node_index = build_node_index(data)
    
    # Build packet flows if requested
    flows = {}
    if packets and show_flows:
        print("Building packet flow paths...")
        flows = build_packet_flows(packets, node_index)
        print(f"Found {sum(len(f) for f in flows.values())} packet flows")

    coverage_time_keys = sorted(coverage.keys(), key=float) if coverage else []
    beams_time_keys = sorted(beams.keys(), key=float) if beams else []
    links_time_keys = sorted(links.keys(), key=float) if links else []
    link_quality_time_keys = sorted(link_quality.keys(), key=float) if link_quality else []
    has_beam_coverage = (
        show_coverage and coverage and
        any(
            any(c.get('coverage_mode') == 'beam' for c in records)
            for records in coverage.values()
        )
    )

    # Prepare frames for animation
    frames = []
    
    for frame_idx, current_time in enumerate(times):
        frame_data = []

        time_key = f"{current_time:.3f}"
        next_time = times[frame_idx + 1] if frame_idx + 1 < len(times) else current_time + 1.0
        ho_events_frame = [e for e in handovers if current_time <= e['time_s'] < next_time] if handovers else []
        coverage_time_key = _latest_time_key_at_or_before(coverage_time_keys, current_time)
        coverage_records = coverage[coverage_time_key] if coverage and coverage_time_key is not None else []
        is_beam_coverage = any(c.get('coverage_mode') == 'beam' for c in coverage_records)
        beam_serving_x, beam_serving_y, beam_serving_z = [], [], []
        beam_non_serving_x, beam_non_serving_y, beam_non_serving_z = [], [], []
        center_x, center_y, center_z, center_h, center_c, center_labels = [], [], [], [], [], []

        if has_beam_coverage and is_beam_coverage:
            serving_beam_ids = set()
            beams_key, beam_records_for_coverage = _records_at_time(
                beams,
                beams_time_keys,
                float(coverage_time_key),
            )
            if beam_records_for_coverage:
                for rec in beam_records_for_coverage:
                    bid = rec.get('beam_id', -1)
                    sid = rec.get('sat_id')
                    state = rec.get('state', '').upper()
                    if (bid >= 0 and sid is not None and rec.get('beam_active', True) and
                            state in SERVING_STATES):
                        serving_beam_ids.add((sid, bid))

            for c in coverage_records:
                if c.get('coverage_mode') != 'beam':
                    continue

                circle = beam_footprint_circle(c['center_lat'], c['center_lon'], c['radius_km'])
                if circle is None:
                    continue

                cx, cy, cz = circle
                is_active = c.get('active_in_slot', True)
                is_serving = (c['satellite_id'], c['beam_id']) in serving_beam_ids
                if is_serving:
                    beam_serving_x.extend(cx + [None])
                    beam_serving_y.extend(cy + [None])
                    beam_serving_z.extend(cz + [None])
                else:
                    beam_non_serving_x.extend(cx + [None])
                    beam_non_serving_y.extend(cy + [None])
                    beam_non_serving_z.extend(cz + [None])

                center_pos = geodetic_to_cartesian(c['center_lat'], c['center_lon'])
                center_x.append(center_pos[0]); center_y.append(center_pos[1]); center_z.append(center_pos[2])
                center_h.append(
                    f"SAT {c['satellite_id']} Beam {c['beam_id']}<br>"
                    f"Cell: {c['cell_id']} | Color: {c['color_group']}<br>"
                    f"Center: ({c['center_lat']:.4f}, {c['center_lon']:.4f}) deg<br>"
                    f"Radius: {c['radius_km']:.1f} km<br>"
                    f"Active slot: {'YES' if is_active else 'NO'}<br>"
                    f"Serving: {'YES' if is_serving else 'NO'}"
                )
                center_c.append('blue' if is_serving else 'green')
                center_labels.append(str(c['beam_id']) if c['beam_id'] >= 0 else '')
        
        # Add Earth to every frame
        frame_data.append(create_earth_sphere())
        
        # Add satellites at current time - ALWAYS add in same order
        satellite_ids = sorted(data['SATELLITE'].keys())
        for sat_id in satellite_ids:
            sat_data = data['SATELLITE'][sat_id]
            idx = np.searchsorted(sat_data['times'], current_time)
            if idx < len(sat_data['positions']):
                pos = sat_data['positions'][idx]
                sat_lat, sat_lon, sat_alt = cartesian_to_geodetic(pos)
                sat_hover = (
                    f"Satellite {sat_id}: {sat_data['name']}<br>"
                    f"Time: {current_time:.1f}s<br>"
                    f"Lat: {sat_lat:.5f} deg<br>"
                    f"Lon: {sat_lon:.5f} deg<br>"
                    f"Alt: {sat_alt:.1f} m"
                )
                frame_data.append(go.Scatter3d(
                    x=[pos[0]], y=[pos[1]], z=[pos[2]],
                    mode='markers+text',
                    marker=dict(size=4, color='red', symbol='circle'),
                    text=[f'SAT{sat_id}'],
                    textposition='top center',
                    name=f'Satellite {sat_id}',
                    legendgroup=f'sat{sat_id}',
                    hovertext=sat_hover,
                    hoverinfo='text',
                    showlegend=True,
                    visible=True
                ))
        
        # Add servers (stationary) - ALWAYS add in same order
        server_ids = sorted(data['SERVER'].keys())
        for srv_id in server_ids:
            srv_data = data['SERVER'][srv_id]
            if len(srv_data['positions']) > 0:
                pos = srv_data['positions'][0]
                marker_pos = lifted_marker_position(pos, GROUND_MARKER_LIFT_M)
                srv_lat, srv_lon, srv_alt = cartesian_to_geodetic(pos)
                srv_hover = (
                    f"Server: {srv_data['name']}<br>"
                    f"Lat: {srv_lat:.5f} deg<br>"
                    f"Lon: {srv_lon:.5f} deg<br>"
                    f"Alt: {srv_alt:.1f} m"
                )
                frame_data.append(go.Scatter3d(
                    x=[marker_pos[0]], y=[marker_pos[1]], z=[marker_pos[2]],
                    mode='markers+text',
                    marker=dict(
                        size=2.5,
                        color=SERVER_MARKER_COLOR,
                        symbol='diamond',
                        line=dict(color=SERVER_MARKER_LINE_COLOR, width=1)
                    ),
                    text=[srv_data['name']],
                    textposition='top center',
                    name=f'Server {srv_id}',
                    hovertext=srv_hover,
                    hoverinfo='text',
                    showlegend=True,
                    visible=True
                ))
        
        # Add UEs (stationary) - ALWAYS add in same order
        ue_ids = sorted(data['UE'].keys())
        for ue_id in ue_ids:
            ue_data = data['UE'][ue_id]
            if len(ue_data['positions']) > 0:
                pos = ue_data['positions'][0]
                marker_pos = lifted_marker_position(pos, UE_MARKER_LIFT_M)
                ue_lat, ue_lon, ue_alt = cartesian_to_geodetic(pos)
                ue_hover = (
                    f"UE: {ue_data['name']}<br>"
                    f"Lat: {ue_lat:.5f} deg<br>"
                    f"Lon: {ue_lon:.5f} deg<br>"
                    f"Alt: {ue_alt:.1f} m"
                )
                frame_data.append(go.Scatter3d(
                    x=[marker_pos[0]], y=[marker_pos[1]], z=[marker_pos[2]],
                    mode='markers+text',
                    marker=dict(
                        size=2.5,
                        color=UE_MARKER_COLOR,
                        symbol='circle',
                        opacity=1.0,
                        line=dict(color=UE_MARKER_LINE_COLOR, width=1)
                    ),
                    text=[ue_data['name']],
                    textposition='top center',
                    name=f'UE {ue_id}',
                    hovertext=ue_hover,
                    hoverinfo='text',
                    showlegend=True,
                    visible=True
                ))

        if has_beam_coverage:
            frame_data.append(go.Scatter3d(
                x=beam_serving_x, y=beam_serving_y, z=beam_serving_z,
                mode='lines',
                line=dict(color='blue', width=4),
                opacity=0.95,
                name='Serving beam boundaries',
                showlegend=True,
                hoverinfo='skip',
                visible=True
            ))
            frame_data.append(go.Scatter3d(
                x=beam_non_serving_x, y=beam_non_serving_y, z=beam_non_serving_z,
                mode='lines',
                line=dict(color='green', width=2),
                opacity=0.8,
                name='Non-serving beam boundaries',
                showlegend=True,
                hoverinfo='skip',
                visible=True
            ))
            frame_data.append(go.Scatter3d(
                x=center_x, y=center_y, z=center_z,
                mode='markers+text',
                marker=dict(size=3, color=center_c),
                text=center_labels,
                textposition='middle center',
                textfont=dict(size=9, color='black'),
                name='All beam centers',
                hovertext=center_h,
                hoverinfo='text',
                showlegend=True,
                visible=True
            ))
        
        # Add established ISL topology links from the unified link-state file.
        # These are satellite mesh links, not packet-flow events.
        links_time_key, link_records = _records_at_time(links, links_time_keys, current_time)
        if show_isl_links and links and link_records:
            drawn_isl_pairs = set()
            for link in link_records:
                if link.get('peer_type') != 'ISL':
                    continue

                link_state = link.get('link_state', 'UNKNOWN')
                if link_state in ('DOWN', 'LEOSIM_LINK_DOWN'):
                    continue

                sat_id = link['sat_id']
                peer_id = link['peer_id']
                pair = tuple(sorted((sat_id, peer_id)))
                if pair in drawn_isl_pairs:
                    continue
                drawn_isl_pairs.add(pair)

                if sat_id not in data['SATELLITE'] or peer_id not in data['SATELLITE']:
                    continue

                sat_pos = get_position_at_time(data['SATELLITE'][sat_id], current_time)
                peer_pos = get_position_at_time(data['SATELLITE'][peer_id], current_time)
                if sat_pos is None or peer_pos is None:
                    continue

                sat_name = data['SATELLITE'][sat_id].get('name', f'SAT{sat_id}')
                peer_name = data['SATELLITE'][peer_id].get('name', f'SAT{peer_id}')
                hover_text = (
                    f"<b>Established ISL</b><br>"
                    f"<b>Satellite 1:</b> {sat_id} ({sat_name})<br>"
                    f"<b>Satellite 2:</b> {peer_id} ({peer_name})<br>"
                    f"<b>State:</b> {link_state}<br>"
                    f"<b>Layer:</b> ISL topology"
                )

                frame_data.append(go.Scatter3d(
                    x=[sat_pos[0], peer_pos[0]],
                    y=[sat_pos[1], peer_pos[1]],
                    z=[sat_pos[2], peer_pos[2]],
                    mode='lines',
                    line=dict(color='rgba(255,215,0,0.95)', width=4),
                    opacity=0.75,
                    showlegend=False,
                    name=f'Established ISL SAT{sat_id}-SAT{peer_id}',
                    hovertext=hover_text,
                    hoverinfo='text',
                    visible=True
                ))

        # Add raw channel-visible access links if available. These are physical/channel
        # visibility records, not authoritative serving access links.
        if show_channel_links and links and link_records:
            for link in link_records:
                if link.get('peer_type') == 'ISL':
                    continue
                sat_id = link['sat_id']
                ground_id = link['peer_id']
                ground_type = link['peer_type']
                beam_id = link['beam_id']
                # Get satellite position
                if sat_id in data['SATELLITE']:
                    sat_data = data['SATELLITE'][sat_id]
                    sat_idx = np.searchsorted(sat_data['times'], current_time)
                    if sat_idx < len(sat_data['positions']):
                        sat_pos = sat_data['positions'][sat_idx]

                        # Get endpoint position
                        other_pos = None
                        link_color = 'rgba(180,180,180,0.7)'
                        link_name = f'Channel-visible SAT{sat_id}-{ground_type}{ground_id}'

                        # Gather more details for hover text
                        sat_name = sat_data.get('name', f'SAT{sat_id}')
                        ground_name = None
                        if ground_type == 'SERVER' and ground_id in data['SERVER']:
                            other_pos = data['SERVER'][ground_id]['positions'][0]
                            ground_name = data['SERVER'][ground_id].get('name', f'SERVER{ground_id}')
                        elif ground_type == 'UE' and ground_id in data['UE']:
                            other_pos = data['UE'][ground_id]['positions'][0]
                            ground_name = data['UE'][ground_id].get('name', f'UE{ground_id}')
                        elif ground_type in ('ISL', 'SATELLITE') and ground_id in data['SATELLITE']:
                            other_data = data['SATELLITE'][ground_id]
                            other_idx = np.searchsorted(other_data['times'], current_time)
                            if other_idx < len(other_data['positions']):
                                other_pos = other_data['positions'][other_idx]
                                ground_name = other_data.get('name', f'SAT{ground_id}')
                                link_color = 'yellow'
                                link_name = f'Channel-visible ISL SAT{sat_id}-SAT{ground_id}'

                        if other_pos is not None:
                            # Compose detailed hover text
                            hover_lines = []
                            if ground_type in ('SERVER', 'UE'):
                                hover_lines.append(f"<b>Satellite:</b> {sat_id} ({sat_name})")
                                hover_lines.append(f"<b>Ground Node:</b> {ground_id} ({ground_name}) [{ground_type}]")
                                if beam_id >= 0:
                                    hover_lines.append(f"<b>Beam:</b> {beam_id}")
                                else:
                                    hover_lines.append(f"<b>Beam:</b> N/A")
                                hover_lines.append(f"<b>Layer:</b> Channel visibility")
                                hover_lines.append(f"<b>Type:</b> SATELLITE-{ground_type}")
                                hover_lines.append("<b>Routing:</b> Not necessarily serving/usable")
                            elif ground_type in ('ISL', 'SATELLITE'):
                                hover_lines.append(f"<b>ISL Link</b>")
                                hover_lines.append(f"<b>Satellite 1:</b> {sat_id} ({sat_name})")
                                hover_lines.append(f"<b>Satellite 2:</b> {ground_id} ({ground_name})")
                                hover_lines.append(f"<b>Layer:</b> Channel visibility")
                                hover_lines.append(f"<b>Type:</b> ISL")
                            hover_text = '<br>'.join(hover_lines)

                            # Add line for the link
                            frame_data.append(go.Scatter3d(
                                x=[sat_pos[0], other_pos[0]],
                                y=[sat_pos[1], other_pos[1]],
                                z=[sat_pos[2], other_pos[2]],
                                mode='lines',
                                line=dict(color=link_color, width=2),
                                opacity=0.45,
                                showlegend=False,
                                name=link_name,
                                hovertext=hover_text,
                                hoverinfo='text',
                                visible=True
                            ))

        # Add ground coverage links and endpoints (UP/DEGRADED states).
        # Use the most recent available coverage sample at or before this frame time,
        # because logger and position timestamps may not align exactly.
        if coverage and coverage_time_key is not None:
            if is_beam_coverage:
                pass
            else:
                cov_up_x, cov_up_y, cov_up_z = [], [], []
                cov_deg_x, cov_deg_y, cov_deg_z = [], [], []
                ep_up_x, ep_up_y, ep_up_z, ep_up_h = [], [], [], []
                ep_deg_x, ep_deg_y, ep_deg_z, ep_deg_h = [], [], [], []

                for c in coverage_records:
                    sat_id = c['satellite_id']
                    ground_id = c['ground_id']
                    ground_type = c['ground_type']
                    link_state = c['link_state']

                    if sat_id not in data['SATELLITE']:
                        continue

                    sat_data = data['SATELLITE'][sat_id]
                    sat_idx = np.searchsorted(sat_data['times'], current_time)
                    if sat_idx >= len(sat_data['positions']):
                        continue
                    sat_pos = sat_data['positions'][sat_idx]

                    ground_pos = None
                    if ground_type == 'SERVER' and ground_id in data['SERVER']:
                        ground_pos = data['SERVER'][ground_id]['positions'][0]
                    elif ground_type == 'UE' and ground_id in data['UE']:
                        ground_pos = data['UE'][ground_id]['positions'][0]

                    if ground_pos is None:
                        continue

                    hover = (
                        f"Coverage {link_state}<br>"
                        f"SAT {sat_id} -> {ground_type} {ground_id}<br>"
                        f"Distance: {c['distance_m']:.1f} m<br>"
                        f"Elevation: {c['elevation_deg']:.2f} deg<br>"
                        f"SNR: {c['snr_db']:.2f} dB"
                    )

                    if link_state == 'UP':
                        cov_up_x.extend([sat_pos[0], ground_pos[0], None])
                        cov_up_y.extend([sat_pos[1], ground_pos[1], None])
                        cov_up_z.extend([sat_pos[2], ground_pos[2], None])
                        marker_pos = lifted_marker_position(ground_pos, GROUND_MARKER_LIFT_M)
                        ep_up_x.append(marker_pos[0]); ep_up_y.append(marker_pos[1]); ep_up_z.append(marker_pos[2]); ep_up_h.append(hover)
                    else:
                        cov_deg_x.extend([sat_pos[0], ground_pos[0], None])
                        cov_deg_y.extend([sat_pos[1], ground_pos[1], None])
                        cov_deg_z.extend([sat_pos[2], ground_pos[2], None])
                        marker_pos = lifted_marker_position(ground_pos, GROUND_MARKER_LIFT_M)
                        ep_deg_x.append(marker_pos[0]); ep_deg_y.append(marker_pos[1]); ep_deg_z.append(marker_pos[2]); ep_deg_h.append(hover)

                if cov_up_x:
                    frame_data.append(go.Scatter3d(
                        x=cov_up_x, y=cov_up_y, z=cov_up_z,
                        mode='lines',
                        line=dict(color='lightgreen', width=2),
                        opacity=0.45,
                        name='Coverage UP',
                        showlegend=True,
                        hoverinfo='skip'
                    ))
                if cov_deg_x:
                    frame_data.append(go.Scatter3d(
                        x=cov_deg_x, y=cov_deg_y, z=cov_deg_z,
                        mode='lines',
                        line=dict(color='orange', width=2, dash='dot'),
                        opacity=0.55,
                        name='Coverage DEGRADED',
                        showlegend=True,
                        hoverinfo='skip'
                    ))
                if ep_up_x:
                    frame_data.append(go.Scatter3d(
                        x=ep_up_x, y=ep_up_y, z=ep_up_z,
                        mode='markers',
                        marker=dict(
                            size=2,
                            color=SERVER_MARKER_COLOR,
                            symbol='circle',
                            line=dict(color=SERVER_MARKER_LINE_COLOR, width=1)
                        ),
                        name='Covered Ground (UP)',
                        hovertext=ep_up_h,
                        hoverinfo='text',
                        showlegend=True
                    ))
                if ep_deg_x:
                    frame_data.append(go.Scatter3d(
                        x=ep_deg_x, y=ep_deg_y, z=ep_deg_z,
                        mode='markers',
                        marker=dict(
                            size=2,
                            color=SERVER_MARKER_COLOR,
                            symbol='circle-open',
                            line=dict(color=SERVER_MARKER_LINE_COLOR, width=1)
                        ),
                        name='Covered Ground (DEGRADED)',
                        hovertext=ep_deg_h,
                        hoverinfo='text',
                        showlegend=True
                    ))

        # Add out-of-threshold link quality details
        link_quality_time_key, link_quality_records = _records_at_time(
            link_quality,
            link_quality_time_keys,
            current_time,
        )
        if show_link_quality and link_quality_records:
            q_down_x, q_down_y, q_down_z = [], [], []
            q_deg_x, q_deg_y, q_deg_z = [], [], []
            q_h_x, q_h_y, q_h_z, q_h_t = [], [], [], []

            for q in link_quality_records:
                node1 = node_index.get(q['node1_id'])
                node2 = node_index.get(q['node2_id'])
                if not node1 or not node2:
                    continue
                p1 = get_position_at_time(node1, current_time)
                p2 = get_position_at_time(node2, current_time)
                if p1 is None or p2 is None:
                    continue

                link_state = str(q.get('link_state', 'UNKNOWN')).upper()
                if link_state in ('DOWN', 'LEOSIM_LINK_DOWN'):
                    q_down_x.extend([p1[0], p2[0], None])
                    q_down_y.extend([p1[1], p2[1], None])
                    q_down_z.extend([p1[2], p2[2], None])
                elif link_state in ('DEGRADED', 'LEOSIM_LINK_DEGRADED'):
                    q_deg_x.extend([p1[0], p2[0], None])
                    q_deg_y.extend([p1[1], p2[1], None])
                    q_deg_z.extend([p1[2], p2[2], None])
                else:
                    # UP/OPERATIONAL (or unknown) links are not out-of-threshold.
                    continue

                # Lookup beam information from links data
                beam_info = "N/A"
                if links and links_time_key and links_time_key in links:
                    for link in links[links_time_key]:
                        sat_id = link['sat_id']
                        ground_id = link['peer_id']
                        beam_id = link['beam_id']
                        # Check if this link matches the current link_quality record
                        if (q['node1_id'] == sat_id and q['node2_id'] == ground_id) or \
                           (q['node2_id'] == sat_id and q['node1_id'] == ground_id):
                            beam_info = f"{beam_id}" if beam_id >= 0 else "ISL"
                            break

                mid = 0.5 * (p1 + p2)
                q_h_x.append(mid[0]); q_h_y.append(mid[1]); q_h_z.append(mid[2])
                q_h_t.append(
                    f"{link_state} {q['link_type']}<br>"
                    f"{q['node1_id']} -> {q['node2_id']}<br>"
                    f"Beam: {beam_info}<br>"
                    f"SNR: {q['snr_db']:.2f} dB<br>"
                    f"Distance: {q['distance_m']:.1f} m<br>"
                    f"Elevation: {q['elevation_deg']:.2f} deg<br>"
                    f"Path loss: {q['path_loss_db']:.2f} dB<br>"
                    f"Signal: {q['signal_strength_dbm']:.2f} dBm<br>"
                    f"Reason: {q['degradation_reason']}"
                )

            if q_down_x:
                frame_data.append(go.Scatter3d(
                    x=q_down_x, y=q_down_y, z=q_down_z,
                    mode='lines',
                    line=dict(color='lightcoral', width=4),
                    opacity=0.9,
                    name='Out-of-threshold (DOWN)',
                    showlegend=True,
                    hoverinfo='skip',
                    visible='legendonly'
                ))
            if q_deg_x:
                frame_data.append(go.Scatter3d(
                    x=q_deg_x, y=q_deg_y, z=q_deg_z,
                    mode='lines',
                    line=dict(color='orange', width=3, dash='dot'),
                    opacity=0.8,
                    name='Out-of-threshold (DEGRADED)',
                    showlegend=True,
                    hoverinfo='skip'
                ))
            if q_h_x:
                frame_data.append(go.Scatter3d(
                    x=q_h_x, y=q_h_y, z=q_h_z,
                    mode='markers',
                    marker=dict(size=4, color='magenta', symbol='diamond'),
                    name='Link quality details',
                    hovertext=q_h_t,
                    hoverinfo='text',
                    showlegend=True
                ))

        # Add beam links and serving ground-node overlays if available
        beams_time_key, beam_records = _records_at_time(beams, beams_time_keys, current_time)
        if show_beams and beam_records:
            # Group beams by satellite for individual legend entries
            beams_by_sat = {}  # sat_id -> {'normal': (x,y,z,h), 'src': (x,y,z), 'tgt': (x,y,z)}
            serving_ue_x, serving_ue_y, serving_ue_z, serving_ue_h = [], [], [], []
            serving_srv_x, serving_srv_y, serving_srv_z, serving_srv_h = [], [], [], []

            for rec in beam_records:
                ground_id = rec.get('ground_id')
                ground_type = rec.get('ground_type', 'UE')
                sat_id = rec.get('sat_id')
                beam_id = rec.get('beam_id', -1)
                state = rec.get('state', '').upper()

                # Serving/access-authority visualization should only use valid
                # actively serving associations, not SEARCHING/default rows.
                if beam_id < 0 or not rec.get('beam_active', True) or state not in SERVING_STATES:
                    continue

                # Resolve by declared ground type to avoid UE/SERVER id collisions.
                if ground_type == 'SERVER':
                    ground_entry = data['SERVER'].get(ground_id)
                else:
                    ground_entry = data['UE'].get(ground_id)
                sat_entry = data['SATELLITE'].get(sat_id)
                if not ground_entry or not sat_entry:
                    continue

                ground_pos = get_position_at_time(ground_entry, current_time)
                sat_pos = get_position_at_time(sat_entry, current_time)
                if ground_pos is None or sat_pos is None:
                    continue

                # Initialize satellite entry if needed
                if sat_id not in beams_by_sat:
                    beams_by_sat[sat_id] = {
                        'normal': {'x': [], 'y': [], 'z': [], 'h': []},
                        'src': {'x': [], 'y': [], 'z': []},
                        'tgt': {'x': [], 'y': [], 'z': []}
                    }

                # Highlight handover transitions by coloring source/target serving links.
                is_handover_src = False
                is_handover_tgt = False
                handover_kind = ''
                if ground_type == 'UE' and ho_events_frame:
                    for e in ho_events_frame:
                        if e.get('ue_id') != ground_id:
                            continue
                        handover_kind = e.get('type', '').upper()
                        if e.get('src_sat') == sat_id:
                            is_handover_src = True
                        if e.get('tgt_sat') == sat_id:
                            is_handover_tgt = True

                rsrp = rec.get('rsrp_dbm', float('nan'))
                sinr = rec.get('sinr_db', float('nan'))
                elv  = rec.get('elevation_deg', float('nan'))
                tte  = rec.get('tte_sec', float('nan'))
                bload = rec.get('beam_load', float('nan'))
                topscore = rec.get('topsis_score', float('nan'))
                hover = (
                    f"Serving {ground_type}<br>"
                    f"Ground: {ground_id}<br>"
                    f"Satellite: {sat_id}<br>"
                    f"Beam: {beam_id} | Cell: {rec.get('cell_id', -1)}<br>"
                    f"RSRP: {rsrp:.1f} dBm | SINR: {sinr:.1f} dB<br>"
                    f"Elevation: {elv:.1f}° | TTE: {tte:.0f}s<br>"
                    f"BeamLoad: {bload:.0f} | TOPSIS: {topscore:.3f}<br>"
                    f"State: {state}"
                )

                if is_handover_tgt:
                    beams_by_sat[sat_id]['tgt']['x'].extend([ground_pos[0], sat_pos[0], None])
                    beams_by_sat[sat_id]['tgt']['y'].extend([ground_pos[1], sat_pos[1], None])
                    beams_by_sat[sat_id]['tgt']['z'].extend([ground_pos[2], sat_pos[2], None])
                    hover += f"<br>Handover: TARGET ({handover_kind or 'acquiring'})"
                elif is_handover_src:
                    beams_by_sat[sat_id]['src']['x'].extend([ground_pos[0], sat_pos[0], None])
                    beams_by_sat[sat_id]['src']['y'].extend([ground_pos[1], sat_pos[1], None])
                    beams_by_sat[sat_id]['src']['z'].extend([ground_pos[2], sat_pos[2], None])
                    hover += f"<br>Handover: SOURCE ({handover_kind or 'releasing'})"
                else:
                    beams_by_sat[sat_id]['normal']['x'].extend([ground_pos[0], sat_pos[0], None])
                    beams_by_sat[sat_id]['normal']['y'].extend([ground_pos[1], sat_pos[1], None])
                    beams_by_sat[sat_id]['normal']['z'].extend([ground_pos[2], sat_pos[2], None])

                if ground_type == 'SERVER':
                    marker_pos = lifted_marker_position(ground_pos, GROUND_MARKER_LIFT_M)
                    serving_srv_x.append(marker_pos[0]); serving_srv_y.append(marker_pos[1]);
                    serving_srv_z.append(marker_pos[2]); serving_srv_h.append(hover)
                else:
                    marker_pos = lifted_marker_position(ground_pos, UE_MARKER_LIFT_M)
                    serving_ue_x.append(marker_pos[0]); serving_ue_y.append(marker_pos[1]);
                    serving_ue_z.append(marker_pos[2]); serving_ue_h.append(hover)

            # Create per-satellite beam traces for individual toggling
            for sat_id in sorted(beams_by_sat.keys()):
                sat_beams = beams_by_sat[sat_id]
                
                # Normal serving access beams (highlighted blue solid)
                if sat_beams['normal']['x']:
                    frame_data.append(go.Scatter3d(
                        x=sat_beams['normal']['x'], y=sat_beams['normal']['y'], z=sat_beams['normal']['z'],
                        mode='lines',
                        line=dict(color='blue', width=4),
                        opacity=0.9,
                        showlegend=True,
                        legendgroup=f'sat{sat_id}',
                        name=f'SAT {sat_id} serving access beams',
                        hoverinfo='skip'
                    ))

                # Handover source links (orange)
                if sat_beams['src']['x']:
                    frame_data.append(go.Scatter3d(
                        x=sat_beams['src']['x'], y=sat_beams['src']['y'], z=sat_beams['src']['z'],
                        mode='lines',
                        line=dict(color='orange', width=5, dash='dash'),
                        opacity=0.95,
                        showlegend=True,
                        legendgroup=f'sat{sat_id}',
                        name=f'SAT {sat_id} serving HO source',
                        hoverinfo='skip'
                    ))

                # Handover target links (blue)
                if sat_beams['tgt']['x']:
                    frame_data.append(go.Scatter3d(
                        x=sat_beams['tgt']['x'], y=sat_beams['tgt']['y'], z=sat_beams['tgt']['z'],
                        mode='lines',
                        line=dict(color='deepskyblue', width=6),
                        opacity=0.95,
                        showlegend=True,
                        legendgroup=f'sat{sat_id}',
                        name=f'SAT {sat_id} serving HO target',
                        hoverinfo='skip'
                    ))

            if serving_ue_x:
                frame_data.append(go.Scatter3d(
                    x=serving_ue_x, y=serving_ue_y, z=serving_ue_z,
                    mode='markers',
                    marker=dict(
                        size=3,
                        color=UE_MARKER_COLOR,
                        symbol='circle',
                        opacity=1.0,
                        line=dict(color=UE_MARKER_LINE_COLOR, width=1)
                    ),
                    name='Serving ground nodes (UE)',
                    hovertext=serving_ue_h,
                    hoverinfo='text',
                    showlegend=True
                ))

            if serving_srv_x:
                frame_data.append(go.Scatter3d(
                    x=serving_srv_x, y=serving_srv_y, z=serving_srv_z,
                    mode='markers',
                    marker=dict(
                        size=3,
                        color=SERVER_MARKER_COLOR,
                        symbol='diamond-open',
                        line=dict(color=SERVER_MARKER_LINE_COLOR, width=1)
                    ),
                    name='Serving ground nodes (SERVER)',
                    hovertext=serving_srv_h,
                    hoverinfo='text',
                    showlegend=True
                ))

        # Add handover event markers within this frame window
        if show_handovers and handovers:
            intra_x, intra_y, intra_z, intra_text = [], [], [], []
            inter_x, inter_y, inter_z, inter_text, inter_labels = [], [], [], [], []
            inter_stem_x, inter_stem_y, inter_stem_z = [], [], []
            inter_line_x, inter_line_y, inter_line_z, inter_line_text = [], [], [], []
            for e in ho_events_frame:
                ue_entry = node_index.get(e['ue_id'])
                if not ue_entry:
                    continue
                ue_pos = get_position_at_time(ue_entry, current_time)
                if ue_pos is None:
                    continue

                sinr_b = e.get('sinr_before', float('nan'))
                sinr_a = e.get('sinr_after', float('nan'))
                lat_ms = e.get('latency_ms', float('nan'))
                src_bm = e.get('src_beam', -1)
                tgt_bm = e.get('tgt_beam', -1)
                ho_type = e.get('type', '').upper()
                is_inter_sat = ho_type in ('INTER_SAT', 'INTER_SATELLITE', 'INTER_ORBIT')
                label = 'Inter-satellite handover' if is_inter_sat else 'Intra-beam handover'
                text = (
                    f"<b>{label}</b><br>"
                    f"Handover @ {e['time_s']:.3f}s<br>"
                    f"UE {e['ue_id']}<br>"
                    f"SAT {e['src_sat']} (beam {src_bm}) → SAT {e['tgt_sat']} (beam {tgt_bm})<br>"
                    f"{e.get('mode','')} / {e.get('type','')} / {e.get('trigger','')}<br>"
                    f"Latency: {lat_ms:.1f} ms | success={e.get('success','')}<br>"
                    f"SINR: {sinr_b:.1f} → {sinr_a:.1f} dB | route_change={e.get('route_change','')}<br>"
                    f"buffered={e.get('buff_pkts',0)} dropped={e.get('drop_pkts',0)}"
                )

                if is_inter_sat:
                    marker_pos = lifted_marker_position(ue_pos, 160000.0)
                    inter_x.append(marker_pos[0]); inter_y.append(marker_pos[1]); inter_z.append(marker_pos[2])
                    inter_text.append(text)
                    inter_labels.append('INTER-SAT HO')
                    inter_stem_x.extend([ue_pos[0], marker_pos[0], None])
                    inter_stem_y.extend([ue_pos[1], marker_pos[1], None])
                    inter_stem_z.extend([ue_pos[2], marker_pos[2], None])

                    src_sat = data['SATELLITE'].get(e.get('src_sat'))
                    tgt_sat = data['SATELLITE'].get(e.get('tgt_sat'))
                    if src_sat and tgt_sat:
                        src_pos = get_position_at_time(src_sat, current_time)
                        tgt_pos = get_position_at_time(tgt_sat, current_time)
                        if src_pos is not None and tgt_pos is not None:
                            inter_line_x.extend([src_pos[0], tgt_pos[0], None])
                            inter_line_y.extend([src_pos[1], tgt_pos[1], None])
                            inter_line_z.extend([src_pos[2], tgt_pos[2], None])
                            inter_line_text.extend([text, text, None])
                else:
                    marker_pos = lifted_marker_position(ue_pos, 90000.0)
                    intra_x.append(marker_pos[0]); intra_y.append(marker_pos[1]); intra_z.append(marker_pos[2])
                    intra_text.append(text)

            if inter_stem_x:
                frame_data.append(go.Scatter3d(
                    x=inter_stem_x, y=inter_stem_y, z=inter_stem_z,
                    mode='lines',
                    line=dict(color='red', width=5),
                    opacity=0.95,
                    name='Inter-sat handover marker stem',
                    hoverinfo='skip',
                    showlegend=False,
                    visible=True
                ))

            if inter_line_x:
                frame_data.append(go.Scatter3d(
                    x=inter_line_x, y=inter_line_y, z=inter_line_z,
                    mode='lines',
                    line=dict(color='red', width=7, dash='dash'),
                    name='Inter-sat handover transfer',
                    hovertext=inter_line_text,
                    hoverinfo='text',
                    showlegend=True,
                    visible=True
                ))

            if intra_x:
                frame_data.append(go.Scatter3d(
                    x=intra_x, y=intra_y, z=intra_z,
                    mode='markers',
                    marker=dict(size=8, color='orange', symbol='diamond', line=dict(color='darkorange', width=2)),
                    name='Intra-beam handovers',
                    hovertext=intra_text,
                    hoverinfo='text',
                    showlegend=True,
                    visible=True
                ))

            if inter_x:
                frame_data.append(go.Scatter3d(
                    x=inter_x, y=inter_y, z=inter_z,
                    mode='markers+text',
                    marker=dict(size=15, color='red', symbol='cross', line=dict(color='darkred', width=5)),
                    text=inter_labels,
                    textposition='top center',
                    textfont=dict(color='red', size=13),
                    name='Inter-sat handovers',
                    hovertext=inter_text,
                    hoverinfo='text',
                    showlegend=True,
                    visible=True
                ))

        # Add packet flow arrows if available
        flow_time_key = int(current_time)  # Flows are indexed by integer seconds
        if flows and flow_time_key in flows:
            for flow in flows[flow_time_key]:
                # Create arrow from source to destination
                src = flow['src_pos']
                dst = flow['dst_pos']
                # Direction vector
                direction = dst - src
                length = np.linalg.norm(direction)
                
                # Determine flow color based on link type
                link_type = flow.get('link_type', 'UNKNOWN')

                flow_color = 'cyan'  # ISL flows in cyan
                
                # Create arrow with cone at destination
                # Line for the flow
                frame_data.append(go.Scatter3d(
                    x=[src[0], dst[0]],
                    y=[src[1], dst[1]],
                    z=[src[2], dst[2]],
                    mode='lines',
                    line=dict(color=flow_color, width=4),
                    opacity=0.8,
                    showlegend=False,
                    name=f'Flow {flow["src_node"]}→{flow["dst_node"]}',
                    hovertext=(f'Packet Flow<br>'
                              f'From: {flow["src_type"]} {flow["src_node"]}<br>'
                              f'To: {flow["dst_type"]} {flow["dst_node"]}<br>'
                              f'Link: {flow.get("link_type", "UNKNOWN")}<br>'
                              f'Size: {flow["size"]} bytes<br>'
                              f'Delay: {(flow["rx_time"] - flow["tx_time"])*1000:.2f} ms<br>'
                              f'SNR: {flow["snr"]:.2f} dB'),
                    hoverinfo='text'
                ))
                
                # Add arrow head using a cone
                arrow_pos = src + 0.85 * direction  # Position arrow near destination
                cone_size = min(length * 0.1, 500000)  # Scale to reasonable size
                
                frame_data.append(go.Cone(
                    x=[arrow_pos[0]], y=[arrow_pos[1]], z=[arrow_pos[2]],
                    u=[direction[0]], v=[direction[1]], w=[direction[2]],
                    sizemode='absolute',
                    sizeref=cone_size,
                    colorscale=[[0, flow_color], [1, flow_color]],
                    showscale=False,
                    opacity=0.8,
                    showlegend=False,
                    hoverinfo='skip'
                ))

        # Add packet events if available for this time
        # Collect all packet events within this frame's time window
        if packets:
            # Determine time window for this frame
            next_time = times[frame_idx + 1] if frame_idx + 1 < len(times) else current_time + 1.0
            
            events = []
            for time_key, time_events in packets.items():
                for ev in time_events:
                    # Include packets from current_time up to (but not including) next_time
                    if current_time <= ev['time'] < next_time:
                        events.append(ev)

            tx_x, tx_y, tx_z, tx_hover, tx_size = [], [], [], [], []
            rx_x, rx_y, rx_z, rx_hover, rx_size = [], [], [], [], []
            dr_x, dr_y, dr_z, dr_hover, dr_size = [], [], [], [], []

            for ev in events:
                node_entry = node_index.get(ev['node_id'])
                if not node_entry:
                    continue
                pos = get_position_at_time(node_entry, current_time)
                if pos is None:
                    continue

                size = max(8.0, min(20.0, ev['size_bytes'] / 10.0))
                
                link_type = ev.get('link_type', 'UNKNOWN')
                peer_node = ev.get('peer_node_id', -1)
                hover = (f"{ev['event']} Node {ev['node_id']} Dev {ev['device_id']}<br>"
                         f"Peer: {peer_node} ({link_type})<br>"
                         f"Size: {ev['size_bytes']} bytes<br>"
                         f"SNR: {ev['snr_db']:.2f} dB<br>"
                         f"Doppler: {ev['doppler_hz']:.2f} Hz")

                if ev['event'] == 'TX':
                    tx_x.append(pos[0]); tx_y.append(pos[1]); tx_z.append(pos[2])
                    tx_hover.append(hover); tx_size.append(size)
                elif ev['event'] == 'RX':
                    rx_x.append(pos[0]); rx_y.append(pos[1]); rx_z.append(pos[2])
                    rx_hover.append(hover); rx_size.append(size)
                else:
                    dr_x.append(pos[0]); dr_y.append(pos[1]); dr_z.append(pos[2])
                    dr_hover.append(hover); dr_size.append(size)

            # Always add packet traces to maintain consistent frame structure
            # TX packets
            frame_data.append(go.Scatter3d(
                x=tx_x, y=tx_y, z=tx_z,
                mode='markers',
                marker=dict(
                    size=tx_size if tx_x else [10],
                    color='orange',
                    symbol='circle',
                    opacity=0.9,
                    line=dict(color='darkorange', width=2)
                ),
                name='TX packets',
                hovertext=tx_hover if tx_x else [],
                showlegend=True,
                visible=True
            ))

            # RX packets
            frame_data.append(go.Scatter3d(
                x=rx_x, y=rx_y, z=rx_z,
                mode='markers',
                marker=dict(
                    size=rx_size if rx_x else [10],
                    color='lime',
                    symbol='circle',
                    opacity=0.9,
                    line=dict(color='green', width=2)
                ),
                name='RX packets',
                hovertext=rx_hover if rx_x else [],
                showlegend=True,
                visible=True
            ))

            # Dropped packets
            frame_data.append(go.Scatter3d(
                x=dr_x, y=dr_y, z=dr_z,
                mode='markers',
                marker=dict(
                    size=dr_size if dr_x else [10],
                    color='red',
                    symbol='x',
                    opacity=0.9,
                    line=dict(color='darkred', width=3)
                ),
                name='Dropped packets',
                hovertext=dr_hover if dr_x else [],
                showlegend=True,
                visible=True
            ))
        
        frames.append(go.Frame(
            data=frame_data,
            name=str(frame_idx),
            layout=go.Layout(
                title_text=f'LEO Satellite Network - Time: {current_time:.1f}s'
            )
        ))
        
        if frame_idx == 0:
            print(f"Frame 0 has {len(frame_data)} traces")
            for i, trace in enumerate(frame_data):
                print(f"  Trace {i}: {trace.name}")
    
    # Create initial figure with first frame data and servers/UEs from initial data
    fig = go.Figure(
        data=list(frames[0].data if frames else []),
        frames=frames
    )
    
    # Set layout
    max_range = EARTH_RADIUS * 1.5
    
    # Create slider steps
    sliders = [dict(
        active=0,
        yanchor="top",
        y=0,
        xanchor="left",
        x=0.1,
        currentvalue=dict(
            prefix="Time: ",
            suffix=" s",
            visible=True,
            xanchor="right"
        ),
        pad=dict(b=10, t=50),
        len=0.8,
        steps=[dict(
            args=[[f.name], dict(
                frame=dict(duration=100, redraw=True),
                mode="immediate",
                transition=dict(duration=0)
            )],
            label=f'{times[int(f.name)]:.1f}',
            method="animate"
        ) for f in frames]
    )]
    
    # Add play/pause and speed control buttons
    updatemenus = [
        # Play/Pause controls
        dict(
            type="buttons",
            direction="left",
            x=0.1,
            y=0,
            showactive=False,
            buttons=[
                dict(label="Play",
                     method="animate",
                     args=[None, dict(
                         frame=dict(duration=500, redraw=True),
                         fromcurrent=True,
                         mode="immediate",
                         transition=dict(duration=0)
                     )]),
                dict(label="Pause",
                     method="animate",
                     args=[[None], dict(
                         frame=dict(duration=0, redraw=False),
                         mode="immediate",
                         transition=dict(duration=0)
                     )])
            ]
        ),
        # Speed controls
        dict(
            type="buttons",
            direction="left",
            x=0.3,
            y=0,
            showactive=False,
            buttons=[
                dict(label="Slow (2s)",
                     method="animate",
                     args=[None, dict(
                         frame=dict(duration=2000, redraw=True),
                         fromcurrent=True,
                         mode="immediate",
                         transition=dict(duration=0)
                     )]),
                dict(label="Normal (0.5s)",
                     method="animate",
                     args=[None, dict(
                         frame=dict(duration=500, redraw=True),
                         fromcurrent=True,
                         mode="immediate",
                         transition=dict(duration=0)
                     )]),
                dict(label="Fast (0.1s)",
                     method="animate",
                     args=[None, dict(
                         frame=dict(duration=100, redraw=True),
                         fromcurrent=True,
                         mode="immediate",
                         transition=dict(duration=0)
                     )])
            ]
        )
    ]
    
    fig.update_layout(
        title='LEO Satellite Network - Interactive 3D Animation',
        scene=dict(
            xaxis=dict(range=[-max_range, max_range], title='X (m)', showgrid=True),
            yaxis=dict(range=[-max_range, max_range], title='Y (m)', showgrid=True),
            zaxis=dict(range=[-max_range, max_range], title='Z (m)', showgrid=True),
            aspectmode='cube',
            camera=dict(
                eye=dict(x=1.5, y=1.5, z=1.2)
            )
        ),
        updatemenus=updatemenus,
        sliders=sliders,
        showlegend=True,
        legend=dict(
            x=1.01,
            y=1,
            xanchor='left',
            yanchor='top',
            bgcolor='rgba(255, 255, 255, 0.85)',
            bordercolor='black',
            borderwidth=1
        ),
        height=900
    )
    
    if output_file:
        print(f"Saving animation to {output_file}...")
        fig.write_html(output_file)
        print(f"Animation saved successfully!")
    else:
        print("Opening interactive animation in browser...")
        fig.show()
    
    return fig

def main():
    parser = argparse.ArgumentParser(description='Visualize LEO satellite network on 3D globe')
    parser.add_argument('--position_file',default="leosim_positions.csv", help='CSV file with position data')
    parser.add_argument('--max-frames', type=int, default=None,
                       help='Maximum number of animation frames')
    parser.add_argument('--links', type=str, default="leosim_link_state.csv",
                       help='CSV file with unified channel/link/beam state data')
    parser.add_argument('--packets', type=str, default="leosim_packets.csv",
                       help='CSV file with packet data')
    parser.add_argument('--beams', type=str, default="leosim_beam_associations.csv",
                       help='CSV file with serving ground-node beam associations')
    parser.add_argument('--handovers', type=str, default="leosim_handovers.csv",
                       help='CSV file with handover events')
    parser.add_argument('--output', type=str, default="visualization.html",
                       help='Output HTML file (if not specified, opens in browser)')
    parser.add_argument('--coverage', type=str, default="leosim_coverage.csv",
                       help='CSV file with satellite ground coverage data')
    parser.add_argument('--link-quality', type=str, default="",
                       help='Optional legacy CSV file with out-of-threshold link quality data')
    parser.add_argument('--show-flows', action='store_true', default=True,
                       help='Show packet flow arrows (default: enabled)')
    parser.add_argument('--no-flows', action='store_false', dest='show_flows',
                       help='Disable packet flow visualization')
    parser.add_argument('--show-channel-links', action='store_true', default=False,
                       help='Show raw channel-visible links (default: disabled)')
    parser.add_argument('--no-channel-links', action='store_false', dest='show_channel_links',
                       help='Disable raw channel-visible links')
    parser.add_argument('--show-isl-links', action='store_true', default=True,
                       help='Show established ISL topology links (default: enabled)')
    parser.add_argument('--no-isl-links', action='store_false', dest='show_isl_links',
                       help='Disable established ISL topology links')
    parser.add_argument('--show-beams', '--show-serving-beams', action='store_true', default=True,
                       help='Show serving beam/access-authority links (default: enabled)')
    parser.add_argument('--no-beams', '--no-serving-beams', action='store_false', dest='show_beams',
                       help='Disable serving beam/access-authority links')
    parser.add_argument('--show-coverage', '--show-all-beams', action='store_true', default=True,
                       help='Show all beam footprint/coverage overlays (default: enabled)')
    parser.add_argument('--no-coverage', '--no-all-beams', action='store_false', dest='show_coverage',
                       help='Disable all beam footprint/coverage overlays')
    parser.add_argument('--show-link-quality', action='store_true', default=True,
                       help='Show out-of-threshold link quality overlays (default: enabled)')
    parser.add_argument('--no-link-quality', action='store_false', dest='show_link_quality',
                       help='Disable out-of-threshold link quality overlays')
    parser.add_argument('--only-participating-nodes', '--participating-nodes-only',
                       action='store_true',
                       help=('Visualize only nodes involved in packets, serving beams, '
                             'handovers, or active/established links'))
    
    args = parser.parse_args()
    
    # Load data
    print(f"Loading data from {args.position_file}...")
    data = load_position_data(args.position_file)
    
    # Try to load link data
    links = {}
    if args.links:
        links = load_link_data(args.links)
    else:
        # Try default link file name
        link_file = args.position_file.replace('.csv', '_links.csv')
        links = load_link_data(link_file)

    packets = {}
    if args.packets:
        packets = load_packet_data(args.packets)

    beams = {}
    if args.beams:
        beams = load_beam_data(args.beams)

    handovers = []
    if args.handovers:
        handovers = load_handover_data(args.handovers)

    coverage = {}
    if args.coverage:
        coverage = load_coverage_data(args.coverage)

    link_quality = {}
    if args.link_quality:
        link_quality = load_link_quality_data(args.link_quality)

    if args.only_participating_nodes:
        original_counts = {node_type: len(nodes) for node_type, nodes in data.items()}
        data = filter_participating_nodes(data, links, packets, beams, handovers)
        retained_counts = {node_type: len(nodes) for node_type, nodes in data.items()}
        print(
            "Participating-node filter: "
            + ", ".join(
                f"{node_type} {retained_counts[node_type]}/{original_counts[node_type]}"
                for node_type in ('SATELLITE', 'SERVER', 'UE')
            )
        )
    
    # Print summary
    print(f"\nData loaded successfully!")
    for node_type in ['SATELLITE', 'SERVER', 'UE']:
        count = len(data[node_type])
        if count > 0:
            sample = next(iter(data[node_type].values()))
            num_points = len(sample['positions'])
            print(f"{node_type}s: {count} ({num_points} points each)")
        else:
            print(f"{node_type}s: 0")
    
    if links:
        print(f"Links loaded for {len(links)} time steps")

    if packets:
        print(f"Packets loaded for {len(packets)} time steps")

    if beams:
        print(f"Beams loaded for {len(beams)} time steps")

    if handovers:
        print(f"Handovers loaded: {len(handovers)} events")
    if coverage:
        print(f"Coverage loaded for {len(coverage)} time steps")
    if link_quality:
        print(f"Link quality loaded for {len(link_quality)} time steps")
    

    visualize_animated(
        data,
        links,
        packets,
        beams,
        handovers,
        coverage,
        link_quality,
        max_frames=args.max_frames,
        output_file=args.output,
        show_flows=args.show_flows,
        show_channel_links=args.show_channel_links,
        show_isl_links=args.show_isl_links,
        show_beams=args.show_beams,
        show_coverage=args.show_coverage,
        show_link_quality=args.show_link_quality,
    )

if __name__ == '__main__':
    main()
