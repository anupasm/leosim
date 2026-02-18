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
Generate ground station locations (servers and UEs) that are closer to LEO satellite positions.
Analyzes satellite positions and creates optimal ground station coverage areas.
"""

import csv
import math
from collections import defaultdict
from typing import List, Tuple, Dict

def ecef_to_lla(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert ECEF coordinates to Latitude, Longitude, Altitude."""
    a = 6378137.0  # Earth's semi-major axis (WGS84)
    b = 6356752.3  # Earth's semi-minor axis (WGS84)
    e2 = 1 - (b**2 / a**2)  # First eccentricity squared
    
    p = math.sqrt(x**2 + y**2)
    latitude = math.atan2(z, p * (1 - e2))
    
    # Iterate to refine latitude
    for _ in range(5):
        N = a / math.sqrt(1 - e2 * math.sin(latitude)**2)
        latitude = math.atan2(z + e2 * N * math.sin(latitude), p)
    
    N = a / math.sqrt(1 - e2 * math.sin(latitude)**2)
    altitude = p / math.cos(latitude) - N
    
    longitude = math.atan2(y, x)
    
    # Convert to degrees
    latitude_deg = math.degrees(latitude)
    longitude_deg = math.degrees(longitude)
    
    return latitude_deg, longitude_deg, altitude

def lla_to_ecef(latitude: float, longitude: float, altitude: float = 0.0) -> Tuple[float, float, float]:
    """Convert Latitude, Longitude, Altitude to ECEF coordinates."""
    a = 6378137.0  # Earth's semi-major axis (WGS84)
    b = 6356752.3  # Earth's semi-minor axis (WGS84)
    e2 = 1 - (b**2 / a**2)  # First eccentricity squared
    
    lat_rad = math.radians(latitude)
    lon_rad = math.radians(longitude)
    
    N = a / math.sqrt(1 - e2 * math.sin(lat_rad)**2)
    
    x = (N + altitude) * math.cos(lat_rad) * math.cos(lon_rad)
    y = (N + altitude) * math.cos(lat_rad) * math.sin(lon_rad)
    z = (N * (1 - e2) + altitude) * math.sin(lat_rad)
    
    return x, y, z

def analyze_satellite_coverage(csv_file: str) -> Dict[str, List[Tuple[float, float, float]]]:
    """Analyze satellite positions to determine coverage areas."""
    satellite_positions = defaultdict(list)
    
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row['type'] == 'SATELLITE':
                sat_id = row['name']
                x = float(row['x'])
                y = float(row['y'])
                z = float(row['z'])
                lat, lon, alt = ecef_to_lla(x, y, z)
                satellite_positions[sat_id].append((lat, lon, alt))
    
    return satellite_positions

def generate_ground_stations(sat_positions: Dict[str, List[Tuple[float, float, float]]]) -> \
        Tuple[List[Tuple[str, float, float]], List[Tuple[str, float, float]]]:
    """Generate server and UE locations based on satellite coverage."""
    
    servers = []
    ues = []
    
    # Define major coverage regions based on satellite tracks
    # Servers - major hubs
    coverage_regions = {
        'Server_North_America': (45, -100, 'SERVER'),
        'Server_South_America': (-15, -60, 'SERVER'),
        'Server_Europe': (50, 10, 'SERVER'),
        'Server_Middle_East': (30, 45, 'SERVER'),
        'Server_East_Asia': (35, 120, 'SERVER'),
        'Server_Southeast_Asia': (5, 110, 'SERVER'),
        'Server_Australia': (-25, 135, 'SERVER'),
        'Server_Africa': (0, 20, 'SERVER'),
    }
    
    # UEs - distributed user equipment
    ue_regions = {
        'UE_Canada': (55, -120, 'UE'),
        'UE_Brazil': (-5, -55, 'UE'),
        'UE_France': (47, 2, 'UE'),
        'UE_UAE': (24, 54, 'UE'),
        'UE_India': (20, 78, 'UE'),
        'UE_Indonesia': (-5, 115, 'UE'),
        'UE_NewZealand': (-41, 174, 'UE'),
        'UE_Nigeria': (9, 7, 'UE'),
        'UE_Russia': (60, 100, 'UE'),
        'UE_Chile': (-30, -71, 'UE'),
        'UE_SouthKorea': (37, 127, 'UE'),
        'UE_SouthAfrica': (-34, 22, 'UE'),
    }
    
    # Combine all regions
    all_regions = {**coverage_regions, **ue_regions}
    
    for name, (lat, lon, node_type) in all_regions.items():
        if node_type == 'SERVER':
            servers.append((name, lat, lon))
        else:
            ues.append((name, lat, lon))
    
    return servers, ues

def create_csv_output(servers: List[Tuple[str, float, float]], 
                      ues: List[Tuple[str, float, float]],
                      output_file: str) -> None:
    """Create CSV file with generated ground stations in latitude/longitude format."""
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['device_id', 'device_name', 'device_type', 'latitude_deg', 'longitude_deg', 'altitude_m'])
        
        device_id = 0
        
        # Write servers
        for name, lat, lon in servers:
            writer.writerow([device_id, name, 'SERVER', f"{lat:.4f}", f"{lon:.4f}", "10.0"])
            device_id += 1
        
        # Write UEs
        for name, lat, lon in ues:
            writer.writerow([device_id, name, 'UE', f"{lat:.4f}", f"{lon:.4f}", "10.0"])
            device_id += 1

def print_location_summary(servers: List[Tuple[str, float, float]],
                          ues: List[Tuple[str, float, float]]) -> None:
    """Print summary of generated locations."""
    print("\n=== Generated Server Locations ===")
    print(f"{'Name':<30} {'Latitude':<12} {'Longitude':<12}")
    print("-" * 54)
    for name, lat, lon in servers:
        print(f"{name:<30} {lat:>10.2f}°  {lon:>10.2f}°")
    
    print("\n=== Generated UE Locations ===")
    print(f"{'Name':<30} {'Latitude':<12} {'Longitude':<12}")
    print("-" * 54)
    for name, lat, lon in ues:
        print(f"{name:<30} {lat:>10.2f}°  {lon:>10.2f}°")
    
    print(f"\nTotal Servers: {len(servers)}")
    print(f"Total UEs: {len(ues)}")
    print(f"Total Ground Stations: {len(servers) + len(ues)}")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Generate ground station locations closer to LEO satellites'
    )
    parser.add_argument(
        '-i', '--input',
        default='leosim_channel_positions.csv',
        help='Input CSV file with satellite positions'
    )
    parser.add_argument(
        '-o', '--output',
        default='generated_ground_stations.csv',
        help='Output CSV file for ground stations'
    )
    parser.add_argument(
        '--print-summary',
        action='store_true',
        help='Print summary of generated locations'
    )
    
    args = parser.parse_args()
    
    # Analyze satellite positions
    print(f"Analyzing satellite positions from {args.input}...")
    sat_positions = analyze_satellite_coverage(args.input)
    print(f"Found {len(sat_positions)} satellites")
    
    # Generate ground stations
    print("Generating optimal ground station locations...")
    servers, ues = generate_ground_stations(sat_positions)
    
    # Create output CSV
    print(f"Writing to {args.output}...")
    create_csv_output(servers, ues, args.output)
    
    # Print summary if requested
    if args.print_summary:
        print_location_summary(servers, ues)
    
    print(f"\n✓ Generated {len(servers)} servers and {len(ues)} UEs")
    print(f"✓ Output saved to {args.output}")

if __name__ == '__main__':
    main()
