import math
import random

# Fixed seed for reproducibility (remove to get fresh random points each time)
random.seed(42)

def generate_ue_locations(n=500000):
    """Generate n uniformly distributed (lat, lon) on a sphere."""
    locations = []
    for _ in range(n):
        u = random.random()
        v = random.random()
        lon = 2 * math.pi * u
        lat = math.acos(2 * v - 1) - math.pi / 2
        locations.append((math.degrees(lat), math.degrees(lon)))
    return locations

def assign_altitudes(locations, min_alt=0, max_alt=200):
    """Assign a random altitude (in meters) to each location."""
    return [(lat, lon, round(random.uniform(min_alt, max_alt), 1))
            for lat, lon in locations]

def split_into_groups(data, groups=3):
    """Split data into roughly equal groups."""
    k = len(data) // groups
    remainder = len(data) % groups
    result = []
    start = 0
    for i in range(groups):
        end = start + k + (1 if i < remainder else 0)
        result.append(data[start:end])
        start = end
    return result

def write_file(filename, prefix, records):
    """Write a CSV file with name, latitude, longitude, altitude."""
    with open(filename, 'w') as f:
        for idx, (lat, lon, alt) in enumerate(records, start=1):
            f.write(f"{prefix}_ue_{idx},{lat:.6f},{lon:.6f},{alt:.1f}\n")

if __name__ == "__main__":
    # 1. Generate 500 raw (lat, lon)
    raw_points = generate_ue_locations(1000)
    1
    # 2. Add altitude
    points_with_alt = assign_altitudes(raw_points)
    
    # 3. Split into 3 groups
    groups = split_into_groups(points_with_alt, 3)
    
    leosimDataDir = "contrib/leosim/data";
    
    # 4. Write each group to a separate file
    write_file(leosimDataDir+"/ues/alpha.txt", "alpha", groups[0])
    write_file(leosimDataDir+"/ues/beta.txt",  "beta",  groups[1])
    write_file(leosimDataDir+"/ues/gamma.txt", "gamma", groups[2])
    
    print("✅ Done! Created:")
    print(f"   alpha_ue_locations.csv – {len(groups[0])} UEs")
    print(f"   beta_ue_locations.csv  – {len(groups[1])} UEs")
    print(f"   gamma_ue_locations.csv – {len(groups[2])} UEs")