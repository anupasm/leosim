#!/usr/bin/env python3
"""Identify satellites geometrically covering a LeoSim UE area or GPS point over time."""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path


EARTH_RADIUS_M = 6_371_000.0
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATA_DIR = SCRIPT_DIR.parent / "data-b"
DEFAULT_TRACE = DEFAULT_DATA_DIR / "prepro" / "satellite_mobility.tcl"
TRACE_PATTERN = re.compile(
    r'^\$ns_ at ([0-9.+-]+) "\$node_\(([0-9]+)\) set ([XYZ])_ ([0-9.eE+-]+)"$'
)


@dataclass(frozen=True)
class Area:
    center_unit: tuple[float, float, float]
    center_lat: float
    center_lon: float
    radius_m: float
    ue_count: int


@dataclass
class SatelliteResult:
    satellite_id: int
    name: str
    matching_times: list[float]
    minimum_center_distance_m: float = math.inf
    maximum_coverage_radius_m: float = 0.0


def unit_from_lat_lon(latitude: float, longitude: float) -> tuple[float, float, float]:
    lat = math.radians(latitude)
    lon = math.radians(longitude)
    return math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat)


def area_for_location(latitude: float, longitude: float) -> Area:
    """Represent a single GPS location as a zero-radius spherical area."""
    if not -90.0 <= latitude <= 90.0:
        raise ValueError("GPS latitude must be between -90 and 90 degrees")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("GPS longitude must be between -180 and 180 degrees")
    return Area(unit_from_lat_lon(latitude, longitude), latitude, longitude, 0.0, 1)


def angular_distance(first: tuple[float, float, float], second: tuple[float, float, float]) -> float:
    dot = sum(a * b for a, b in zip(first, second))
    return math.acos(max(-1.0, min(1.0, dot)))


def load_ue_area(data_dir: Path, center_node: str | None, excluded_ues: set[str]) -> Area:
    """Return a spherical enclosing cap for all UEs in file-loader order."""
    coordinates: list[tuple[float, float]] = []
    ue_dir = data_dir / "ues"
    for path in sorted(ue_dir.glob("*.txt")):
        with path.open(newline="", encoding="utf-8") as stream:
            for line_number, row in enumerate(csv.reader(stream), start=1):
                if not row or not row[0].strip() or row[0].lstrip().startswith("#"):
                    continue
                if row[0].strip().lower() in {"name", "device_name"}:
                    continue
                if len(row) < 3:
                    raise ValueError(f"{path}:{line_number}: expected name,latitude,longitude")
                if row[0].strip() in excluded_ues:
                    continue
                latitude, longitude = float(row[1]), float(row[2])
                if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
                    raise ValueError(f"{path}:{line_number}: invalid latitude/longitude")
                coordinates.append((latitude, longitude))
    if not coordinates:
        raise ValueError(f"no UEs found under {ue_dir}")

    if center_node:
        center_coordinates = None
        for path in sorted((data_dir / "gss").glob("*.txt")):
            with path.open(newline="", encoding="utf-8") as stream:
                for row in csv.reader(stream):
                    if len(row) >= 3 and row[0].strip() == center_node:
                        center_coordinates = float(row[1]), float(row[2])
                        break
            if center_coordinates:
                break
        if center_coordinates is None:
            raise ValueError(f"center node {center_node!r} was not found under {data_dir / 'gss'}")
        center_lat, center_lon = center_coordinates
        center = unit_from_lat_lon(center_lat, center_lon)
    else:
        vectors = [unit_from_lat_lon(lat, lon) for lat, lon in coordinates]
        mean = tuple(sum(vector[index] for vector in vectors) for index in range(3))
        norm = math.sqrt(sum(value * value for value in mean))
        if norm == 0:
            raise ValueError("UE area has no unique spherical center")
        center = tuple(value / norm for value in mean)
        center_lat = math.degrees(math.asin(center[2]))
        center_lon = math.degrees(math.atan2(center[1], center[0]))
    vectors = [unit_from_lat_lon(lat, lon) for lat, lon in coordinates]
    radius_m = max(angular_distance(center, vector) for vector in vectors) * EARTH_RADIUS_M
    return Area(center, center_lat, center_lon, radius_m, len(coordinates))


def load_satellite_names(path: Path | None) -> dict[int, str]:
    if path is None or not path.exists():
        return {}
    names: dict[int, str] = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            lowered = {str(key).strip().lower(): value for key, value in row.items()}
            id_text = (
                lowered.get("id")
                or lowered.get("node_id")
                or lowered.get("sat_id")
                or lowered.get("index")
            )
            name = lowered.get("name") or lowered.get("satellite_name") or lowered.get("sat_name")
            if id_text not in (None, "") and name:
                names[int(id_text)] = name
    return names


def visibility_radius_m(
    satellite_radius_m: float, minimum_elevation_deg: float, maximum_distance_m: float
) -> float:
    """Ground radius visible under elevation and slant-range constraints."""
    if satellite_radius_m <= EARTH_RADIUS_M:
        return 0.0
    elevation = math.radians(minimum_elevation_deg)
    elevation_angle = math.acos(
        min(1.0, EARTH_RADIUS_M / satellite_radius_m * math.cos(elevation))
    ) - elevation
    distance_cosine = (
        satellite_radius_m**2 + EARTH_RADIUS_M**2 - maximum_distance_m**2
    ) / (2.0 * satellite_radius_m * EARTH_RADIUS_M)
    if distance_cosine >= 1.0:
        distance_angle = 0.0
    elif distance_cosine <= -1.0:
        distance_angle = math.pi
    else:
        distance_angle = math.acos(distance_cosine)
    return EARTH_RADIUS_M * max(0.0, min(elevation_angle, distance_angle))


def position_covers_area(
    xyz: tuple[float, float, float],
    area: Area,
    mode: str,
    minimum_elevation_deg: float,
    maximum_distance_m: float,
) -> tuple[bool, float, float]:
    satellite_radius = math.sqrt(sum(value * value for value in xyz))
    satellite_unit = tuple(value / satellite_radius for value in xyz)
    center_distance = angular_distance(area.center_unit, satellite_unit) * EARTH_RADIUS_M
    coverage_radius = visibility_radius_m(
        satellite_radius, minimum_elevation_deg, maximum_distance_m
    )
    if mode == "all":
        covered = center_distance + area.radius_m <= coverage_radius
    elif mode == "any":
        covered = center_distance <= coverage_radius + area.radius_m
    else:
        covered = center_distance <= coverage_radius
    return covered, center_distance, coverage_radius


def scan_trace(
    trace: Path,
    area: Area,
    names: dict[int, str],
    start_time: float,
    end_time: float,
    sample_interval: float,
    satellite_limit: int,
    mode: str,
    minimum_elevation_deg: float,
    maximum_distance_m: float,
) -> list[SatelliteResult]:
    """Stream the large ns-2 trace without retaining satellite trajectories."""
    results: dict[int, SatelliteResult] = {}
    coordinates: dict[tuple[int, float], dict[str, float]] = {}
    next_sample: dict[int, float] = defaultdict_time(start_time)

    with trace.open(encoding="utf-8") as stream:
        for raw_line in stream:
            match = TRACE_PATTERN.match(raw_line.strip())
            if not match:
                continue
            time_s = float(match.group(1))
            satellite_id = int(match.group(2))
            axis = match.group(3)
            value = float(match.group(4))
            if satellite_limit and satellite_id >= satellite_limit:
                break
            if time_s < start_time or time_s > end_time:
                continue
            if time_s + 1e-9 < next_sample[satellite_id]:
                continue
            key = satellite_id, time_s
            coordinates.setdefault(key, {})[axis] = value
            if len(coordinates[key]) != 3:
                continue
            completed_coordinates = coordinates.pop(key)
            xyz = tuple(completed_coordinates[axis_name] for axis_name in "XYZ")
            next_sample[satellite_id] = time_s + sample_interval
            covered, center_distance, coverage_radius = position_covers_area(
                xyz,
                area,
                mode,
                minimum_elevation_deg,
                maximum_distance_m,
            )
            result = results.setdefault(
                satellite_id,
                SatelliteResult(satellite_id, names.get(satellite_id, f"SAT-{satellite_id}"), []),
            )
            result.minimum_center_distance_m = min(result.minimum_center_distance_m, center_distance)
            result.maximum_coverage_radius_m = max(result.maximum_coverage_radius_m, coverage_radius)
            if covered:
                result.matching_times.append(time_s)
    return [result for result in results.values() if result.matching_times]


def defaultdict_time(value: float):
    """Create a float-valued defaultdict without exposing a mutable lambda loop variable."""
    from collections import defaultdict

    return defaultdict(lambda: value)


def continuous_intervals(times: list[float], sample_interval: float) -> list[tuple[float, float]]:
    if not times:
        return []
    intervals: list[tuple[float, float]] = []
    start = previous = times[0]
    tolerance = sample_interval * 1.5 + 1e-9
    for current in times[1:]:
        if current - previous > tolerance:
            intervals.append((start, previous))
            start = current
        previous = current
    intervals.append((start, previous))
    return intervals


def write_results(
    output: Path,
    results: list[SatelliteResult],
    sample_interval: float,
    minimum_continuous_s: float,
) -> tuple[int, int]:
    output.parent.mkdir(parents=True, exist_ok=True)
    satellite_count = interval_count = 0
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "satellite_id",
                "satellite_name",
                "start_s",
                "end_s",
                "sampled_duration_s",
                "samples",
                "minimum_area_center_distance_km",
                "maximum_visibility_radius_km",
            ]
        )
        for result in sorted(results, key=lambda item: item.satellite_id):
            written_for_satellite = False
            for start, end in continuous_intervals(result.matching_times, sample_interval):
                duration = end - start
                if duration + 1e-9 < minimum_continuous_s:
                    continue
                samples = sum(start <= time <= end for time in result.matching_times)
                writer.writerow(
                    [
                        result.satellite_id,
                        result.name,
                        f"{start:.3f}",
                        f"{end:.3f}",
                        f"{duration:.3f}",
                        samples,
                        f"{result.minimum_center_distance_m / 1000.0:.3f}",
                        f"{result.maximum_coverage_radius_m / 1000.0:.3f}",
                    ]
                )
                interval_count += 1
                written_for_satellite = True
            satellite_count += int(written_for_satellite)
    return satellite_count, interval_count


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--center-node", default="alpha_gs", help="GSS name used as area center")
    parser.add_argument(
        "--location",
        type=float,
        nargs=2,
        metavar=("LATITUDE", "LONGITUDE"),
        help="select coverage for one GPS point instead of loading the UE area",
    )
    parser.add_argument(
        "--exclude-ue",
        action="append",
        default=["alpha_ue_South_India"],
        help="UE name excluded from the geographic service area; repeat as needed",
    )
    parser.add_argument("--trace", type=Path, default=DEFAULT_TRACE)
    parser.add_argument("--names", type=Path, default=None)
    parser.add_argument("--start", type=float, default=0.0, help="window start time in seconds")
    parser.add_argument("--duration", type=float, required=True, help="window duration in seconds")
    parser.add_argument("--sample-interval", type=float, default=1.0)
    parser.add_argument("--num-satellites", type=int, default=500, help="0 means all trace satellites")
    parser.add_argument("--minimum-elevation", type=float, default=10.0)
    parser.add_argument("--maximum-distance", type=float, default=2_500_000.0, help="metres")
    parser.add_argument(
        "--mode",
        choices=("any", "center", "all"),
        default="all",
        help="require visibility of any part, center, or the complete UE area",
    )
    parser.add_argument("--beam-radius-km", type=float, default=250.0)
    parser.add_argument("--minimum-continuous", type=float, default=0.0, help="seconds")
    parser.add_argument("--output", type=Path, default=Path("ue-area-satellite-coverage.csv"))
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration <= 0 or args.sample_interval <= 0:
        raise SystemExit("--duration and --sample-interval must be positive")
    if args.start < 0 or args.num_satellites < 0 or args.minimum_continuous < 0:
        raise SystemExit("start, satellite count, and minimum continuous duration cannot be negative")
    if not 0 <= args.minimum_elevation <= 90 or args.maximum_distance <= 0:
        raise SystemExit("invalid elevation or maximum distance")
    try:
        if args.location is not None:
            area = area_for_location(*args.location)
            area_description = "GPS location"
        else:
            area = load_ue_area(args.data_dir, args.center_node or None, set(args.exclude_ue))
            area_description = "UE area"
        names_path = args.names
        if names_path is None:
            candidate = args.trace.with_suffix(args.trace.suffix + ".names.csv")
            names_path = candidate if candidate.exists() else None
        names = load_satellite_names(names_path)
        if args.mode == "all" and area.radius_m > args.beam_radius_km * 1000.0 + 1.0:
            raise ValueError(
                f"UE enclosing radius {area.radius_m / 1000.0:.1f} km exceeds the "
                f"one-beam radius {args.beam_radius_km:.1f} km"
            )
        results = scan_trace(
            args.trace,
            area,
            names,
            args.start,
            args.start + args.duration,
            args.sample_interval,
            args.num_satellites,
            args.mode,
            args.minimum_elevation,
            args.maximum_distance,
        )
        satellite_count, interval_count = write_results(
            args.output, results, args.sample_interval, args.minimum_continuous
        )
    except (OSError, ValueError) as exc:
        raise SystemExit(f"Error: {exc}") from exc

    node_label = "node" if area.ue_count == 1 else "nodes"
    print(
        f"{area_description}: {area.ue_count:,} {node_label}, center=({area.center_lat:.5f}, "
        f"{area.center_lon:.5f}), enclosing radius={area.radius_m / 1000.0:.2f} km"
    )
    print(
        f"Found {satellite_count} satellites in {interval_count} qualifying intervals; "
        f"wrote {args.output}"
    )


if __name__ == "__main__":
    main()
