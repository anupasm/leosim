#!/usr/bin/env python3
"""Visualize one satellite node over the lifetime recorded in a result set.

The input is the position CSV stored under ``<results>/_preprocessed``.  The
file may contain millions of rows, so it is read as a stream.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize


EARTH_RADIUS_KM = 6371.0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot the position and motion history of one node."
    )
    parser.add_argument(
        "results",
        type=Path,
        help="result directory (for example ns3/results11514_3)",
    )
    node = parser.add_mutually_exclusive_group()
    node.add_argument("--node-id", type=int, default=None, help="sat_id to plot")
    node.add_argument("--node-name", help="exact sat_name to plot")
    parser.add_argument(
        "--positions",
        type=Path,
        help="position CSV; defaults to the only CSV in RESULTS/_preprocessed",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="output PNG (default: RESULTS/node-lifetime-<id>.png)",
    )
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def find_positions(results: Path, supplied: Path | None) -> Path:
    if supplied:
        return supplied
    candidates = sorted((results / "_preprocessed").glob("*.csv"))
    if len(candidates) != 1:
        raise SystemExit(
            f"Expected one position CSV in {results / '_preprocessed'}, "
            f"found {len(candidates)}; pass --positions."
        )
    return candidates[0]


def read_node(
    path: Path, node_id: int | None, node_name: str | None
) -> tuple[int, str, list[float], list[float], list[float], list[float]]:
    times: list[float] = []
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    selected_id = node_id
    selected_name = node_name

    with path.open(newline="") as source:
        rows = csv.DictReader(source)
        required = {"sat_id", "sat_name", "time_s", "x_m", "y_m", "z_m"}
        missing = required.difference(rows.fieldnames or ())
        if missing:
            raise SystemExit(f"{path} is missing columns: {', '.join(sorted(missing))}")

        for row in rows:
            row_id = int(row["sat_id"])
            row_name = row["sat_name"]
            # With no selector, use the first node in the file.
            if selected_id is None and selected_name is None:
                selected_id, selected_name = row_id, row_name
            matches = (
                row_id == selected_id if selected_id is not None else row_name == selected_name
            )
            if not matches:
                continue
            selected_id, selected_name = row_id, row_name
            times.append(float(row["time_s"]))
            xs.append(float(row["x_m"]) / 1000.0)
            ys.append(float(row["y_m"]) / 1000.0)
            zs.append(float(row["z_m"]) / 1000.0)

    if not times or selected_id is None or selected_name is None:
        selector = f"id {node_id}" if node_id is not None else f"name {node_name!r}"
        raise SystemExit(f"No samples found for node {selector} in {path}")

    ordered = sorted(zip(times, xs, ys, zs))
    times, xs, ys, zs = (list(values) for values in zip(*ordered))
    return selected_id, selected_name, times, xs, ys, zs


def colored_line(
    axis: plt.Axes, x: list[float], y: list[float], time: list[float]
) -> LineCollection:
    points = list(zip(x, y))
    segments = [[points[index], points[index + 1]] for index in range(len(points) - 1)]
    collection = LineCollection(
        segments,
        cmap="viridis",
        norm=Normalize(min(time), max(time) if max(time) > min(time) else min(time) + 1),
        linewidth=2.2,
    )
    collection.set_array(time[:-1])
    axis.add_collection(collection)
    axis.autoscale()
    return collection


def draw_earth(axis: plt.Axes) -> None:
    circle = plt.Circle(
        (0, 0), EARTH_RADIUS_KM, facecolor="#dceaf4", edgecolor="#7697ad", alpha=0.8
    )
    axis.add_patch(circle)


def plot(
    output: Path,
    node_id: int,
    node_name: str,
    time: list[float],
    x: list[float],
    y: list[float],
    z: list[float],
    dpi: int,
) -> None:
    radius = [math.sqrt(a * a + b * b + c * c) for a, b, c in zip(x, y, z)]
    altitude = [value - EARTH_RADIUS_KM for value in radius]
    speed = [math.nan]
    for index in range(1, len(time)):
        dt = time[index] - time[index - 1]
        distance = math.dist(
            (x[index - 1], y[index - 1], z[index - 1]),
            (x[index], y[index], z[index]),
        )
        speed.append(distance / dt if dt > 0 else math.nan)

    figure, axes = plt.subplots(2, 2, figsize=(13, 9), constrained_layout=True)
    figure.suptitle(
        f"Node {node_id}: {node_name}\n"
        f"recorded lifetime {time[0]:g}–{time[-1]:g} s "
        f"({time[-1] - time[0]:g} s, {len(time)} samples)",
        fontsize=15,
    )

    track = colored_line(axes[0, 0], x, y, time)
    draw_earth(axes[0, 0])
    axes[0, 0].scatter(x[0], y[0], marker="o", color="#1b9e77", label="start", zorder=4)
    axes[0, 0].scatter(x[-1], y[-1], marker="X", color="#d95f02", label="end", zorder=4)
    axes[0, 0].set(title="Earth-centred XY trajectory", xlabel="x (km)", ylabel="y (km)")
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 0].legend()
    figure.colorbar(track, ax=axes[0, 0], label="simulation time (s)")

    axes[0, 1].plot(time, altitude, color="#377eb8")
    axes[0, 1].set(
        title="Altitude over recorded lifetime",
        xlabel="simulation time (s)",
        ylabel="altitude above mean Earth radius (km)",
    )

    axes[1, 0].plot(time, x, label="x")
    axes[1, 0].plot(time, y, label="y")
    axes[1, 0].plot(time, z, label="z")
    axes[1, 0].set(
        title="Earth-centred coordinates",
        xlabel="simulation time (s)",
        ylabel="position (km)",
    )
    axes[1, 0].legend(ncol=3)

    axes[1, 1].plot(time, speed, color="#984ea3")
    axes[1, 1].set(
        title="Estimated node speed",
        xlabel="simulation time (s)",
        ylabel="speed (km/s)",
    )

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi)
    plt.close(figure)


def main() -> None:
    args = arguments()
    positions = find_positions(args.results, args.positions)
    node_id, node_name, time, x, y, z = read_node(
        positions, args.node_id, args.node_name
    )
    output = args.output or args.results / f"node-lifetime-{node_id}.png"
    plot(output, node_id, node_name, time, x, y, z, args.dpi)
    print(f"Wrote {output} ({len(time)} samples for {node_name}, node {node_id})")


if __name__ == "__main__":
    main()
