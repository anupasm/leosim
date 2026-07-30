#!/usr/bin/env python3
"""Plot the link or handover lifetime of one LeoSim UE or ground server.

When visualization output is available, this consumes ``*-positions.csv`` and
``*-links.csv``.  It can also consume one or more ``result-handovers.csv``
files, including runs made with ``--enableVisualization=0``.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize satellite access links throughout one ground node's lifetime."
    )
    parser.add_argument(
        "results",
        type=Path,
        help=(
            "run directory containing visualization CSVs, or a comparison directory "
            "whose immediate subdirectories contain result-handovers.csv"
        ),
    )
    node = parser.add_mutually_exclusive_group(required=True)
    node.add_argument("--ground-id", type=int, help="ns-3 ground node ID")
    node.add_argument("--ground-name", help="exact ground node name")
    parser.add_argument(
        "--ground-type",
        choices=("UE", "SERVER"),
        help="optional type filter (useful if names are duplicated)",
    )
    parser.add_argument("--positions", type=Path, help="override positions CSV")
    parser.add_argument("--links", type=Path, help="override unified link-state CSV")
    parser.add_argument(
        "--handovers",
        type=Path,
        action="append",
        help="override handover CSV (repeat to compare runs)",
    )
    parser.add_argument("--output", type=Path, help="output PNG path")
    parser.add_argument("--dpi", type=int, default=180)
    return parser.parse_args()


def discover(results: Path, supplied: Path | None, suffix: str) -> Path:
    if supplied:
        return supplied
    matches = sorted(results.glob(f"*-{suffix}.csv"))
    if len(matches) != 1:
        raise SystemExit(
            f"Expected one *-{suffix}.csv in {results}, found {len(matches)}; "
            f"pass --{suffix}."
        )
    return matches[0]


def discover_handovers(results: Path, supplied: list[Path] | None) -> list[Path]:
    if supplied:
        return supplied
    direct = sorted(results.glob("*-handovers.csv"))
    if direct:
        return direct
    return sorted(results.glob("*/result-handovers.csv"))


def resolve_ground(
    positions: Path,
    requested_id: int | None,
    requested_name: str | None,
    requested_type: str | None,
) -> tuple[int, str, str, float, float, float, float | None, float | None]:
    candidates: dict[int, tuple[str, str, float, float, float, float | None, float | None]] = {}
    with positions.open(newline="") as source:
        rows = csv.DictReader(source)
        required = {"time", "type", "id", "name", "x", "y", "z"}
        missing = required.difference(rows.fieldnames or ())
        if missing:
            raise SystemExit(f"{positions} is missing: {', '.join(sorted(missing))}")
        for row in rows:
            kind = row["type"].upper()
            if kind not in {"UE", "SERVER"} or (requested_type and kind != requested_type):
                continue
            node_id = int(row["id"])
            if requested_id is not None and node_id != requested_id:
                continue
            if requested_name is not None and row["name"] != requested_name:
                continue
            latitude = float(row["latitude_deg"]) if row.get("latitude_deg") else None
            longitude = float(row["longitude_deg"]) if row.get("longitude_deg") else None
            candidates[node_id] = (
                row["name"],
                kind,
                float(row["x"]),
                float(row["y"]),
                float(row["z"]),
                latitude,
                longitude,
            )

    if not candidates:
        selector = (
            f"id {requested_id}" if requested_id is not None else f"name {requested_name!r}"
        )
        raise SystemExit(f"No ground node with {selector} found in {positions}")
    if len(candidates) > 1:
        ids = ", ".join(map(str, sorted(candidates)))
        raise SystemExit(f"Ground name is ambiguous (node IDs: {ids}); use --ground-id.")
    node_id, details = next(iter(candidates.items()))
    return (node_id, *details)


def truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes"}


def read_links(links: Path, ground_id: int) -> list[dict[str, str]]:
    selected: list[dict[str, str]] = []
    with links.open(newline="") as source:
        rows = csv.DictReader(source)
        required = {
            "time",
            "ground_id",
            "sat_id",
            "channel_state",
            "snr_db",
            "elevation_deg",
            "path_loss_db",
            "signal_strength_dbm",
        }
        missing = required.difference(rows.fieldnames or ())
        if missing:
            raise SystemExit(
                f"{links} is not a unified link-state CSV; missing: "
                f"{', '.join(sorted(missing))}"
            )
        for row in rows:
            if row["ground_id"] and int(row["ground_id"]) == ground_id:
                selected.append(row)
    if not selected:
        raise SystemExit(f"No access-link samples for ground node {ground_id} in {links}")
    return selected


def read_handovers(handovers: Path, ground_id: int) -> list[dict[str, str]]:
    selected: list[dict[str, str]] = []
    with handovers.open(newline="") as source:
        rows = csv.DictReader(source)
        required = {
            "time_ms",
            "ue_id",
            "src_sat",
            "tgt_sat",
            "type",
            "trigger",
            "latency_ms",
            "success",
            "route_change",
        }
        missing = required.difference(rows.fieldnames or ())
        if missing:
            raise SystemExit(f"{handovers} is missing: {', '.join(sorted(missing))}")
        for row in rows:
            if row["ue_id"] and int(row["ue_id"]) == ground_id:
                selected.append(row)
    return selected


def number(row: dict[str, str], field: str) -> float:
    try:
        return float(row[field])
    except (KeyError, TypeError, ValueError):
        return float("nan")


def plot(
    output: Path,
    node_id: int,
    name: str,
    kind: str,
    xyz: tuple[float, float, float],
    latitude: float | None,
    longitude: float | None,
    rows: list[dict[str, str]],
    dpi: int,
) -> None:
    rows.sort(key=lambda row: (number(row, "time"), number(row, "sat_id")))
    times = [number(row, "time") for row in rows]
    satellites = [number(row, "sat_id") for row in rows]
    states = [row["channel_state"].upper() for row in rows]
    serving = [truthy(row.get("is_serving_access", "")) for row in rows]
    colors = {
        "UP": "#2ca02c",
        "DEGRADED": "#ff7f0e",
        "DOWN": "#d62728",
    }
    point_colors = [colors.get(state, "#7f7f7f") for state in states]
    sizes = [42 if active else 13 for active in serving]

    figure, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    t0, t1 = min(times), max(times)
    location = (
        f"{latitude:.4f}°, {longitude:.4f}°"
        if latitude is not None and longitude is not None
        else f"ECEF ({xyz[0]:.0f}, {xyz[1]:.0f}, {xyz[2]:.0f}) m"
    )
    figure.suptitle(
        f"{kind} node {node_id}: {name}\n"
        f"access-link lifetime {t0:g}–{t1:g} s · {location}",
        fontsize=15,
    )

    axes[0, 0].scatter(times, satellites, c=point_colors, s=sizes, alpha=0.8)
    for state, color in colors.items():
        axes[0, 0].scatter([], [], color=color, label=state, s=25)
    if any(serving):
        axes[0, 0].scatter([], [], facecolors="none", edgecolors="black", s=42,
                           label="larger = serving")
    axes[0, 0].set(
        title="Satellite access links", xlabel="simulation time (s)", ylabel="satellite node ID"
    )
    axes[0, 0].legend(ncol=2, fontsize=8)

    metric_specs = (
        (axes[0, 1], "snr_db", "SNR (dB)", "Link SNR"),
        (axes[1, 0], "elevation_deg", "elevation (degrees)", "Satellite elevation"),
        (axes[1, 1], "path_loss_db", "path loss (dB)", "Path loss"),
    )
    for axis, field, ylabel, title in metric_specs:
        for state, color in colors.items():
            indexes = [i for i, value in enumerate(states) if value == state]
            axis.scatter(
                [times[i] for i in indexes],
                [number(rows[i], field) for i in indexes],
                color=color,
                s=[sizes[i] for i in indexes],
                alpha=0.75,
                label=state,
            )
        axis.set(title=title, xlabel="simulation time (s)", ylabel=ylabel)

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
        axis.set_xlim(t0, t1 if t1 > t0 else t0 + 1)

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi)
    plt.close(figure)


def plot_handovers(
    output: Path,
    node_id: int,
    datasets: list[tuple[str, list[dict[str, str]]]],
    dpi: int,
) -> None:
    figure, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
    figure.suptitle(f"Ground node {node_id}: handover lifetime", fontsize=15)
    palette = plt.get_cmap("tab10")

    all_triggers: dict[str, int] = {}
    all_types: dict[str, int] = {}
    for run_index, (label, rows) in enumerate(datasets):
        rows.sort(key=lambda row: number(row, "time_ms"))
        color = palette(run_index % 10)
        times = [number(row, "time_ms") / 1000.0 for row in rows]
        successes = [truthy(row["success"]) for row in rows]
        source_sats = [number(row, "src_sat") for row in rows]
        target_sats = [number(row, "tgt_sat") for row in rows]
        latencies = [number(row, "latency_ms") for row in rows]

        # Each segment shows the source and attempted target at an event. A
        # failed event is marked with an X and does not imply target service.
        for time, source_sat, target_sat, success in zip(
            times, source_sats, target_sats, successes
        ):
            axes[0, 0].plot(
                [time, time],
                [source_sat, target_sat],
                color=color,
                alpha=0.25,
                linewidth=0.8,
            )
            axes[0, 0].scatter(
                [time],
                [target_sat],
                color=color if success else "#d62728",
                marker="o" if success else "x",
                s=26 if success else 55,
            )

        axes[0, 1].scatter(
            times,
            latencies,
            color=[color if success else "#d62728" for success in successes],
            s=24,
            alpha=0.8,
            label=label,
        )
        axes[1, 0].step(
            times,
            range(1, len(times) + 1),
            where="post",
            color=color,
            label=label,
        )
        for row in rows:
            all_triggers[row["trigger"]] = all_triggers.get(row["trigger"], 0) + 1
            all_types[row["type"]] = all_types.get(row["type"], 0) + 1

    axes[0, 0].set(
        title="Satellite transitions (X = rejected target)",
        xlabel="simulation time (s)",
        ylabel="satellite node ID",
    )
    axes[0, 1].set(
        title="Handover latency",
        xlabel="simulation time (s)",
        ylabel="latency (ms)",
    )
    axes[1, 0].set(
        title="Cumulative handovers",
        xlabel="simulation time (s)",
        ylabel="event count",
    )

    categories = list(all_types) + list(all_triggers)
    counts = [all_types[value] for value in all_types] + [
        all_triggers[value] for value in all_triggers
    ]
    axes[1, 1].bar(range(len(categories)), counts, color="#4c78a8")
    axes[1, 1].set(
        title="Event type and trigger counts",
        ylabel="events across selected runs",
        xticks=range(len(categories)),
        xticklabels=categories,
    )
    axes[1, 1].tick_params(axis="x", rotation=30)

    for axis in axes.flat:
        axis.grid(True, alpha=0.25)
    if len(datasets) > 1:
        axes[0, 1].legend(fontsize=7)
        axes[1, 0].legend(fontsize=7)

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=dpi)
    plt.close(figure)


def main() -> None:
    args = parse_args()
    handover_files = discover_handovers(args.results, args.handovers)
    position_matches = sorted(args.results.glob("*-positions.csv"))
    link_matches = sorted(args.results.glob("*-links.csv"))

    # Prefer the richer link-state view when its two inputs are available or
    # explicitly supplied. Otherwise fall back to handover-only output.
    use_link_view = bool(args.positions or args.links or (position_matches and link_matches))
    if not use_link_view:
        if args.ground_id is None:
            raise SystemExit(
                "Handover-only input has no node names; select the node with --ground-id."
            )
        if not handover_files:
            raise SystemExit(
                f"No *-handovers.csv found in {args.results} or its immediate subdirectories."
            )
        datasets: list[tuple[str, list[dict[str, str]]]] = []
        for handovers in handover_files:
            rows = read_handovers(handovers, args.ground_id)
            if rows:
                label = handovers.parent.name if handovers.parent != args.results else handovers.stem
                datasets.append((label, rows))
        if not datasets:
            raise SystemExit(
                f"No handover events for ground node {args.ground_id} in "
                f"{len(handover_files)} handover file(s)."
            )
        output = args.output or args.results / f"ground-node-lifetime-{args.ground_id}.png"
        plot_handovers(output, args.ground_id, datasets, args.dpi)
        count = sum(len(rows) for _, rows in datasets)
        print(
            f"Wrote {output} ({count} handover events for ground node "
            f"{args.ground_id} across {len(datasets)} run(s))"
        )
        return

    positions = discover(args.results, args.positions, "positions")
    links = discover(args.results, args.links, "links")
    node_id, name, kind, x, y, z, latitude, longitude = resolve_ground(
        positions, args.ground_id, args.ground_name, args.ground_type
    )
    link_rows = read_links(links, node_id)
    output = args.output or args.results / f"ground-node-lifetime-{node_id}.png"
    plot(
        output,
        node_id,
        name,
        kind,
        (x, y, z),
        latitude,
        longitude,
        link_rows,
        args.dpi,
    )
    print(f"Wrote {output} ({len(link_rows)} link samples for {kind} {node_id}: {name})")


if __name__ == "__main__":
    main()
