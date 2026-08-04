#!/usr/bin/env python3
"""Create publication-ready figures from a LeoSim routing-metric sweep.

The input is the output directory produced by ``run_routing_sweep.sh`` (or an
equivalent collection of ``metric-*/result-statistics.csv`` directories).
Figures are written as vector PDF and high-resolution PNG by default.
"""

from __future__ import annotations

import argparse
import csv
import ipaddress
import json
import re
import warnings
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.ticker import PercentFormatter


METRIC_ORDER = (
    "hop",
    "distance",
    "path-loss",
    "snr",
    "signal-strength",
    "lifetime",
    "load",
)
LABELS = {
    "hop": "Hop count",
    "distance": "Distance",
    "path-loss": "Path loss",
    "snr": "SNR",
    "signal-strength": "Signal strength",
    "lifetime": "Lifetime",
    "load": "Load-aware",
}
COLORS = {
    "hop": "#0072B2",
    "distance": "#D50000",
    "path-loss": "#009E73",
    "snr": "#CC79A7",
    "signal-strength": "#E69F00",
    "lifetime": "#52D30D",
    "load": "#56B4E9",
}
MARKERS = dict(zip(METRIC_ORDER, ("o", "s", "^", "D", "P", "X", "v")))
RUN_PATTERN = re.compile(
    r"^metric-(.+)_sats-(\d+)_(static|dynamic)_duration-(\d+)s"
    r"(?:-interval-(\d+)s)?$"
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "results_dir",
        nargs="?",
        type=Path,
        default=Path("results11514"),
        help="routing sweep directory (default: results11514)",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        type=Path,
        help="figure directory (default: RESULTS_DIR/paper-figures)",
    )
    parser.add_argument(
        "--formats",
        nargs="+",
        default=("pdf", "png"),
        choices=("pdf", "png", "svg"),
        help="output formats (default: pdf png)",
    )
    parser.add_argument(
        "--metrics",
        nargs="+",
        choices=METRIC_ORDER,
        metavar="METRIC",
        help=(
            "routing metrics to visualize (default: all discovered metrics); "
            "choices: %(choices)s"
        ),
    )
    parser.add_argument("--dpi", type=int, default=300, help="raster DPI")
    parser.add_argument(
        "--warmup",
        type=float,
        default=60.0,
        help="exclude earlier samples from distribution plots, in seconds",
    )
    parser.add_argument(
        "--smooth",
        type=int,
        default=15,
        help="centred rolling-median window for time-series curves",
    )
    return parser.parse_args()


def discover_runs(results_dir: Path) -> dict[str, dict]:
    runs: dict[str, dict] = {}
    for run_dir in sorted(results_dir.glob("metric-*")):
        match = RUN_PATTERN.match(run_dir.name) if run_dir.is_dir() else None
        stats_file = run_dir / "result-statistics.csv"
        if not match or not stats_file.is_file():
            continue
        metric, satellites, mode, duration, interval = match.groups()
        frame = pd.read_csv(stats_file)
        required = {
            "time_s",
            "pdr",
            "throughput_mbps",
            "delay_ms",
            "jitter_ms",
            "hop_count",
        }
        missing = required.difference(frame.columns)
        if missing:
            raise ValueError(f"{stats_file} lacks columns: {', '.join(sorted(missing))}")
        frame = frame.sort_values("time_s").drop_duplicates("time_s")
        runs[metric] = {
            "dir": run_dir,
            "data": frame,
            "traffic_pairs": read_traffic_pairs(run_dir / "result-statistics.json"),
            "satellites": int(satellites),
            "mode": mode,
            "duration": float(duration),
            "interval": float(interval or 0),
        }
    if not runs:
        raise FileNotFoundError(
            f"No metric-*/result-statistics.csv runs found below {results_dir}"
        )
    return runs


def read_traffic_pairs(statistics_file: Path) -> pd.DataFrame | None:
    """Aggregate FlowMonitor statistics by bidirectional ground-node pair."""
    if not statistics_file.is_file():
        warnings.warn(
            f"{statistics_file} is missing; omitting this run from "
            "per-ground-node-pair traffic plots",
            stacklevel=2,
        )
        return None

    with statistics_file.open() as stream:
        statistics = json.load(stream)
    flows = statistics.get("flows", [])
    if not flows:
        raise ValueError(f"{statistics_file} contains no per-flow statistics")

    pairs: dict[tuple[str, str], dict[str, float | str]] = {}
    for flow in flows:
        source = str(flow["source"])
        destination = str(flow["destination"])
        endpoints = tuple(
            sorted(
                (source, destination),
                key=lambda address: int(ipaddress.ip_address(address)),
            )
        )
        pair = pairs.setdefault(
            endpoints,
            {
                "endpoint_a": endpoints[0],
                "endpoint_b": endpoints[1],
                "tx_packets": 0.0,
                "rx_packets": 0.0,
                "lost_packets": 0.0,
                "throughput_mbps": 0.0,
                "delay_weighted": 0.0,
                "hop_weighted": 0.0,
            },
        )
        received = float(flow["rx_packets"])
        pair["tx_packets"] += float(flow["tx_packets"])
        pair["rx_packets"] += received
        pair["lost_packets"] += float(flow["lost_packets"])
        pair["throughput_mbps"] += float(flow["throughput_mbps"])
        pair["delay_weighted"] += float(flow["delay_ms"]) * received
        pair["hop_weighted"] += float(flow["mean_hop_count"]) * received

    records = []
    for pair in pairs.values():
        transmitted = float(pair["tx_packets"])
        received = float(pair["rx_packets"])
        records.append(
            {
                "endpoint_a": pair["endpoint_a"],
                "endpoint_b": pair["endpoint_b"],
                "throughput_mbps": pair["throughput_mbps"],
                "pdr": received / transmitted if transmitted else np.nan,
                "delay_ms": (
                    float(pair["delay_weighted"]) / received if received else np.nan
                ),
                "hop_count": (
                    float(pair["hop_weighted"]) / received if received else np.nan
                ),
                "lost_packets": pair["lost_packets"],
            }
        )

    return (
        pd.DataFrame(records)
        .sort_values(["endpoint_a", "endpoint_b"])
        .set_index(["endpoint_a", "endpoint_b"])
    )


def read_summary(results_dir: Path, runs: dict[str, dict]) -> pd.DataFrame:
    summary_file = results_dir / "analysis" / "routing-metric-summary.csv"
    if summary_file.is_file():
        summary = pd.read_csv(summary_file)
        summary = summary[summary["metric"].isin(runs)].copy()
        if len(summary) == len(runs):
            return summary.set_index("metric")

    # The final statistics row contains the cumulative network results.
    records = []
    for metric, run in runs.items():
        row = run["data"].iloc[-1]
        records.append(
            {
                "metric": metric,
                "pdr": row["pdr"],
                "throughput_mbps": row["throughput_mbps"],
                "delay_ms": row["delay_ms"],
                "flow_mean_hop_count": row["hop_count"],
                "route_changes": np.nan,
                "unique_paths": np.nan,
                "mean_route_distance_km": np.nan,
                "mean_route_min_snr_db": np.nan,
            }
        )
    return pd.DataFrame(records).set_index("metric")


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 8.5,
            "axes.labelsize": 9,
            "axes.titlesize": 9,
            "legend.fontsize": 7.5,
            "xtick.labelsize": 8,
            "ytick.labelsize": 8,
            "axes.linewidth": 0.7,
            "grid.linewidth": 0.45,
            "lines.linewidth": 1.35,
            "lines.markersize": 4,
            "figure.dpi": 120,
            "savefig.bbox": "tight",
            "savefig.pad_inches": 0.03,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def ordered_metrics(runs: dict[str, dict]) -> list[str]:
    known = [metric for metric in METRIC_ORDER if metric in runs]
    return known + sorted(set(runs).difference(known))


def select_metrics(runs: dict[str, dict], requested: list[str] | None) -> dict[str, dict]:
    """Return only requested routing metrics, preserving canonical plot order."""
    if requested is None:
        return runs

    missing = sorted(set(requested).difference(runs))
    if missing:
        available = ", ".join(ordered_metrics(runs))
        raise ValueError(
            f"Requested metric(s) not found: {', '.join(missing)}. "
            f"Available metrics: {available}"
        )
    requested_set = set(requested)
    return {
        metric: runs[metric]
        for metric in ordered_metrics(runs)
        if metric in requested_set
    }


def add_grid(axis: plt.Axes) -> None:
    axis.grid(True, color="#D7D7D7", alpha=0.8)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)


def plot_timeseries(runs: dict[str, dict], smooth: int) -> plt.Figure:
    fields = (
        ("throughput_mbps", "Cumulative throughput (Mbit/s)"),
        ("delay_ms", "Mean delay (ms)"),
        ("jitter_ms", "Mean jitter (ms)"),
        ("hop_count", "Mean hop count"),
        ("pdr", "Packet delivery ratio"),
    )
    figure, axes = plt.subplots(3, 2, figsize=(7.15, 6.4), sharex=True)
    for axis, (field, ylabel) in zip(axes.flat, fields):
        for metric in ordered_metrics(runs):
            data = runs[metric]["data"]
            values = data[field].rolling(
                max(1, smooth), center=True, min_periods=1
            ).median()
            axis.plot(
                data["time_s"] / 60.0,
                values,
                color=COLORS.get(metric),
                label=LABELS.get(metric, metric),
            )
        axis.set_ylabel(ylabel)
        add_grid(axis)
    axes[2, 1].set_visible(False)
    axes[2, 0].set_xlabel("Simulation time (min)")
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(
        handles,
        legend_labels,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.01),
        ncol=min(5, len(runs)),
        frameon=False,
    )
    figure.subplots_adjust(bottom=0.13, hspace=0.18, wspace=0.24)
    return figure


def plot_steady_state(runs: dict[str, dict], warmup: float) -> plt.Figure:
    metrics = ordered_metrics(runs)
    fields = (
        ("throughput_mbps", "Cumulative throughput (Mbit/s)"),
        ("delay_ms", "Mean delay (ms)"),
        ("jitter_ms", "Mean jitter (ms)"),
        ("hop_count", "Mean hop count"),
    )
    figure, axes = plt.subplots(1, 4, figsize=(9.2, 2.65))
    positions = np.arange(1, len(metrics) + 1)
    for axis, (field, ylabel) in zip(axes, fields):
        samples = []
        for metric in metrics:
            data = runs[metric]["data"]
            values = data.loc[data["time_s"] >= warmup, field].dropna().to_numpy()
            if values.size == 0:
                raise ValueError(
                    f"No {metric} samples remain after --warmup={warmup:g}s"
                )
            samples.append(values)
        parts = axis.violinplot(
            samples,
            positions=positions,
            widths=0.82,
            showmeans=False,
            showmedians=True,
            showextrema=False,
            points=150,
        )
        for body, metric in zip(parts["bodies"], metrics):
            body.set_facecolor(COLORS.get(metric, "#777777"))
            body.set_edgecolor("black")
            body.set_alpha(0.72)
            body.set_linewidth(0.45)
        parts["cmedians"].set_color("black")
        parts["cmedians"].set_linewidth(1.1)
        axis.set_xticks(positions, [LABELS.get(m, m) for m in metrics])
        axis.tick_params(axis="x", rotation=35)
        axis.set_ylabel(ylabel)
        add_grid(axis)
    figure.subplots_adjust(bottom=0.28, wspace=0.32)
    return figure


def plot_tradeoffs(summary: pd.DataFrame, metrics: list[str]) -> plt.Figure:
    required = (
        "mean_route_distance_km",
        "flow_mean_hop_count",
        "route_changes",
        "throughput_mbps",
    )
    if any(field not in summary or summary[field].isna().all() for field in required):
        raise ValueError(
            "Route-level fields are absent; run analyze_routing_sweep.py first"
        )
    data = summary.loc[metrics]
    figure, (left, right) = plt.subplots(1, 2, figsize=(7.15, 3.05))

    sizes = 35 + 100 * (
        (data["throughput_mbps"] - data["throughput_mbps"].min())
        / max(data["throughput_mbps"].max() - data["throughput_mbps"].min(), 1e-12)
    )
    for metric in metrics:
        left.scatter(
            data.at[metric, "mean_route_distance_km"] / 1000.0,
            data.at[metric, "flow_mean_hop_count"],
            s=sizes[metric],
            marker=MARKERS.get(metric, "o"),
            color=COLORS.get(metric),
            edgecolor="black",
            linewidth=0.5,
            label=LABELS.get(metric, metric),
            zorder=3,
        )
    left.set_xlabel(r"Mean route distance ($10^3$ km)")
    left.set_ylabel("Mean flow hop count")
    left.legend(frameon=False, loc="best")
    add_grid(left)

    baseline = data.loc["hop"] if "hop" in data.index else data.iloc[0]
    fields = ("throughput_mbps", "delay_ms", "route_changes")
    titles = ("Throughput", "Delay", "Route changes")
    values = np.column_stack(
        [
            100.0 * (data[field].to_numpy() / baseline[field] - 1.0)
            for field in fields
        ]
    )
    x = np.arange(len(metrics))
    width = 0.23
    hatches = ("", "//", "xx")
    for index, title in enumerate(titles):
        right.bar(
            x + (index - 1) * width,
            values[:, index],
            width,
            label=title,
            facecolor="white",
            edgecolor=("#0072B2", "#D55E00", "#009E73")[index],
            hatch=hatches[index],
            linewidth=1.0,
        )
    right.axhline(0, color="black", linewidth=0.7)
    right.set_xticks(x, [LABELS.get(m, m) for m in metrics])
    right.tick_params(axis="x", rotation=35)
    right.set_ylabel("Relative change vs. hop routing")
    right.yaxis.set_major_formatter(PercentFormatter())
    right.legend(frameon=False, ncol=3, loc="upper left")
    add_grid(right)
    figure.subplots_adjust(bottom=0.25, wspace=0.32)
    return figure


def compact_pair_label(pair: tuple[str, str]) -> str:
    """Return a compact but unambiguous IPv4 pair label for the x axis."""
    left, right = pair

    def compact(address: str) -> str:
        octets = address.split(".")
        return ".".join(octets[-2:]) if len(octets) == 4 else address

    return f"{compact(left)}↔{compact(right)}"


def plot_pair_traffic(runs: dict[str, dict]) -> plt.Figure:
    """Plot all observed ground-node pairs, leaving missing values as gaps."""
    metrics = [
        metric
        for metric in ordered_metrics(runs)
        if runs[metric]["traffic_pairs"] is not None
    ]
    if not metrics:
        raise ValueError(
            "No result-statistics.json files with ground-node traffic pairs were found"
        )
    pair_sets = [set(runs[metric]["traffic_pairs"].index) for metric in metrics]
    all_pairs = set.union(*pair_sets)
    if not all_pairs:
        raise ValueError("No ground-node traffic pairs were found")

    pairs = sorted(
        all_pairs,
        key=lambda pair: (
            int(ipaddress.ip_address(pair[0])),
            int(ipaddress.ip_address(pair[1])),
        ),
    )
    fields = (
        ("throughput_mbps", "Combined throughput (Mbit/s)"),
        ("delay_ms", "Packet-weighted delay (ms)"),
        ("pdr", "Packet delivery ratio"),
        ("lost_packets", "Lost packets"),
    )
    figure, axes = plt.subplots(2, 2, figsize=(7.15, 5.2), sharex=True)
    x = np.arange(len(pairs))
    for axis, (field, ylabel) in zip(axes.flat, fields):
        for metric in metrics:
            pair_data = runs[metric]["traffic_pairs"]
            # Reindex against the union so every observed pair is retained.
            # Matplotlib renders pairs absent from a particular run as gaps.
            values = pair_data[field].reindex(pairs).to_numpy()
            axis.plot(
                x,
                values,
                color=COLORS.get(metric),
                marker=MARKERS.get(metric, "o"),
                label=LABELS.get(metric, metric),
            )
        axis.set_ylabel(ylabel)
        if field == "pdr":
            axis.yaxis.set_major_formatter(PercentFormatter(xmax=1.0, decimals=2))
        add_grid(axis)

    labels = [compact_pair_label(pair) for pair in pairs]
    for axis in axes[1]:
        axis.set_xticks(x, labels)
        axis.tick_params(axis="x", rotation=42, labelsize=7)
    figure.supxlabel("Bidirectional ground-node pair", y=0.105)
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(
        handles,
        legend_labels,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.005),
        ncol=min(5, len(metrics)),
        frameon=False,
    )
    figure.subplots_adjust(bottom=0.28, hspace=0.14, wspace=0.28)
    return figure


def save_figure(
    figure: plt.Figure,
    output_dir: Path,
    stem: str,
    formats: list[str],
    dpi: int,
) -> list[Path]:
    outputs = []
    for extension in formats:
        filename = output_dir / f"{stem}.{extension}"
        figure.savefig(filename, dpi=dpi)
        outputs.append(filename)
    plt.close(figure)
    return outputs


def write_manifest(
    output_dir: Path, results_dir: Path, runs: dict[str, dict], outputs: list[Path]
) -> None:
    with (output_dir / "figure-manifest.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("figure", "source_results", "metrics"))
        for output in outputs:
            writer.writerow(
                (
                    output.name,
                    str(results_dir.resolve()),
                    ";".join(ordered_metrics(runs)),
                )
            )


def main() -> int:
    args = parse_args()
    results_dir = args.results_dir.resolve()
    output_dir = (args.output_dir or results_dir / "paper-figures").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()

    runs = select_metrics(discover_runs(results_dir), args.metrics)
    metrics = ordered_metrics(runs)
    summary = read_summary(results_dir, runs)
    outputs: list[Path] = []
    outputs += save_figure(
        plot_timeseries(runs, args.smooth),
        output_dir,
        "routing-performance-timeseries",
        args.formats,
        args.dpi,
    )
    outputs += save_figure(
        plot_steady_state(runs, args.warmup),
        output_dir,
        "routing-performance-distributions",
        args.formats,
        args.dpi,
    )
    outputs += save_figure(
        plot_pair_traffic(runs),
        output_dir,
        "ground-pair-traffic",
        args.formats,
        args.dpi,
    )
    try:
        tradeoff_figure = plot_tradeoffs(summary, metrics)
    except ValueError as error:
        print(f"Skipping route trade-off figure: {error}")
    else:
        outputs += save_figure(
            tradeoff_figure,
            output_dir,
            "routing-tradeoffs",
            args.formats,
            args.dpi,
        )
    write_manifest(output_dir, results_dir, runs, outputs)
    print(f"Wrote {len(outputs)} figure files to {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
