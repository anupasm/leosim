#!/usr/bin/env python3
"""Compare LeoSim routing metrics from run_routing_sweep.sh outputs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
from collections import defaultdict
from pathlib import Path
from statistics import mean


METRIC_ORDER = {name: index for index, name in enumerate(
    ("hop", "distance", "path-loss", "snr", "signal-strength"))}


def number(value, default=0.0):
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def resolve_output(base: Path, value: str, suffix: str = "") -> Path:
    path = Path(value + suffix)
    # If the given path is absolute but doesn't exist on this host (for
    # example the manifest recorded container paths like `/results/...`),
    # attempt to map it into the provided `base` results tree by trying a
    # few reasonable candidates.
    if path.is_absolute():
        if path.exists():
            return path
        # Drop the leading root and try to resolve relative to cwd and the
        # provided base paths. If the absolute path begins with a
        # container-mounted prefix like 'results', strip that component so
        # that '/results/metric-...' maps to '<results_dir>/metric-...'.
        rel = Path(*path.parts[1:]) if len(path.parts) > 1 else Path()
        if rel.parts and rel.parts[0] == "results":
            rel = Path(*rel.parts[1:]) if len(rel.parts) > 1 else Path()
        candidates = (Path.cwd() / rel, base / rel, base.parent / rel)
        return next((candidate for candidate in candidates if candidate.exists()), candidates[1])

    candidates = (Path.cwd() / path, base.parent / path, base / path)
    return next((candidate for candidate in candidates if candidate.exists()), candidates[1])


def read_route_metrics(filename: Path | None,
                       satellites: int,
                       servers: int,
                       ues: int) -> dict:
    empty = {
        "route_data_available": False,
        "route_samples": 0,
        "route_valid_ratio": 0.0,
        "route_changes": 0,
        "unique_paths": 0,
        "mean_route_hops": 0.0,
        "mean_route_distance_km": 0.0,
        "mean_route_min_snr_db": 0.0,
        "minimum_route_snr_db": 0.0,
        "mean_route_path_loss_db": 0.0,
        "mean_route_min_signal_dbm": 0.0,
    }
    # External destination-tree runs deliberately disable the verbose route
    # CSV. Network and per-flow statistics are still complete, so do not drop
    # an otherwise successful run merely because result-routes.csv is absent.
    if filename is None or not filename.is_file():
        return empty

    # leosim-param-scenario creates all server nodes first, followed by UE nodes.
    # Aggregate every UE -> server pair instead of assuming one server and one UE.
    server_nodes = set(range(satellites, satellites + servers))
    ue_nodes = set(range(satellites + servers, satellites + servers + ues))
    rows = []
    with filename.open(newline="") as stream:
        for row in csv.DictReader(stream):
            if (int(row["source_node"]) in ue_nodes and
                    int(row["destination_node"]) in server_nodes):
                rows.append(row)

    valid = [row for row in rows if row["valid"] == "1"]
    values = lambda field: [number(row[field]) for row in valid]
    return {
        # A disabled logger can leave a stale header-only CSV from an earlier
        # run. Treat it as unavailable so the compact JSON routing summary wins.
        "route_data_available": bool(rows),
        "route_samples": len(rows),
        "route_valid_ratio": len(valid) / len(rows) if rows else 0.0,
        "route_changes": sum(int(row["route_changed"]) for row in rows),
        "unique_paths": len({row["path"] for row in valid}),
        "mean_route_hops": mean(values("hop_count")) if valid else 0.0,
        "mean_route_distance_km": mean(values("total_distance_m")) / 1000.0 if valid else 0.0,
        "mean_route_min_snr_db": mean(values("min_snr_db")) if valid else 0.0,
        "minimum_route_snr_db": min(values("min_snr_db")) if valid else 0.0,
        "mean_route_path_loss_db": mean(values("total_path_loss_db")) if valid else 0.0,
        "mean_route_min_signal_dbm": mean(values("min_signal_strength_dbm")) if valid else 0.0,
    }


def load_runs(results_dir: Path) -> list[dict]:
    manifest = results_dir / "manifest.csv"
    runs = []
    if manifest.exists():
        with manifest.open(newline="") as stream:
            items = list(csv.DictReader(stream))
    else:
        # Fall back to discovering completed run directories when a manifest
        # is missing (for example runs produced directly on the host or by
        # other tooling). This keeps analysis robust to missing manifests.
        items = []

    # Also discover completed run directories. This makes analysis resilient to
    # an interrupted/truncated manifest when all per-run outputs were completed.
    known = {item["run_id"] for item in items}
    pattern = re.compile(
        r"^metric-(.+)_sats-(\d+)_(static|dynamic)_duration-(\d+)s(?:-interval-(\d+)s)?$")
    for run_dir in results_dir.iterdir():
        match = pattern.match(run_dir.name) if run_dir.is_dir() else None
        if not match or run_dir.name in known:
            continue
        metric, satellites, mode, duration, interval = match.groups()
        if not (run_dir / "result-statistics.json").exists():
            continue
        status_file = run_dir / "status.txt"
        status = status_file.read_text().strip() if status_file.exists() else "UNKNOWN"
        items.append({
            "run_id": run_dir.name,
            "metric": metric,
            "satellites": satellites,
            "mode": mode,
            "update_interval_s": interval or "0",
            "sim_time_s": duration,
            "status": status,
            "wall_time_s": "0",
            "output_prefix": str(run_dir / "result"),
            "route_log_file": str(run_dir / "result-routes.csv"),
            "_run_dir": str(run_dir),
        })

    for item in items:
        prefix = ((Path(item["_run_dir"]) / "result") if "_run_dir" in item
                  else resolve_output(results_dir, item["output_prefix"]))
        statistics_file = Path(str(prefix) + "-statistics.json")
        if "_run_dir" in item:
            candidate_route_file = Path(item["_run_dir"]) / "result-routes.csv"
        elif item.get("route_log_file"):
            candidate_route_file = resolve_output(results_dir, item["route_log_file"])
        else:
            candidate_route_file = Path(str(prefix) + "-routes.csv")
        route_file = candidate_route_file if candidate_route_file.is_file() else None
        if statistics_file.exists():
            with statistics_file.open() as stats_stream:
                stats = json.load(stats_stream)
        else:
            # Fallback: some runs only provide a CSV statistics file. Read the
            # final row and synthesise the `network` summary expected by the
            # rest of the script.
            csv_stats = Path(str(prefix) + "-statistics.csv")
            if csv_stats.exists():
                with csv_stats.open(newline="") as csv_stream:
                    reader = list(csv.DictReader(csv_stream))
                last = reader[-1] if reader else {}
                stats = {
                    "network": {
                        "pdr": number(last.get("pdr", 0)),
                        "throughput_mbps": number(last.get("throughput_mbps", 0)),
                        "delay_ms": number(last.get("delay_ms", 0)),
                        "jitter_ms": number(last.get("jitter_ms", 0)),
                        "mean_hop_count": number(last.get("hop_count", 0)),
                        "tx_packets": int(float(last.get("tx_packets", 0))) if last.get("tx_packets") is not None else 0,
                        "rx_packets": int(float(last.get("rx_packets", 0))) if last.get("rx_packets") is not None else 0,
                        "lost_packets": int(float(last.get("lost_packets", 0))) if last.get("lost_packets") is not None else 0,
                    },
                    "handover": {
                        "total": int(number(last.get("handovers"))),
                        # The periodic CSV does not contain the successful
                        # count, but it does contain the corresponding ratio.
                        "success_ratio": number(last.get("ho_success_ratio"), 1.0),
                        "latency_mean_ms": number(last.get("ho_latency_mean_ms")),
                        "latency_p95_ms": number(last.get("ho_latency_p95_ms")),
                        "ping_pongs": int(number(last.get("ping_pongs"))),
                    },
                }
            else:
                raise FileNotFoundError(f"Missing statistics file for run: {prefix}")

        network = stats["network"]
        satellites = int(item["satellites"])
        run = {
            "run_id": item["run_id"],
            "metric": item["metric"],
            "satellites": satellites,
            "mode": item["mode"],
            "update_interval_s": number(item["update_interval_s"]),
            "sim_time_s": number(item["sim_time_s"]),
            "status": item["status"],
            "wall_time_s": number(item["wall_time_s"]),
            "pdr": number(network["pdr"]),
            "throughput_mbps": number(network["throughput_mbps"]),
            "delay_ms": number(network["delay_ms"]),
            "jitter_ms": number(network["jitter_ms"]),
            "flow_mean_hop_count": number(network["mean_hop_count"]),
            "tx_packets": int(network["tx_packets"]),
            "rx_packets": int(network["rx_packets"]),
            "lost_packets": int(network["lost_packets"]),
        }
        # Forward application flows use an ephemeral source port and a lower
        # server listening port. Their unique endpoint addresses reveal the
        # resolved counts when command-line zero means "all loaded endpoints".
        forward_flows = [flow for flow in stats.get("flows", [])
                         if int(flow.get("source_port", 0)) >
                         int(flow.get("destination_port", 0))]
        server_count = len({flow.get("destination") for flow in forward_flows})
        ue_count = len({flow.get("source") for flow in forward_flows})
        handover = stats.get("handover")
        handover_available = isinstance(handover, dict)
        handover = handover or {}
        handovers = int(number(handover.get("total")))
        success_ratio = number(handover.get("success_ratio"), 1.0)
        successful = int(number(
            handover.get("successful"),
            round(handovers * success_ratio),
        ))
        run.update({
            "handover_data_available": handover_available,
            "handovers": handovers,
            "successful_handovers": successful,
            "failed_handovers": max(0, handovers - successful),
            "handover_success_ratio": success_ratio,
            "mean_handover_latency_ms": number(handover.get("latency_mean_ms")),
            "p95_handover_latency_ms": number(handover.get("latency_p95_ms")),
            "ping_pongs": int(number(handover.get("ping_pongs"))),
            "handovers_per_minute": (
                handovers * 60.0 / run["sim_time_s"] if run["sim_time_s"] > 0 else 0.0
            ),
            "handovers_per_ue": handovers / ue_count if ue_count else 0.0,
        })
        route_metrics = read_route_metrics(route_file,
                                           satellites,
                                           server_count or 1,
                                           ue_count or 1)
        routing = stats.get("routing", {})
        if not route_metrics["route_data_available"] and number(routing.get("samples")) > 0:
            route_metrics = {
                "route_data_available": True,
                "route_samples": int(number(routing.get("samples"))),
                "route_valid_ratio": number(routing.get("valid_ratio")),
                "route_changes": int(number(routing.get("changes"))),
                "unique_paths": int(number(routing.get("unique_paths"))),
                "mean_route_hops": number(routing.get("mean_hops")),
                "mean_route_distance_km": number(routing.get("mean_distance_km")),
                "mean_route_min_snr_db": number(routing.get("mean_min_snr_db")),
                "minimum_route_snr_db": number(routing.get("minimum_snr_db")),
                "mean_route_path_loss_db": number(routing.get("mean_path_loss_db")),
                "mean_route_min_signal_dbm": number(routing.get("mean_min_signal_dbm")),
            }
        run.update(route_metrics)
        runs.append(run)
    return runs


def add_hop_deltas(runs: list[dict]) -> None:
    groups = defaultdict(list)
    for run in runs:
        key = (run["satellites"], run["sim_time_s"], run["mode"], run["update_interval_s"])
        groups[key].append(run)

    delta_fields = (
        "pdr", "throughput_mbps", "delay_ms", "flow_mean_hop_count",
        "route_changes", "mean_route_distance_km", "mean_route_min_snr_db",
        "mean_route_path_loss_db", "handovers", "handover_success_ratio",
        "mean_handover_latency_ms", "p95_handover_latency_ms", "ping_pongs",
        "handovers_per_minute", "handovers_per_ue",
    )
    for group in groups.values():
        baseline = next((run for run in group if run["metric"] == "hop"), None)
        for run in group:
            for field in delta_fields:
                run[f"delta_vs_hop_{field}"] = run[field] - baseline[field] if baseline else 0.0


def write_csv(filename: Path, runs: list[dict]) -> None:
    fields = list(runs[0]) if runs else []
    with filename.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(runs)


def fmt(value, digits=3):
    return f"{number(value):.{digits}f}"


def write_markdown(filename: Path, runs: list[dict]) -> None:
    groups = defaultdict(list)
    for run in runs:
        groups[(run["satellites"], run["sim_time_s"], run["update_interval_s"])].append(run)

    with filename.open("w") as stream:
        stream.write("# LeoSim routing metric comparison\n\n")
        route_run_count = sum(bool(run["route_data_available"]) for run in runs)
        handover_run_count = sum(bool(run["handover_data_available"]) for run in runs)
        stream.write(
            f"Runs analysed: {len(runs)}. Route CSV data available for "
            f"{route_run_count} run(s); handover data available for "
            f"{handover_run_count} run(s). Unavailable fields are shown as —.\n\n"
        )
        for key in sorted(groups):
            satellites, duration, interval = key
            group = sorted(groups[key], key=lambda run: METRIC_ORDER.get(run["metric"], 99))
            stream.write(f"## {satellites} satellites, {duration:g}s, update {interval:g}s\n\n")
            stream.write("| Metric | PDR | Throughput Mbps | Delay ms | Flow hops | Route changes | "
                         "Unique paths | Route km | Min SNR dB | Path loss dB | Handovers | "
                         "HO/min | HO success | HO latency mean/P95 ms | Ping-pongs |\n")
            stream.write(
                "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"
                "---:|---:|---:|---:|\n"
            )
            for run in group:
                route_changes = str(run["route_changes"]) if run["route_data_available"] else "—"
                unique_paths = str(run["unique_paths"]) if run["route_data_available"] else "—"
                route_km = (fmt(run["mean_route_distance_km"])
                            if run["route_data_available"] else "—")
                route_snr = (fmt(run["minimum_route_snr_db"])
                             if run["route_data_available"] else "—")
                route_loss = (fmt(run["mean_route_path_loss_db"])
                              if run["route_data_available"] else "—")
                if run["handover_data_available"]:
                    handovers = str(run["handovers"])
                    handover_rate = fmt(run["handovers_per_minute"])
                    handover_success = fmt(run["handover_success_ratio"], 4)
                    handover_latency = (
                        f"{fmt(run['mean_handover_latency_ms'])}/"
                        f"{fmt(run['p95_handover_latency_ms'])}"
                    )
                    ping_pongs = str(run["ping_pongs"])
                else:
                    handovers = handover_rate = handover_success = "—"
                    handover_latency = ping_pongs = "—"
                stream.write(
                    f"| {run['metric']} | {fmt(run['pdr'], 4)} | {fmt(run['throughput_mbps'], 6)} | "
                    f"{fmt(run['delay_ms'])} | {fmt(run['flow_mean_hop_count'])} | "
                    f"{route_changes} | {unique_paths} | {route_km} | {route_snr} | "
                    f"{route_loss} | {handovers} | {handover_rate} | "
                    f"{handover_success} | {handover_latency} | {ping_pongs} |\n"
                )

            best_pdr = max(group, key=lambda run: (run["pdr"], run["throughput_mbps"]))
            best_delay = min((run for run in group if run["rx_packets"] > 0),
                             key=lambda run: run["delay_ms"], default=None)
            route_group = [run for run in group if run["route_data_available"]]
            best_snr = (max(route_group, key=lambda run: run["minimum_route_snr_db"])
                        if route_group else None)
            handover_group = [run for run in group if run["handover_data_available"]]
            best_handover = (
                min(handover_group,
                    key=lambda run: (-run["handover_success_ratio"],
                                     run["p95_handover_latency_ms"],
                                     run["ping_pongs"]))
                if handover_group else None
            )
            stream.write(f"\nBest delivery: **{best_pdr['metric']}**")
            if best_delay:
                stream.write(f"; lowest delay: **{best_delay['metric']}**")
            if best_snr:
                stream.write(f"; strongest worst-link SNR: **{best_snr['metric']}**")
            if best_handover:
                stream.write(f"; best handover outcome: **{best_handover['metric']}**")
            stream.write(".\n\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results_dir", type=Path, help="Sweep directory containing manifest.csv")
    parser.add_argument("--output-dir", type=Path, help="Directory for analysis files")
    args = parser.parse_args()

    results_dir = args.results_dir.resolve()
    output_dir = (args.output_dir or (results_dir / "analysis")).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    runs = load_runs(results_dir)
    add_hop_deltas(runs)
    runs.sort(key=lambda run: (run["satellites"], run["sim_time_s"],
                               run["update_interval_s"], METRIC_ORDER.get(run["metric"], 99)))
    write_csv(output_dir / "routing-metric-summary.csv", runs)
    write_markdown(output_dir / "routing-metric-comparison.md", runs)
    print(f"Analysed {len(runs)} runs")
    print(output_dir / "routing-metric-summary.csv")
    print(output_dir / "routing-metric-comparison.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
