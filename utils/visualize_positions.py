import argparse
import csv
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np
try:
    import plotly.graph_objects as go
except ModuleNotFoundError as e:
    raise SystemExit(
        "Missing dependency: plotly. Install it with: pip install plotly"
    ) from e

EARTH_RADIUS_M = 6.371e6  # meters


def _round_time_s(t: float) -> float:
    # LeoSim writes times with 3 decimals in positions/beams.
    return round(t, 3)


def _round_time_ms_to_s(t_ms: float) -> float:
    # LeoSim handover file uses 0.1ms precision.
    return _round_time_s(t_ms / 1000.0)


@dataclass(frozen=True)
class NodePose:
    node_type: str
    node_id: int
    node_name: str
    x: float
    y: float
    z: float


@dataclass(frozen=True)
class HandoverEvent:
    time_s: float
    ue_id: int
    src_sat: int
    tgt_sat: int
    mode: str
    ho_type: str
    trigger: str
    success: bool

def parse_position_line(line):
    """Parse line like: +0ns:0:1.57284e+06:3.85377e+06:5.52478e+06"""
    line = line.strip()
    if not line:
        return None
    parts = line.lstrip('+').split(':')
    if len(parts) < 5:
        return None
    try:
        time_str = parts[0]
        sat_id = parts[1]
        x = float(parts[2])
        y = float(parts[3])
        z = float(parts[4])
        return (time_str, sat_id, x, y, z)
    except (ValueError, IndexError):
        return None


def _sniff_is_csv_positions_file(filename: str) -> bool:
    try:
        with open(filename, "r", newline="") as f:
            header = f.readline().strip().lower()
    except OSError:
        return False
    # New LeoSim CSV header: time,type,id,name,x,y,z
    return header.startswith("time,") and ",type," in header and ",x," in header


def _read_positions_legacy_trace(filename: str) -> Dict[str, List[Tuple[str, float, float, float]]]:
    """Legacy format: +time:sat_id:x:y:z (used by old TLE utilities)."""
    data_by_sat: Dict[str, List[Tuple[str, float, float, float]]] = {}
    with open(filename, "r") as f:
        for line in f:
            parsed = parse_position_line(line)
            if parsed:
                time_str, sat_id, x, y, z = parsed
                data_by_sat.setdefault(sat_id, []).append((time_str, x, y, z))
    return data_by_sat


def _read_positions_csv(filename: str) -> Tuple[List[float], Dict[float, Dict[int, NodePose]]]:
    """Read LeoSim CSV: time,type,id,name,x,y,z."""
    poses_by_time: Dict[float, Dict[int, NodePose]] = {}
    times_set = set()

    with open(filename, "r", newline="") as f:
        reader = csv.DictReader(f)
        required = {"time", "type", "id", "name", "x", "y", "z"}
        if reader.fieldnames is None or not required.issubset({h.strip().lower() for h in reader.fieldnames}):
            raise ValueError(
                f"{filename} does not look like LeoSim positions CSV; expected columns {sorted(required)}"
            )

        # Normalize field access (DictReader preserves original header casing).
        field_map = {h.strip().lower(): h for h in reader.fieldnames}

        for row in reader:
            try:
                t = _round_time_s(float(row[field_map["time"]]))
                node_type = str(row[field_map["type"]]).strip()
                node_id = int(row[field_map["id"]])
                node_name = str(row[field_map["name"]]).strip()
                x = float(row[field_map["x"]])
                y = float(row[field_map["y"]])
                z = float(row[field_map["z"]])
            except (KeyError, TypeError, ValueError):
                continue

            times_set.add(t)
            poses_by_time.setdefault(t, {})[node_id] = NodePose(
                node_type=node_type,
                node_id=node_id,
                node_name=node_name,
                x=x,
                y=y,
                z=z,
            )

    times = sorted(times_set)
    return times, poses_by_time


def _read_beams_csv(filename: str) -> Dict[float, Dict[int, int]]:
    """Read LeoSim beams CSV: time,ue_id,sat_id,...

    Returns mapping: time_s -> {ue_id: sat_id}.
    """
    beams_by_time: Dict[float, Dict[int, int]] = {}
    with open(filename, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return beams_by_time
        field_map = {h.strip().lower(): h for h in reader.fieldnames}
        if "time" not in field_map or "ue_id" not in field_map or "sat_id" not in field_map:
            return beams_by_time

        for row in reader:
            try:
                t = _round_time_s(float(row[field_map["time"]]))
                ue_id = int(row[field_map["ue_id"]])
                sat_id = int(row[field_map["sat_id"]])
            except (KeyError, TypeError, ValueError):
                continue

            # Keep last-seen assignment per (time, ue).
            beams_by_time.setdefault(t, {})[ue_id] = sat_id

    return beams_by_time


def _read_handovers_csv(filename: str) -> Dict[float, List[HandoverEvent]]:
    """Read LeoSim handover events CSV: time_ms,ue_id,src_sat,tgt_sat,mode,type,trigger,..."""
    events_by_time: Dict[float, List[HandoverEvent]] = {}
    with open(filename, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return events_by_time
        field_map = {h.strip().lower(): h for h in reader.fieldnames}

        required = {"time_ms", "ue_id", "src_sat", "tgt_sat", "mode", "type", "trigger", "success"}
        if not required.issubset(field_map.keys()):
            return events_by_time

        for row in reader:
            try:
                time_s = _round_time_ms_to_s(float(row[field_map["time_ms"]]))
                ue_id = int(row[field_map["ue_id"]])
                src_sat = int(row[field_map["src_sat"]])
                tgt_sat = int(row[field_map["tgt_sat"]])
                mode = str(row[field_map["mode"]]).strip()
                ho_type = str(row[field_map["type"]]).strip()
                trigger = str(row[field_map["trigger"]]).strip()
                success = str(row[field_map["success"]]).strip() in ("1", "true", "yes")
            except (KeyError, TypeError, ValueError):
                continue

            evt = HandoverEvent(
                time_s=time_s,
                ue_id=ue_id,
                src_sat=src_sat,
                tgt_sat=tgt_sat,
                mode=mode,
                ho_type=ho_type,
                trigger=trigger,
                success=success,
            )
            events_by_time.setdefault(time_s, []).append(evt)

    return events_by_time

def plotly_3d_globe_navigation(
    filename: str,
    show_earth: bool = True,
    output_file: Optional[str] = None,
    beams_file: Optional[str] = None,
    handovers_file: Optional[str] = None,
):
    """Visualize node positions.

    Supports:
    - Legacy trace format: +time:sat_id:x:y:z (satellites only)
    - LeoSim CSV: time,type,id,name,x,y,z (satellites + UEs + servers)

    If given, overlays:
    - `beams_file` (leosim_beams.csv): draws UE→serving-satellite lines per timestep
    - `handovers_file` (leosim_handovers.csv): marks UE handover events at event times
    """
    if not _sniff_is_csv_positions_file(filename):
        # Keep old behavior for legacy trace input.
        data_by_sat = _read_positions_legacy_trace(filename)
        if not data_by_sat:
            print(f"No valid legacy position data found in {filename}")
            return

        all_x = [x for sat_data in data_by_sat.values() for (_, x, _, _) in sat_data]
        all_y = [y for sat_data in data_by_sat.values() for (_, _, y, _) in sat_data]
        all_z = [z for sat_data in data_by_sat.values() for (_, _, _, z) in sat_data]

        fig = go.Figure()

        # Add Earth sphere
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)

        max_coord = max(np.max(np.abs(all_x)), np.max(np.abs(all_y)), np.max(np.abs(all_z)))
        sphere_radius = EARTH_RADIUS_M if show_earth else max_coord * 0.5

        x_sphere = sphere_radius * np.outer(np.cos(u), np.sin(v))
        y_sphere = sphere_radius * np.outer(np.sin(u), np.sin(v))
        z_sphere = sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v))

        if show_earth:
            fig.add_trace(
                go.Surface(
                    x=x_sphere,
                    y=y_sphere,
                    z=z_sphere,
                    colorscale=[[0, "rgba(100,150,255,0.3)"], [1, "rgba(100,150,255,0.3)"]],
                    opacity=0.3,
                    showscale=False,
                    name="Earth",
                    hoverinfo="skip",
                )
            )

        colors = ["red", "blue", "green", "orange", "purple", "cyan", "magenta", "yellow"]
        max_points = max(len(sat_data) for sat_data in data_by_sat.values()) if data_by_sat else 1

        for idx, (sat_id, sat_data) in enumerate(sorted(data_by_sat.items())):
            times = [d[0] for d in sat_data]
            xs = [d[1] for d in sat_data]
            ys = [d[2] for d in sat_data]
            zs = [d[3] for d in sat_data]
            color = colors[idx % len(colors)]

            fig.add_trace(
                go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode="lines+markers",
                    line=dict(color=color, width=3),
                    marker=dict(size=1, color=color),
                    name=f"Satellite {sat_id}",
                    hovertemplate=(
                        "Sat: "
                        + sat_id
                        + "<br>Time: %{text}<br>X: %{x:.2e}<br>Y: %{y:.2e}<br>Z: %{z:.2e}<extra></extra>"
                    ),
                    text=times,
                )
            )

            fig.add_trace(
                go.Scatter3d(
                    x=[xs[0]],
                    y=[ys[0]],
                    z=[zs[0]],
                    mode="markers",
                    marker=dict(size=1, color=color, symbol="diamond"),
                    name=f"Start Sat {sat_id}",
                    hovertemplate=(
                        f"Start Sat {sat_id}<br>X: %{{x:.2e}}<br>Y: %{{y:.2e}}<br>Z: %{{z:.2e}}<extra></extra>"
                    ),
                    showlegend=False,
                )
            )

            fig.add_trace(
                go.Scatter3d(
                    x=[xs[-1]],
                    y=[ys[-1]],
                    z=[zs[-1]],
                    mode="markers",
                    marker=dict(size=1, color=color, symbol="circle"),
                    name=f"End Sat {sat_id}",
                    hovertemplate=(
                        f"End Sat {sat_id}<br>X: %{{x:.2e}}<br>Y: %{{y:.2e}}<br>Z: %{{z:.2e}}<extra></extra>"
                    ),
                    showlegend=False,
                )
            )

        frames = []
        for frame_idx in range(max_points):
            frame_data = []
            if show_earth:
                frame_data.append(
                    go.Surface(
                        x=x_sphere,
                        y=y_sphere,
                        z=z_sphere,
                        colorscale=[[0, "rgba(100,150,255,0.3)"], [1, "rgba(100,150,255,0.3)"]],
                        opacity=0.3,
                        showscale=False,
                        name="Earth",
                        hoverinfo="skip",
                    )
                )

            for idx, (sat_id, sat_data) in enumerate(sorted(data_by_sat.items())):
                times = [d[0] for d in sat_data]
                xs = [d[1] for d in sat_data]
                ys = [d[2] for d in sat_data]
                zs = [d[3] for d in sat_data]
                color = colors[idx % len(colors)]

                current_frame_idx = min(frame_idx, len(xs) - 1)
                xs_frame = xs[: current_frame_idx + 1]
                ys_frame = ys[: current_frame_idx + 1]
                zs_frame = zs[: current_frame_idx + 1]
                times_frame = times[: current_frame_idx + 1]

                frame_data.append(
                    go.Scatter3d(
                        x=xs_frame,
                        y=ys_frame,
                        z=zs_frame,
                        mode="lines+markers",
                        line=dict(color=color, width=1),
                        marker=dict(size=1, color=color),
                        name=f"Satellite {sat_id}",
                        hovertemplate=(
                            "Sat: "
                            + sat_id
                            + "<br>Time: %{text}<br>X: %{x:.2e}<br>Y: %{y:.2e}<br>Z: %{z:.2e}<extra></extra>"
                        ),
                        text=times_frame,
                    )
                )

                if xs_frame:
                    frame_data.append(
                        go.Scatter3d(
                            x=[xs_frame[-1]],
                            y=[ys_frame[-1]],
                            z=[zs_frame[-1]],
                            mode="markers",
                            marker=dict(size=1, color=color, symbol="circle"),
                            name=f"Current Sat {sat_id}",
                            hovertemplate=(
                                f"Current Sat {sat_id}<br>Time: {times_frame[-1]}<br>X: %{{x:.2e}}<br>Y: %{{y:.2e}}<br>Z: %{{z:.2e}}<extra></extra>"
                            ),
                            showlegend=False,
                        )
                    )

            frames.append(go.Frame(data=frame_data, name=str(frame_idx)))

        fig.frames = frames

        fig.update_layout(
            title=f"Satellite Orbits from TLE Data ({len(data_by_sat)} satellites) - TIME ANIMATION",
            scene=dict(
                xaxis_title="X (m)",
                yaxis_title="Y (m)",
                zaxis_title="Z (m)",
                aspectmode="data",
            ),
            width=1000,
            height=800,
            showlegend=True,
            updatemenus=[
                dict(
                    type="buttons",
                    showactive=False,
                    y=0.0,
                    x=0.0,
                    xanchor="left",
                    yanchor="bottom",
                    buttons=[
                        dict(
                            label="▶ Play",
                            method="animate",
                            args=[
                                None,
                                dict(
                                    frame=dict(duration=100, redraw=True),
                                    fromcurrent=True,
                                    transition=dict(duration=50),
                                ),
                            ],
                        ),
                        dict(
                            label="⏸ Pause",
                            method="animate",
                            args=[
                                [None],
                                dict(
                                    frame=dict(duration=0, redraw=False),
                                    mode="immediate",
                                    transition=dict(duration=0),
                                ),
                            ],
                        ),
                    ],
                )
            ],
            sliders=[
                dict(
                    active=0,
                    yanchor="top",
                    y=-0.1,
                    xanchor="left",
                    x=0.1,
                    currentvalue=dict(
                        font=dict(size=14),
                        prefix="Time frame: ",
                        visible=True,
                        xanchor="center",
                    ),
                    pad=dict(b=10, t=50),
                    len=0.8,
                    transition=dict(duration=50),
                    steps=[
                        dict(
                            args=[
                                [f.name],
                                dict(
                                    frame=dict(duration=50, redraw=True),
                                    mode="immediate",
                                    transition=dict(duration=50),
                                ),
                            ],
                            method="animate",
                            label=str(i),
                        )
                        for i, f in enumerate(frames)
                    ],
                )
            ],
        )

        if output_file:
            fig.write_html(output_file)
            print(f"Animation saved to {output_file}")
        else:
            fig.show()
        return

    # LeoSim CSV visualization (satellites + ground nodes, plus optional beams/handovers).
    times, poses_by_time = _read_positions_csv(filename)
    if not times:
        print(f"No valid CSV position data found in {filename}")
        return

    beams_by_time: Dict[float, Dict[int, int]] = {}
    if beams_file:
        try:
            beams_by_time = _read_beams_csv(beams_file)
        except OSError:
            beams_by_time = {}

    handovers_by_time: Dict[float, List[HandoverEvent]] = {}
    if handovers_file:
        try:
            handovers_by_time = _read_handovers_csv(handovers_file)
        except OSError:
            handovers_by_time = {}

    # Collect all coordinates for bounds.
    all_x: List[float] = []
    all_y: List[float] = []
    all_z: List[float] = []
    for t in times:
        for pose in poses_by_time.get(t, {}).values():
            all_x.append(pose.x)
            all_y.append(pose.y)
            all_z.append(pose.z)

    fig = go.Figure()
    
    u = np.linspace(0, 2 * np.pi, 50)
    v = np.linspace(0, np.pi, 50)

    max_coord = max(np.max(np.abs(all_x)), np.max(np.abs(all_y)), np.max(np.abs(all_z))) if all_x else 1.0
    sphere_radius = EARTH_RADIUS_M if show_earth else max_coord * 0.5
    x_sphere = sphere_radius * np.outer(np.cos(u), np.sin(v))
    y_sphere = sphere_radius * np.outer(np.sin(u), np.sin(v))
    z_sphere = sphere_radius * np.outer(np.ones(np.size(u)), np.cos(v))

    colors = ["red", "blue", "green", "orange", "purple", "cyan", "magenta", "yellow"]
    sat_ids = sorted(
        {
            pose.node_id
            for t in times
            for pose in poses_by_time.get(t, {}).values()
            if pose.node_type.upper() == "SATELLITE"
        }
    )

    # Frames.
    frames = []
    for frame_idx, t in enumerate(times):
        frame_data = []

        if show_earth:
            frame_data.append(
                go.Surface(
                    x=x_sphere,
                    y=y_sphere,
                    z=z_sphere,
                    colorscale=[[0, "rgba(100,150,255,0.3)"], [1, "rgba(100,150,255,0.3)"]],
                    opacity=0.3,
                    showscale=False,
                    name="Earth",
                    hoverinfo="skip",
                )
            )

        # Satellites: progressive trail.
        for idx, sat_id in enumerate(sat_ids):
            xs: List[float] = []
            ys: List[float] = []
            zs: List[float] = []
            ts_text: List[str] = []

            for t_hist in times[: frame_idx + 1]:
                pose = poses_by_time.get(t_hist, {}).get(sat_id)
                if not pose:
                    continue
                xs.append(pose.x)
                ys.append(pose.y)
                zs.append(pose.z)
                ts_text.append(f"{t_hist:.3f}s")

            color = colors[idx % len(colors)]
            frame_data.append(
                go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode="lines",
                    line=dict(color=color, width=2),
                    name=f"SAT {sat_id}",
                    hovertemplate=(
                        f"SAT {sat_id}<br>Time: %{{text}}<br>X: %{{x:.2e}}<br>Y: %{{y:.2e}}<br>Z: %{{z:.2e}}<extra></extra>"
                    ),
                    text=ts_text,
                )
            )

            if xs:
                frame_data.append(
                    go.Scatter3d(
                        x=[xs[-1]],
                        y=[ys[-1]],
                        z=[zs[-1]],
                        mode="markers",
                        marker=dict(size=3, color=color, symbol="circle"),
                        name=f"SAT {sat_id} (current)",
                        hovertemplate=(
                            f"SAT {sat_id}<br>Time: {t:.3f}s<br>X: %{{x:.2e}}<br>Y: %{{y:.2e}}<br>Z: %{{z:.2e}}<extra></extra>"
                        ),
                        showlegend=False,
                    )
                )

        # Current non-sat nodes at this time.
        ues_x: List[float] = []
        ues_y: List[float] = []
        ues_z: List[float] = []
        ues_text: List[str] = []
        ues_ids: List[int] = []

        servers_x: List[float] = []
        servers_y: List[float] = []
        servers_z: List[float] = []
        servers_text: List[str] = []

        poses_now = poses_by_time.get(t, {})
        for node_id, pose in poses_now.items():
            t_upper = pose.node_type.upper()
            if t_upper == "UE":
                ues_x.append(pose.x)
                ues_y.append(pose.y)
                ues_z.append(pose.z)
                ues_ids.append(node_id)
                ues_text.append(f"UE {node_id} ({pose.node_name})<br>t={t:.3f}s")
            elif t_upper == "SERVER":
                servers_x.append(pose.x)
                servers_y.append(pose.y)
                servers_z.append(pose.z)
                servers_text.append(f"SERVER {node_id} ({pose.node_name})<br>t={t:.3f}s")

        if servers_x:
            frame_data.append(
                go.Scatter3d(
                    x=servers_x,
                    y=servers_y,
                    z=servers_z,
                    mode="markers",
                    marker=dict(size=4, color="green", symbol="diamond"),
                    name="Servers",
                    text=servers_text,
                    hovertemplate="%{text}<extra></extra>",
                )
            )

        if ues_x:
            frame_data.append(
                go.Scatter3d(
                    x=ues_x,
                    y=ues_y,
                    z=ues_z,
                    mode="markers",
                    marker=dict(size=4, color="black", symbol="circle"),
                    name="UEs",
                    text=ues_text,
                    hovertemplate="%{text}<extra></extra>",
                )
            )

        # Beam links: UE -> serving satellite.
        if beams_by_time:
            beam_lines_x: List[Optional[float]] = []
            beam_lines_y: List[Optional[float]] = []
            beam_lines_z: List[Optional[float]] = []

            assigns = beams_by_time.get(t, {})
            for ue_id, sat_id in assigns.items():
                ue_pose = poses_now.get(ue_id)
                sat_pose = poses_now.get(sat_id)
                if not ue_pose or not sat_pose:
                    continue
                beam_lines_x.extend([ue_pose.x, sat_pose.x, None])
                beam_lines_y.extend([ue_pose.y, sat_pose.y, None])
                beam_lines_z.extend([ue_pose.z, sat_pose.z, None])

            if beam_lines_x:
                frame_data.append(
                    go.Scatter3d(
                        x=beam_lines_x,
                        y=beam_lines_y,
                        z=beam_lines_z,
                        mode="lines",
                        line=dict(color="gray", width=2, dash="dot"),
                        name="Beams",
                        hoverinfo="skip",
                    )
                )

        # Handover events at this time (mark UE position).
        if handovers_by_time:
            evts = handovers_by_time.get(t, [])
            if evts:
                ho_x: List[float] = []
                ho_y: List[float] = []
                ho_z: List[float] = []
                ho_text: List[str] = []

                for evt in evts:
                    ue_pose = poses_now.get(evt.ue_id)
                    if not ue_pose:
                        continue
                    ho_x.append(ue_pose.x)
                    ho_y.append(ue_pose.y)
                    ho_z.append(ue_pose.z)
                    ho_text.append(
                        "<br>".join(
                            [
                                f"Handover @ {evt.time_s:.3f}s",
                                f"UE {evt.ue_id}",
                                f"{evt.src_sat} → {evt.tgt_sat}",
                                f"{evt.mode} / {evt.ho_type} / {evt.trigger}",
                                f"success={1 if evt.success else 0}",
                            ]
                        )
                    )

                if ho_x:
                    frame_data.append(
                        go.Scatter3d(
                            x=ho_x,
                            y=ho_y,
                            z=ho_z,
                            mode="markers",
                            marker=dict(size=6, color="red", symbol="x"),
                            name="Handovers",
                            text=ho_text,
                            hovertemplate="%{text}<extra></extra>",
                        )
                    )

        frames.append(go.Frame(data=frame_data, name=f"{t:.3f}"))

    fig.frames = frames
    if frames:
        fig.add_traces(frames[0].data)

    # Slider labels show actual time values.
    fig.update_layout(
        title=f"LeoSim Node Positions ({len(sat_ids)} sats, {len(times)} frames)",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
        ),
        width=1000,
        height=800,
        showlegend=True,
        updatemenus=[
            dict(
                type="buttons",
                showactive=False,
                y=0.0,
                x=0.0,
                xanchor="left",
                yanchor="bottom",
                buttons=[
                    dict(
                        label="▶ Play",
                        method="animate",
                        args=[
                            None,
                            dict(
                                frame=dict(duration=100, redraw=True),
                                fromcurrent=True,
                                transition=dict(duration=0),
                            ),
                        ],
                    ),
                    dict(
                        label="⏸ Pause",
                        method="animate",
                        args=[
                            [None],
                            dict(
                                frame=dict(duration=0, redraw=False),
                                mode="immediate",
                                transition=dict(duration=0),
                            ),
                        ],
                    ),
                ],
            )
        ],
        sliders=[
            dict(
                active=0,
                yanchor="top",
                y=-0.1,
                xanchor="left",
                x=0.1,
                currentvalue=dict(
                    font=dict(size=14),
                    prefix="Time (s): ",
                    visible=True,
                    xanchor="center",
                ),
                pad=dict(b=10, t=50),
                len=0.8,
                steps=[
                    dict(
                        args=[
                            [f.name],
                            dict(
                                frame=dict(duration=0, redraw=True),
                                mode="immediate",
                                transition=dict(duration=0),
                            ),
                        ],
                        method="animate",
                        label=f.name,
                    )
                    for f in frames
                ],
            )
        ],
    )

    if output_file:
        fig.write_html(output_file)
        print(f"Animation saved to {output_file}")
    else:
        fig.show()

def main():
    parser = argparse.ArgumentParser(
        description=(
            "Visualize LeoSim positions in 3D (CSV or legacy trace). "
            "For CSV runs, you can optionally overlay beam links and handover events."
        )
    )
    parser.add_argument(
        "filename",
        help=(
            "Positions file. Supports LeoSim CSV (time,type,id,name,x,y,z) or legacy trace (+time:sat_id:x:y:z)."
        ),
    )
    parser.add_argument("--beams", help="Optional leosim_beams.csv for UE→sat beam overlays")
    parser.add_argument("--handovers", help="Optional leosim_handovers.csv for handover event markers")
    parser.add_argument("--no-earth", action="store_true", help="Hide Earth globe")
    parser.add_argument("--output", "-o", help="Save to HTML file instead of showing interactive plot")
    args = parser.parse_args()

    plotly_3d_globe_navigation(
        args.filename,
        show_earth=not args.no_earth,
        output_file=args.output,
        beams_file=args.beams,
        handovers_file=args.handovers,
    )

if __name__ == '__main__':
    main()