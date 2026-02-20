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
    python3 visualize_3d.py <position_file.csv> [--static]

Requirements:
    pip install plotly numpy
"""

import argparse
import sys
import numpy as np
import plotly.graph_objects as go

# Earth radius in meters
EARTH_RADIUS = 6371000.0

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
    """Load link/channel data from CSV file."""
    links = {}  # time -> list of (sat_id, ground_id, ground_type)
    
    try:
        with open(filename, 'r') as f:
            # Skip header
            next(f)
            
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 4:
                    continue
                
                time = float(parts[0])
                sat_id = int(parts[1])
                ground_id = int(parts[2])
                ground_type = parts[3]
                
                if time not in links:
                    links[time] = []
                
                links[time].append((sat_id, ground_id, ground_type))
        
        return links
    
    except FileNotFoundError:
        print(f"Link file '{filename}' not found, skipping link visualization.")
        return {}
    except Exception as e:
        print(f"Error loading link file: {e}")
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
    except Exception as e:
        print(f"Error loading packet file: {e}")
        return {}

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
        hoverinfo='skip'
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

def visualize_animated(data, links, packets, max_frames=None, output_file=None, show_flows=True):
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
    
    node_index = build_node_index(data)
    
    # Build packet flows if requested
    flows = {}
    if packets and show_flows:
        print("Building packet flow paths...")
        flows = build_packet_flows(packets, node_index)
        print(f"Found {sum(len(f) for f in flows.values())} packet flows")

    # Prepare frames for animation
    frames = []
    
    for frame_idx, current_time in enumerate(times):
        frame_data = []
        
        # Add Earth to every frame
        frame_data.append(create_earth_sphere())
        
        # Add satellites at current time - ALWAYS add in same order
        satellite_ids = sorted(data['SATELLITE'].keys())
        for sat_id in satellite_ids:
            sat_data = data['SATELLITE'][sat_id]
            idx = np.searchsorted(sat_data['times'], current_time)
            if idx < len(sat_data['positions']):
                pos = sat_data['positions'][idx]
                frame_data.append(go.Scatter3d(
                    x=[pos[0]], y=[pos[1]], z=[pos[2]],
                    mode='markers+text',
                    marker=dict(size=4, color='red', symbol='circle'),
                    text=[f'SAT{sat_id}'],
                    textposition='top center',
                    name=f'Satellite {sat_id}',
                    hovertext=f'Satellite {sat_id}: {sat_data["name"]}<br>Time: {current_time:.1f}s',
                    showlegend=(frame_idx == 0),
                    visible=True
                ))
        
        # Add servers (stationary) - ALWAYS add in same order
        server_ids = sorted(data['SERVER'].keys())
        for srv_id in server_ids:
            srv_data = data['SERVER'][srv_id]
            if len(srv_data['positions']) > 0:
                pos = srv_data['positions'][0]
                frame_data.append(go.Scatter3d(
                    x=[pos[0]], y=[pos[1]], z=[pos[2]],
                    mode='markers+text',
                    marker=dict(size=6, color='green', symbol='diamond'),
                    text=[srv_data['name']],
                    textposition='top center',
                    name=f'Server {srv_id}',
                    hovertext=f'Server: {srv_data["name"]}',
                    showlegend=(frame_idx == 0),
                    visible=True
                ))
        
        # Add UEs (stationary) - ALWAYS add in same order
        ue_ids = sorted(data['UE'].keys())
        for ue_id in ue_ids:
            ue_data = data['UE'][ue_id]
            if len(ue_data['positions']) > 0:
                pos = ue_data['positions'][0]
                frame_data.append(go.Scatter3d(
                    x=[pos[0]], y=[pos[1]], z=[pos[2]],
                    mode='markers+text',
                    marker=dict(size=3, color='blue', symbol='square'),
                    text=[ue_data['name']],
                    textposition='top center',
                    name=f'UE {ue_id}',
                    hovertext=f'UE: {ue_data["name"]}',
                    showlegend=(frame_idx == 0),
                    visible=True
                ))
        
        # Add channel links if available for this time
        if links and current_time in links:
            for sat_id, ground_id, ground_type in links[current_time]:
                # Get satellite position
                if sat_id in data['SATELLITE']:
                    sat_data = data['SATELLITE'][sat_id]
                    sat_idx = np.searchsorted(sat_data['times'], current_time)
                    if sat_idx < len(sat_data['positions']):
                        sat_pos = sat_data['positions'][sat_idx]

                        # Get endpoint position
                        other_pos = None
                        link_color = 'yellow'
                        link_name = f'Link SAT{sat_id}-{ground_type}{ground_id}'

                        if ground_type == 'SERVER' and ground_id in data['SERVER']:
                            other_pos = data['SERVER'][ground_id]['positions'][0]
                        elif ground_type == 'UE' and ground_id in data['UE']:
                            other_pos = data['UE'][ground_id]['positions'][0]
                        elif ground_type in ('ISL', 'SATELLITE') and ground_id in data['SATELLITE']:
                            other_data = data['SATELLITE'][ground_id]
                            other_idx = np.searchsorted(other_data['times'], current_time)
                            if other_idx < len(other_data['positions']):
                                other_pos = other_data['positions'][other_idx]
                                link_color = 'yellow'
                                link_name = f'ISL SAT{sat_id}-SAT{ground_id}'

                        if other_pos is not None:
                            # Add line for the link
                            frame_data.append(go.Scatter3d(
                                x=[sat_pos[0], other_pos[0]],
                                y=[sat_pos[1], other_pos[1]],
                                z=[sat_pos[2], other_pos[2]],
                                mode='lines',
                                line=dict(color=link_color, width=2),
                                opacity=0.6,
                                showlegend=(frame_idx == 0 and link_color == 'yellow'),
                                name=link_name,
                                hoverinfo='skip'
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
                showlegend=(frame_idx == 0),
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
                showlegend=(frame_idx == 0),
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
                showlegend=(frame_idx == 0),
                visible=True
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
                showlegend=(frame_idx == 0),
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
                showlegend=(frame_idx == 0),
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
                showlegend=(frame_idx == 0),
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
                dict(label="▶ Play",
                     method="animate",
                     args=[None, dict(
                         frame=dict(duration=500, redraw=True),
                         fromcurrent=True,
                         mode="immediate",
                         transition=dict(duration=0)
                     )]),
                dict(label="⏸ Pause",
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
                dict(label="🐢 Slow (2s)",
                     method="animate",
                     args=[None, dict(
                         frame=dict(duration=2000, redraw=True),
                         fromcurrent=True,
                         mode="immediate",
                         transition=dict(duration=0)
                     )]),
                dict(label="🚶 Normal (0.5s)",
                     method="animate",
                     args=[None, dict(
                         frame=dict(duration=500, redraw=True),
                         fromcurrent=True,
                         mode="immediate",
                         transition=dict(duration=0)
                     )]),
                dict(label="🚀 Fast (0.1s)",
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
    parser.add_argument('--links', type=str, default="leosim_links.csv",
                       help='CSV file with channel/link data')
    parser.add_argument('--packets', type=str, default="leosim_packets.csv",
                       help='CSV file with packet data')
    parser.add_argument('--output', type=str, default="visualization.html",
                       help='Output HTML file (if not specified, opens in browser)')
    parser.add_argument('--show-flows', action='store_true', default=True,
                       help='Show packet flow arrows (default: enabled)')
    parser.add_argument('--no-flows', action='store_false', dest='show_flows',
                       help='Disable packet flow visualization')
    
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
    

    visualize_animated(data, links, packets, max_frames=args.max_frames, 
                      output_file=args.output, show_flows=args.show_flows)

if __name__ == '__main__':
    main()
