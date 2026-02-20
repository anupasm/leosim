# LeoSim TCP/IP Network Simulation Example

This example demonstrates a comprehensive LEO (Low Earth Orbit) satellite network simulation using the LeoSim module for ns-3. It simulates a multi-node topology with satellites, user equipment (UEs), and servers, with ground communication links and inter-satellite links (ISLs).

## Overview

The simulation creates a realistic LEO constellation network where:
- **Satellites** relay traffic between ground stations
- **User Equipment (UEs)** act as source nodes generating traffic
- **Servers** act as destination nodes for traffic
- **Ground Links** connect satellites to UEs and servers
- **Inter-Satellite Links (ISLs)** (optional) create a mesh network between satellites for optimized routing

## Features and Functionalities Used from LeoSim

### 1. **Data Loading & Configuration** (`LeoSimLoaderHelper`, `LeoSimLoader`)
   - **Satellite Data Loading**: Loads satellite orbital data from either:
     - NS-3 trace files (`.tcl` format) for precise mobility traces
     - CSV format for custom satellite configurations
   - **Ground Device Loading**: Loads ground station configurations from CSV files
   - **Device Classification**: Automatically categorizes ground devices as:
     - `SERVER`: Destination nodes
     - `UE`: Source/User Equipment nodes
   - **Metadata Management**: Retrieves satellite names, device names, and geographic positions
   - **Configuration Access**: Validates and provides access to loaded constellation data

### 2. **Mobility Modeling** (`LeoSimMobilityHelper`, `LeoSimMobilityModel`)
   - **Satellite Mobility**: Installs mobility models for satellites using:
     - Loaded orbital parameters
     - Velocity calculations for dynamic link predictions
     - Realistic trajectory updates at each simulation timestep
   - **Ground Station Mobility**: Installs static/fixed mobility models for ground devices:
     - UEs positioned at specified geographic coordinates
     - Servers positioned at specified geographic coordinates
   - **Velocity Tracking**: Optionally calculates and tracks satellite velocities for Doppler calculations
   - **Lifecycle Management**: Starts and manages all mobility model timers

### 3. **Dynamic Channel Modeling** (`LeoSimChannelHelper`, `LeoSimChannelModel`, `LeoSimChannel`)
   - **Link Visibility Calculation**: Determines satellite-to-ground visibility based on:
     - Minimum elevation angle (default: 10°)
     - Line-of-sight geometry
     - Real-time position updates
   - **Ground Link Configuration**:
     - Carrier frequency (default: 12 GHz) for link propagation modeling
     - Maximum link distance constraints (default: 2.5 million meters)
     - Transmit power settings (default: 40 dBm)
     - Periodic update intervals (default: 1 second)
   - **Link Management**: 
     - Automatically tracks active links as topology changes
     - Triggers channel updates when link status changes
     - Provides link state to device installers
   - **Dynamic Updates**: Continuously recalculates link availability throughout simulation

### 4. **Inter-Satellite Link (ISL) Mesh** (`LeoSimChannelModel` for ISLs)
   - **ISL Mesh Creation**: Creates a full mesh of potential inter-satellite links with:
     - ISL carrier frequency (default: 26 GHz)
     - Maximum connection distance (default: 5000 km)
     - ISL-specific transmit power (default: 30 dBm)
     - ISL antenna gains (default: 35 dB)
   - **Satellite-to-Satellite Routing**: Enables high-speed, low-latency paths between satellites
   - **Parallel Link Support**: Maintains both ground and ISL channel models simultaneously
   - **Visualization Integration**: Provides ISL topology for visualization output
   - **Dynamic Topology**: Updates ISL connectivity as satellite positions change

### 5. **Network Device Installation** (`LeoSimDeviceInstaller`)
   - **Ground Link Devices**:
     - Installs point-to-point network devices on all active ground links
     - Configurable data rate (default: 100 Mbps)
     - Configurable link delay (default: 1 ms)
     - Configurable MTU size (default: 1500 bytes)
   - **ISL Devices**:
     - Installs high-performance network devices for inter-satellite links
     - Higher data rate (default: 10 Gbps) for ISL compared to ground links
     - Lower latency (default: 100 µs) for ISL links
   - **Automatic Pair Management**: Links devices based on channel model connectivity
   - **Error Model Support**: Integrated with channel model error characteristics

### 6. **IP Addressing & Network Configuration**
   - **Automated IP Assignment**:
     - Assigns unique subnets to each link to avoid address collisions
     - Ground links use `10.X.0.0/24` subnets
     - ISL links use `10.(100+X).0.0/24` subnets
   - **Internet Stack Installation**: Deploys full TCP/IP stack on all nodes
   - **Per-Link Addressing**: Ensures proper point-to-point connectivity semantics

### 7. **Dynamic Routing Calculation** (`LeoSimRoutingCalculatorHelper`, `LeoSimRoutingCalculator`)
   - **Unified Route Calculator**:
     - Creates a single routing calculation engine handling both ground and ISL links
     - Computes shortest paths across the hybrid topology
   - **Periodic Route Updates**:
     - Recalculates routes at configurable intervals (default: 10 seconds)
     - Automatically detects topology changes from channel model
     - Updates all nodes' routing tables with new paths
   - **Dual-Link Support**: Computes optimal paths considering:
     - Ground satellite-to-ground links
     - Inter-satellite ISL links
     - Mixed terrestrial and satellite segments
   - **Static Routing Table Installation**: Installs computed routes into node routing tables
   - **Lifetime Management**: Operates throughout the entire simulation period

### 8. **Visualization & Data Logging** (`LeoSimVisualizationHelper`)
   - **Position File Logging**:
     - Outputs satellite positions at regular intervals (configurable, default: 1 second)
     - Outputs ground device positions
     - Generates time-series position data for visualization
   - **Link File Logging**:
     - Records active satellite-to-ground links at each update interval
     - Captures temporal evolution of network topology
     - Used for topology visualization
   - **Packet File Logging** (optional):
     - Optionally logs all transmitted/received packets
     - Records source, destination, packet ID, size, and timestamp
     - Enables packet-level visibility and performance analysis
   - **ISL Topology Tracking**:
     - Records inter-satellite links for visualization
     - Shows ISL mesh evolution over time
   - **Data Format**: Outputs to CSV for easy analysis and visualization
   - **Integration**: Works with LeoSimLoader for geographic mapping

### 9. **Traffic Generation** (`LeoSimTrafficGeneratorHelper`)
   - **Ping Traffic**:
     - Generates ICMP ping packets from UEs to servers
     - Configurable ping interval (default: 10 seconds)
     - Standard ping payload size (default: 56 bytes)
     - Configurable start/stop times for traffic generation
   - **Application Management**:
     - Creates and manages ping client applications on source nodes
     - Manages ping server applications on destination nodes
   - **Flexible Deployment**:
     - Can install UE-to-Server traffic patterns
     - Supports multiple simultaneous traffic flows
   - **Performance Metrics**: Ping responses provide latency and loss metrics

### 10. **Packet-Level Monitoring & Tracing**
   - **Packet Hooks**: Installs callbacks on all nodes for packet transmission/reception
   - **Packet Logging**: Records packet flow events with:
     - Timestamp
     - Source and destination nodes
     - Packet size
     - Protocol information
   - **Performance Analysis**: Direct observation of network behavior at packet level
   - **Visualization Support**: Packet traces exported for timeline visualization

## Configuration Parameters

The simulation accepts the following command-line parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `--satellites` | string | `contrib/leosim/utils/satellite_mobility.tcl` | Path to satellite data file |
| `--groundDevices` | string | `contrib/leosim/utils/ground_devices.csv` | Path to ground devices CSV |
| `--simTime` | double | 60.0 | Simulation duration in seconds |
| `--logInterval` | double | 1.0 | Position/link logging interval (seconds) |
| `--minElevation` | double | 10.0 | Minimum elevation angle for visibility (degrees) |
| `--frequency` | double | 12e9 | Ground link carrier frequency (Hz) |
| `--positions` | string | `leosim_positions.csv` | Position output file |
| `--links` | string | `leosim_links.csv` | Link topology output file |
| `--packets` | string | `leosim_packets.csv` | Packet log output file |
| `--logPackets` | bool | true | Enable packet-level logging |
| `--verbose` | bool | true | Enable verbose console output |
| `--useTrace` | bool | true | Load satellites from trace file vs CSV |
| `--enablePeriodicRouting` | bool | true | Enable dynamic routing updates |
| `--routingUpdateInterval` | double | 10.0 | Routing recalculation interval (seconds) |
| `--pathPrintInterval` | double | 10.0 | UE→Server path printing interval (seconds) |
| `--numSatellites` | uint32 | 300 | Number of satellites to simulate |
| `--numServers` | uint32 | 1 | Number of server ground stations |
| `--numUes` | uint32 | 1 | Number of user equipment |
| `--enableIsl` | bool | true | Enable inter-satellite links |
| `--islFrequency` | double | 26e9 | ISL carrier frequency (Hz) |
| `--islMaxDistance` | double | 5e6 | ISL maximum connection distance (meters) |
| `--islTransmitPower` | double | 30.0 | ISL transmit power (dBm) |
| `--islAntennaGain` | double | 35.0 | ISL antenna gain (dB) |

## Running the Simulation

### Basic Run
```bash
./ns3 run leosim-example
```

### Custom Configuration Example
```bash
./ns3 run "leosim-example --simTime=100.0 --numSatellites=50 --numUes=5 --numServers=2 --enableIsl=true --logInterval=0.5"
```

### With Reduced Logging
```bash
./ns3 run "leosim-example --verbose=false --logPackets=false"
```

## Output Files

The simulation generates the following output files in the current directory:

1. **leosim_positions.csv**: Satellite and ground device positions at each logging interval
   - Columns: Time, NodeID, Type, Latitude, Longitude, Altitude, X, Y, Z

2. **leosim_links.csv**: Active satellite-to-ground links at each logging interval
   - Columns: Time, SatelliteID, GroundDeviceID, Elevation, Distance, AvailableDataRate

3. **leosim_packets.csv**: (Optional) Packet-level transmission events
   - Columns: Time, SourceNode, DestinationNode, PacketID, Size, Protocol

## Simulation Architecture

```
┌─────────────────────────────────────────────────────────┐
│              Network Simulation Setup                    │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  ┌──────────────────────────────────────────────────┐   │
│  │ Data Loading (LeoSimLoaderHelper)               │   │
│  │  - Load satellite orbital data                   │   │
│  │  - Load ground device configurations              │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │ Mobility Models (LeoSimMobilityHelper)           │   │
│  │  - Install satellite orbital mechanics          │   │
│  │  - Install ground station fixed positions        │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │ Channel Models (LeoSimChannelHelper)             │   │
│  │  - Ground Links: Satellite ↔ Ground            │   │
│  │  - ISL Mesh: Satellite ↔ Satellite              │   │
│  │  - Link Visibility Calculation                   │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │ Network Devices (LeoSimDeviceInstaller)          │   │
│  │  - Install P2P devices on all links              │   │
│  │  - Configure data rates and delays               │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │ IP Configuration                                 │   │
│  │  - Assign unique subnets per link               │   │
│  │  - Install Internet stack on all nodes          │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │ Routing (LeoSimRoutingCalculatorHelper)          │   │
│  │  - Compute optimal paths                         │   │
│  │  - Periodic route updates every 10 seconds      │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │ Applications & Traffic                           │   │
│  │  - Ping from UEs to Servers                      │   │
│  │  - Traffic generation                            │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │ Logging & Visualization Setup                    │   │
│  │  - Position logging                              │   │
│  │  - Link topology logging                         │   │
│  │  - Packet-level monitoring                       │   │
│  └────────────────┬─────────────────────────────────┘   │
│                   │                                       │
│  ┌────────────────▼─────────────────────────────────┐   │
│  │        Run Simulation                            │   │
│  │   (Generate detailed CSV outputs)                │   │
│  └──────────────────────────────────────────────────┘   │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

## Key Simulation Features

### Dynamic Network Topology
- **Continuous Updates**: Satellite links are recomputed every second based on actual position and elevation
- **Automatic Link Discovery**: Links appear/disappear based on visibility constraints
- **Optimized Paths**: Routes recalculated every 10 seconds to adapt to changing topology

### Realistic LEO Characteristics
- **High Dynamics**: Satellite constellations constantly evolving positions
- **Link Latency**: Models realistic propagation delays (1ms ground, 100µs ISL)
- **Capacity Constraints**: Distinct data rates for different link types
- **Elevation-Based Visibility**: Only connections above minimum elevation angle

### Hybrid Ground-ISL Network
- **Ground Links**: Direct satellite-to-ground and ground-to-ground via satellite
- **ISL Mesh**: Inter-satellite backbone for improved throughput and latency
- **Dual-Channel Tracking**: Maintains separate channel models for ground and ISL links
- **Unified Routing**: Single routing engine optimizes across both network types

### Observability
- **Multi-Level Monitoring**: Application, packet, and network topology visibility
- **Time-Series Data**: Outputs at configurable intervals for temporal analysis
- **CSV Export**: Standard format for external analysis and visualization
- **Performance Metrics**: Ping responses provide direct QoS measurements

## LeoSim Module Components Used

| Component | File | Purpose |
|-----------|------|---------|
| LeoSimLoaderHelper | `leosim-loader-helper.h/cc` | Load satellite and ground device data |
| LeoSimLoader | `leosim-loader.h/cc` | Core data management for constellation |
| LeoSimMobilityHelper | `leosim-mobility-helper.h/cc` | Manage node mobility installation |
| LeoSimMobilityModel | `leosim-mobility-model.h/cc` | Orbital mechanics and position calculation |
| LeoSimChannelHelper | `leosim-channel-helper.h/cc` | Configure channel models |
| LeoSimChannelModel | `leosim-channel-model.h/cc` | Manage link availability and topology |
| LeoSimChannel | `leosim-channel.h/cc` | Low-level channel representation |
| LeoSimDeviceInstaller | `leosim-device-installer.h/cc` | Install network devices on links |
| LeoSimRoutingCalculatorHelper | `leosim-routing-calculator-helper.h/cc` | Configure routing engine |
| LeoSimRoutingCalculator | `leosim-routing-calculator.h/cc` | Compute shortest paths |
| LeoSimIslRoutingModel | `leosim-isl-routing-model.h/cc` | ISL-specific routing logic |
| LeoSimVisualizationHelper | `leosim-visualization-helper.h/cc` | Logging and data export |
| LeoSimTrafficGeneratorHelper | `leosim-traffic-generator-helper.h/cc` | Traffic generation |

## Example Use Cases

1. **LEO Constellation Performance**: Evaluate latency, throughput, and path diversity in various constellation configurations
2. **Routing Algorithm Testing**: Benchmark different routing strategies on dynamic LEO topologies
3. **Ground Station Network Design**: Optimize placement and coverage of ground stations
4. **ISL Effectiveness**: Compare performance with and without inter-satellite links
5. **Service Coverage**: Analyze continuous service availability under different minimum elevation angles
6. **Traffic Load Analysis**: Evaluate network capacity under various data load patterns

## Building and Testing

```bash
# Configure and build ns-3 with LeoSim examples
./ns3 configure --enable-examples --enable-tests
./ns3 build

# Run the leosim-example
./ns3 run leosim-example

# Run LeoSim tests
./test.py -s leosim
```

## Performance Considerations

- **Node Count**: Simulation performance scales quadratically with satellite count (O(n²) for link calculations)
- **Update Frequency**: More frequent position/route updates increase CPU demand
- **Link Density**: Denser constellations with more simultaneous links use more memory
- **Logging Overhead**: Packet-level logging significantly increases output size and CPU usage

## License

This simulation code follows the ns-3 license (GNU General Public License version 2).

## Author

Copyright (c) 2024 Anupa De Silva

## References

- ns-3 Project: https://www.nsnam.org/
- LEO Satellite Networks Research
- ns-3 LeoSim Module Documentation

