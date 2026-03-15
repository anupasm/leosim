# LeoSim: LEO Satellite Network Simulator

Comprehensive ns-3 module for simulating Low Earth Orbit (LEO) satellite communication networks with ground stations, inter-satellite links, dynamic routing, and traffic generation.

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Core Components](#core-components)
4. [Implementation Process](#implementation-process)
5. [Example Walkthrough](#example-walkthrough)
6. [Usage Guide](#usage-guide)
7. [Output Files](#output-files)
8. [Testing](#testing)
9. [TODO - Features & Improvements](#todo---features--improvements)
10. [Contributing](#contributing)

---

## Overview

LeoSim is a sophisticated ns-3 module designed to simulate realistic LEO satellite networks. It provides:

- **Realistic satellite dynamics** with TLE (Two-Line Element) data and orbital mechanics
- **Ground station modeling** for servers, UEs (User Equipment), and gateways
- **Dynamic channel modeling** with elevation angle constraints, path loss, and line-of-sight calculations
- **Inter-Satellite Links (ISLs)** for satellite-to-satellite communication
- **Dynamic routing** with periodic topology updates as satellites move
- **3GPP NTN Handover Management** with conditional handover (CHO) and basic handover (BHO) modes
- **Beam Management** supporting multi-criteria decision making (TOPSIS) for handover optimization
- **Traffic generation** capabilities (ping, custom applications)
- **Position and link logging** for visualization
- **Packet tracing** for detailed analysis
- **Flow monitoring integration** for packet-level KPI capture

**Target Scenarios:**
- LEO constellation connectivity analysis
- Satellite network routing protocols evaluation
- Ground-to-satellite and satellite-to-satellite latency studies
- Handover and beam switching impact analysis with 3GPP compliance
- Traffic engineering in dynamic satellite networks

---

## Architecture

### System Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    NS-3 Simulation Core                      │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │            LeoSim Module (./contrib/leosim/)           │ │
│  │                                                        │ │
│  │  ┌──────────────────┐        ┌─────────────────────┐ │ │
│  │  │   Model Layer    │        │  Helper Layer       │ │ │
│  │  ├──────────────────┤        ├─────────────────────┤ │ │
│  │  │- LeoSimLoader    │        │- LeoSimLoaderHelper │ │ │
│  │  │- LeoSimChannel   │        │- LeoSimChannelHelp  │ │ │
│  │  │- LeoSimMobility  │        │- LeoSimMobilityHelp │ │ │
│  │  │- LeoSimRoutingCa │        │- LeoSimDeviceInst   │ │ │
│  │  │- LeoSimISLRouting        │- LeoSimRoutingCalcH │ │ │
│  │  │- LeoSimBeamMgr   │        │- LeoSimTrafficGenH  │ │ │
│  │  │                  │        │- LeoSimVisualizH    │ │ │
│  │  │                  │        │- LeoSimBeamMgrHelp  │ │ │
│  │  └──────────────────┘        └─────────────────────┘ │ │
│  │                                                        │ │
│  │  ┌────────────────────────────────────────────────┐  │ │
│  │  │           Example Programs                      │  │ │
│  │  ├────────────────────────────────────────────────┤  │ │
│  │  │  leosim-example.cc (Main TCP/Ping example)    │  │ │
│  │  └────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────┘ │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐ │
│  │        Supporting NS-3 Modules                         │ │
│  │  (internet, network, mobility, propagation, etc.)     │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow in Simulation

```
Input Data
    ↓
┌─────────────────────────────────────┐
│  LeoSimLoader                       │
│  - Load satellite mobility (TLE)    │
│  - Load ground devices from CSV     │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Node Creation & Mobility Setup     │
│  - Create satellite nodes           │
│  - Create UE/Server nodes           │
│  - Install mobility models          │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Channel Model & Link Management    │
│  - LeoSimChannelModel               │
│  - Compute elevation angles         │
│  - Calculate path loss              │
│  - Manage link states               │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Network Device Installation        │
│  - LeoSimDeviceInstaller            │
│  - Create point-to-point links      │
│  - Assign IP addresses              │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Routing Setup                      │
│  - LeoSimRoutingCalculator          │
│  - Compute paths (Dijkstra)         │
│  - Schedule periodic updates        │
│  - Install static routes            │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Traffic Injection                  │
│  - Install applications (ping, TCP) │
│  - Schedule starts/stops            │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│  Beam Manager Setup (Optional)      │
│  - LeoSimBeamManager                │
│  - Configure handover parameters    │
│  - Initialize handover logic        │
│  - Start periodic beam updates      │
└──────────────┬──────────────────────┘
               ↓
        Simulation Runs
               ↓
┌─────────────────────────────────────┐
│  Output & Logging                   │
│  - Position tracking (CSV)          │
│  - Link state (CSV)                 │
│  - Packet traces (CSV/PCap)         │
│  - Handover events & statistics     │
│  - Flow monitoring (if enabled)     │
└─────────────────────────────────────┘
```

---

## Core Components

### 1. **LeoSimLoader** (model/leosim-loader.*)
**Purpose:** Load satellite and ground device data

**Key Features:**
- Load satellite data from **trace files** (`.tcl` format)
- Load satellite data from **CSV files** with timestep data
- Load ground devices from **CSV files**
- Query by device type (SERVER, UE, GATEWAY)
- Get satellite names and ground device names
- Ground device position retrieval
- Support for **geodetic coordinates** (latitude, longitude, altitude) and Cartesian (x, y, z)
- Trajectory tracking with timestep information

**Data Structures:**
```
Satellite:
  - ID
  - Name (satellite name)
  - Mobility model
  - Position updates

Ground Device:
  - ID
  - Name
  - Type (SERVER, UE, GATEWAY)
  - Fixed position (latitude, longitude, altitude)
```

**Example Usage:**
```cpp
LeoSimLoaderHelper loaderHelper;
loaderHelper.SetVerbose(true);
loaderHelper.LoadSatellitesFromTrace("satellite_mobility.tcl");
loaderHelper.LoadGroundDevicesFromCsv("ground_devices.csv");
Ptr<LeoSimLoader> loader = loaderHelper.GetLoader();
```

### 2. **LeoSimChannel** (model/leosim-channel.*)
**Purpose:** Model individual communication links between satellites and ground nodes

**Key Features:**
- Calculate distance between nodes
- Compute elevation angle
- Calculate path loss using free-space path loss formula
- Determine if elevation angle meets minimum threshold
- Track link state (UP, DOWN, DEGRADED)

**Formulas Used:**
- **Path Loss (dB):** `20*log10(distance) + 20*log10(frequency/c) + constant`
- **Elevation Angle:** `arcsin(cos(sat_lat) * cos(sat_lon - gnd_lon) * cos(gnd_lat) + sin(sat_lat) * sin(gnd_lat))`

### 3. **LeoSimChannelModel** (model/leosim-channel-model.*)
**Purpose:** Manage all links in the simulation and provide dynamic updates

**Key Features:**
- Maintain link quality metrics for all node pairs
- Update links at configurable intervals
- Support for ground links (satellite ↔ ground) and ISL links (satellite ↔ satellite)
- Query available links for a node
- Track link type and quality

**Configuration:**
```cpp
LeoSimChannelHelper channelHelper;
channelHelper.SetMinElevationAngle(10.0);      // degrees
channelHelper.SetFrequency(12.0e9);            // Hz (Ku band)
channelHelper.SetTransmitPower(40.0);          // dBm
channelHelper.SetMaxLinkDistance(2500000.0);   // meters
channelHelper.SetUpdateInterval(Seconds(1.0)); // Update frequency
```

### 4. **LeoSimMobilityModel** (model/leosim-mobility-model.*)
**Purpose:** Model satellite orbital motion and static ground positions

**Key Features:**
- Satellite trajectory from TLE data
- Ground device fixed positions
- Velocity calculations (optional)
- Position updates driven by simulation time

**Supported Models:**
- **SatelliteMotion:** Dynamic motion following orbital mechanics
- **StaticGateway:** Fixed ground station positions

### 5. **LeoSimRoutingCalculator** (model/leosim-routing-calculator.*)
**Purpose:** Compute optimal routes through the satellite network

**Key Features:**
- **Dijkstra's algorithm** with weighted shortest path computation
- **Multiple routing metrics** for flexible path optimization:
  - `LEOSIM_METRIC_HOP_COUNT`: Minimize number of hops
  - `LEOSIM_METRIC_PATH_LOSS`: Minimize accumulated path loss
  - `LEOSIM_METRIC_SNR`: Maximize signal-to-noise ratio
  - `LEOSIM_METRIC_DISTANCE`: Minimize total distance
  - `LEOSIM_METRIC_SIGNAL_STRENGTH`: Maximize signal strength
- **Path constraints** for link type filtering:
  - `LEOSIM_PATH_ANY`: Allow mixed ISL and ground links
  - `LEOSIM_PATH_ISL_ONLY`: Route only through ISL links
  - `LEOSIM_PATH_GROUND_ONLY`: Route only through ground links
- **Unified routing** supporting both ground and ISL links
- Route quality metrics (hop count, path loss, SNR)
- Path validation based on link availability

**Route Structure:**
```cpp
struct LeoSimRoute {
    std::vector<Ptr<Node>> path;          // Ordered nodes in path
    uint32_t hopCount;                    // Number of hops
    double totalPathLoss;                 // Accumulated path loss in dB
    double totalDistance;                 // Total distance in meters
    double minSignalStrength;             // Minimum signal strength (dBm)
    double minSnr;                        // Minimum SNR along path (dB)
    bool hasIslLinks;                     // Path contains ISL links
    bool hasGroundLinks;                  // Path contains ground links
    std::vector<LeoSimLinkType> linkTypes;// Type of each link in path
    bool valid;                           // Whether route is valid
};
```

**Algorithm:**
1. Initialize distances and visited set
2. Dijkstra iteration: expand lowest-cost unvisited node
3. Update neighbors based on selected routing metric
4. Respect path type constraints (ISL-only, ground-only, or mixed)
5. Backtrack from destination to rebuild ordered path
6. Validate all links in path are UP
7. Calculate comprehensive route metrics

### 6. **LeoSimISLRoutingModel** (model/leosim-isl-routing-model.*)
**Purpose:** Manage inter-satellite link topology

**Key Features:**
- Create ISL mesh between satellites
- Enforce maximum distance constraints
- Manage ISL-specific metrics
- Support dynamic ISL creation/destruction

### 7. **LeoSimDeviceInstaller** (helper/leosim-device-installer.*)
**Purpose:** Install network devices on nodes based on channel model

**Key Features:**
- Create point-to-point links between nodes
- Automatically detect link pairs from channel model
- Configure device data rates
- Set link delays
- Set MTUs

**Configuration:**
```cpp
LeoSimDeviceInstaller deviceInstaller;
deviceInstaller.SetChannelModel(channelModel);
deviceInstaller.SetDeviceDataRate("100Mbps");     // Ground links
deviceInstaller.SetDeviceDelay("1ms");
deviceInstaller.SetDeviceMtu(1500);
NetDeviceContainer devices = deviceInstaller.Install(satellites, groundNodes);
```

### 8. **LeoSimTrafficGeneratorHelper** (helper/leosim-traffic-generator-helper.*)
**Purpose:** Install traffic-generating applications

**Key Features:**
- **Ping (ICMP echo)** application generation
- Configurable ping intervals and payload sizes
- Flexible traffic installation methods:
  - `InstallPingClient()`: Single source to destination
  - `InstallPingClients()`: Multiple sources to single destination
  - `InstallBidirectionalPing()`: Two-way ping traffic
  - `InstallUeToServerPing()`: UEs to server topology
- Start/stop time scheduling
- Verbose logging support

**Configuration:**
```cpp
LeoSimTrafficGeneratorHelper trafficHelper;
trafficHelper.SetPingInterval(Seconds(10.0));
trafficHelper.SetPingDataSize(56);  // ICMP payload
trafficHelper.SetVerbose(true);

// Option 1: Single client
ApplicationContainer ping = trafficHelper.InstallPingClient(
    ueNode, serverAddr, Seconds(1.0), Seconds(59.0)
);

// Option 2: Multiple clients
ApplicationContainer pings = trafficHelper.InstallPingClients(
    ueNodes, serverAddr, Seconds(1.0), Seconds(59.0)
);

// Option 3: Bidirectional
ApplicationContainer bidPings = trafficHelper.InstallBidirectionalPing(
    node1, node2, addr1, addr2, Seconds(1.0), Seconds(59.0)
);

// Option 4: UE-to-server pattern
ApplicationContainer serverPings = trafficHelper.InstallUeToServerPing(
    ueNodes, serverNodes.Get(0), serverAddr, Seconds(1.0), Seconds(59.0)
);
```

### 9. **LeoSimVisualizationHelper** (helper/leosim-visualization-helper.*)
**Purpose:** Log simulation data for visualization and analysis

**Key Features:**
- Position logging (satellites, UEs, servers) at intervals
- Link state logging (which nodes can communicate)
- Packet-level logging (optional, when enabled)
- CSV output format for post-processing
- Support for Python visualization scripts

**Output Files:**
- `leosim_positions.csv` - Node positions over time
- `leosim_links.csv` - Active links over time
- `leosim_packets.csv` - Individual packet traces

**Configuration:**
```cpp
LeoSimVisualizationHelper vizHelper;
vizHelper.SetOutputFile("leosim_positions.csv");
vizHelper.SetLinkFile("leosim_links.csv");
vizHelper.SetPacketFile("leosim_packets.csv");
vizHelper.EnablePacketLogging(true);
vizHelper.Initialize();
vizHelper.SchedulePositionLogging(sats, servers, ues, 1.0, simTime);
```

### 10. **LeoSimRoutingCalculatorHelper** (helper/leosim-routing-calculator-helper.*)
**Purpose:** Simplify routing calculator creation and management

**Key Features:**
- Create unified routing calculators
- Enable dynamic routing with periodic updates
- Handle timer scheduling
- Manage multiple routing calculators

**Configuration:**
```cpp
LeoSimRoutingCalculatorHelper routingHelper;
Ptr<LeoSimRoutingCalculator> calc = 
    routingHelper.CreateUnifiedRoutingCalculator(
        groundChannelModel, islChannelModel, verbose
    );
routingHelper.EnableDynamicRouting(calc, allNodes, allNodes, 
    updateInterval, simTime, verbose);
```

### 11. **LeoSimBeamManager** (model/leosim-beam-manager.*)
**Purpose:** Manage beam selection, handover decisions, and handover execution using 3GPP NTN recommendations

**Key Features:**
- **3GPP-Compliant Handover Modes:**
  - BHO (Basic Handover): Reactive handover on link failure
  - CHO (Conditional Handover): Proactive 3GPP Rel-17 NTN handover
- **3GPP Timer Management (TS 38.321):**
  - TTT (Time-To-Trigger): Measurement report filtering
  - T310: Radio Link Failure timer
  - N310/N311: RLF counters for event-based triggering
- **Handover Trigger Conditions:**
  - A3 Event: Neighboring cell stronger than serving + offset
  - A4 Event: Absolute RSRP threshold
  - Elevation Angle: Minimum elevation constraints
  - Time-To-Exist (TTE): Ephemeris-based handover prediction
  - Load Balancing: Satellite load distribution
- **TOPSIS Multi-Criteria Decision Making:**
  - Weighted criteria: RSRP, TTE, Satellite Load, Latency, Elevation
  - Configurable weighting coefficients
  - Automated candidate ranking
- **Packet Buffering During Handover:**
  - Automatic packet buffering at source
  - Configurable buffer size and retention
  - Flow monitoring integration for KPI capture
- **Advanced Features:**
  - Ping-pong detection with rolling window
  - Per-UE beam tracking
  - Handover history and statistics
  - Verbose logging for debugging

**Setter Methods Available:**

*Dependency Injection:*
- `SetChannelModel(Ptr<LeoSimChannelModel>)` - Propagation channel model
- `SetIslChannelModel(Ptr<LeoSimChannelModel>)` - Inter-satellite link model
- `SetRoutingCalculator(Ptr<LeoSimRoutingCalculator>)` - Routing calculator
- `SetLoader(Ptr<LeoSimLoader>)` - Satellite/beam loader
- `SetVerbose(bool)` - Enable debug output

*Handover Modes:*
- `SetHandoverMode(LeoSimHandoverMode)` - CHO or BHO
- `SetBeamMode(bool earthFixed)` - Earth-fixed or body-fixed beams

*3GPP Timers:*
- `SetTttDuration(Time)` - Time-To-Trigger duration
- `SetT310Duration(Time)` - RLF timer duration
- `SetN310Count(uint32_t)` - RLF event counter
- `SetN311Count(uint32_t)` - RLF recovery counter

*Handover Thresholds:*
- `SetA3Offset(double)` - A3 event offset (dB)
- `SetA4Threshold(double)` - A4 absolute threshold (dBm)
- `SetElevationThreshold(double)` - Minimum elevation (degrees)
- `SetTteThreshold(Time)` - Time-to-exist threshold

*TOPSIS Configuration:*
- `SetTopsisWeights(double w1-w5)` - Weighting coefficients
- `SetMaxCandidates(uint32_t)` - Maximum candidate count

*CHO Timing:*
- `SetChoPreparationDelay(Time)` - Phase 1 preparation delay
- `SetChoExecutionDelay(Time)` - Phase 2 execution delay

*Load Balancing:*
- `EnableLoadBalancing(bool)` - Enable/disable load balancing
- `SetLoadImbalanceThreshold(uint32_t)` - Load difference threshold
- `EnableHandoverBuffering(bool)` - Enable/disable packet buffering
- `SetMaxBufferSize(uint32_t)` - Maximum buffer size (packets)

*Flow Monitoring:*
- `SetFlowMonitor(Ptr<FlowMonitor>)` - Flow monitor instance
- `SetFlowClassifier(Ptr<Ipv4FlowClassifier>)` - IPv4 flow classifier

**Configuration:**
```cpp
LeoSimBeamManagerHelper beamHelper;
beamHelper.SetChannelModel(channelModel);
beamHelper.SetIslChannelModel(islChannelModel);
beamHelper.SetRoutingCalculator(routingCalculator);
beamHelper.SetLoader(loader);

// Handover mode
beamHelper.SetHandoverMode(LEOSIM_HO_MODE_CHO);
beamHelper.SetEarthFixedBeamMode(false);

// 3GPP timers
beamHelper.SetTtt(Seconds(1.0));
beamHelper.SetT310(Seconds(1.0));
beamHelper.SetN310(3);
beamHelper.SetN311(3);

// Thresholds
beamHelper.SetA3Offset(3.0);
beamHelper.SetA4Threshold(-110.0);
beamHelper.SetElevationThreshold(10.0);
beamHelper.SetTteThreshold(Seconds(30.0));

// TOPSIS weighting
beamHelper.SetTopsisWeights(0.30, 0.30, 0.15, 0.15, 0.10);
beamHelper.SetMaxCandidates(3);

// CHO timing
beamHelper.SetChoPreparationDelay(MilliSeconds(100));
beamHelper.SetChoExecutionDelay(MilliSeconds(150));

// Load balancing & buffering
beamHelper.EnableLoadBalancing(true);
beamHelper.EnableHandoverBuffering(true);

// Install and start beam manager
Ptr<LeoSimBeamManager> beamMgr = beamHelper.Install(ueNodes, satNodes, simTime);
```

### 12. **LeoSimBeamManagerHelper** (helper/leosim-beam-manager-helper.*)
**Purpose:** Simplify beam manager configuration and installation

**Key Features:**
- Centralized parameter configuration
- Easy installation with preconfigured settings
- Support for all beam manager features
- Verbose logging support

---

## Implementation Process

### Phase 1: Setup & Data Loading
1. **Create simulator** and set time resolution
2. **Load data** using LeoSimLoader
   - Satellite positions from trace files or CSV
   - Ground device locations from CSV
3. **Create nodes**
   - Satellite nodes
   - UE nodes
   - Server nodes

### Phase 2: Mobility Installation
1. Create LeoSimMobilityHelper
2. Install satellite mobility (orbital motion)
3. Install gateway mobility (static)
4. Start mobility models

### Phase 3: Channel Modeling
1. Create channel models
2. Configure parameters (frequency, elevation angles, etc.)
3. Create ground links (satellite ↔ ground)
4. Create ISL mesh (satellite ↔ satellite) if enabled
5. Start channel updates

### Phase 4: Network Setup
1. Install point-to-point devices
   - Ground devices (satellite-ground links)
   - ISL devices (satellite-satellite links)
2. Install internet stack (IPv4/IPv6)
3. Assign IP addresses (unique per-link subnets)

### Phase 5: Routing
1. Create routing calculator
2. Enable dynamic routing
3. Schedule periodic routing updates
4. Install routes into static routing tables

### Phase 6: Traffic Injection
1. Create traffic applications
   - Ping applications
   - Custom TCP/UDP applications
2. Schedule application start/stop times
3. Configure application parameters

### Phase 7: Beam Manager & Handover (Optional)
1. Create LeoSimBeamManagerHelper
2. Configure handover parameters:
   - Handover mode (BHO or CHO)
   - 3GPP timers and counters
   - Trigger thresholds (A3, A4, elevation, TTE)
   - TOPSIS weighting coefficients
3. Enable optional features:
   - Load balancing
   - Packet buffering
   - Flow monitoring integration
4. Install and start beam manager
5. Beam manager runs periodic update cycles for handover decisions

### Phase 8: Logging & Visualization
1. Setup visualization helper
2. Schedule position logging
3. Enable packet logging (optional)
4. Setup output files

### Phase 9: Execution & Analysis
1. Run simulator
2. Stop channel updates and beam manager
3. Finalize visualization
4. Destroy simulator
5. Post-process output files
   - Analyze handover history and statistics
   - Process KPI captures from flow monitoring
   - Visualize beam switching events

---

## Example Walkthrough

### Program Structure: leosim-example.cc

#### Command-Line Arguments
```bash
./ns3 run leosim-example -- [options]

Key options:
  --satellites="path/to/satellite_mobility.tcl"   # Satellite data
  --groundDevices="contrib/leosim/utils/ground_devices.csv"  # Ground stations
  --simTime=60                                     # Simulation duration (seconds)
  --numSatellites=300                             # Number of satellites to use
  --numServers=1                                  # Number of server nodes
  --numUes=1                                      # Number of UE nodes
  --enableIsl=1                                   # Enable inter-satellite links
  --logPackets=1                                  # Enable packet logging
  --verbose=1                                     # Enable verbose output
```

#### Execution Phases

**1. Initialization**
```cpp
// Set time resolution
Time::SetResolution(Time::NS);

// Load data
LeoSimLoaderHelper loaderHelper;
loaderHelper.SetVerbose(verbose);

// Load satellites from trace file OR CSV
if (useTrace) {
    loaderHelper.LoadSatellitesFromTrace(satelliteFile);
} else {
    loaderHelper.LoadSatellitesFromCsv(satelliteFile);
}
loaderHelper.LoadGroundDevicesFromCsv(groundDeviceFile);

// Get device lists
Ptr<LeoSimLoader> loader = loaderHelper.GetLoader();
auto serverDeviceIds = loader->GetGroundDeviceIdsByType("SERVER");
auto ueDeviceIds = loader->GetGroundDeviceIdsByType("UE");
```

**2. Node Creation**
```cpp
// Create node containers
NodeContainer ueNodes;
ueNodes.Create(numUes);

NodeContainer satelliteNodes;
satelliteNodes.Create(numSatellites);

NodeContainer serverNodes;
serverNodes.Create(numServers);
```

**3. Mobility Setup**
```cpp
// Setup mobility
LeoSimMobilityHelper mobilityHelper;
mobilityHelper.SetLoader(loader);
mobilityHelper.SetVerbose(verbose);
mobilityHelper.SetVelocityCalculation(true);

// Install satellite mobility
for (uint32_t i = 0; i < numSatellites; i++) {
    mobilityHelper.InstallSatellite(satelliteNodes.Get(i), i, 
                                    loader->GetSatelliteName(i));
}

// Install ground station mobility
for (uint32_t i = 0; i < numUes; i++) {
    uint32_t ueId = ueDeviceIds[i];
    mobilityHelper.InstallGateway(ueNodes.Get(i), ueId,
                                  loader->GetGroundDeviceName(ueId),
                                  loader->GetGroundDevicePosition(ueId));
}

mobilityHelper.StartAll();
```

**4. Channel Model Creation**
```cpp
// Ground channel
LeoSimChannelHelper channelHelper;
channelHelper.SetMinElevationAngle(minElevation);
channelHelper.SetFrequency(frequency);
channelHelper.SetTransmitPower(40.0);
channelHelper.SetMaxLinkDistance(2500000.0);
channelHelper.SetUpdateInterval(Seconds(1.0));

NodeContainer allGroundNodes;
allGroundNodes.Add(ueNodes);
allGroundNodes.Add(serverNodes);

Ptr<LeoSimChannelModel> channelModel =
    channelHelper.CreateChannels(satelliteNodes, allGroundNodes);
channelModel->StartUpdates();

// ISL channel (if enabled)
if (enableIsl) {
    LeoSimChannelHelper islHelper;
    islHelper.SetIslFrequency(islFrequency);
    islHelper.SetIslMaxDistance(islMaxDistance);
    islHelper.SetIslTransmitPower(islTransmitPower);
    islHelper.SetIslAntennaGain(islAntennaGain);
    
    Ptr<LeoSimChannelModel> islChannelModel = 
        islHelper.CreateIslMesh(satelliteNodes);
    islChannelModel->StartUpdates();
}
```

**5. Device Installation**
```cpp
// Ground devices
LeoSimDeviceInstaller deviceInstaller;
deviceInstaller.SetChannelModel(channelModel);
deviceInstaller.SetDeviceDataRate("100Mbps");
deviceInstaller.SetDeviceDelay("1ms");
deviceInstaller.SetDeviceMtu(1500);

NetDeviceContainer groundDevices = 
    deviceInstaller.Install(satelliteNodes, allGroundNodes);

// ISL devices
if (enableIsl && islChannelModel) {
    LeoSimDeviceInstaller islDeviceInstaller;
    islDeviceInstaller.SetChannelModel(islChannelModel);
    islDeviceInstaller.SetDeviceDataRate("10Gbps");
    islDeviceInstaller.SetDeviceDelay("100us");
    
    NetDeviceContainer islDevices = 
        islDeviceInstaller.Install(satelliteNodes, NodeContainer());
}
```

**6. Network Stack & Addressing**
```cpp
// Install Internet stack
InternetStackHelper stack;
stack.Install(satelliteNodes);
stack.Install(ueNodes);
stack.Install(serverNodes);

// Assign IP addresses (per-link subnets)
uint32_t groundSubnetIndex = 0;
uint32_t islSubnetIndex = 100;

for (uint32_t i = 0; i < groundDevices.GetN(); i += 2) {
    NetDeviceContainer linkDevices;
    linkDevices.Add(groundDevices.Get(i));
    if (i + 1 < groundDevices.GetN())
        linkDevices.Add(groundDevices.Get(i + 1));
    
    Ipv4AddressHelper groundIpv4;
    char baseAddr[32];
    snprintf(baseAddr, sizeof(baseAddr), "10.%d.0.0", groundSubnetIndex);
    groundIpv4.SetBase(Ipv4Address(baseAddr), Ipv4Mask("255.255.255.0"));
    groundIpv4.Assign(linkDevices);
    
    groundSubnetIndex++;
}
```

**7. Routing Setup**
```cpp
// Create routing calculator with Dijkstra algorithm
LeoSimRoutingCalculatorHelper routingHelper;
Ptr<LeoSimRoutingCalculator> unifiedCalc = 
    routingHelper.CreateUnifiedRoutingCalculator(
        channelModel, islChannelModel, verbose);

// Set preferred routing metric (HOP_COUNT, PATH_LOSS, SNR, DISTANCE, SIGNAL_STRENGTH)
// unifiedCalc->SetRoutingMetric(LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT);

// Enable dynamic routing with periodic updates
NodeContainer allNodes;
allNodes.Add(satelliteNodes);
allNodes.Add(ueNodes);
allNodes.Add(serverNodes);

routingHelper.EnableDynamicRouting(unifiedCalc, allNodes, allNodes,
    Seconds(routingUpdateInterval), simTime, verbose);
```

**8. Traffic Injection**
```cpp
// Ping traffic
LeoSimTrafficGeneratorHelper trafficHelper;
trafficHelper.SetPingInterval(Seconds(10.0));
trafficHelper.SetPingDataSize(56);
trafficHelper.SetVerbose(verbose);

Ptr<Ipv4> serverIpv4 = serverNodes.Get(0)->GetObject<Ipv4>();
Ipv4Address serverAddress = serverIpv4->GetAddress(1, 0).GetLocal();

ApplicationContainer pingApps = trafficHelper.InstallUeToServerPing(
    ueNodes, serverNodes.Get(0), serverAddress,
    Seconds(1.0), Seconds(simTime - 1.0));
```

**9. Execution**
```cpp
// Setup visualization and logging
LeoSimVisualizationHelper vizHelper;
vizHelper.SetOutputFile(positionFile);
vizHelper.SetLinkFile(linkFile);
vizHelper.SetPacketFile(packetFile);
vizHelper.EnablePacketLogging(logPackets);
vizHelper.Initialize();
vizHelper.SchedulePositionLogging(satelliteNodes, serverNodes, ueNodes, 
                                  logInterval, simTime);

// Run simulation
Simulator::Stop(Seconds(simTime));
Simulator::Run();

// Cleanup
channelModel->StopUpdates();
if (enableIsl && islChannelModel)
    islChannelModel->StopUpdates();
vizHelper.Finalize();
Simulator::Destroy();
```

---

## Usage Guide

### Building the Module

```bash
# Configure ns-3 with examples and tests enabled
cd /home/anupa/UCD/LeoSim/ns3
./ns3 configure --enable-examples --enable-tests

# Build
./ns3 build
```

### Running the Example

```bash
# Basic run with default parameters
./ns3 run leosim-example

# With specific satellite count and ISL enabled
./ns3 run "leosim-example --numSatellites=100 --enableIsl=1"

# Full verbose output with packet logging
./ns3 run "leosim-example --verbose=1 --logPackets=1 --simTime=30"

# Custom output files
./ns3 run "leosim-example --positions=my_positions.csv \
           --links=my_links.csv --packets=my_packets.csv"
```

### Running Tests

```bash
# Run all LeoSim tests
./test.py -s leosim

# Run with specific verbosity
./test.py -v -s leosim
```

### Input Files

**1. Satellite Mobility File** (Trace or CSV format)

**Format 1: Trace File** (`satellite_mobility.tcl`)
- Format: ns-3 trace file format
- Contains satellite positions over time
- Generated from TLE (Two-Line Element) data
- Path: `contrib/leosim/utils/satellite_mobility.tcl`
- Load with: `loaderHelper.LoadSatellitesFromTrace(file)`

**Format 2: CSV File** (`satellite_positions.csv`)
- CSV with headers: ID, Time, Name, X(or Lat), Y(or Lon), Z(or Alt)
- Contains one line per satellite per timestep
- Timestep data allows trajectory tracking
- Load with: `loaderHelper.LoadSatellitesFromCsv(file)`

**2. Ground Devices CSV** (`ground_devices.csv`)
- Format: CSV with headers
- Columns: ID, Name, Type, Latitude, Longitude, Altitude
- Types: SERVER, UE, GATEWAY
- Path: `contrib/leosim/utils/ground_devices.csv`

Example:
```
ID,Name,Type,Latitude,Longitude,Altitude
1,Gateway-1,SERVER,37.7749,-122.4194,17
2,User-1,UE,34.0522,-118.2437,20
3,User-2,UE,41.8781,-87.6298,15
```

---

## Output Files

### 1. Position Log (`leosim_positions.csv`)

**Format:**
```
Time(s),NodeType,NodeID,NodeName,X(m),Y(m),Z(m)
0.0,SATELLITE,0,Satellite0,1234567.8,2345678.9,3456789.0
0.0,SERVER,300,Gateway-1,4567890.1,5678901.2,6789012.3
0.0,UE,301,User-1,7890123.4,8901234.5,9012345.6
1.0,SATELLITE,0,Satellite0,1234568.1,2345679.2,3456789.3
...
```

**Usage:**
- Track satellite and ground station positions
- Verify mobility models
- Generate animations
- Analyze coverage patterns

### 2. Link Log (`leosim_links.csv`)

**Format:**
```
Time(s),SourceNode,SourceID,DestNode,DestID,Distance(m),Elevation(deg),PathLoss(dB),SignalStrength(dBm),SNR(dB),LinkType,LinkState
0.0,Satellite,0,Gateway,300,580234.5,15.2,-185.3,-131.0,18.9,GROUND,UP
0.0,Satellite,0,Satellite,1,450000.0,N/A,-180.5,-126.2,23.6,ISL,UP
...
```

**Usage:**
- Track active connections
- Analyze link quality metrics
- Study elevation angle impacts
- Identify connectivity gaps

### 3. Packet Log (`leosim_packets.csv`)

**Format:**
```
Time(s),EventType,Direction,SourceID,DestID,Size(bytes),UID
0.5,TX,TX,1,300,100,1
0.50015,RX,RX,300,1,100,1
0.501,ECHO_REPLY,TX,300,1,100,1
...
```

**Usage:**
- Packet-level tracing
- Latency analysis
- Loss detection
- Throughput verification

---

## Testing

### Unit Test Suite

**Location:** `test/leosim-test-suite.cc`

**Current State:** Minimal test coverage (test case that does nothing)

**Running Tests:**
```bash
./test.py -s leosim
```

### Test Coverage Areas (TODO)

1. **Loader Tests:**
   - Load satellite data from trace files
   - Load ground devices from CSV
   - Query by device type
   - Handle missing files

2. **Channel Model Tests:**
   - Elevation angle calculation accuracy
   - Path loss formula verification
   - Link state transitions
   - Distance calculations

3. **Routing Tests:**
   - Dijkstra pathfinding with multiple metrics
   - Route validation
   - Hop count accuracy
   - Path quality metrics
   - Path type constraints (ISL-only, ground-only, mixed)

4. **Device Installer Tests:**
   - Device creation
   - IP address assignment
   - Multiple link support

5. **Integration Tests:**
   - Full simulation runs
   - Traffic generation
   - Output file generation
   - Logging verification

---

## TODO - Features & Improvements

### High Priority

#### 1. **Comprehensive Testing** 🔴
- [ ] Implement unit tests for all major components
- [ ] Add integration tests for full simulation workflow
- [ ] Create regression tests for updated components
- [ ] Test error handling and edge cases
- [ ] Performance/scalability tests with large constellations

**Estimated Impact:** Critical for production use
**Implementation Effort:** High
**Dependencies:** None

#### 2. **Advanced Routing Algorithms** ✅
- [x] Implement Dijkstra's algorithm for weighted shortest path
- [x] Add multiple routing metrics (hop count, SNR, path loss, distance, signal strength)
- [ ] Support k-shortest paths (backup routes)
- [ ] Implement dynamic routing with congestion awareness
- [ ] Add route caching mechanisms
- [ ] Optimize routing table updates

**Current:** Dijkstra with multiple metrics (COMPLETED)
**Next:** k-shortest paths and congestion awareness
**Estimated Effort for Remaining Items:** Medium

#### 3. **Handover Management** ✅
- [x] Detect handover opportunities (A3, A4, elevation, TTE-based triggers)
- [x] Manage handover triggers and timing (3GPP TTT, T310 timers)
- [x] Implement soft/hard handover strategies (BHO and CHO modes)
- [x] Track handover impact on traffic (packet buffering, flow monitoring integration)
- [x] Log handover events (complete history with metrics, KPI capture)
- [x] Multi-criteria handover decision making (TOPSIS ranking)
- [x] 3GPP NTN compliance (TS 38.321 timers and RLF procedures)
- [x] Ping-pong detection and prevention

**Current:** LeoSimBeamManager fully implemented with all features (COMPLETED)
**Features:** 
- 3GPP timer management (TTT, T310, N310, N311)
- Multi-trigger handover (A3, A4, elevation, TTE, load balancing, RLF)
- TOPSIS multi-criteria decision making
- CHO and BHO modes
- Packet buffering and flow monitoring
- Detailed logging and statistics
**Impact:** Critical for realistic LEO operation
**Implementation Status:** COMPLETED - see LeoSimBeamManager (Section 11)

#### 4. **Link Quality Adaptation** 🟠
- [ ] Adaptive modulation/coding based on SNR
- [ ] Dynamic data rate adjustment
- [ ] Link retry mechanisms
- [ ] Quality-of-Service (QoS) classes
- [ ] Congestion detection and response

**Current:** Static rates
**Impact:** More realistic performance
**Estimated Effort:** Medium

### Medium Priority

#### 5. **Advanced Traffic Models** 🟠
- [ ] TCP/UDP applications beyond ping
- [ ] File transfer simulations
- [ ] Video streaming models
- [ ] Real-time communication (voice/video)
- [ ] Background traffic generation
- [ ] Traffic burst patterns

**Current:** Ping only
**Impact:** Better understanding of system performance
**Estimated Effort:** Medium

#### 6. **Enhanced ISL Topology** 🟠
- [ ] Dynamic ISL creation/destruction based on distance
- [ ] ISL handover mechanisms
- [ ] Cross-link interference modeling
- [ ] ISL-specific propagation models
- [ ] ISL frequency/bandwidth management

**Current:** Static ISL mesh
**Impact:** More realistic ISL dynamics
**Estimated Effort:** Medium

#### 7. **Performance Optimization** 🟡
- [ ] Optimize channel model update frequency
- [ ] Cache routing calculations
- [ ] Reduce memory footprint for large constellations
- [ ] Parallel processing for independent calculations
- [ ] Efficient data structure for link lookups

**Current:** Basic implementation
**Impact:** Enable larger-scale simulations
**Estimated Effort:** Medium

#### 8. **Flow Monitor Integration** 🟠
- [ ] Collect end-to-end delay statistics
- [ ] Track packet loss rates
- [ ] Monitor throughput per flow
- [ ] Generate flow-based reports
- [ ] Integration with ns-3 FlowMonitor

**Current:** Basic packet logging
**Impact:** Standard ns-3 traffic analysis
**Estimated Effort:** Low

### Medium-Low Priority

#### 9. **Visualization Enhancements** 🟡
- [ ] Python script for 3D visualization
- [ ] Animation of satellite motion
- [ ] Link connectivity animation
- [ ] Performance metrics dashboard
- [ ] Interactive parameter adjustment

**Current:** CSV output only
**Impact:** Better insight into simulation behavior
**Estimated Effort:** Medium

#### 10. **Error Handling & Validation** 🟡
- [ ] Input file validation
- [ ] Range checks for all parameters
- [ ] Better error messages
- [ ] Graceful degradation
- [ ] Recovery mechanisms

**Current:** Basic checks
**Impact:** Robustness and debuggability
**Estimated Effort:** Low-Medium

### Low Priority

#### 11. **Additional Propagation Models** 🟢
- [ ] Rain attenuation model
- [ ] Scintillation effects
- [ ] Multipath propagation
- [ ] Doppler shift calculations
- [ ] Atmospheric absorption

**Current:** Free-space path loss only
**Impact:** Higher accuracy
**Estimated Effort:** Medium

#### 12. **Satellite Failure Scenarios** 🟢
- [ ] Model satellite breakdowns
- [ ] Temporary link failures
- [ ] Recovery procedures
- [ ] Impact on routing

**Current:** All satellites always functional
**Impact:** Reliability analysis
**Estimated Effort:** Low

#### 13. **Ground Station Mobility** 🟢
- [ ] Support for moving ground stations
- [ ] Aerial platforms (aircraft, balloons)
- [ ] Mobile users in vehicles
- [ ] Maritime terminals

**Current:** Static ground stations only
**Impact:** More diverse scenarios
**Estimated Effort:** Medium

#### 14. **Configuration Files** 🟢
- [ ] Support XML/JSON configuration
- [ ] Scenario templates
- [ ] Parameter presets
- [ ] Configuration validation

**Current:** Command-line arguments only
**Impact:** Easier scenario management
**Estimated Effort:** Low

#### 15. **Documentation** 🟢
- [ ] API documentation (Doxygen)
- [ ] User manual with examples
- [ ] Tutorial for new users
- [ ] Troubleshooting guide

**Current:** Basic README
**Impact:** Community adoption
**Estimated Effort:** Low-Medium

---

## Implementation Roadmap

### Phase 1: Stabilization (Weeks 1-2)
1. Comprehensive unit testing
2. Error handling improvements
3. Input validation
4. Bug fixes

### Phase 2: Core Features (Weeks 3-6)
1. Advanced routing algorithms
2. Handover management
3. Link quality adaptation
4. Flow monitor integration

### Phase 3: Enhancement (Weeks 7-10)
1. Advanced traffic models
2. ISL topology improvements
3. Performance optimization
4. Visualization enhancements

### Phase 4: Polish (Weeks 11-12)
1. Documentation completion
2. Example scenarios
3. Beta testing
4. Release preparation

---

## Contributing

### Adding New Features

1. **Create a branch** from `main`
2. **Implement feature** following ns-3 coding standards
3. **Add tests** for new functionality
4. **Update documentation** (README, comments, Doxygen)
5. **Submit merge request** with description

### Code Structure

```
contrib/leosim/
├── model/                      # Core simulation models
│   ├── leosim-*.h/cc          # Model classes
│   └── leosim.h               # Module header
├── helper/                     # Helper classes for setup
│   ├── leosim-*-helper.h/cc   # Helper classes
├── examples/                   # Example programs
│   ├── leosim-example.cc      # Main example
├── test/                       # Unit tests
│   └── leosim-test-suite.cc
├── utils/                      # Data files and utilities
│   ├── satellite_mobility.tcl  # Satellite trace data
│   ├── ground_devices.csv      # Ground stations
│   └── *.py                    # Python utilities
├── CMakeLists.txt             # Build configuration
└── README.md                  # Module documentation
```

### Coding Standards

- Follow ns-3 coding style guide
- Use Doxygen comments for public APIs
- Include unit tests for new features
- Test with valgrind for memory leaks
- Update documentation and examples

---

## License

This module is released under the GNU General Public License v2.0.

```
Copyright (c) 2024 Anupa De Silva

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License version 2 as
published by the Free Software Foundation.
```

---

## Quick Reference

### Command-Line Options (leosim-example)

```
General:
  --satellites=FILE              Satellite mobility file (trace or CSV)
  --groundDevices=FILE           Ground devices CSV
  --simTime=SECONDS              Simulation duration (default: 60)
  --verbose=0|1                  Verbose logging (default: 1)
  --logPackets=0|1               Enable packet logging (default: 1)
  --useTrace=0|1                 Use trace format for satellites (default: 1)

Network:
  --numSatellites=N              Number of satellites (default: 300)
  --numServers=N                 Number of servers (default: 1)
  --numUes=N                     Number of UEs (default: 1)

Frequency & Range:
  --minElevation=DEG             Min elevation angle (default: 10)
  --frequency=HZ                 Ground link frequency (default: 12e9)
  --maxDistance=M                Max ground link distance

ISL (Inter-Satellite Links):
  --enableIsl=0|1                Enable ISL mesh (default: 1)
  --islFrequency=HZ              ISL frequency (default: 26e9)
  --islMaxDistance=M             ISL max distance (default: 5000000)
  --islTransmitPower=DBM         ISL transmit power (default: 30)
  --islAntennaGain=DB            ISL antenna gain (default: 35)

Routing:
  --enablePeriodicRouting=0|1    Dynamic routing (default: 1)
  --routingUpdateInterval=S      Route update interval (default: 10)

Logging:
  --positions=FILE               Position output file
  --links=FILE                   Link output file
  --packets=FILE                 Packet output file
  --logInterval=S                Log interval (default: 1)
  --pathPrintInterval=S          Path print interval (default: 10)
```

### Key Classes & Methods

```cpp
// Loader
LeoSimLoaderHelper::LoadSatellitesFromTrace(file)
LeoSimLoaderHelper::LoadSatellitesFromCsv(file)
LeoSimLoaderHelper::LoadGroundDevicesFromCsv(file)
LeoSimLoader::GetGroundDeviceIdsByType(type)
LeoSimLoader::GetNumSatellites()

// Channel
LeoSimChannelHelper::SetMinElevationAngle(degrees)
LeoSimChannelHelper::CreateChannels(sats, grounds)
LeoSimChannelModel::StartUpdates()

// Mobility
LeoSimMobilityHelper::InstallSatellite(node, id, name)
LeoSimMobilityHelper::InstallGateway(node, id, name, pos)

// Routing
LeoSimRoutingCalculatorHelper::CreateUnifiedRoutingCalculator(...)
LeoSimRoutingCalculator::ComputeRoute(source, dest, metric, pathType)
LeoSimRoutingCalculator::SetChannelModel(model)
LeoSimRoutingCalculator::SetIslChannelModel(islModel)
routingHelper.EnableDynamicRouting(calc, nodes, nodes, interval, ...)

// Traffic
LeoSimTrafficGeneratorHelper::InstallPingClient(node, addr, startTime, stopTime)
LeoSimTrafficGeneratorHelper::InstallPingClients(nodes, addr, startTime, stopTime)
LeoSimTrafficGeneratorHelper::InstallBidirectionalPing(node1, node2, addr1, addr2, ...)
LeoSimTrafficGeneratorHelper::InstallUeToServerPing(ueNodes, serverNode, addr, ...)

// Visualization
LeoSimVisualizationHelper::SchedulePositionLogging(...)
LeoSimVisualizationHelper::InstallPacketLogging(...)
```

---

## Support & Contact

For questions, bug reports, or feature requests, please contact the development team or open an issue in the project repository.

**Project Status:** Active Development ✓  
**Last Updated:** March 2026  
**Version:** ns-3.45  
**Status:** Active Development with Dijkstra Routing ✓
