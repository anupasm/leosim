# Inter-Satellite Link (ISL) Support in LeoSim

## Overview

LeoSim now includes comprehensive support for **Inter-Satellite Links (ISLs)**, enabling realistic modeling of communication between satellites in a LEO constellation. ISLs are critical for:

- Multi-hop routing through satellite networks
- Reducing reliance on ground stations
- Providing global coverage with fewer ground stations
- Improving latency for long-distance communication
- Creating resilient satellite mesh networks

## Key Features

### 1. ISL-Specific Channel Modeling

ISLs have fundamentally different characteristics compared to ground-to-satellite links:

- **No atmospheric attenuation**: ISLs operate in vacuum
- **No elevation angle constraints**: Satellites communicate peer-to-peer
- **Different frequency bands**: Typically Ka-band (26 GHz) or optical
- **Higher antenna gains**: Directional antennas for point-to-point links
- **Longer maximum distances**: Up to 5000+ km between satellites

### 2. Dual Channel Model

The LeoSim channel model now supports two distinct link types:

- `LEOSIM_LINK_SATELLITE_TO_GROUND`: Traditional satellite-to-ground links
- `LEOSIM_LINK_ISL`: Inter-Satellite Links with ISL-specific parameters

### 3. ISL Mesh Topology

Easy creation of full mesh connectivity between satellites:

```cpp
LeoSimChannelHelper islHelper;
islHelper.SetIslFrequency(26.0e9);        // Ka-band
islHelper.SetIslMaxDistance(5000000.0);    // 5000 km
islHelper.SetIslTransmitPower(30.0);       // 30 dBm
islHelper.SetIslAntennaGain(35.0);         // 35 dB high-gain antenna

Ptr<LeoSimChannelModel> islModel = islHelper.CreateIslMesh(satelliteNodes);
```

## Configuration Parameters

### ISL-Specific Parameters

| Parameter | Default Value | Description |
|-----------|--------------|-------------|
| `IslFrequency` | 26 GHz | Ka-band commonly used for ISL |
| `IslMaxDistance` | 5000 km | Maximum ISL range |
| `IslTransmitPower` | 30 dBm | Lower than ground links due to better link budget |
| `IslAntennaGain` | 30 dB | High-gain directional antennas |

### Ground Link Parameters

| Parameter | Default Value | Description |
|-----------|--------------|-------------|
| `Frequency` | 12 GHz | Ku-band for ground links |
| `MaxLinkDistance` | 2500 km | Maximum ground link range |
| `TransmitPower` | 40 dBm | Higher power for ground links |
| `MinElevationAngle` | 10° | Minimum elevation for ground visibility |

## Usage Examples

### Example 1: Basic ISL Mesh

```cpp
// Create satellite nodes
NodeContainer satellites;
satellites.Create(10);

// Configure ISL channel
LeoSimChannelHelper islHelper;
islHelper.SetIslFrequency(26.0e9);
islHelper.SetIslMaxDistance(5000000.0);

// Create full mesh of ISLs
Ptr<LeoSimChannelModel> islModel = islHelper.CreateIslMesh(satellites);
islModel->StartUpdates();
```

### Example 2: Combined Ground and ISL Links

```cpp
// Ground-to-satellite links
LeoSimChannelHelper groundHelper;
groundHelper.SetFrequency(12.0e9);
groundHelper.SetMinElevationAngle(10.0);
Ptr<LeoSimChannelModel> groundModel = groundHelper.CreateChannels(satellites, groundStations);

// ISL links
LeoSimChannelHelper islHelper;
islHelper.SetIslFrequency(26.0e9);
Ptr<LeoSimChannelModel> islModel = islHelper.CreateIslMesh(satellites);

// Start both channel updates
groundModel->StartUpdates();
islModel->StartUpdates();
```

### Example 3: Topology with ISL

```cpp
LeoSimTopologyHelper topoHelper;
topoHelper.SetFrequency(12.0e9);
topoHelper.SetDataRate("10Mb/s");

// Create ISL mesh topology (creates both channel model and network topology)
auto islAddresses = topoHelper.CreateIslMeshTopology(satellites, islChannelModel);

// Populate routing tables for multi-hop through ISL
topoHelper.PopulateRoutingTables();
```

### Example 4: ISL Link State Monitoring

```cpp
void OnIslStateChange(Ptr<Node> sat1, Ptr<Node> sat2, LeoSimLinkState state)
{
    if (state == LEOSIM_LINK_UP)
    {
        std::cout << "ISL UP between satellite " << sat1->GetId() 
                  << " and " << sat2->GetId() << std::endl;
    }
}

islHelper.InstallLinkStateChangeCallback(islModel, MakeCallback(&OnIslStateChange));
```

## Channel Quality Calculation

### ISL Link Budget

ISLs use free-space path loss without atmospheric attenuation:

```
FSPL (dB) = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
RSS (dBm) = TxPower + TxGain + RxGain - FSPL
SNR (dB) = RSS - NoisePower
```

Where:
- `d` = distance in meters
- `f` = ISL frequency in Hz
- `c` = speed of light
- No atmospheric loss is added for ISL

### Link State Determination

Links are classified based on SNR:

- `LEOSIM_LINK_UP`: SNR > 10 dB
- `LEOSIM_LINK_DEGRADED`: 0 dB < SNR ≤ 10 dB  
- `LEOSIM_LINK_DOWN`: SNR ≤ 0 dB or distance > max distance

## Example Scenarios

### Scenario 1: Global Coverage with Minimal Ground Stations

```
Ground Station A → Satellite 1 → Satellite 2 → ... → Satellite N → Ground Station B
```

ISLs enable communication between ground stations on opposite sides of the Earth.

### Scenario 2: Constellation Mesh Network

```
    Sat1 ─── Sat2
     │ ╲   ╱ │
     │  ╲ ╱  │  
     │  ╱ ╲  │
     │ ╱   ╲ │
    Sat3 ─── Sat4
```

Full mesh provides redundancy and multiple routing paths.

### Scenario 3: Orbital Plane ISL

Connect satellites in the same orbital plane for intra-plane routing:

```cpp
// Only connect adjacent satellites in same plane
for (uint32_t i = 0; i < satellitesInPlane.GetN() - 1; ++i)
{
    islHelper.AddIslLink(islModel, satellitesInPlane.Get(i), satellitesInPlane.Get(i+1));
}
```

## Performance Considerations

### ISL Advantages

1. **Better Link Budget**: No atmospheric attenuation
2. **Higher Data Rates**: Direct line-of-sight, minimal interference
3. **Reduced Latency**: Direct satellite-to-satellite communication
4. **Greater Range**: Can maintain links up to 5000 km

### ISL Challenges Modeled

1. **Distance Limitations**: Links break when satellites move too far apart
2. **Dynamic Topology**: Links constantly change as satellites orbit
3. **Routing Complexity**: Multi-hop routing required

## API Reference

### LeoSimChannelModel

```cpp
// Add single ISL
uint32_t AddIslLink(Ptr<Node> sat1, Ptr<Node> sat2);

// Create full ISL mesh
uint32_t CreateIslMesh(NodeContainer satellites);

// Configure ISL parameters
void SetIslMaxDistance(double distance);
void SetIslTransmitPower(double power);
void SetIslAntennaGain(double gain);
void SetIslFrequency(double frequency);
```

### LeoSimChannelHelper

```cpp
// Create ISL mesh
Ptr<LeoSimChannelModel> CreateIslMesh(NodeContainer satellites);

// Add ISLs to existing model
uint32_t AddIslLinks(Ptr<LeoSimChannelModel> model, NodeContainer satellites);
uint32_t AddIslLink(Ptr<LeoSimChannelModel> model, Ptr<Node> sat1, Ptr<Node> sat2);

// Configure ISL parameters
void SetIslMaxDistance(double distance);
void SetIslTransmitPower(double power);
void SetIslAntennaGain(double gain);
void SetIslFrequency(double frequency);
```

### LeoSimTopologyHelper

```cpp
// Create ISL mesh topology with routing
std::map<Ptr<Node>, std::vector<Ipv4Address>> CreateIslMeshTopology(
    const NodeContainer& satelliteNodes,
    Ptr<LeoSimChannelModel> channelModel = nullptr);
```

## Running the ISL Example

```bash
cd /home/anupa/UCD/LeoSim/ns3
./ns3 run "leosim-isl-example --simTime=600 --numSatellites=4 --verbose=true"
```

### Example Output

```
=== ISL Statistics at t=60s ===
Total ISL link up events: 6
Total ISL link down events: 0
Packets sent: 58
Packets received: 56
Active ISL links: 6 / 6

=== Simulation Complete ===
Total packets transmitted: 100
Total packets received: 98
Packet delivery ratio: 98%
```

## Future Enhancements

Potential future additions to ISL support:

1. **Optical ISL**: Model laser-based optical inter-satellite links
2. **Pointing Errors**: Simulate antenna misalignment effects
3. **Doppler Compensation**: Model frequency shifts for high-speed ISL
4. **Power Control**: Adaptive transmit power based on link quality
5. **Link Scheduling**: Time-division multiplexing for shared ISL resources
6. **Interference Modeling**: Co-channel interference between ISLs

## References

- ITU-R Recommendation S.1328: Satellite System Characteristics
- Ka-band ISL specifications: 23-27 GHz frequency range
- Typical ISL data rates: 1-10 Gbps
- LEO ISL ranges: 600-5000 km depending on constellation design

## See Also

- [leosim-isl-example.cc](../examples/leosim-isl-example.cc) - Complete ISL example
- [leosim-channel-model.h](../model/leosim-channel-model.h) - Channel model implementation
- [leosim-channel-helper.h](../helper/leosim-channel-helper.h) - Helper class API
- [leosim-topology-helper.h](../helper/leosim-topology-helper.h) - Topology creation
