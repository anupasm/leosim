# LeoSim Dynamic Channel Model

## Overview

The LeoSim dynamic channel model provides realistic satellite-to-ground channel simulation for LEO satellite networks. It computes link quality based on physical properties including distance, elevation angle, atmospheric conditions, and propagation characteristics.

## Features

- **Dynamic Link Management**: Automatically tracks and updates links between satellites and ground nodes
- **Distance-based Path Loss**: Free space path loss calculation using physical parameters
- **Elevation Angle Constraints**: Enforces minimum elevation angle for ground-to-satellite visibility
- **Atmospheric Attenuation**: Optional atmospheric loss modeling for realistic channel conditions
- **Link State Tracking**: Monitors link states (UP, DOWN, DEGRADED) based on SNR thresholds
- **Periodic Updates**: Configurable update intervals for channel recalculation
- **Trace Callbacks**: Link state change and path loss callbacks for monitoring

## Architecture

### LeoSimChannelModel

The core model that manages channel quality calculations:

- Tracks links between node pairs
- Calculates free space path loss
- Computes elevation angles
- Applies atmospheric attenuation
- Updates link states based on SNR
- Provides channel quality metrics

### LeoSimChannelHelper

Helper class for easy channel setup:

- Simplified API for creating channels
- Automatic link creation between satellite and ground nodes
- Pre-configured channel parameters
- Support for custom topologies
- Callback installation utilities

## Usage

### Basic Usage

```cpp
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-channel-model.h"

// Create nodes with mobility models
NodeContainer satellites;
NodeContainer groundNodes;
// ... create and configure nodes ...

// Create channel helper
LeoSimChannelHelper channelHelper;
channelHelper.SetMinElevationAngle(10.0);      // 10 degrees minimum
channelHelper.SetFrequency(12.0e9);            // 12 GHz (Ku-band)
channelHelper.SetTransmitPower(40.0);          // 40 dBm
channelHelper.SetUpdateInterval(Seconds(1.0)); // Update every second

// Create channels between all satellites and ground nodes
Ptr<LeoSimChannelModel> channelModel = 
    channelHelper.CreateChannels(satellites, groundNodes);

// Start periodic updates
channelModel->StartUpdates();
```

### Advanced Configuration

```cpp
// Separate gateway and UE channels
Ptr<LeoSimChannelModel> gwChannels = 
    channelHelper.CreateSatelliteToGatewayChannels(satellites, gateways);
Ptr<LeoSimChannelModel> ueChannels = 
    channelHelper.CreateSatelliteToUeChannels(satellites, ues);

// Custom topology
std::map<uint32_t, std::vector<uint32_t>> topology;
topology[0] = {0, 1, 2}; // Satellite 0 connects to ground nodes 0, 1, 2
topology[1] = {2, 3};    // Satellite 1 connects to ground nodes 2, 3

Ptr<LeoSimChannelModel> customChannel = 
    channelHelper.CreateCustomTopology(topology, satellites, groundNodes);
```

### Installing Callbacks

```cpp
void OnLinkStateChange(Ptr<Node> node1, Ptr<Node> node2, LeoSimLinkState state)
{
    // Handle link state changes
    std::cout << "Link " << node1->GetId() << "->" << node2->GetId() 
              << " changed to " << state << std::endl;
}

// Install callback
channelHelper.InstallLinkStateChangeCallback(
    channelModel, 
    MakeCallback(&OnLinkStateChange)
);
```

### Querying Channel Quality

```cpp
// Get channel quality metrics
LeoSimChannelQuality quality = 
    channelModel->GetChannelQuality(satellite, groundStation);

std::cout << "Distance: " << quality.distance / 1000.0 << " km\n"
          << "Path Loss: " << quality.pathLoss << " dB\n"
          << "Elevation: " << quality.elevationAngle << " degrees\n"
          << "SNR: " << quality.snr << " dB\n"
          << "Link State: " << quality.linkState << std::endl;

// Check if link is available
if (channelModel->IsLinkUp(satellite, groundStation))
{
    // Transmit data
}
```

### Printing Statistics

```cpp
// Print detailed channel statistics
channelHelper.PrintChannelStatistics(channelModel);

// Get all active links
auto activeLinks = channelModel->GetActiveLinks();
std::cout << "Active links: " << activeLinks.size() << std::endl;
```

## Channel Parameters

### Configurable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| MinElevationAngle | 10.0° | Minimum elevation angle for ground-to-satellite links |
| MaxLinkDistance | 2500 km | Maximum link distance |
| Frequency | 12 GHz | Carrier frequency (Ku-band) |
| TransmitPower | 40 dBm | Transmit power |
| NoiseTemperature | 290 K | System noise temperature |
| AtmosphericAttenuation | Enabled | Enable/disable atmospheric loss model |
| UpdateInterval | 1.0 s | Interval for periodic channel updates |

### Link State Thresholds

- **UP**: SNR > 10 dB
- **DEGRADED**: 0 dB < SNR ≤ 10 dB
- **DOWN**: SNR ≤ 0 dB or constraint violated

## Channel Quality Metrics

The `LeoSimChannelQuality` structure provides:

- `distance`: Distance between nodes (meters)
- `pathLoss`: Total path loss (dB)
- `elevationAngle`: Elevation angle from ground node (degrees)
- `signalStrength`: Received signal strength (dBm)
- `snr`: Signal-to-noise ratio (dB)
- `linkState`: Current link state (UP/DOWN/DEGRADED)
- `lastUpdate`: Time of last update

## Example Program

See `contrib/leosim/examples/leosim-channel-example.cc` for a complete example demonstrating:

- Channel creation between satellites and ground nodes
- Periodic channel statistics
- Link state change monitoring
- Channel quality logging

Run the example:

```bash
./ns3 run "leosim-channel-example --simTime=60 --statsInterval=10 --verbose=true"
```

## Channel Model Details

### Free Space Path Loss

The model uses the standard free space path loss equation:

```
FSPL = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
```

Where:
- d = distance (meters)
- f = frequency (Hz)
- c = speed of light (m/s)

### Atmospheric Attenuation

When enabled, the model adds atmospheric attenuation based on:
- Slant path through atmosphere
- Elevation angle
- Frequency-dependent loss

### Elevation Angle Calculation

Elevation angle is computed considering Earth's curvature:
1. Calculate line-of-sight vector from ground to satellite
2. Determine local vertical (ground position normal)
3. Compute angle between LOS and horizontal plane

### SNR Calculation

```
SNR = TransmitPower - PathLoss - NoiseFloor
```

Where noise floor is calculated from system noise temperature and bandwidth.

## Integration with Other Modules

The channel model integrates seamlessly with:
- **LeoSimMobilityModel**: Uses node positions for distance/angle calculations
- **ns-3 Network Module**: Compatible with standard ns-3 nodes
- **Propagation Models**: Can be extended with existing ns-3 propagation models

## Future Enhancements

Potential improvements include:
- Rain fade modeling (ITU-R P.618)
- Doppler shift calculation
- Handover prediction
- Multi-path fading
- Interference modeling
- Link budget analysis tools

## References

- ITU-R P.676: Attenuation by atmospheric gases
- ITU-R P.618: Propagation data for satellite systems
- ns-3 Propagation Module documentation
