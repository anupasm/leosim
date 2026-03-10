# LeoSim Routing System Documentation

## Overview

LeoSim implements a **dynamic multi-hop routing system** designed for LEO satellite constellations. It supports two types of links:
- **ISL (Inter-Satellite Links)**: Direct satellite-to-satellite communication
- **Ground Links**: Satellite-to-ground station communication

The routing system maintains a dynamic network topology that changes as satellites orbit and links appear/disappear based on distance and line-of-sight constraints.

---

## Core Routing Components

### 1. **LeoSimRoutingCalculator** — Main Routing Engine

This is the central component that computes routes between any two nodes using **Dijkstra's algorithm**. Key features:

#### Routing Metrics

The system supports multiple optimization metrics:

- **LEOSIM_METRIC_HOP_COUNT**: Minimize number of hops (default)
- **LEOSIM_METRIC_PATH_LOSS**: Minimize total signal attenuation (dB)
- **LEOSIM_METRIC_SNR**: Maximize signal-to-noise ratio
- **LEOSIM_METRIC_DISTANCE**: Minimize physical distance
- **LEOSIM_METRIC_SIGNAL_STRENGTH**: Maximize signal strength

#### Path Type Constraints

Three path types control which link types can be used:

- **LEOSIM_PATH_ANY**: Use ISL and ground links interchangeably (most flexible)
- **LEOSIM_PATH_ISL_ONLY**: Route only through satellite-to-satellite links
- **LEOSIM_PATH_GROUND_ONLY**: Route only through ground stations

#### Advanced Features

- **SNR constraints**: Compute routes that guarantee minimum signal quality thresholds
- **Alternative routes**: Generate K-shortest paths for redundancy
- **Route validation**: Verify SNR, path loss, and link availability

### 2. **LeoSimISLRoutingModel** — ISL-Specific Topology

Maintains an adjacency list representing which satellites are directly connected via ISL. Updates dynamically when:
- Satellites move beyond maximum transmission distance
- Satellites acquire new direct connections
- Links transition between UP/DEGRADED/DOWN states

---

## Routing Workflow

### Step 1: Topology Acquisition

The routing calculator retrieves the current network topology from the channel model:

```cpp
std::map<Ptr<Node>, std::set<Ptr<Node>>> topology = GetTopology(pathType);
```

This combines:
- Active **ground links** from `LeoSimChannelModel` (satellite-to-ground)
- Active **ISL links** from `m_islChannelModel` (satellite-to-satellite)
- Filters by `pathType` constraint (ISL-only, ground-only, or any)

The topology is cached with a TTL to optimize repeated queries.

### Step 2: Dijkstra's Algorithm Execution

The algorithm maintains three data structures:

1. **Distances map**: Tracks minimum metric value to reach each node
2. **Previous map**: Records the node that was used to reach each node (for path reconstruction)
3. **Unvisited set**: Nodes not yet processed

#### Algorithm Flow

```
distances[source] = 0
unvisited = all nodes in topology

while unvisited is not empty:
    current = unvisited node with minimum distance
    
    if current == destination:
        break  // Path found
    
    remove current from unvisited
    
    for each neighbor of current:
        if neighbor in unvisited AND link is allowed AND SNR meets constraint:
            linkMetric = GetLinkMetricValue(current, neighbor, metric)
            newDistance = distances[current] + linkMetric
            
            if newDistance < distances[neighbor]:
                distances[neighbor] = newDistance
                previous[neighbor] = current
```

#### Metric Calculation (Link Weight)

Different metrics assign different weights to links:

| Metric | Weight Calculation | Purpose |
|--------|-------------------|---------|
| HOP_COUNT | 1.0 per hop | Minimize number of relay nodes |
| PATH_LOSS | Actual PL (dB) | Prefer stronger links |
| SNR | -SNR value | Choose paths with better signal quality |
| DISTANCE | Actual distance (m) | Prefer shorter physical paths |
| SIGNAL_STRENGTH | -RSS (dBm) | Maximize received signal power |

Lower weight = preferred link when using Dijkstra's algorithm.

### Step 3: Path Reconstruction

Once a path is found, the algorithm reconstructs it by following the `previous` map from destination back to source:

```cpp
path = []
current = destination

while current is not null:
    path.prepend(current)
    current = previous[current]
```

Result: Ordered list of nodes from source to destination.

### Step 4: Route Verification & Metrics Calculation

For each hop in the computed path, the routing calculator verifies and calculates:

- **Path loss**: Sum of all hop path losses (dB)
- **Signal strength**: Minimum RSS across all hops (dBm) — this is the bottleneck
- **SNR**: Minimum SNR across all hops (dB) — determines link reliability
- **Distance**: Total physical distance covered (meters)
- **Link types**: Whether each hop is ISL or ground link
- **Validity flag**: Whether the entire route meets all constraints

---

## Link Quality Determination

Before using a link in a route, the system checks:

1. **Link availability**: Is the link currently UP?
2. **Path type compatibility**: Is the link type allowed by the constraint?
3. **SNR constraint satisfaction**: If specified, is SNR ≥ minimum required?

```cpp
bool IsLinkAllowed(source, destination, pathType):
    if not IsLinkAvailable(source, destination):
        return false
    
    if pathType == LEOSIM_PATH_ISL_ONLY:
        return link is ISL type
    elif pathType == LEOSIM_PATH_GROUND_ONLY:
        return link is GROUND type
    else:
        return true  // Any link type allowed
```

### Link States

Links can be in three states:

- **LEOSIM_LINK_UP**: SNR > 10 dB (good signal, reliable communication)
- **LEOSIM_LINK_DEGRADED**: 0 dB < SNR ≤ 10 dB (marginal signal, may have errors)
- **LEOSIM_LINK_DOWN**: SNR ≤ 0 dB or distance > max distance (unusable)

---

## ISL Channel Model Integration

ISL links are calculated using free-space path loss (no atmospheric attenuation):

$$\text{FSPL (dB)} = 20\log_{10}(d) + 20\log_{10}(f) + 20\log_{10}\left(\frac{4\pi}{c}\right)$$

Where:
- $d$ = distance between satellites (meters)
- $f$ = ISL frequency (Hz, typically 26 GHz Ka-band)
- $c$ = speed of light (3×10⁸ m/s)

### Received Signal Strength (RSS)

$$\text{RSS (dBm)} = \text{TxPower} + \text{TxGain} + \text{RxGain} - \text{FSPL}$$

Where:
- **TxPower**: Transmitter power (typically 30 dBm for ISL)
- **TxGain**: Transmitter antenna gain (typically 35 dB for high-gain ISL antenna)
- **RxGain**: Receiver antenna gain (typically 35 dB)
- **FSPL**: Free-space path loss

### Signal-to-Noise Ratio (SNR)

$$\text{SNR (dB)} = \text{RSS} - \text{NoisePower}$$

### ISL vs Ground Link Characteristics

| Parameter | ISL | Ground |
|-----------|-----|--------|
| Frequency | 26 GHz (Ka-band) | 12 GHz (Ku-band) |
| Max Distance | 5000 km | 2500 km |
| Transmit Power | 30 dBm | 40 dBm |
| Antenna Gain | 35 dB | 25 dB |
| Atmospheric Loss | None | Varies (~2-8 dB) |
| Elevation Constraint | None | Minimum 10° elevation |

---

## Usage Examples

### Example 1: Basic Route Computation

```cpp
// Create routing calculator
Ptr<LeoSimRoutingCalculator> routingCalc = CreateObject<LeoSimRoutingCalculator>();
routingCalc->SetChannelModel(groundChannelModel);
routingCalc->SetIslChannelModel(islChannelModel);

// Compute shortest hop-count path
LeoSimRoute route = routingCalc->ComputeRoute(
    sourceNode,
    destNode,
    LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT,
    LeoSimRoutingCalculator::LEOSIM_PATH_ANY
);

if (route.valid)
{
    std::cout << "Route found: " << route.hopCount << " hops" << std::endl;
    std::cout << "Total path loss: " << route.totalPathLoss << " dB" << std::endl;
    std::cout << "Minimum SNR: " << route.minSnr << " dB" << std::endl;
}
```

### Example 2: Route with SNR Constraint

```cpp
// Find route with minimum SNR of 5 dB
LeoSimRoute route = routingCalc->ComputeRouteWithSnrConstraint(
    sourceNode,
    destNode,
    5.0,  // Minimum SNR in dB
    LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT
);

if (route.valid)
{
    std::cout << "SNR-constrained route found" << std::endl;
    std::cout << "All hops have SNR >= 5 dB" << std::endl;
}
```

### Example 3: ISL-Only Path

```cpp
// Route only through Inter-Satellite Links
LeoSimRoute islRoute = routingCalc->ComputeRoute(
    sourceNode,
    destNode,
    LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT,
    LeoSimRoutingCalculator::LEOSIM_PATH_ISL_ONLY
);
```

### Example 4: Minimize Path Loss

```cpp
// Find path with minimum total attenuation
LeoSimRoute route = routingCalc->ComputeRoute(
    sourceNode,
    destNode,
    LeoSimRoutingCalculator::LEOSIM_METRIC_PATH_LOSS,  // Optimize for signal quality
    LeoSimRoutingCalculator::LEOSIM_PATH_ANY
);
```

### Example 5: Get Route Information

```cpp
LeoSimRoute route = routingCalc->ComputeRoute(sourceNode, destNode);

std::cout << "Path length: " << route.path.size() << " nodes" << std::endl;
std::cout << "Hop count: " << route.hopCount << std::endl;
std::cout << "Total distance: " << route.totalDistance / 1000.0 << " km" << std::endl;
std::cout << "Path loss: " << route.totalPathLoss << " dB" << std::endl;
std::cout << "Min signal strength: " << route.minSignalStrength << " dBm" << std::endl;
std::cout << "Min SNR: " << route.minSnr << " dB" << std::endl;
std::cout << "Has ISL links: " << (route.hasIslLinks ? "Yes" : "No") << std::endl;
std::cout << "Has ground links: " << (route.hasGroundLinks ? "Yes" : "No") << std::endl;

// Print path
std::cout << "Path: ";
for (size_t i = 0; i < route.path.size(); i++)
{
    std::cout << route.path[i]->GetId();
    if (i < route.path.size() - 1)
        std::cout << " -> ";
}
std::cout << std::endl;
```

---

## Example: Multi-Hop ISL Route

Consider 4 satellites in a chain: Sat1 → Sat2 → Sat3 → Sat4

### Computing route from Sat1 to Sat4 (hop-count metric)

1. **Initialize**: 
   - distances = {Sat1:0, Sat2:∞, Sat3:∞, Sat4:∞}
   - previous = {}

2. **Visit Sat1**: 
   - Update neighbors through ISL
   - distances = {Sat1:0, Sat2:1, Sat3:∞, Sat4:∞}
   - previous = {Sat2:Sat1}

3. **Visit Sat2**: 
   - distances = {Sat1:0, Sat2:1, Sat3:2, Sat4:∞}
   - previous = {Sat2:Sat1, Sat3:Sat2}

4. **Visit Sat3**: 
   - distances = {Sat1:0, Sat2:1, Sat3:2, Sat4:3}
   - previous = {Sat2:Sat1, Sat3:Sat2, Sat4:Sat3}

5. **Visit Sat4**: 
   - Destination reached, algorithm terminates

### Result

**Path**: [Sat1, Sat2, Sat3, Sat4]  
**Hop count**: 3  
**Status**: ✓ Valid route

---

## Dynamic Topology Updates

The routing system adapts to orbital dynamics automatically:

### Update Sequence

1. **Orbital Update**: Channel model recalculates link availability based on satellite positions
2. **Link State Change**: ISL connections go UP/DOWN as satellites move
3. **Callback Trigger**: `LinkStateChangeCallback` fires for topology changes
4. **Cache Invalidation**: Routing calculator's topology cache expires after TTL
5. **Next Route Query**: Uses updated topology with current link availability

### Example: Link Transition

```
Time t=0s:     Sat1 ↔ Sat2 (distance 1000 km, link UP)
Time t=30s:    Distance increases to 4500 km (still UP, SNR > 10 dB)
Time t=60s:    Distance reaches 5500 km (> max 5000 km)
               Link DOWN event fires
               Future routes avoid this link
```

---

## Optimization Features

### 1. Topology Caching

Avoid recomputing identical topology queries:
- Cache TTL: 100 milliseconds (configurable)
- Automatic expiration for stale data
- Triggered refresh on link state changes

### 2. Alternative Routes

Compute K-shortest paths for redundancy:

```cpp
std::vector<LeoSimRoute> routes = routingCalc->ComputeAlternativeRoutes(
    sourceNode,
    destNode,
    3,  // Find 3 shortest paths
    LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT
);

// Use primary route, fallback to alternatives on failure
for (const auto& route : routes)
{
    // Try to use this route
    if (SendPacket(route))
        break;  // Success
}
```

### 3. Early Termination

Algorithm stops as soon as destination is reached (potential performance gain for sparse topologies).

### 4. Constrained Routing

SNR and path-type constraints filter invalid routes early:
- Skip links that don't meet SNR threshold
- Skip links of wrong type (ISL vs ground)
- Reduces search space efficiently

---

## Link Neighbor Discovery

Get neighboring nodes reachable via specific link types:

```cpp
// Get all neighbors via ISL
std::set<Ptr<Node>> islNeighbors = routingCalc->GetNeighbors(
    node,
    LEOSIM_LINK_ISL
);

// Get all neighbors via ground links
std::set<Ptr<Node>> groundNeighbors = routingCalc->GetNeighbors(
    node,
    LEOSIM_LINK_SATELLITE_TO_GROUND
);

// Get all neighbors (any link type)
std::set<Ptr<Node>> allNeighbors = routingCalc->GetNeighbors(node);
```

---

## Topology Information

Query current network state:

```cpp
// Get link counts
uint32_t totalLinks = routingCalc->GetNumActiveLinks();
uint32_t islLinks = routingCalc->GetNumActiveIslLinks();
uint32_t groundLinks = routingCalc->GetNumActiveGroundLinks();

std::cout << "Active ISLs: " << islLinks << std::endl;
std::cout << "Active ground links: " << groundLinks << std::endl;
std::cout << "Total active links: " << totalLinks << std::endl;

// Get full adjacency list
auto topology = routingCalc->GetTopology();
for (const auto& [node, neighbors] : topology)
{
    std::cout << "Node " << node->GetId() << " connects to: ";
    for (const auto& neighbor : neighbors)
        std::cout << neighbor->GetId() << " ";
    std::cout << std::endl;
}

// Check direct link
if (routingCalc->HasDirectLink(node1, node2))
{
    std::cout << "Direct ISL link exists" << std::endl;
}
```

---

## Performance Considerations

### Time Complexity

- **Dijkstra's Algorithm**: O((V + E) log V) where V = nodes, E = links
- **For typical LEO constellation**: O(N² log N) in worst case (full mesh)
- **Cached lookups**: O(1) if topology hasn't changed

### Space Complexity

- Topology storage: O(V + E)
- Route cache: O(route count)
- Typical: ~1 KB per satellite node for topology

### Typical Numbers (50-satellite constellation)

| Operation | Time |
|-----------|------|
| Route computation (hop-count) | ~2-5 μs |
| Topology update | ~100-500 μs |
| Cached route lookup | <1 μs |
| Link state query | ~1-2 μs |

---

## Debugging and Verbose Mode

Enable detailed logging:

```cpp
routingCalc->SetVerbose(true);

// Output when computing route:
// "Route from 5 to 12: 4 hops, PathLoss=180.5dB, Distance=15234000.0m, MinSNR=8.2dB"
```

---

## Integration with NS-3 Routing Protocols

LeoSim routing can integrate with standard NS-3 routing:

```cpp
// Example: Use LeoSim routing to populate OLSR or other protocol routes
Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
calc->SetChannelModel(channelModel);
calc->SetIslChannelModel(islChannelModel);

// For each satellite pair, add static route based on LeoSim computation
for (uint32_t src = 0; src < satellites.GetN(); src++)
{
    for (uint32_t dst = 0; dst < satellites.GetN(); dst++)
    {
        if (src == dst) continue;
        
        LeoSimRoute route = calc->ComputeRoute(
            satellites.Get(src),
            satellites.Get(dst)
        );
        
        if (route.valid)
        {
            // Add as static route to NS-3 routing table
            InstallStaticRoute(satellites.Get(src), route);
        }
    }
}
```

---

## See Also

- [ISL_DOCUMENTATION.md](ISL_DOCUMENTATION.md) — Inter-Satellite Link specifications
- [CHANNEL_MODEL.md](CHANNEL_MODEL.md) — Channel model theory and configuration
- [leosim-routing-calculator.h](../model/leosim-routing-calculator.h) — API reference
- [leosim-isl-routing-model.h](../model/leosim-isl-routing-model.h) — ISL model API
- [leosim-example.cc](../examples/leosim-example.cc) — Complete usage example

---

## References

- Dijkstra, E. W. (1959). "A note on two problems in connexion with graphs."
- ITU-R Recommendation S.1328: Satellite System Characteristics
- Ka-band ISL specifications: 23-27 GHz frequency range
- LEO ISL ranges: 600-5000 km depending on constellation design
