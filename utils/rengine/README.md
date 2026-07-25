# LeoSim Routing Engine

Standalone thread-parallel routing engine for LeoSim. It performs graph
routing over binary CSR sparse weighted matrices and has no ns-3 dependency.

## Build

```sh
cd ns3/contrib/leosim/utils/rengine
make
make self-test
```

## Runtime Model

The ns-3 process should:

1. Export a graph snapshot to `*.graph`.
2. Export route requests to `*.requests`.
3. Run this engine as a supervisor.
4. Import the `*.results` file on the simulator thread.
5. Install only changed `Ipv4StaticRouting` entries.

The supervisor reads each snapshot once, constructs one shared reverse graph,
and uses a dynamically scheduled thread pool. Workers operate only on immutable
graph data and private shortest-path scratch arrays; they never touch ns-3
objects. The `--worker` command remains available for compatibility, but normal
`--workers N` operation does not fork child processes.

```sh
./leosim-rengine \
  --graph /tmp/leosim-routing/snapshot-123.graph \
  --requests /tmp/leosim-routing/snapshot-123.requests \
  --result-prefix /tmp/leosim-routing/snapshot-123 \
  --results /tmp/leosim-routing/snapshot-123.results \
  --workers 40
```

## Files

- `rengine-format.h`: fixed binary ABI shared with the future ns-3 exporter.
- `leosim-rengine.cc`: supervisor, thread pool, CSR loader, BFS/Dijkstra routing.
- `Makefile`: standalone build.

## Routing Modes

The request file selects the mode:

- `PAIR`: each request is routed independently from `src` to `dst` and is
  dynamically scheduled across the worker threads.
- `DESTINATION_TREE`: requests are grouped by destination. Each worker runs
  reverse Dijkstra for dynamically assigned destinations and emits next-hop
  results for all requested sources. Hop-count trees use reverse BFS instead.

`DESTINATION_TREE` is the default architecture for many sources routing to a
smaller set of gateways or servers.

## Binary Format

The graph file contains:

```text
GraphHeader
uint64_t rowOffsets[nodeCount + 1]
uint32_t colIndices[edgeCount]
float    weights[edgeCount]
```

The request file contains:

```text
RequestHeader
RouteRequest[requestCount]
```

The result file contains:

```text
ResultHeader
RouteResult[resultCount]
```

`RouteResult` returns only `src`, `dst`, `nextHop`, `cost`, `hopCount`, and
`valid`. ns-3 maps `nextHop` to gateway IP/interface locally.

## How To Use With ns-3

This engine is designed to be called from a LeoSim routing update event. The
engine is not linked into ns-3 and workers must never receive ns-3 objects such
as `Ptr<Node>`, `Ipv4`, `Simulator`, or `LeoSimChannelModel`.

LeoSim provides `LeoSimExternalRoutingHelper` as the simulator-side integration
point. It exports snapshots, invokes this engine, imports results, and updates
`Ipv4StaticRouting`.

Minimal usage:

```cpp
#include "ns3/leosim.h"

LeoSimRoutingCalculatorHelper routingFactory;
Ptr<LeoSimRoutingCalculator> calculator =
    routingFactory.CreateUnifiedRoutingCalculator(groundChannelModel,
                                                  islChannelModel,
                                                  false);

LeoSimExternalRoutingHelper externalRouting;
externalRouting.SetEnginePath("contrib/leosim/utils/rengine/leosim-rengine");
externalRouting.SetWorkingDirectory("/tmp/leosim-routing");
externalRouting.SetWorkerCount(40);
externalRouting.SetMode(LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_DESTINATION_TREE);
externalRouting.SetMetric(LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_DISTANCE);

externalRouting.SetStaticRoutes(calculator, sources, destinations, true);
```

Periodic dynamic routing:

```cpp
externalRouting.EnableDynamicRouting(calculator,
                                     sources,
                                     destinations,
                                     Seconds(5.0),
                                     stopTime,
                                     true);
```

Build the external engine before running a simulation that uses the helper:

```sh
cd ns3/contrib/leosim/utils/rengine
make
```

The ns-3 side should do all simulator interaction in this order:

1. Build the current LeoSim topology on the simulator thread.
2. Convert valid links into CSR arrays.
3. Write `snapshot-N.graph`.
4. Write `snapshot-N.requests`.
5. Launch `leosim-rengine`.
6. Wait/poll for `snapshot-N.results`.
7. Import results on the simulator thread.
8. Install only changed static routes.

Recommended event flow:

```text
Simulator::Schedule(updateInterval, RoutingUpdate)

RoutingUpdate:
  channelModel->UpdateAllLinks()
  export graph + requests
  launch rengine supervisor process
  Simulator::Schedule(pollInterval, CheckRoutingResult)

CheckRoutingResult:
  if result is not ready:
      reschedule CheckRoutingResult
  else:
      verify snapshotId
      load RouteResult records
      diff with previous next-hop table
      update Ipv4StaticRouting
      schedule next RoutingUpdate
```

The exporter should include only links that routing is allowed to use. For
example, beam-manager serving-link policy, CHO candidate policy, operator
sharing constraints, weather penalties, and down/degraded link handling should
already be reflected in the exported edge list and weights.

### Exporting The Graph

Map ns-3 node IDs to dense routing IDs before writing CSR. The engine expects
node IDs in the range `[0, nodeCount)`.

```cpp
std::unordered_map<uint32_t, uint32_t> nodeIdToRoutingId;
std::vector<uint32_t> routingIdToNodeId;
```

For every allowed directed edge:

```cpp
uint32_t u = nodeIdToRoutingId[srcNode->GetId()];
uint32_t v = nodeIdToRoutingId[dstNode->GetId()];
float weight = ComputeRoutingWeight(srcNode, dstNode);

adj[u].push_back({v, weight});
```

For undirected links, write both directions:

```cpp
adj[u].push_back({v, weightUv});
adj[v].push_back({u, weightVu});
```

Then flatten to CSR:

```cpp
rowOffsets.resize(nodeCount + 1);
for (uint32_t u = 0; u < nodeCount; ++u)
{
    rowOffsets[u + 1] = rowOffsets[u] + adj[u].size();
    for (const auto& edge : adj[u])
    {
        colIndices.push_back(edge.to);
        weights.push_back(edge.weight);
    }
}
```

Write the file using the structs in `rengine-format.h`:

```cpp
GraphHeader header{};
header.magic = GRAPH_MAGIC;
header.version = FORMAT_VERSION;
header.snapshotId = snapshotId;
header.simTimeSeconds = Simulator::Now().GetSeconds();
header.nodeCount = nodeCount;
header.edgeCount = colIndices.size();
header.rowOffsetCount = rowOffsets.size();
header.colIndexCount = colIndices.size();
header.weightCount = weights.size();
```

### Exporting Requests

Use `DESTINATION_TREE` when many sources route to a smaller destination set
such as gateways or servers:

```cpp
RequestHeader header{};
header.magic = REQUEST_MAGIC;
header.version = FORMAT_VERSION;
header.snapshotId = snapshotId;
header.requestCount = requests.size();
header.mode = static_cast<uint32_t>(RoutingMode::DESTINATION_TREE);
header.metric = static_cast<uint32_t>(RoutingMetric::WEIGHT);
```

Each request uses dense routing IDs:

```cpp
RouteRequest req{};
req.src = nodeIdToRoutingId[sourceNode->GetId()];
req.dst = nodeIdToRoutingId[destinationNode->GetId()];
```

### Launching The Engine

Initial integration can use a blocking child process if the simulation is
allowed to pause during routing updates:

```cpp
std::string cmd =
    "contrib/leosim/utils/rengine/leosim-rengine"
    " --graph " + graphPath +
    " --requests " + requestPath +
    " --result-prefix " + resultPrefix +
    " --results " + resultPath +
    " --workers 40";

int rc = std::system(cmd.c_str());
```

For long simulations, prefer non-blocking launch plus polling from scheduled
events. The important rule is the same: route installation must happen inside
the ns-3 simulator thread after results are complete.

### Importing Results

Reject stale results:

```cpp
ResultHeader header;
read(header);
if (header.snapshotId != expectedSnapshotId)
{
    return; // stale result
}
```

Convert dense routing IDs back to ns-3 node IDs:

```cpp
uint32_t srcNodeId = routingIdToNodeId[result.src];
uint32_t dstNodeId = routingIdToNodeId[result.dst];
uint32_t nextHopNodeId = routingIdToNodeId[result.nextHop];
```

Then find the gateway IP/interface exactly as current
`LeoSimRoutingCalculatorHelper::SetStaticRoutes()` does: locate the source
interface and next-hop interface on the same subnet, then install a `/32` host
route for each destination address.

Install only route changes:

```text
old: src,dst -> nextHop A
new: src,dst -> nextHop B

if A != B:
  remove old computed /32 route
  add new /32 route through B
```

### Operational Defaults

For a 40-core machine with at least 40 unique destinations:

```text
workers = 40
mode = DESTINATION_TREE
metric = WEIGHT
graph = sparse CSR
result apply = diff only
fallback = existing in-process LeoSimRoutingCalculator
```

Use `PAIR` mode only when the request set is small or irregular. Use
`DESTINATION_TREE` for gateway/server traffic and large satellite populations.
