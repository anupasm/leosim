/*
 * Standalone LeoSim routing engine.
 *
 * This process performs pure graph routing over a binary CSR sparse matrix.
 * It must not include or call ns-3 APIs. ns-3 should export snapshots and
 * import RouteResult records on the simulator thread.
 */

#include "rengine-format.h"

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace leosim_rengine
{
namespace
{

struct Options
{
    std::string graphPath;
    std::string requestPath;
    std::string resultPath;
    std::string resultPrefix;
    std::string executablePath;
    uint32_t workers = 1;
    uint32_t workerId = 0;
    uint32_t workerCount = 1;
    bool worker = false;
    bool selfTest = false;
};

struct SparseGraph
{
    uint64_t snapshotId = 0;
    double simTimeSeconds = 0.0;
    uint32_t nodeCount = 0;
    std::vector<uint64_t> rowOffsets;
    std::vector<uint32_t> colIndices;
    std::vector<float> weights;
};

struct Requests
{
    uint64_t snapshotId = 0;
    RoutingMode mode = RoutingMode::DESTINATION_TREE;
    RoutingMetric metric = RoutingMetric::WEIGHT;
    std::vector<RouteRequest> requests;
};

struct DestinationWork
{
    uint32_t dst = INVALID_NODE;
    std::vector<size_t> requestIndices;
};

struct TreeScratch
{
    explicit TreeScratch(uint32_t nodeCount)
        : dist(nodeCount),
          nextHop(nodeCount),
          hops(nodeCount),
          bfsQueue(nodeCount)
    {
    }

    std::vector<float> dist;
    std::vector<uint32_t> nextHop;
    std::vector<uint16_t> hops;
    std::vector<uint32_t> bfsQueue;
};

[[noreturn]] void
Fail(const std::string& message)
{
    throw std::runtime_error(message);
}

template <typename T>
void
ReadExact(std::ifstream& input, T* data, size_t count)
{
    input.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!input)
    {
        Fail("short read while loading binary file");
    }
}

template <typename T>
void
WriteExact(std::ofstream& output, const T* data, size_t count)
{
    output.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(sizeof(T) * count));
    if (!output)
    {
        Fail("short write while writing binary file");
    }
}

SparseGraph
ReadGraph(const std::string& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        Fail("cannot open graph file: " + path);
    }

    GraphHeader header{};
    ReadExact(input, &header, 1);
    if (header.magic != GRAPH_MAGIC || header.version != FORMAT_VERSION)
    {
        Fail("invalid graph header: " + path);
    }
    if (header.rowOffsetCount != static_cast<uint64_t>(header.nodeCount) + 1 ||
        header.colIndexCount != header.edgeCount ||
        header.weightCount != header.edgeCount)
    {
        Fail("inconsistent graph dimensions: " + path);
    }

    SparseGraph graph;
    graph.snapshotId = header.snapshotId;
    graph.simTimeSeconds = header.simTimeSeconds;
    graph.nodeCount = header.nodeCount;
    graph.rowOffsets.resize(static_cast<size_t>(header.rowOffsetCount));
    graph.colIndices.resize(header.edgeCount);
    graph.weights.resize(header.edgeCount);

    ReadExact(input, graph.rowOffsets.data(), graph.rowOffsets.size());
    ReadExact(input, graph.colIndices.data(), graph.colIndices.size());
    ReadExact(input, graph.weights.data(), graph.weights.size());

    if (graph.rowOffsets.empty() || graph.rowOffsets.back() != graph.colIndices.size())
    {
        Fail("CSR row offsets do not match edge count");
    }
    for (uint32_t node = 0; node < graph.nodeCount; ++node)
    {
        if (graph.rowOffsets[node] > graph.rowOffsets[node + 1] ||
            graph.rowOffsets[node + 1] > graph.colIndices.size())
        {
            Fail("CSR row offsets are not monotonic");
        }
    }
    for (uint32_t edge = 0; edge < graph.colIndices.size(); ++edge)
    {
        if (graph.colIndices[edge] >= graph.nodeCount)
        {
            Fail("CSR edge points outside node range");
        }
        if (!std::isfinite(graph.weights[edge]) || graph.weights[edge] < 0.0f)
        {
            Fail("CSR edge has invalid weight");
        }
    }

    return graph;
}

Requests
ReadRequests(const std::string& path)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        Fail("cannot open request file: " + path);
    }

    RequestHeader header{};
    ReadExact(input, &header, 1);
    if (header.magic != REQUEST_MAGIC || header.version != FORMAT_VERSION)
    {
        Fail("invalid request header: " + path);
    }
    if (header.mode != static_cast<uint32_t>(RoutingMode::PAIR) &&
        header.mode != static_cast<uint32_t>(RoutingMode::DESTINATION_TREE))
    {
        Fail("invalid routing mode in request file");
    }

    Requests requests;
    requests.snapshotId = header.snapshotId;
    requests.mode = static_cast<RoutingMode>(header.mode);
    requests.metric = static_cast<RoutingMetric>(header.metric);
    requests.requests.resize(header.requestCount);
    ReadExact(input, requests.requests.data(), requests.requests.size());
    return requests;
}

void
WriteResults(const std::string& path,
             uint64_t snapshotId,
             RoutingMode mode,
             RoutingMetric metric,
             uint32_t workerId,
             const std::vector<RouteResult>& results)
{
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
    {
        Fail("cannot open result file for write: " + path);
    }

    ResultHeader header{};
    header.magic = RESULT_MAGIC;
    header.version = FORMAT_VERSION;
    header.snapshotId = snapshotId;
    header.resultCount = static_cast<uint32_t>(results.size());
    header.mode = static_cast<uint32_t>(mode);
    header.metric = static_cast<uint32_t>(metric);
    header.workerId = workerId;

    WriteExact(output, &header, 1);
    WriteExact(output, results.data(), results.size());
}

std::vector<RouteResult>
ReadResults(const std::string& path, uint64_t snapshotId)
{
    std::ifstream input(path, std::ios::binary);
    if (!input)
    {
        Fail("cannot open worker result file: " + path);
    }

    ResultHeader header{};
    ReadExact(input, &header, 1);
    if (header.magic != RESULT_MAGIC || header.version != FORMAT_VERSION)
    {
        Fail("invalid result header: " + path);
    }
    if (header.snapshotId != snapshotId)
    {
        Fail("stale result file rejected: " + path);
    }

    std::vector<RouteResult> results(header.resultCount);
    ReadExact(input, results.data(), results.size());
    return results;
}

SparseGraph
BuildReverseGraph(const SparseGraph& graph)
{
    SparseGraph reverse;
    reverse.snapshotId = graph.snapshotId;
    reverse.simTimeSeconds = graph.simTimeSeconds;
    reverse.nodeCount = graph.nodeCount;
    reverse.rowOffsets.assign(static_cast<size_t>(graph.nodeCount) + 1, 0);
    reverse.colIndices.resize(graph.colIndices.size());
    reverse.weights.resize(graph.weights.size());

    for (uint32_t src = 0; src < graph.nodeCount; ++src)
    {
        for (uint64_t edge = graph.rowOffsets[src]; edge < graph.rowOffsets[src + 1]; ++edge)
        {
            ++reverse.rowOffsets[graph.colIndices[edge] + 1];
        }
    }
    for (uint32_t node = 1; node <= graph.nodeCount; ++node)
    {
        reverse.rowOffsets[node] += reverse.rowOffsets[node - 1];
    }

    std::vector<uint64_t> cursor = reverse.rowOffsets;
    for (uint32_t src = 0; src < graph.nodeCount; ++src)
    {
        for (uint64_t edge = graph.rowOffsets[src]; edge < graph.rowOffsets[src + 1]; ++edge)
        {
            uint32_t dst = graph.colIndices[edge];
            uint64_t out = cursor[dst]++;
            reverse.colIndices[out] = src;
            reverse.weights[out] = graph.weights[edge];
        }
    }

    return reverse;
}

float
EdgeCost(float weight, RoutingMetric metric)
{
    return metric == RoutingMetric::HOP_COUNT ? 1.0f : weight;
}

RouteResult
ComputePairRoute(const SparseGraph& graph,
                 uint32_t src,
                 uint32_t dst,
                 RoutingMetric metric)
{
    RouteResult result{};
    result.src = src;
    result.dst = dst;
    result.nextHop = INVALID_NODE;
    result.cost = std::numeric_limits<float>::infinity();
    result.hopCount = 0;
    result.valid = 0;

    if (src >= graph.nodeCount || dst >= graph.nodeCount || src == dst)
    {
        return result;
    }

    using QueueEntry = std::pair<float, uint32_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    std::vector<float> dist(graph.nodeCount, std::numeric_limits<float>::infinity());
    std::vector<uint32_t> parent(graph.nodeCount, INVALID_NODE);

    dist[src] = 0.0f;
    queue.push({0.0f, src});
    while (!queue.empty())
    {
        auto [cost, node] = queue.top();
        queue.pop();
        if (cost > dist[node])
        {
            continue;
        }
        if (node == dst)
        {
            break;
        }
        for (uint64_t edge = graph.rowOffsets[node]; edge < graph.rowOffsets[node + 1]; ++edge)
        {
            uint32_t next = graph.colIndices[edge];
            float nextCost = cost + EdgeCost(graph.weights[edge], metric);
            if (nextCost < dist[next])
            {
                dist[next] = nextCost;
                parent[next] = node;
                queue.push({nextCost, next});
            }
        }
    }

    if (!std::isfinite(dist[dst]))
    {
        return result;
    }

    uint32_t current = dst;
    uint16_t hops = 0;
    while (parent[current] != INVALID_NODE && parent[current] != src)
    {
        current = parent[current];
        if (++hops == std::numeric_limits<uint16_t>::max())
        {
            return result;
        }
    }
    if (parent[current] != src)
    {
        return result;
    }

    result.nextHop = current;
    result.cost = dist[dst];
    result.hopCount = static_cast<uint16_t>(hops + 1);
    result.valid = 1;
    return result;
}

std::vector<RouteResult>
ComputeDestinationTree(const SparseGraph& reverseGraph,
                       const std::vector<RouteRequest>& requests,
                       uint32_t dst,
                       RoutingMetric metric)
{
    using QueueEntry = std::pair<float, uint32_t>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
    std::vector<float> dist(reverseGraph.nodeCount, std::numeric_limits<float>::infinity());
    std::vector<uint32_t> nextHop(reverseGraph.nodeCount, INVALID_NODE);
    std::vector<uint16_t> hops(reverseGraph.nodeCount, 0);

    dist[dst] = 0.0f;
    nextHop[dst] = dst;
    queue.push({0.0f, dst});

    while (!queue.empty())
    {
        auto [cost, node] = queue.top();
        queue.pop();
        if (cost > dist[node])
        {
            continue;
        }

        for (uint64_t edge = reverseGraph.rowOffsets[node]; edge < reverseGraph.rowOffsets[node + 1]; ++edge)
        {
            uint32_t predecessor = reverseGraph.colIndices[edge];
            float nextCost = cost + EdgeCost(reverseGraph.weights[edge], metric);
            if (nextCost < dist[predecessor])
            {
                dist[predecessor] = nextCost;
                nextHop[predecessor] = node;
                hops[predecessor] = static_cast<uint16_t>(
                    std::min<uint32_t>(static_cast<uint32_t>(hops[node]) + 1,
                                       std::numeric_limits<uint16_t>::max()));
                queue.push({nextCost, predecessor});
            }
        }
    }

    std::vector<RouteResult> results;
    results.reserve(requests.size());
    for (const auto& request : requests)
    {
        if (request.dst != dst)
        {
            continue;
        }
        RouteResult result{};
        result.src = request.src;
        result.dst = request.dst;
        result.nextHop = INVALID_NODE;
        result.cost = std::numeric_limits<float>::infinity();
        result.valid = 0;
        if (request.src < reverseGraph.nodeCount &&
            std::isfinite(dist[request.src]) &&
            request.src != request.dst)
        {
            result.nextHop = nextHop[request.src];
            result.cost = dist[request.src];
            result.hopCount = hops[request.src];
            result.valid = result.nextHop == INVALID_NODE ? 0 : 1;
        }
        results.push_back(result);
    }
    return results;
}

RouteResult
InvalidResult(const RouteRequest& request)
{
    RouteResult result{};
    result.src = request.src;
    result.dst = request.dst;
    result.nextHop = INVALID_NODE;
    result.cost = std::numeric_limits<float>::infinity();
    result.hopCount = 0;
    result.valid = 0;
    return result;
}

void
ComputeDestinationWork(const SparseGraph& reverseGraph,
                       const Requests& requests,
                       const DestinationWork& work,
                       TreeScratch& scratch,
                       std::vector<RouteResult>& results)
{
    if (work.dst >= reverseGraph.nodeCount)
    {
        return;
    }

    const float infinity = std::numeric_limits<float>::infinity();
    std::fill(scratch.dist.begin(), scratch.dist.end(), infinity);
    std::fill(scratch.nextHop.begin(), scratch.nextHop.end(), INVALID_NODE);
    std::fill(scratch.hops.begin(), scratch.hops.end(), 0);

    scratch.dist[work.dst] = 0.0f;
    scratch.nextHop[work.dst] = work.dst;

    if (requests.metric == RoutingMetric::HOP_COUNT)
    {
        size_t head = 0;
        size_t tail = 0;
        scratch.bfsQueue[tail++] = work.dst;
        while (head < tail)
        {
            const uint32_t node = scratch.bfsQueue[head++];
            for (uint64_t edge = reverseGraph.rowOffsets[node];
                 edge < reverseGraph.rowOffsets[node + 1];
                 ++edge)
            {
                const uint32_t predecessor = reverseGraph.colIndices[edge];
                if (std::isfinite(scratch.dist[predecessor]))
                {
                    continue;
                }
                scratch.dist[predecessor] = scratch.dist[node] + 1.0f;
                scratch.nextHop[predecessor] = node;
                scratch.hops[predecessor] = static_cast<uint16_t>(
                    std::min<uint32_t>(static_cast<uint32_t>(scratch.hops[node]) + 1,
                                       std::numeric_limits<uint16_t>::max()));
                scratch.bfsQueue[tail++] = predecessor;
            }
        }
    }
    else
    {
        using QueueEntry = std::pair<float, uint32_t>;
        std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
        queue.push({0.0f, work.dst});
        while (!queue.empty())
        {
            auto [cost, node] = queue.top();
            queue.pop();
            if (cost > scratch.dist[node])
            {
                continue;
            }
            for (uint64_t edge = reverseGraph.rowOffsets[node];
                 edge < reverseGraph.rowOffsets[node + 1];
                 ++edge)
            {
                const uint32_t predecessor = reverseGraph.colIndices[edge];
                const float nextCost = cost + reverseGraph.weights[edge];
                if (nextCost < scratch.dist[predecessor])
                {
                    scratch.dist[predecessor] = nextCost;
                    scratch.nextHop[predecessor] = node;
                    scratch.hops[predecessor] = static_cast<uint16_t>(
                        std::min<uint32_t>(static_cast<uint32_t>(scratch.hops[node]) + 1,
                                           std::numeric_limits<uint16_t>::max()));
                    queue.push({nextCost, predecessor});
                }
            }
        }
    }

    for (size_t requestIndex : work.requestIndices)
    {
        const RouteRequest& request = requests.requests[requestIndex];
        RouteResult& result = results[requestIndex];
        if (request.src < reverseGraph.nodeCount &&
            request.src != request.dst &&
            std::isfinite(scratch.dist[request.src]))
        {
            result.nextHop = scratch.nextHop[request.src];
            result.cost = scratch.dist[request.src];
            result.hopCount = scratch.hops[request.src];
            result.valid = result.nextHop == INVALID_NODE ? 0 : 1;
        }
    }
}

std::vector<RouteResult>
ComputeParallelResults(const SparseGraph& graph,
                       const Requests& requests,
                       uint32_t requestedWorkers)
{
    std::vector<RouteResult> results;
    results.reserve(requests.requests.size());
    for (const RouteRequest& request : requests.requests)
    {
        results.push_back(InvalidResult(request));
    }
    if (requests.requests.empty())
    {
        return results;
    }

    if (requests.mode == RoutingMode::PAIR)
    {
        const uint32_t workerCount =
            std::min<uint32_t>(std::max<uint32_t>(1, requestedWorkers), requests.requests.size());
        std::atomic<size_t> nextRequest{0};
        std::vector<std::thread> threads;
        threads.reserve(workerCount);
        for (uint32_t worker = 0; worker < workerCount; ++worker)
        {
            threads.emplace_back([&]() {
                while (true)
                {
                    const size_t index = nextRequest.fetch_add(1, std::memory_order_relaxed);
                    if (index >= requests.requests.size())
                    {
                        break;
                    }
                    const RouteRequest& request = requests.requests[index];
                    results[index] =
                        ComputePairRoute(graph, request.src, request.dst, requests.metric);
                }
            });
        }
        for (std::thread& thread : threads)
        {
            thread.join();
        }
        return results;
    }

    const SparseGraph reverseGraph = BuildReverseGraph(graph);
    std::map<uint32_t, size_t> destinationToWork;
    std::vector<DestinationWork> workItems;
    for (size_t index = 0; index < requests.requests.size(); ++index)
    {
        const uint32_t dst = requests.requests[index].dst;
        auto inserted = destinationToWork.emplace(dst, workItems.size());
        if (inserted.second)
        {
            DestinationWork work;
            work.dst = dst;
            workItems.push_back(std::move(work));
        }
        workItems[inserted.first->second].requestIndices.push_back(index);
    }

    const uint32_t workerCount =
        std::min<uint32_t>(std::max<uint32_t>(1, requestedWorkers), workItems.size());
    std::atomic<size_t> nextWork{0};
    std::vector<std::thread> threads;
    threads.reserve(workerCount);
    for (uint32_t worker = 0; worker < workerCount; ++worker)
    {
        threads.emplace_back([&]() {
            TreeScratch scratch(reverseGraph.nodeCount);
            while (true)
            {
                const size_t index = nextWork.fetch_add(1, std::memory_order_relaxed);
                if (index >= workItems.size())
                {
                    break;
                }
                ComputeDestinationWork(reverseGraph,
                                       requests,
                                       workItems[index],
                                       scratch,
                                       results);
            }
        });
    }
    for (std::thread& thread : threads)
    {
        thread.join();
    }
    return results;
}

std::vector<RouteRequest>
PartitionRequests(const std::vector<RouteRequest>& requests,
                  RoutingMode mode,
                  uint32_t workerId,
                  uint32_t workerCount)
{
    std::vector<RouteRequest> out;
    if (workerCount == 0)
    {
        Fail("workerCount must be nonzero");
    }

    if (mode == RoutingMode::PAIR)
    {
        for (size_t i = 0; i < requests.size(); ++i)
        {
            if ((i % workerCount) == workerId)
            {
                out.push_back(requests[i]);
            }
        }
        return out;
    }

    std::set<uint32_t> selectedDsts;
    std::vector<uint32_t> uniqueDsts;
    for (const auto& request : requests)
    {
        if (selectedDsts.insert(request.dst).second)
        {
            uniqueDsts.push_back(request.dst);
        }
    }

    selectedDsts.clear();
    for (size_t i = 0; i < uniqueDsts.size(); ++i)
    {
        if ((i % workerCount) == workerId)
        {
            selectedDsts.insert(uniqueDsts[i]);
        }
    }

    for (const auto& request : requests)
    {
        if (selectedDsts.count(request.dst) != 0)
        {
            out.push_back(request);
        }
    }
    return out;
}

std::vector<RouteResult>
ComputeWorkerResults(const SparseGraph& graph,
                     const Requests& requests,
                     uint32_t workerId,
                     uint32_t workerCount)
{
    std::vector<RouteRequest> partition =
        PartitionRequests(requests.requests, requests.mode, workerId, workerCount);

    std::vector<RouteResult> results;
    if (requests.mode == RoutingMode::PAIR)
    {
        results.reserve(partition.size());
        for (const auto& request : partition)
        {
            results.push_back(ComputePairRoute(graph, request.src, request.dst, requests.metric));
        }
        return results;
    }

    SparseGraph reverseGraph = BuildReverseGraph(graph);
    std::map<uint32_t, std::vector<RouteRequest>> byDestination;
    for (const auto& request : partition)
    {
        byDestination[request.dst].push_back(request);
    }

    for (const auto& item : byDestination)
    {
        if (item.first >= graph.nodeCount)
        {
            for (const auto& request : item.second)
            {
                RouteResult result{};
                result.src = request.src;
                result.dst = request.dst;
                result.nextHop = INVALID_NODE;
                result.cost = std::numeric_limits<float>::infinity();
                result.valid = 0;
                results.push_back(result);
            }
            continue;
        }
        std::vector<RouteResult> tree =
            ComputeDestinationTree(reverseGraph, item.second, item.first, requests.metric);
        results.insert(results.end(), tree.begin(), tree.end());
    }
    return results;
}

Options
ParseArgs(int argc, char** argv)
{
    Options options;
    options.executablePath = argv[0];

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        auto needValue = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc)
            {
                Fail("missing value for " + name);
            }
            return argv[++i];
        };

        if (arg == "--graph")
        {
            options.graphPath = needValue(arg);
        }
        else if (arg == "--requests")
        {
            options.requestPath = needValue(arg);
        }
        else if (arg == "--results")
        {
            options.resultPath = needValue(arg);
        }
        else if (arg == "--result-prefix")
        {
            options.resultPrefix = needValue(arg);
        }
        else if (arg == "--workers")
        {
            options.workers = static_cast<uint32_t>(std::stoul(needValue(arg)));
        }
        else if (arg == "--worker")
        {
            options.worker = true;
        }
        else if (arg == "--worker-id")
        {
            options.workerId = static_cast<uint32_t>(std::stoul(needValue(arg)));
        }
        else if (arg == "--worker-count")
        {
            options.workerCount = static_cast<uint32_t>(std::stoul(needValue(arg)));
        }
        else if (arg == "--self-test")
        {
            options.selfTest = true;
        }
        else
        {
            Fail("unknown argument: " + arg);
        }
    }
    return options;
}

int
RunWorker(const Options& options)
{
    SparseGraph graph = ReadGraph(options.graphPath);
    Requests requests = ReadRequests(options.requestPath);
    if (graph.snapshotId != requests.snapshotId)
    {
        Fail("graph and request snapshot IDs differ");
    }

    std::vector<RouteResult> results =
        ComputeWorkerResults(graph, requests, options.workerId, options.workerCount);

    WriteResults(options.resultPath,
                 requests.snapshotId,
                 requests.mode,
                 requests.metric,
                 options.workerId,
                 results);
    return 0;
}

int
RunSupervisor(const Options& options)
{
    if (options.graphPath.empty() || options.requestPath.empty() ||
        options.resultPath.empty() || options.resultPrefix.empty())
    {
        Fail("supervisor requires --graph, --requests, --results, and --result-prefix");
    }
    if (options.workers == 0)
    {
        Fail("--workers must be greater than zero");
    }

    SparseGraph graph = ReadGraph(options.graphPath);
    Requests requests = ReadRequests(options.requestPath);
    if (graph.snapshotId != requests.snapshotId)
    {
        Fail("graph and request snapshot IDs differ");
    }
    std::vector<RouteResult> results =
        ComputeParallelResults(graph, requests, options.workers);

    WriteResults(options.resultPath,
                 requests.snapshotId,
                 requests.mode,
                 requests.metric,
                 std::numeric_limits<uint32_t>::max(),
                 results);
    return 0;
}

void
WriteSelfTestGraph(const std::string& graphPath,
                   const std::string& requestPath,
                   RoutingMetric metric)
{
    // Directed graph:
    // 0 -> 1 -> 3 costs 2
    // 0 -> 2 -> 3 costs 6
    // 0 -> 3 costs 10 (preferred only by hop-count routing)
    // 1 -> 2 cost 1
    SparseGraph graph;
    graph.snapshotId = 7;
    graph.nodeCount = 4;
    graph.rowOffsets = {0, 3, 5, 6, 6};
    graph.colIndices = {1, 2, 3, 2, 3, 3};
    graph.weights = {1.0f, 5.0f, 10.0f, 1.0f, 1.0f, 1.0f};

    std::ofstream graphOut(graphPath, std::ios::binary | std::ios::trunc);
    GraphHeader graphHeader{};
    graphHeader.magic = GRAPH_MAGIC;
    graphHeader.version = FORMAT_VERSION;
    graphHeader.snapshotId = graph.snapshotId;
    graphHeader.nodeCount = graph.nodeCount;
    graphHeader.edgeCount = static_cast<uint32_t>(graph.colIndices.size());
    graphHeader.rowOffsetCount = graph.rowOffsets.size();
    graphHeader.colIndexCount = graph.colIndices.size();
    graphHeader.weightCount = graph.weights.size();
    WriteExact(graphOut, &graphHeader, 1);
    WriteExact(graphOut, graph.rowOffsets.data(), graph.rowOffsets.size());
    WriteExact(graphOut, graph.colIndices.data(), graph.colIndices.size());
    WriteExact(graphOut, graph.weights.data(), graph.weights.size());

    std::vector<RouteRequest> requests = {{0, 3}, {1, 3}, {2, 3}, {3, 0}};
    std::ofstream reqOut(requestPath, std::ios::binary | std::ios::trunc);
    RequestHeader reqHeader{};
    reqHeader.magic = REQUEST_MAGIC;
    reqHeader.version = FORMAT_VERSION;
    reqHeader.snapshotId = graph.snapshotId;
    reqHeader.requestCount = static_cast<uint32_t>(requests.size());
    reqHeader.mode = static_cast<uint32_t>(RoutingMode::DESTINATION_TREE);
    reqHeader.metric = static_cast<uint32_t>(metric);
    WriteExact(reqOut, &reqHeader, 1);
    WriteExact(reqOut, requests.data(), requests.size());
}

int
RunSelfTest(const Options& options)
{
    // Multiple Slurm array tasks can invoke the self-test concurrently on the
    // same node. Use a per-process prefix so their binary fixtures cannot
    // truncate or overwrite one another.
    const std::string prefix =
        "/tmp/leosim-rengine-selftest-" + std::to_string(static_cast<long long>(getpid()));
    const std::string graphPath = prefix + ".graph";
    const std::string requestPath = prefix + ".requests";
    auto run = [&](RoutingMetric metric, uint32_t workers, const std::string& suffix) {
        WriteSelfTestGraph(graphPath, requestPath, metric);
        Options supervisor;
        supervisor.executablePath = options.executablePath;
        supervisor.graphPath = graphPath;
        supervisor.requestPath = requestPath;
        supervisor.resultPath = prefix + suffix + ".results";
        supervisor.resultPrefix = prefix + suffix;
        supervisor.workers = workers;
        if (RunSupervisor(supervisor) != 0)
        {
            Fail("self-test supervisor failed");
        }
        return ReadResults(supervisor.resultPath, 7);
    };

    std::vector<RouteResult> results = run(RoutingMetric::WEIGHT, 3, "-weighted");
    std::vector<RouteResult> serial = run(RoutingMetric::WEIGHT, 1, "-serial");
    if (results.size() != serial.size() ||
        !std::equal(results.begin(),
                    results.end(),
                    serial.begin(),
                    [](const RouteResult& lhs, const RouteResult& rhs) {
                        return std::memcmp(&lhs, &rhs, sizeof(RouteResult)) == 0;
                    }))
    {
        Fail("self-test parallel weighted results differ from serial results");
    }

    std::map<std::pair<uint32_t, uint32_t>, RouteResult> byPair;
    for (const auto& result : results)
    {
        byPair[{result.src, result.dst}] = result;
    }

    auto require = [&](uint32_t src, uint32_t dst, uint32_t nextHop, bool valid) {
        auto it = byPair.find({src, dst});
        if (it == byPair.end())
        {
            Fail("self-test missing route result");
        }
        if ((it->second.valid != 0) != valid || (valid && it->second.nextHop != nextHop))
        {
            std::ostringstream msg;
            msg << "self-test route mismatch for " << src << "->" << dst;
            Fail(msg.str());
        }
    };

    require(0, 3, 1, true);
    require(1, 3, 3, true);
    require(2, 3, 3, true);
    require(3, 0, INVALID_NODE, false);

    results = run(RoutingMetric::HOP_COUNT, 3, "-hop");
    byPair.clear();
    for (const auto& result : results)
    {
        byPair[{result.src, result.dst}] = result;
    }
    require(0, 3, 3, true);
    require(1, 3, 3, true);
    require(2, 3, 3, true);
    require(3, 0, INVALID_NODE, false);

    std::cout << "self-test passed: weighted and hop-count, serial and parallel"
              << std::endl;
    return 0;
}

} // namespace
} // namespace leosim_rengine

int
main(int argc, char** argv)
{
    try
    {
        leosim_rengine::Options options = leosim_rengine::ParseArgs(argc, argv);
        if (options.selfTest)
        {
            return leosim_rengine::RunSelfTest(options);
        }
        if (options.worker)
        {
            return leosim_rengine::RunWorker(options);
        }
        return leosim_rengine::RunSupervisor(options);
    }
    catch (const std::exception& ex)
    {
        std::cerr << "leosim-rengine: " << ex.what() << std::endl;
        return 1;
    }
}
