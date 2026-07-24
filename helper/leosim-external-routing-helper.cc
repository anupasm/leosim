/*
 * Copyright (c) 2024 Anupa De Silva
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 */

#include "leosim-external-routing-helper.h"

#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/leosim-routing-calculator.h"
#include "ns3/leosim-task-profiler.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

#include "../utils/rengine/rengine-format.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <vector>

NS_LOG_COMPONENT_DEFINE("LeoSimExternalRoutingHelper");

namespace ns3
{
namespace
{

template <typename T>
bool
WriteExact(std::ofstream& output, const T* data, size_t count)
{
    output.write(reinterpret_cast<const char*>(data),
                 static_cast<std::streamsize>(sizeof(T) * count));
    return static_cast<bool>(output);
}

template <typename T>
bool
ReadExact(std::ifstream& input, T* data, size_t count)
{
    input.read(reinterpret_cast<char*>(data),
               static_cast<std::streamsize>(sizeof(T) * count));
    return static_cast<bool>(input);
}

std::string
ShellQuote(const std::string& value)
{
    std::string quoted = "'";
    for (char c : value)
    {
        if (c == '\'')
        {
            quoted += "'\\''";
        }
        else
        {
            quoted += c;
        }
    }
    quoted += "'";
    return quoted;
}

void
EnsureDirectory(const std::string& path)
{
    if (path.empty())
    {
        return;
    }
    mkdir(path.c_str(), 0755);
}

} // namespace

LeoSimExternalRoutingHelper::LeoSimExternalRoutingHelper()
    : m_enginePath("contrib/leosim/utils/rengine/leosim-rengine"),
      m_workingDirectory("/tmp/leosim-routing"),
      m_workerCount(1),
      m_mode(LEOSIM_EXTERNAL_DESTINATION_TREE),
      m_metric(LEOSIM_EXTERNAL_WEIGHT_DISTANCE),
      m_maxRouteRequests(10000000ULL),
      m_destinationTreeAllNodes(true),
      m_nextSnapshotId(1),
      m_reactiveDebounceInterval(MilliSeconds(200)),
      m_reactiveVerbose(false)
{
}

LeoSimExternalRoutingHelper::~LeoSimExternalRoutingHelper()
{
}

void
LeoSimExternalRoutingHelper::SetEnginePath(const std::string& enginePath)
{
    m_enginePath = enginePath;
}

void
LeoSimExternalRoutingHelper::SetWorkingDirectory(const std::string& workingDirectory)
{
    m_workingDirectory = workingDirectory;
}

void
LeoSimExternalRoutingHelper::SetWorkerCount(uint32_t workerCount)
{
    m_workerCount = std::max<uint32_t>(1, workerCount);
}

void
LeoSimExternalRoutingHelper::SetMode(ExternalRoutingMode mode)
{
    m_mode = mode;
}

void
LeoSimExternalRoutingHelper::SetMetric(ExternalRoutingMetric metric)
{
    m_metric = metric;
}

void
LeoSimExternalRoutingHelper::SetMaxRouteRequests(uint64_t maxRequests)
{
    m_maxRouteRequests = std::max<uint64_t>(1, maxRequests);
}

void
LeoSimExternalRoutingHelper::SetDestinationTreeAllNodes(bool enable)
{
    m_destinationTreeAllNodes = enable;
}

bool
LeoSimExternalRoutingHelper::SetStaticRoutes(Ptr<LeoSimRoutingCalculator> calculator,
                                             const NodeContainer& sources,
                                             const NodeContainer& destinations,
                                             bool verbose)
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.external_routing.update");

    if (!calculator)
    {
        NS_LOG_ERROR("External routing skipped: calculator is null");
        return false;
    }

    GraphExport exportInfo;
    if (!ExportSnapshot(calculator, sources, destinations, exportInfo, verbose))
    {
        return false;
    }
    if (!RunEngine(exportInfo, verbose))
    {
        return false;
    }

    std::vector<RouteResultRecord> results;
    if (!ImportResults(exportInfo, results, verbose))
    {
        return false;
    }
    return ApplyResults(exportInfo, results, verbose);
}

void
LeoSimExternalRoutingHelper::EnableDynamicRouting(Ptr<LeoSimRoutingCalculator> calculator,
                                                  const NodeContainer& sources,
                                                  const NodeContainer& destinations,
                                                  Time updateInterval,
                                                  double stopTime,
                                                  bool verbose)
{
    SetStaticRoutes(calculator, sources, destinations, verbose);
    m_dynamicRoutingUpdate =
        Simulator::Schedule(updateInterval,
                            &LeoSimExternalRoutingHelper::UpdateRoutesAndReschedule,
                            this,
                            calculator,
                            sources,
                            destinations,
                            updateInterval,
                            stopTime,
                            verbose);
}

bool
LeoSimExternalRoutingHelper::HasPendingDynamicRoutingUpdate() const
{
    return m_dynamicRoutingUpdate.IsPending();
}

void
LeoSimExternalRoutingHelper::EnableReactiveRouteRefresh(Ptr<LeoSimRoutingCalculator> calculator,
                                                        const NodeContainer& sources,
                                                        const NodeContainer& destinations,
                                                        Time debounceInterval,
                                                        bool verbose)
{
    m_reactiveCalculator = calculator;
    m_reactiveSources = sources;
    m_reactiveDestinations = destinations;
    m_reactiveDebounceInterval = debounceInterval;
    m_reactiveVerbose = verbose;
}

void
LeoSimExternalRoutingHelper::RequestRouteRefresh()
{
    if (!m_reactiveCalculator)
    {
        return;
    }

    if (m_pendingReactiveUpdate.IsPending())
    {
        m_pendingReactiveUpdate.Cancel();
    }

    m_pendingReactiveUpdate =
        Simulator::Schedule(m_reactiveDebounceInterval,
                            &LeoSimExternalRoutingHelper::DoReactiveUpdate,
                            this);
}

void
LeoSimExternalRoutingHelper::UpdateRoutesAndReschedule(Ptr<LeoSimRoutingCalculator> calculator,
                                                       const NodeContainer& sources,
                                                       const NodeContainer& destinations,
                                                       Time updateInterval,
                                                       double stopTime,
                                                       bool verbose)
{
    const double now = Simulator::Now().GetSeconds();
    SetStaticRoutes(calculator, sources, destinations, verbose);

    if (stopTime > 0.0 && now + updateInterval.GetSeconds() >= stopTime)
    {
        m_dynamicRoutingUpdate = EventId();
        return;
    }

    m_dynamicRoutingUpdate =
        Simulator::Schedule(updateInterval,
                            &LeoSimExternalRoutingHelper::UpdateRoutesAndReschedule,
                            this,
                            calculator,
                            sources,
                            destinations,
                            updateInterval,
                            stopTime,
                            verbose);
}

void
LeoSimExternalRoutingHelper::DoReactiveUpdate()
{
    if (!m_reactiveCalculator)
    {
        return;
    }

    SetStaticRoutes(m_reactiveCalculator,
                    m_reactiveSources,
                    m_reactiveDestinations,
                    m_reactiveVerbose);
}

void
LeoSimExternalRoutingHelper::RegisterNode(Ptr<Node> node, GraphExport& exportInfo) const
{
    if (!node)
    {
        return;
    }

    const uint32_t nodeId = node->GetId();
    if (exportInfo.nodeIdToRoutingId.find(nodeId) != exportInfo.nodeIdToRoutingId.end())
    {
        return;
    }

    const uint32_t routingId = exportInfo.routingIdToNode.size();
    exportInfo.nodeIdToRoutingId[nodeId] = routingId;
    exportInfo.routingIdToNode.push_back(node);
}

float
LeoSimExternalRoutingHelper::GetExportedWeight(Ptr<LeoSimRoutingCalculator> calculator,
                                               Ptr<Node> source,
                                               Ptr<Node> destination) const
{
    if (m_metric == LEOSIM_EXTERNAL_HOP_COUNT)
    {
        return 1.0f;
    }

    LeoSimChannelQuality quality = calculator->GetLinkQuality(source, destination);
    double weight = 1.0;
    switch (m_metric)
    {
    case LEOSIM_EXTERNAL_WEIGHT_DISTANCE:
        weight = quality.distance;
        break;
    case LEOSIM_EXTERNAL_WEIGHT_PATH_LOSS:
        weight = quality.pathLoss;
        break;
    case LEOSIM_EXTERNAL_WEIGHT_SNR:
        // Match LeoSimRoutingCalculator::GetLinkMetricValue: reciprocal SNR
        // gives Dijkstra a non-negative cost while preferring stronger links.
        weight = 1.0 / std::max(quality.snr, 1e-9);
        break;
    case LEOSIM_EXTERNAL_WEIGHT_SIGNAL_STRENGTH:
        // Signal strength is normally negative dBm, so negating it turns
        // stronger (less-negative) signals into smaller positive costs.
        weight = -quality.signalStrength;
        break;
    case LEOSIM_EXTERNAL_HOP_COUNT:
        weight = 1.0;
        break;
    }

    // Invalid channel samples must never poison the external Dijkstra queue.
    // Use the same neutral fallback historically used for missing distance.
    if (!std::isfinite(weight) || weight < 0.0)
    {
        return 1.0f;
    }
    return static_cast<float>(weight);
}

bool
LeoSimExternalRoutingHelper::ExportSnapshot(Ptr<LeoSimRoutingCalculator> calculator,
                                            const NodeContainer& sources,
                                            const NodeContainer& destinations,
                                            GraphExport& exportInfo,
                                            bool verbose)
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.external_routing.export_snapshot");
    using namespace leosim_rengine;

    EnsureDirectory(m_workingDirectory);
    exportInfo.snapshotId = m_nextSnapshotId++;
    {
        std::ostringstream prefix;
        prefix << m_workingDirectory << "/snapshot-" << exportInfo.snapshotId;
        exportInfo.prefix = prefix.str();
    }
    exportInfo.graphPath = exportInfo.prefix + ".graph";
    exportInfo.requestPath = exportInfo.prefix + ".requests";
    exportInfo.resultPath = exportInfo.prefix + ".results";

    for (uint32_t i = 0; i < sources.GetN(); ++i)
    {
        RegisterNode(sources.Get(i), exportInfo);
    }
    for (uint32_t i = 0; i < destinations.GetN(); ++i)
    {
        RegisterNode(destinations.Get(i), exportInfo);
    }

    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> links = calculator->GetActiveLinks();
    for (const auto& link : links)
    {
        RegisterNode(link.first, exportInfo);
        RegisterNode(link.second, exportInfo);
    }

    const uint32_t nodeCount = exportInfo.routingIdToNode.size();

    // Destination-tree mode computes one reverse shortest-path tree per
    // endpoint. Every registered graph node needs a result because satellites
    // are forwarding nodes even when they do not originate application data.
    const uint64_t routeSourceCount =
        (m_mode == LEOSIM_EXTERNAL_DESTINATION_TREE && m_destinationTreeAllNodes)
            ? static_cast<uint64_t>(nodeCount)
            : static_cast<uint64_t>(sources.GetN());
    const uint64_t requestedPairCount =
        routeSourceCount * static_cast<uint64_t>(destinations.GetN());
    if (requestedPairCount > m_maxRouteRequests)
    {
        NS_LOG_ERROR("Refusing to materialize up to "
                     << requestedPairCount << " route requests (limit "
                     << m_maxRouteRequests
                     << "). Reduce endpoint destinations or raise MaxRouteRequests deliberately.");
        return false;
    }

    std::vector<std::vector<std::pair<uint32_t, float>>> adjacency(nodeCount);
    for (const auto& link : links)
    {
        const auto srcIt = exportInfo.nodeIdToRoutingId.find(link.first->GetId());
        const auto dstIt = exportInfo.nodeIdToRoutingId.find(link.second->GetId());
        if (srcIt == exportInfo.nodeIdToRoutingId.end() ||
            dstIt == exportInfo.nodeIdToRoutingId.end())
        {
            continue;
        }

        const uint32_t u = srcIt->second;
        const uint32_t v = dstIt->second;
        adjacency[u].push_back(std::make_pair(v, GetExportedWeight(calculator, link.first, link.second)));
        adjacency[v].push_back(std::make_pair(u, GetExportedWeight(calculator, link.second, link.first)));
    }

    std::vector<uint64_t> rowOffsets(nodeCount + 1, 0);
    std::vector<uint32_t> colIndices;
    std::vector<float> weights;
    for (uint32_t node = 0; node < nodeCount; ++node)
    {
        rowOffsets[node + 1] = rowOffsets[node] + adjacency[node].size();
        for (const auto& edge : adjacency[node])
        {
            colIndices.push_back(edge.first);
            weights.push_back(edge.second);
        }
    }

    std::ofstream graphOut(exportInfo.graphPath, std::ios::binary | std::ios::trunc);
    if (!graphOut)
    {
        NS_LOG_ERROR("Unable to write external routing graph: " << exportInfo.graphPath);
        return false;
    }

    GraphHeader graphHeader{};
    graphHeader.magic = GRAPH_MAGIC;
    graphHeader.version = FORMAT_VERSION;
    graphHeader.snapshotId = exportInfo.snapshotId;
    graphHeader.simTimeSeconds = Simulator::Now().GetSeconds();
    graphHeader.nodeCount = nodeCount;
    graphHeader.edgeCount = colIndices.size();
    graphHeader.rowOffsetCount = rowOffsets.size();
    graphHeader.colIndexCount = colIndices.size();
    graphHeader.weightCount = weights.size();

    if (!WriteExact(graphOut, &graphHeader, 1) ||
        !WriteExact(graphOut, rowOffsets.data(), rowOffsets.size()) ||
        !WriteExact(graphOut, colIndices.data(), colIndices.size()) ||
        !WriteExact(graphOut, weights.data(), weights.size()))
    {
        NS_LOG_ERROR("Failed while writing external routing graph");
        return false;
    }

    std::vector<RouteRequest> requests;
    requests.reserve(static_cast<size_t>(requestedPairCount));
    const bool useAllGraphNodes =
        m_mode == LEOSIM_EXTERNAL_DESTINATION_TREE && m_destinationTreeAllNodes;
    const uint32_t sourceCount = useAllGraphNodes ? nodeCount : sources.GetN();
    for (uint32_t i = 0; i < sourceCount; ++i)
    {
        Ptr<Node> src = useAllGraphNodes ? exportInfo.routingIdToNode[i] : sources.Get(i);
        auto srcIt = exportInfo.nodeIdToRoutingId.find(src->GetId());
        if (srcIt == exportInfo.nodeIdToRoutingId.end())
        {
            continue;
        }

        for (uint32_t j = 0; j < destinations.GetN(); ++j)
        {
            Ptr<Node> dst = destinations.Get(j);
            if (src == dst)
            {
                continue;
            }
            auto dstIt = exportInfo.nodeIdToRoutingId.find(dst->GetId());
            if (dstIt == exportInfo.nodeIdToRoutingId.end())
            {
                continue;
            }

            RouteRequest request{};
            request.src = srcIt->second;
            request.dst = dstIt->second;
            requests.push_back(request);
        }
    }

    std::ofstream requestOut(exportInfo.requestPath, std::ios::binary | std::ios::trunc);
    if (!requestOut)
    {
        NS_LOG_ERROR("Unable to write external routing requests: " << exportInfo.requestPath);
        return false;
    }

    RequestHeader requestHeader{};
    requestHeader.magic = REQUEST_MAGIC;
    requestHeader.version = FORMAT_VERSION;
    requestHeader.snapshotId = exportInfo.snapshotId;
    requestHeader.requestCount = requests.size();
    requestHeader.mode = static_cast<uint32_t>(
        m_mode == LEOSIM_EXTERNAL_PAIR ? RoutingMode::PAIR : RoutingMode::DESTINATION_TREE);
    requestHeader.metric = static_cast<uint32_t>(
        m_metric == LEOSIM_EXTERNAL_HOP_COUNT ? RoutingMetric::HOP_COUNT : RoutingMetric::WEIGHT);

    if (!WriteExact(requestOut, &requestHeader, 1) ||
        !WriteExact(requestOut, requests.data(), requests.size()))
    {
        NS_LOG_ERROR("Failed while writing external routing requests");
        return false;
    }

    if (verbose)
    {
        NS_LOG_DEBUG("External routing snapshot " << exportInfo.snapshotId
                                                  << ": nodes=" << nodeCount
                                                  << " edges=" << colIndices.size()
                                                  << " requests=" << requests.size());
    }

    return true;
}

std::string
LeoSimExternalRoutingHelper::BuildShellCommand(const GraphExport& exportInfo) const
{
    std::ostringstream cmd;
    cmd << ShellQuote(m_enginePath)
        << " --graph " << ShellQuote(exportInfo.graphPath)
        << " --requests " << ShellQuote(exportInfo.requestPath)
        << " --result-prefix " << ShellQuote(exportInfo.prefix)
        << " --results " << ShellQuote(exportInfo.resultPath)
        << " --workers " << m_workerCount;
    return cmd.str();
}

bool
LeoSimExternalRoutingHelper::RunEngine(const GraphExport& exportInfo, bool verbose) const
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.external_routing.run_engine");
    const std::string command = BuildShellCommand(exportInfo);
    if (verbose)
    {
        NS_LOG_DEBUG("Running external routing command: " << command);
    }

    const int rc = std::system(command.c_str());
    if (rc != 0)
    {
        NS_LOG_ERROR("External routing engine failed with status " << rc);
        return false;
    }
    return true;
}

bool
LeoSimExternalRoutingHelper::ImportResults(const GraphExport& exportInfo,
                                           std::vector<RouteResultRecord>& results,
                                           bool verbose) const
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.external_routing.import_results");
    using namespace leosim_rengine;

    std::ifstream input(exportInfo.resultPath, std::ios::binary);
    if (!input)
    {
        NS_LOG_ERROR("Unable to open external routing result: " << exportInfo.resultPath);
        return false;
    }

    ResultHeader header{};
    if (!ReadExact(input, &header, 1))
    {
        NS_LOG_ERROR("Unable to read external routing result header");
        return false;
    }
    if (header.magic != RESULT_MAGIC ||
        header.version != FORMAT_VERSION ||
        header.snapshotId != exportInfo.snapshotId)
    {
        NS_LOG_ERROR("Rejecting invalid or stale external routing result");
        return false;
    }

    std::vector<RouteResult> raw(header.resultCount);
    if (!ReadExact(input, raw.data(), raw.size()))
    {
        NS_LOG_ERROR("Unable to read external routing result records");
        return false;
    }

    results.clear();
    results.reserve(raw.size());
    for (const auto& item : raw)
    {
        RouteResultRecord result;
        result.src = item.src;
        result.dst = item.dst;
        result.nextHop = item.nextHop;
        result.cost = item.cost;
        result.hopCount = item.hopCount;
        result.valid = item.valid != 0;
        results.push_back(result);
    }

    if (verbose)
    {
        NS_LOG_DEBUG("Imported " << results.size()
                                  << " external routing results for snapshot "
                                  << exportInfo.snapshotId);
    }

    return true;
}

std::vector<std::string>
LeoSimExternalRoutingHelper::GetDestinationAddresses(Ptr<Node> node) const
{
    std::vector<std::string> addresses;
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        return addresses;
    }

    for (uint32_t ifIdx = 1; ifIdx < ipv4->GetNInterfaces(); ++ifIdx)
    {
        for (uint32_t addrIdx = 0; addrIdx < ipv4->GetNAddresses(ifIdx); ++addrIdx)
        {
            Ipv4InterfaceAddress addr = ipv4->GetAddress(ifIdx, addrIdx);
            Ipv4Address local = addr.GetLocal();
            if (local != Ipv4Address::GetZero() && local != Ipv4Address("127.0.0.1"))
            {
                std::ostringstream os;
                os << local;
                addresses.push_back(os.str());
            }
        }
    }
    return addresses;
}

void
LeoSimExternalRoutingHelper::RemoveComputedHostRoutes(Ptr<Node> node, bool verbose) const
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        return;
    }

    Ptr<Ipv4StaticRouting> staticRouting =
        Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(ipv4->GetRoutingProtocol());
    if (!staticRouting)
    {
        return;
    }

    std::vector<uint32_t> removeIndices;
    for (uint32_t routeIdx = 0; routeIdx < staticRouting->GetNRoutes(); ++routeIdx)
    {
        Ipv4RoutingTableEntry entry = staticRouting->GetRoute(routeIdx);
        if (entry.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
            entry.GetGateway() != Ipv4Address::GetZero())
        {
            removeIndices.push_back(routeIdx);
        }
    }

    for (int i = static_cast<int>(removeIndices.size()) - 1; i >= 0; --i)
    {
        staticRouting->RemoveRoute(removeIndices[i]);
    }

    if (verbose && !removeIndices.empty())
    {
        NS_LOG_DEBUG("External routing removed " << removeIndices.size()
                                                  << " old host routes on node "
                                                  << node->GetId());
    }
}

bool
LeoSimExternalRoutingHelper::FindNextHopAddress(Ptr<Node> source,
                                                Ptr<Node> nextHop,
                                                uint32_t& sourceInterface,
                                                std::string& nextHopAddress) const
{
    Ptr<Ipv4> srcIpv4 = source->GetObject<Ipv4>();
    Ptr<Ipv4> nextHopIpv4 = nextHop->GetObject<Ipv4>();
    if (!srcIpv4 || !nextHopIpv4)
    {
        return false;
    }

    for (uint32_t srcIfIdx = 1; srcIfIdx < srcIpv4->GetNInterfaces(); ++srcIfIdx)
    {
        if (srcIpv4->GetNAddresses(srcIfIdx) == 0)
        {
            continue;
        }
        Ipv4InterfaceAddress srcAddr = srcIpv4->GetAddress(srcIfIdx, 0);
        Ipv4Address srcNetwork = srcAddr.GetLocal().CombineMask(srcAddr.GetMask());

        for (uint32_t nhIfIdx = 1; nhIfIdx < nextHopIpv4->GetNInterfaces(); ++nhIfIdx)
        {
            for (uint32_t nhAddrIdx = 0; nhAddrIdx < nextHopIpv4->GetNAddresses(nhIfIdx); ++nhAddrIdx)
            {
                Ipv4InterfaceAddress nhAddr = nextHopIpv4->GetAddress(nhIfIdx, nhAddrIdx);
                Ipv4Address nhNetwork = nhAddr.GetLocal().CombineMask(nhAddr.GetMask());
                if (srcNetwork == nhNetwork && nhAddr.GetLocal() != srcAddr.GetLocal())
                {
                    sourceInterface = srcIfIdx;
                    std::ostringstream os;
                    os << nhAddr.GetLocal();
                    nextHopAddress = os.str();
                    return true;
                }
            }
        }
    }

    return false;
}

bool
LeoSimExternalRoutingHelper::ApplyResults(const GraphExport& exportInfo,
                                          const std::vector<RouteResultRecord>& results,
                                          bool verbose)
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.external_routing.apply_results");
    std::set<uint32_t> cleanedSourceNodeIds;
    uint32_t installed = 0;
    uint32_t skipped = 0;

    for (Ptr<Node> node : exportInfo.routingIdToNode)
    {
        if (node && cleanedSourceNodeIds.insert(node->GetId()).second)
        {
            RemoveComputedHostRoutes(node, verbose);
        }
    }

    for (const auto& result : results)
    {
        if (!result.valid ||
            result.src >= exportInfo.routingIdToNode.size() ||
            result.dst >= exportInfo.routingIdToNode.size() ||
            result.nextHop >= exportInfo.routingIdToNode.size())
        {
            ++skipped;
            continue;
        }

        Ptr<Node> srcNode = exportInfo.routingIdToNode[result.src];
        Ptr<Node> dstNode = exportInfo.routingIdToNode[result.dst];
        Ptr<Node> nextHopNode = exportInfo.routingIdToNode[result.nextHop];
        if (!srcNode || !dstNode || !nextHopNode)
        {
            ++skipped;
            continue;
        }

        Ptr<Ipv4> srcIpv4 = srcNode->GetObject<Ipv4>();
        if (!srcIpv4)
        {
            ++skipped;
            continue;
        }

        Ptr<Ipv4StaticRouting> staticRouting =
            Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(srcIpv4->GetRoutingProtocol());
        if (!staticRouting)
        {
            ++skipped;
            continue;
        }

        uint32_t sourceInterface = 0;
        std::string nextHopAddressString;
        if (!FindNextHopAddress(srcNode, nextHopNode, sourceInterface, nextHopAddressString))
        {
            ++skipped;
            continue;
        }

        Ipv4Address nextHopAddress(nextHopAddressString.c_str());
        Ipv4Mask hostMask("255.255.255.255");
        for (const auto& dstAddressString : GetDestinationAddresses(dstNode))
        {
            Ipv4Address dstAddress(dstAddressString.c_str());
            staticRouting->AddNetworkRouteTo(dstAddress, hostMask, nextHopAddress, sourceInterface, 100);
            ++installed;
        }
    }

    if (verbose)
    {
        NS_LOG_DEBUG("External routing installed " << installed
                                                    << " routes, skipped "
                                                    << skipped << " results");
    }

    return true;
}

std::string
LeoSimExternalRoutingHelper::MakeSnapshotPrefix() const
{
    std::ostringstream prefix;
    prefix << m_workingDirectory << "/snapshot-" << m_nextSnapshotId;
    return prefix.str();
}

} // namespace ns3
