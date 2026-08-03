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
#include "ns3/net-device.h"
#include "ns3/net-device-queue-interface.h"
#include "ns3/node.h"
#include "ns3/queue-disc.h"
#include "ns3/simulator.h"
#include "ns3/traffic-control-layer.h"

#include "../utils/rengine/rengine-format.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
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

void
LeoSimExternalRoutingHelper::SetStatisticsEndpoints(const NodeContainer& sources,
                                                    const NodeContainer& destinations)
{
    m_statisticsSources = sources;
    m_statisticsDestinations = destinations;
}

void
LeoSimExternalRoutingHelper::EnableRouteLogging(const std::string& filename)
{
    if (m_routeLog.is_open())
    {
        m_routeLog.close();
    }
    m_previousLoggedPaths.clear();
    m_routeLog.open(filename, std::ios::out | std::ios::trunc);
    if (!m_routeLog.is_open())
    {
        NS_LOG_ERROR("Could not open external route log: " << filename);
        return;
    }
    m_routeLog << "time_s,source_node,destination_node,metric,valid,path,hop_count,"
                  "total_distance_m,min_snr_db,total_path_loss_db,min_signal_strength_dbm,"
                  "route_changed"
               << std::endl;
}

void
LeoSimExternalRoutingHelper::LogStatisticsRoute(
    Ptr<Node> source,
    Ptr<Node> destination,
    bool valid,
    const std::vector<uint32_t>& path,
    double distanceKm,
    double minSnr,
    double pathLoss,
    double minSignal)
{
    if (!m_routeLog.is_open() || !source || !destination)
    {
        return;
    }

    const char* metricName = "hop";
    switch (m_metric)
    {
    case LEOSIM_EXTERNAL_WEIGHT_DISTANCE: metricName = "distance"; break;
    case LEOSIM_EXTERNAL_WEIGHT_PATH_LOSS: metricName = "path-loss"; break;
    case LEOSIM_EXTERNAL_WEIGHT_SNR: metricName = "snr"; break;
    case LEOSIM_EXTERNAL_WEIGHT_SIGNAL_STRENGTH: metricName = "signal-strength"; break;
    case LEOSIM_EXTERNAL_WEIGHT_REMAINING_LIFETIME: metricName = "lifetime"; break;
    case LEOSIM_EXTERNAL_WEIGHT_LOAD: metricName = "load"; break;
    default: break;
    }

    std::ostringstream pathText;
    if (valid)
    {
        for (std::size_t i = 0; i < path.size(); ++i)
        {
            if (i)
            {
                pathText << '>';
            }
            pathText << path[i];
        }
    }
    else
    {
        pathText << "NO_ROUTE";
    }

    const auto key = std::make_pair(source->GetId(), destination->GetId());
    const auto previous = m_previousLoggedPaths.find(key);
    const bool changed =
        previous != m_previousLoggedPaths.end() && previous->second != pathText.str();
    m_previousLoggedPaths[key] = pathText.str();

    m_routeLog << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds() << ','
               << source->GetId() << ',' << destination->GetId() << ',' << metricName << ','
               << (valid ? 1 : 0) << ',' << pathText.str() << ','
               << (valid ? path.size() - 1 : 0) << ','
               << (valid ? distanceKm * 1000.0 : 0.0) << ','
               << (valid ? minSnr : 0.0) << ','
               << (valid ? pathLoss : 0.0) << ','
               << (valid ? minSignal : 0.0) << ',' << (changed ? 1 : 0) << std::endl;
}

LeoSimRouteStatistics
LeoSimExternalRoutingHelper::GetRouteStatistics() const
{
    LeoSimRouteStatistics statistics;
    statistics.samples = m_routeSamples;
    statistics.validSamples = m_routeValidSamples;
    statistics.changes = m_routeChanges;
    statistics.uniquePaths = m_uniquePaths.size();
    if (m_routeValidSamples)
    {
        const double count = static_cast<double>(m_routeValidSamples);
        statistics.meanHops = m_routeHopSum / count;
        statistics.meanDistanceKm = m_routeDistanceKmSum / count;
        statistics.meanMinSnrDb = m_routeMinSnrSum / count;
        statistics.minimumSnrDb = m_routeMinimumSnr;
        statistics.meanPathLossDb = m_routePathLossSum / count;
        statistics.meanMinSignalDbm = m_routeMinSignalSum / count;
    }
    return statistics;
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
    UpdateRouteStatistics(calculator, exportInfo, results);
    return ApplyResults(exportInfo, results, verbose);
}

void
LeoSimExternalRoutingHelper::UpdateRouteStatistics(
    Ptr<LeoSimRoutingCalculator> calculator,
    const GraphExport& exportInfo,
    const std::vector<RouteResultRecord>& results)
{
    LeoSimTaskProfiler::ScopedEvent profile(
        "run_simulation.external_routing.update_route_statistics");

    if (!calculator || m_statisticsSources.GetN() == 0 ||
        m_statisticsDestinations.GetN() == 0)
    {
        return;
    }

    std::map<std::pair<uint32_t, uint32_t>, const RouteResultRecord*> nextHops;
    for (const auto& result : results)
    {
        nextHops[{result.src, result.dst}] = &result;
    }

    for (uint32_t i = 0; i < m_statisticsSources.GetN(); ++i)
    {
        Ptr<Node> source = m_statisticsSources.Get(i);
        const auto sourceIt = exportInfo.nodeIdToRoutingId.find(source->GetId());
        if (sourceIt == exportInfo.nodeIdToRoutingId.end())
        {
            continue;
        }
        for (uint32_t j = 0; j < m_statisticsDestinations.GetN(); ++j)
        {
            Ptr<Node> destination = m_statisticsDestinations.Get(j);
            if (source == destination)
            {
                continue;
            }
            ++m_routeSamples;
            const auto destinationIt =
                exportInfo.nodeIdToRoutingId.find(destination->GetId());
            if (destinationIt == exportInfo.nodeIdToRoutingId.end())
            {
                continue;
            }

            const uint32_t destinationId = destinationIt->second;
            uint32_t current = sourceIt->second;
            std::vector<uint32_t> path{source->GetId()};
            std::set<uint32_t> visited{current};
            double distanceKm = 0.0;
            double pathLoss = 0.0;
            double minSnr = std::numeric_limits<double>::max();
            double minSignal = std::numeric_limits<double>::max();
            bool valid = true;

            while (current != destinationId)
            {
                const auto hopIt = nextHops.find({current, destinationId});
                if (hopIt == nextHops.end() || !hopIt->second->valid ||
                    hopIt->second->nextHop >= exportInfo.routingIdToNode.size())
                {
                    valid = false;
                    break;
                }
                const uint32_t next = hopIt->second->nextHop;
                if (!visited.insert(next).second)
                {
                    valid = false;
                    break;
                }
                Ptr<Node> currentNode = exportInfo.routingIdToNode[current];
                Ptr<Node> nextNode = exportInfo.routingIdToNode[next];
                if (!currentNode || !nextNode)
                {
                    valid = false;
                    break;
                }
                const LeoSimChannelQuality quality =
                    calculator->GetLinkQuality(currentNode, nextNode);
                distanceKm += quality.distance / 1000.0;
                pathLoss += quality.pathLoss;
                minSnr = std::min(minSnr, quality.snr);
                minSignal = std::min(minSignal, quality.signalStrength);
                path.push_back(nextNode->GetId());
                current = next;
                if (path.size() > exportInfo.routingIdToNode.size())
                {
                    valid = false;
                    break;
                }
            }

            if (!valid || path.size() < 2)
            {
                LogStatisticsRoute(source,
                                   destination,
                                   false,
                                   path,
                                   0.0,
                                   0.0,
                                   0.0,
                                   0.0);
                continue;
            }
            LogStatisticsRoute(source,
                               destination,
                               true,
                               path,
                               distanceKm,
                               minSnr,
                               pathLoss,
                               minSignal);
            ++m_routeValidSamples;
            m_routeHopSum += path.size() - 1;
            m_routeDistanceKmSum += distanceKm;
            m_routeMinSnrSum += minSnr;
            m_routePathLossSum += pathLoss;
            m_routeMinSignalSum += minSignal;
            if (m_routeValidSamples == 1 || minSnr < m_routeMinimumSnr)
            {
                m_routeMinimumSnr = minSnr;
            }
            const auto pair = std::make_pair(source->GetId(), destination->GetId());
            const auto previous = m_previousPaths.find(pair);
            if (previous != m_previousPaths.end() && previous->second != path)
            {
                ++m_routeChanges;
            }
            m_previousPaths[pair] = path;
            m_uniquePaths.insert(path);
        }
    }
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

    // A topology/access callback can be emitted while a periodic update at the
    // same simulation timestamp is being applied. That update already observes
    // the current state, so scheduling another full snapshot is redundant.
    if (m_routeUpdateInProgress ||
        (m_hasCompletedRouteUpdate && m_lastRouteUpdateTime == Simulator::Now()))
    {
        LeoSimTaskProfiler::ScopedEvent coalesced(
            "run_simulation.external_routing.coalesced_reactive_update");
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
    // Prefer the periodic update when a zero-delay reactive refresh is queued
    // for this same timestamp. This also prevents the reactive callback from
    // running immediately after this (potentially long) wall-clock operation.
    if (m_pendingReactiveUpdate.IsPending() &&
        m_pendingReactiveUpdate.GetTs() ==
            static_cast<uint64_t>(Simulator::Now().GetTimeStep()))
    {
        m_pendingReactiveUpdate.Cancel();
        m_pendingReactiveUpdate = EventId();
        LeoSimTaskProfiler::ScopedEvent coalesced(
            "run_simulation.external_routing.coalesced_reactive_update");
    }

    const auto wallStart = std::chrono::steady_clock::now();
    m_routeUpdateInProgress = true;
    const bool succeeded = SetStaticRoutes(calculator, sources, destinations, verbose);
    m_routeUpdateInProgress = false;
    m_lastRouteUpdateTime = Simulator::Now();
    m_hasCompletedRouteUpdate = true;
    const double wallSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
    std::cout << "[timing] dynamic routing update completed; sim=" << std::fixed
              << std::setprecision(3) << now << "s, interval="
              << updateInterval.GetSeconds() << "s, wall=" << wallSeconds
              << "s, success=" << (succeeded ? "true" : "false") << std::endl;

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
    m_pendingReactiveUpdate = EventId();
    if (!m_reactiveCalculator)
    {
        LeoSimTaskProfiler::ScopedEvent coalesced(
            "run_simulation.external_routing.coalesced_reactive_update");
        return;
    }


    // If a periodic callback is queued for now, let it own this timestamp. If
    // an update already completed now, the requested state is already covered.
    if ((m_dynamicRoutingUpdate.IsPending() &&
         m_dynamicRoutingUpdate.GetTs() ==
             static_cast<uint64_t>(Simulator::Now().GetTimeStep())) ||
        m_routeUpdateInProgress ||
        (m_hasCompletedRouteUpdate && m_lastRouteUpdateTime == Simulator::Now()))
    {
        return;
    }

    m_routeUpdateInProgress = true;
    SetStaticRoutes(m_reactiveCalculator,
                    m_reactiveSources,
                    m_reactiveDestinations,
                    m_reactiveVerbose);
    m_routeUpdateInProgress = false;
    m_lastRouteUpdateTime = Simulator::Now();
    m_hasCompletedRouteUpdate = true;
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
    case LEOSIM_EXTERNAL_WEIGHT_REMAINING_LIFETIME:
        if (quality.remainingConnectionTimeSeconds < 0.0)
        {
            weight = 1.0;
        }
        else if (quality.remainingConnectionTimeSeconds == 0.0)
        {
            weight = 1.0e12;
        }
        else if (std::isinf(quality.remainingConnectionTimeSeconds))
        {
            weight = 1.0e-12;
        }
        else
        {
            weight = 1.0 / quality.remainingConnectionTimeSeconds;
        }
        break;
    case LEOSIM_EXTERNAL_WEIGHT_LOAD:
        weight = calculator->GetIslLoadCost(source, destination);
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

    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.register_endpoints");
        for (uint32_t i = 0; i < sources.GetN(); ++i)
        {
            RegisterNode(sources.Get(i), exportInfo);
        }
        for (uint32_t i = 0; i < destinations.GetN(); ++i)
        {
            RegisterNode(destinations.Get(i), exportInfo);
        }
    }

    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> links;
    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.get_active_links");
        links = calculator->GetActiveLinks();
    }
    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.register_link_nodes");
        for (const auto& link : links)
        {
            RegisterNode(link.first, exportInfo);
            RegisterNode(link.second, exportInfo);
        }
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
    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.build_weighted_adjacency");
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
            adjacency[u].push_back(
                std::make_pair(v, GetExportedWeight(calculator, link.first, link.second)));
            adjacency[v].push_back(
                std::make_pair(u, GetExportedWeight(calculator, link.second, link.first)));
        }
    }

    std::vector<uint64_t> rowOffsets(nodeCount + 1, 0);
    std::vector<uint32_t> colIndices;
    std::vector<float> weights;
    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.build_csr");
        colIndices.reserve(links.size() * 2);
        weights.reserve(links.size() * 2);
        for (uint32_t node = 0; node < nodeCount; ++node)
        {
            rowOffsets[node + 1] = rowOffsets[node] + adjacency[node].size();
            for (const auto& edge : adjacency[node])
            {
                colIndices.push_back(edge.first);
                weights.push_back(edge.second);
            }
        }
    }

    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.write_graph_file");
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
    }

    std::vector<RouteRequest> requests;
    const bool useAllGraphNodes =
        m_mode == LEOSIM_EXTERNAL_DESTINATION_TREE && m_destinationTreeAllNodes;
    const uint32_t sourceCount = useAllGraphNodes ? nodeCount : sources.GetN();
    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.build_requests");
        requests.reserve(static_cast<size_t>(requestedPairCount));
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
    }

    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.export_snapshot.write_request_file");
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
            m_metric == LEOSIM_EXTERNAL_HOP_COUNT ? RoutingMetric::HOP_COUNT
                                                 : RoutingMetric::WEIGHT);

        if (!WriteExact(requestOut, &requestHeader, 1) ||
            !WriteExact(requestOut, requests.data(), requests.size()))
        {
            NS_LOG_ERROR("Failed while writing external routing requests");
            return false;
        }
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

    uint32_t removed = 0;
    for (uint32_t routeIdx = 0; routeIdx < staticRouting->GetNRoutes();)
    {
        Ipv4RoutingTableEntry entry = staticRouting->GetRoute(routeIdx);
        if (entry.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
            entry.GetGateway() != Ipv4Address::GetZero())
        {
            // Ipv4StaticRouting stores routes in a linked list, so removing
            // collected indices in reverse repeatedly walks almost the entire
            // list and becomes quadratic.  After erasing this entry, the next
            // entry occupies the same index and can be checked immediately.
            staticRouting->RemoveRoute(routeIdx);
            ++removed;
        }
        else
        {
            ++routeIdx;
        }
    }

    if (verbose && removed != 0)
    {
        NS_LOG_DEBUG("External routing removed " << removed
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
    using RouteMap = std::map<uint32_t, uint32_t>;
    std::map<uint32_t, RouteMap> desiredBySource;
    std::map<uint32_t, std::vector<const RouteResultRecord*>> resultsBySource;
    std::map<uint32_t, uint32_t> routingIdByNodeId;
    uint32_t installed = 0;
    uint32_t skipped = 0;
    uint32_t unchangedSources = 0;

    // Convert routing IDs to stable ns-3 node IDs before comparing snapshots.
    // Routing IDs are rebuilt for every exported graph and are not a safe cache key.
    for (uint32_t routingId = 0; routingId < exportInfo.routingIdToNode.size(); ++routingId)
    {
        Ptr<Node> node = exportInfo.routingIdToNode[routingId];
        if (node)
        {
            routingIdByNodeId[node->GetId()] = routingId;
        }
    }
    for (const auto& result : results)
    {
        if (result.src >= exportInfo.routingIdToNode.size())
        {
            ++skipped;
            continue;
        }
        Ptr<Node> srcNode = exportInfo.routingIdToNode[result.src];
        if (!srcNode)
        {
            ++skipped;
            continue;
        }
        const uint32_t srcNodeId = srcNode->GetId();
        desiredBySource[srcNodeId];
        resultsBySource[srcNodeId].push_back(&result);
        if (result.valid && result.dst < exportInfo.routingIdToNode.size() &&
            result.nextHop < exportInfo.routingIdToNode.size())
        {
            Ptr<Node> dstNode = exportInfo.routingIdToNode[result.dst];
            Ptr<Node> nextHopNode = exportInfo.routingIdToNode[result.nextHop];
            if (dstNode && nextHopNode)
            {
                desiredBySource[srcNodeId][dstNode->GetId()] = nextHopNode->GetId();
            }
        }
    }

    std::set<uint32_t> changedSources;
    for (const auto& [sourceId, desired] : desiredBySource)
    {
        const auto previous = m_installedNextHopsBySource.find(sourceId);
        if (previous == m_installedNextHopsBySource.end() || previous->second != desired)
        {
            changedSources.insert(sourceId);
        }
        else
        {
            ++unchangedSources;
        }
    }
    // A source omitted from a later result set must have its old computed routes removed.
    for (const auto& [sourceId, previous] : m_installedNextHopsBySource)
    {
        if (desiredBySource.find(sourceId) == desiredBySource.end() && !previous.empty())
        {
            changedSources.insert(sourceId);
        }
    }

    if (verbose)
    {
        std::cout << "[routing-debug] apply begin"
                  << "; sim_s=" << std::fixed << std::setprecision(6)
                  << Simulator::Now().GetSeconds()
                  << "; snapshot=" << exportInfo.snapshotId
                  << "; results=" << results.size()
                  << "; desired_sources=" << desiredBySource.size()
                  << "; changed_sources=" << changedSources.size()
                  << "; unchanged_sources=" << unchangedSources
                  << "; previously_tracked_sources="
                  << m_installedNextHopsBySource.size() << std::endl;
    }

    uint64_t resolvedNextHops = 0;
    uint64_t missingOutputDevices = 0;
    uint64_t outputDevicesWithRootQueueDisc = 0;
    uint64_t outputDevicesWithoutQueueInterface = 0;
    uint64_t outputDevicesWithZeroTxQueues = 0;
    uint64_t routeFingerprint = 1469598103934665603ULL;

    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.apply_results.remove_old_routes");
        for (uint32_t sourceId : changedSources)
        {
            const auto routingId = routingIdByNodeId.find(sourceId);
            if (routingId != routingIdByNodeId.end())
            {
                RemoveComputedHostRoutes(exportInfo.routingIdToNode[routingId->second], verbose);
            }
        }
    }

    {
        LeoSimTaskProfiler::ScopedEvent phase(
            "run_simulation.external_routing.apply_results.validate_and_install_routes");
        std::map<uint32_t, std::vector<std::string>> destinationAddressCache;
        struct ResolvedNextHop
        {
            bool valid{false};
            uint32_t sourceInterface{0};
            Ipv4Address address;
        };
        std::map<std::pair<uint32_t, uint32_t>, ResolvedNextHop> nextHopCache;
        std::map<uint32_t, RouteMap> successfullyInstalled;

        for (uint32_t sourceId : changedSources)
        {
            const auto sourceResults = resultsBySource.find(sourceId);
            if (sourceResults == resultsBySource.end())
            {
                continue;
            }
            Ptr<Node> srcNode = exportInfo.routingIdToNode[sourceResults->second.front()->src];
            Ptr<Ipv4> srcIpv4 = srcNode ? srcNode->GetObject<Ipv4>() : nullptr;
            Ptr<Ipv4StaticRouting> staticRouting =
                srcIpv4 ? Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(
                              srcIpv4->GetRoutingProtocol())
                        : nullptr;
            if (!staticRouting)
            {
                skipped += sourceResults->second.size();
                continue;
            }

            for (const RouteResultRecord* resultPtr : sourceResults->second)
            {
                const auto& result = *resultPtr;
                if (!result.valid || result.dst >= exportInfo.routingIdToNode.size() ||
                    result.nextHop >= exportInfo.routingIdToNode.size())
                {
                    ++skipped;
                    continue;
                }
                Ptr<Node> dstNode = exportInfo.routingIdToNode[result.dst];
                Ptr<Node> nextHopNode = exportInfo.routingIdToNode[result.nextHop];
                if (!dstNode || !nextHopNode)
                {
                    ++skipped;
                    continue;
                }

                const auto hopKey = std::make_pair(sourceId, nextHopNode->GetId());
                auto [hopIt, insertedHop] = nextHopCache.try_emplace(hopKey);
                if (insertedHop)
                {
                    std::string addressString;
                    hopIt->second.valid = FindNextHopAddress(srcNode,
                                                              nextHopNode,
                                                              hopIt->second.sourceInterface,
                                                              addressString);
                    if (hopIt->second.valid)
                    {
                        hopIt->second.address = Ipv4Address(addressString.c_str());
                        ++resolvedNextHops;

                        Ptr<NetDevice> outputDevice =
                            hopIt->second.sourceInterface < srcIpv4->GetNInterfaces()
                                ? srcIpv4->GetNetDevice(hopIt->second.sourceInterface)
                                : nullptr;
                        if (!outputDevice)
                        {
                            ++missingOutputDevices;
                            std::cerr << "[routing-debug] missing output device"
                                      << "; sim_s=" << std::fixed << std::setprecision(6)
                                      << Simulator::Now().GetSeconds()
                                      << "; snapshot=" << exportInfo.snapshotId
                                      << "; source_node=" << sourceId
                                      << "; next_hop_node=" << nextHopNode->GetId()
                                      << "; source_interface="
                                      << hopIt->second.sourceInterface << std::endl;
                        }
                        else
                        {
                            Ptr<TrafficControlLayer> trafficControl =
                                srcNode->GetObject<TrafficControlLayer>();
                            if (trafficControl &&
                                trafficControl->GetRootQueueDiscOnDevice(outputDevice))
                            {
                                ++outputDevicesWithRootQueueDisc;
                            }

                            Ptr<NetDeviceQueueInterface> queueInterface =
                                outputDevice->GetObject<NetDeviceQueueInterface>();
                            if (!queueInterface)
                            {
                                ++outputDevicesWithoutQueueInterface;
                            }
                            else if (queueInterface->GetNTxQueues() == 0)
                            {
                                ++outputDevicesWithZeroTxQueues;
                            }
                        }
                    }
                }
                if (!hopIt->second.valid)
                {
                    ++skipped;
                    continue;
                }

                auto [addressesIt, insertedAddresses] =
                    destinationAddressCache.try_emplace(dstNode->GetId());
                if (insertedAddresses)
                {
                    addressesIt->second = GetDestinationAddresses(dstNode);
                }
                const Ipv4Mask hostMask("255.255.255.255");
                for (const auto& dstAddressString : addressesIt->second)
                {
                    staticRouting->AddNetworkRouteTo(Ipv4Address(dstAddressString.c_str()),
                                                     hostMask,
                                                     hopIt->second.address,
                                                     hopIt->second.sourceInterface,
                                                     100);
                    ++installed;
                }
                successfullyInstalled[sourceId][dstNode->GetId()] = nextHopNode->GetId();
                // Stable, compact identity for the installed route set.  This
                // makes a failing snapshot comparable across reruns without
                // emitting millions of per-route log lines.
                for (uint64_t value : {static_cast<uint64_t>(sourceId),
                                       static_cast<uint64_t>(dstNode->GetId()),
                                       static_cast<uint64_t>(nextHopNode->GetId()),
                                       static_cast<uint64_t>(hopIt->second.sourceInterface)})
                {
                    routeFingerprint ^= value;
                    routeFingerprint *= 1099511628211ULL;
                }
            }
        }

        for (uint32_t sourceId : changedSources)
        {
            m_installedNextHopsBySource[sourceId] = successfullyInstalled[sourceId];
        }
    }

    if (verbose)
    {
        uint64_t trackedDestinations = 0;
        for (const auto& [sourceId, destinations] : m_installedNextHopsBySource)
        {
            (void)sourceId;
            trackedDestinations += destinations.size();
        }
        std::cout << "[routing-debug] apply complete"
                  << "; sim_s=" << std::fixed << std::setprecision(6)
                  << Simulator::Now().GetSeconds()
                  << "; snapshot=" << exportInfo.snapshotId
                  << "; installed_host_routes=" << installed
                  << "; skipped_results=" << skipped
                  << "; tracked_sources=" << m_installedNextHopsBySource.size()
                  << "; tracked_destinations=" << trackedDestinations
                  << "; resolved_next_hops=" << resolvedNextHops
                  << "; missing_output_devices=" << missingOutputDevices
                  << "; root_qdisc_devices=" << outputDevicesWithRootQueueDisc
                  << "; missing_queue_interfaces=" << outputDevicesWithoutQueueInterface
                  << "; zero_tx_queue_devices=" << outputDevicesWithZeroTxQueues
                  << "; route_fingerprint=0x" << std::hex << routeFingerprint << std::dec
                  << std::endl;
    }

    if (verbose)
    {
        NS_LOG_DEBUG("External routing installed " << installed
                                                    << " routes, skipped " << skipped
                                                    << " results; " << unchangedSources
                                                    << " source tables unchanged");
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
