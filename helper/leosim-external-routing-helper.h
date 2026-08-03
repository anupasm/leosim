/*
 * Copyright (c) 2024 Anupa De Silva
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 */

#ifndef LEOSIM_EXTERNAL_ROUTING_HELPER_H
#define LEOSIM_EXTERNAL_ROUTING_HELPER_H

#include "ns3/event-id.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/leosim-statistics-helper.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace ns3
{

class LeoSimRoutingCalculator;
class Node;

/**
 * \ingroup leosim
 * \brief Helper that offloads LeoSim route calculation to a process-parallel
 * standalone routing engine.
 *
 * ns-3 remains single-threaded: this helper exports a plain CSR graph snapshot,
 * launches contrib/leosim/utils/rengine/leosim-rengine, imports next-hop
 * results, and installs static routes on the simulator thread.
 */
class LeoSimExternalRoutingHelper
{
  public:
    enum ExternalRoutingMode
    {
        LEOSIM_EXTERNAL_PAIR = 1,
        LEOSIM_EXTERNAL_DESTINATION_TREE = 2
    };

    enum ExternalRoutingMetric
    {
        LEOSIM_EXTERNAL_WEIGHT_DISTANCE = 1,
        LEOSIM_EXTERNAL_HOP_COUNT = 2,
        LEOSIM_EXTERNAL_WEIGHT_PATH_LOSS = 3,
        LEOSIM_EXTERNAL_WEIGHT_SNR = 4,
        LEOSIM_EXTERNAL_WEIGHT_SIGNAL_STRENGTH = 5,
        LEOSIM_EXTERNAL_WEIGHT_REMAINING_LIFETIME = 6,
        LEOSIM_EXTERNAL_WEIGHT_LOAD = 7
    };

    LeoSimExternalRoutingHelper();
    ~LeoSimExternalRoutingHelper();

    void SetEnginePath(const std::string& enginePath);
    void SetWorkingDirectory(const std::string& workingDirectory);
    void SetWorkerCount(uint32_t workerCount);
    void SetMode(ExternalRoutingMode mode);
    void SetMetric(ExternalRoutingMetric metric);
    /** Set the maximum number of route records materialized in one snapshot. */
    void SetMaxRouteRequests(uint64_t maxRequests);
    /**
     * In destination-tree mode, generate a next hop from every graph node to
     * each destination. Disable only when callers deliberately want the
     * legacy source-container restriction.
     */
    void SetDestinationTreeAllNodes(bool enable);
    /** Restrict compact route statistics to these application path pairs. */
    void SetStatisticsEndpoints(const NodeContainer& sources,
                                const NodeContainer& destinations);
    /** Log the compact application endpoint routes produced by each snapshot. */
    void EnableRouteLogging(const std::string& filename);
    LeoSimRouteStatistics GetRouteStatistics() const;

    /**
     * \brief Export current topology, invoke the external routing engine, and
     * install returned static routes.
     */
    bool SetStaticRoutes(Ptr<LeoSimRoutingCalculator> calculator,
                         const NodeContainer& sources,
                         const NodeContainer& destinations,
                         bool verbose = false);

    /**
     * \brief Enable periodic external routing updates.
     */
    void EnableDynamicRouting(Ptr<LeoSimRoutingCalculator> calculator,
                              const NodeContainer& sources,
                              const NodeContainer& destinations,
                              Time updateInterval,
                              double stopTime = 0.0,
                              bool verbose = false);

    bool HasPendingDynamicRoutingUpdate() const;

    /**
     * \brief Store context for explicit debounced refresh requests.
     *
     * This is intended for non-channel topology changes such as beam-manager
     * serving access updates.
     */
    void EnableReactiveRouteRefresh(Ptr<LeoSimRoutingCalculator> calculator,
                                    const NodeContainer& sources,
                                    const NodeContainer& destinations,
                                    Time debounceInterval = MilliSeconds(200),
                                    bool verbose = false);

    /**
     * \brief Request a debounced external route refresh.
     */
    void RequestRouteRefresh();

  private:
    struct GraphExport
    {
        uint64_t snapshotId = 0;
        std::string prefix;
        std::string graphPath;
        std::string requestPath;
        std::string resultPath;
        std::map<uint32_t, uint32_t> nodeIdToRoutingId;
        std::vector<Ptr<Node>> routingIdToNode;
    };

    struct RouteResultRecord
    {
        uint32_t src = 0;
        uint32_t dst = 0;
        uint32_t nextHop = 0;
        float cost = 0.0f;
        uint16_t hopCount = 0;
        bool valid = false;
    };

    bool ExportSnapshot(Ptr<LeoSimRoutingCalculator> calculator,
                        const NodeContainer& sources,
                        const NodeContainer& destinations,
                        GraphExport& exportInfo,
                        bool verbose);
    bool RunEngine(const GraphExport& exportInfo, bool verbose) const;
    bool ImportResults(const GraphExport& exportInfo,
                       std::vector<RouteResultRecord>& results,
                       bool verbose) const;
    bool ApplyResults(const GraphExport& exportInfo,
                      const std::vector<RouteResultRecord>& results,
                      bool verbose);
    void UpdateRouteStatistics(Ptr<LeoSimRoutingCalculator> calculator,
                               const GraphExport& exportInfo,
                               const std::vector<RouteResultRecord>& results);
    void LogStatisticsRoute(Ptr<Node> source,
                            Ptr<Node> destination,
                            bool valid,
                            const std::vector<uint32_t>& path,
                            double distanceKm,
                            double minSnr,
                            double pathLoss,
                            double minSignal);

    void UpdateRoutesAndReschedule(Ptr<LeoSimRoutingCalculator> calculator,
                                   const NodeContainer& sources,
                                   const NodeContainer& destinations,
                                   Time updateInterval,
                                   double stopTime,
                                   bool verbose);
    void DoReactiveUpdate();

    void RegisterNode(Ptr<Node> node, GraphExport& exportInfo) const;
    float GetExportedWeight(Ptr<LeoSimRoutingCalculator> calculator,
                            Ptr<Node> source,
                            Ptr<Node> destination) const;
    bool FindNextHopAddress(Ptr<Node> source,
                            Ptr<Node> nextHop,
                            uint32_t& sourceInterface,
                            std::string& nextHopAddress) const;
    void RemoveComputedHostRoutes(Ptr<Node> node, bool verbose) const;
    std::vector<std::string> GetDestinationAddresses(Ptr<Node> node) const;
    std::string BuildShellCommand(const GraphExport& exportInfo) const;
    std::string MakeSnapshotPrefix() const;

    std::string m_enginePath;
    std::string m_workingDirectory;
    uint32_t m_workerCount;
    ExternalRoutingMode m_mode;
    ExternalRoutingMetric m_metric;
    uint64_t m_maxRouteRequests;
    bool m_destinationTreeAllNodes;
    NodeContainer m_statisticsSources;
    NodeContainer m_statisticsDestinations;
    uint64_t m_routeSamples{0};
    uint64_t m_routeValidSamples{0};
    uint64_t m_routeChanges{0};
    double m_routeHopSum{0.0};
    double m_routeDistanceKmSum{0.0};
    double m_routeMinSnrSum{0.0};
    double m_routePathLossSum{0.0};
    double m_routeMinSignalSum{0.0};
    double m_routeMinimumSnr{0.0};
    std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t>> m_previousPaths;
    std::set<std::vector<uint32_t>> m_uniquePaths;
    std::ofstream m_routeLog;
    std::map<std::pair<uint32_t, uint32_t>, std::string> m_previousLoggedPaths;
    // Last successfully installed destination->next-hop map for each source.
    // Dynamic updates use this to avoid deleting and rebuilding unchanged
    // static host-route tables.
    std::map<uint32_t, std::map<uint32_t, uint32_t>> m_installedNextHopsBySource;
    uint64_t m_nextSnapshotId;
    EventId m_dynamicRoutingUpdate;
    EventId m_pendingReactiveUpdate;
    // Periodic and reactive triggers may land at the same simulator timestamp.
    // Remember the last completed update so those triggers share one snapshot.
    Time m_lastRouteUpdateTime;
    bool m_hasCompletedRouteUpdate{false};
    bool m_routeUpdateInProgress{false};
    Ptr<LeoSimRoutingCalculator> m_reactiveCalculator;
    NodeContainer m_reactiveSources;
    NodeContainer m_reactiveDestinations;
    Time m_reactiveDebounceInterval;
    bool m_reactiveVerbose;
};

} // namespace ns3

#endif /* LEOSIM_EXTERNAL_ROUTING_HELPER_H */
