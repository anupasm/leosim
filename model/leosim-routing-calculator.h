/*
 * Copyright (c) 2024 Anupa De Silva
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef LEOSIM_ROUTING_CALCULATOR_H
#define LEOSIM_ROUTING_CALCULATOR_H

#include "leosim-channel-model.h"
#include "leosim-operator-model.h"

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/callback.h"

#include <map>
#include <mutex>
#include <set>
#include <vector>
#include <queue>

namespace ns3
{

class LeoSimBeamManager;
class LeoSimDijkstraRoutingModel;
class LeoSimRouteProvider;
struct LeoSimRoutingContext;
struct LeoSimRoutingRequest;

using LeoSimEdgeCostCallback =
    Callback<double, Ptr<Node>, Ptr<Node>, const LeoSimChannelQuality&>;

/**
 * \ingroup leosim
 * \brief Route information containing path and metrics
 */
struct LeoSimRoute
{
    std::vector<Ptr<Node>> path;              //!< Ordered list of nodes in the path
    uint32_t hopCount;                        //!< Number of hops
    double totalPathLoss;                     //!< Accumulated path loss in dB
    double totalDistance;                     //!< Total distance in meters
    double minSignalStrength;                 //!< Minimum signal strength along path
    double minSnr;                            //!< Minimum SNR along path
    bool hasIslLinks;                         //!< Whether path contains ISL links
    bool hasGroundLinks;                      //!< Whether path contains ground links
    std::vector<LeoSimLinkType> linkTypes;    //!< Type of each link in path
    bool valid;                               //!< Whether route is valid

    LeoSimRoute() : hopCount(0), totalPathLoss(0.0), totalDistance(0.0),
                    minSignalStrength(std::numeric_limits<double>::lowest()),
                    minSnr(std::numeric_limits<double>::lowest()),
                    hasIslLinks(false), hasGroundLinks(false), valid(false) {}
};

/**
 * \ingroup leosim
 * \brief Central routing calculator for LEO satellite networks
 *
 * This class provides routing calculations using the links defined in the
 * channel model. It supports:
 * - Routing through ISL (Inter-Satellite Links)
 * - Routing through ground links (satellite-to-ground)
 * - Mixed ISL and ground link paths
 * - Path computation based on hop count, path loss, or SNR
 * - Dynamic topology support
 *
 * The class uses Dijkstra's algorithm for path computation with customizable
 * metrics (hop count, path loss, SNR, distance).
 */
class LeoSimRoutingCalculator : public Object
{
  public:
    /**
     * \brief Routing metric for path computation
     */
    enum RoutingMetric
    {
        LEOSIM_METRIC_HOP_COUNT,      //!< Minimize hop count
        LEOSIM_METRIC_PATH_LOSS,      //!< Minimize path loss
        LEOSIM_METRIC_SNR,            //!< Maximize SNR (minimize negative SNR)
        LEOSIM_METRIC_DISTANCE,       //!< Minimize distance
        LEOSIM_METRIC_SIGNAL_STRENGTH //!< Maximize signal strength
    };

    /**
     * \brief Path type constraint
     */
    enum PathType
    {
        LEOSIM_PATH_ANY,         //!< Allow any combination of ISL and ground links
        LEOSIM_PATH_ISL_ONLY,    //!< Only use ISL links
      LEOSIM_PATH_GROUND_ONLY, //!< Only use ground links
      LEOSIM_PATH_SAME_OPERATOR_ONLY //!< Never cross operator boundary; intra-op only
    };

    /**
     * \brief Policy for admitting satellite-ground access links into routing.
     */
    enum AccessLinkPolicy
    {
        LEOSIM_ACCESS_SERVING_ONLY,        //!< Current serving beam only.
        LEOSIM_ACCESS_SERVING_AND_CHO,     //!< Serving beam plus prepared CHO candidates.
        LEOSIM_ACCESS_MULTI_CONNECTIVITY   //!< Top-N ranked valid beams per ground node.
    };

    /**
     * \brief Routing access state for satellite-ground links.
     *
     * This is deliberately distinct from LeoSimLinkState (physical channel
     * visibility) and LeoSimBeamState (beam/cell association lifecycle).
     */
    enum LeoSimAccessLinkState
    {
        LEOSIM_ACCESS_NOT_APPLICABLE = 0,   //!< Not a satellite-ground access link.
        LEOSIM_ACCESS_CHANNEL_DOWN,         //!< Physical channel is not UP/DEGRADED.
        LEOSIM_ACCESS_NO_ELIGIBLE_BEAM,     //!< No active feasible beam covers the ground node.
        LEOSIM_ACCESS_BEAM_DARK,            //!< Selected beam exists but is dark this slot.
        LEOSIM_ACCESS_NOT_SELECTED,         //!< Channel/beam feasible but not routing-authorized.
        LEOSIM_ACCESS_SERVING,              //!< Current serving access link.
        LEOSIM_ACCESS_PREPARED,             //!< Prepared CHO candidate access link.
        LEOSIM_ACCESS_MULTI_CONNECTIVITY_CANDIDATE //!< Policy-admitted multi-connectivity link.
    };

    /**
     * \brief Get the type ID
     * \return The TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief Constructor
     */
    LeoSimRoutingCalculator();

    /**
     * \brief Destructor
     */
    ~LeoSimRoutingCalculator();

    /**
     * \brief Set the channel model to use for link information (ground links)
     * \param channelModel Pointer to the channel model
     */
    void SetChannelModel(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Get the channel model
     * \return Pointer to the channel model
     */
    Ptr<LeoSimChannelModel> GetChannelModel() const;

    /**
     * \brief Set the ISL channel model for inter-satellite links
     * \param islChannelModel Pointer to the ISL channel model
     *
     * When set, this enables unified routing calculations that combine
     * both ground and ISL links in the same network.
     */
    void SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel);

    /**
     * \brief Get the ISL channel model
     * \return Pointer to the ISL channel model (null if not set)
     */
    Ptr<LeoSimChannelModel> GetIslChannelModel() const;

    /**
     * \brief Set beam manager used as the access-link authority.
     * \param beamManager Pointer to the beam manager.
     */
    void SetBeamManager(Ptr<LeoSimBeamManager> beamManager);

    /**
     * \brief Set satellite-ground access-link admission policy.
     * \param policy Access policy. Defaults to LEOSIM_ACCESS_SERVING_ONLY.
     */
    void SetAccessLinkPolicy(AccessLinkPolicy policy);

    /**
     * \brief Set maximum access links per ground node in multi-connectivity mode.
     * \param maxLinks Maximum ranked links to admit.
     */
    void SetMultiConnectivityMaxLinks(uint32_t maxLinks);

    /**
     * \brief Enable or bypass beam-manager access-link authority.
     * \param enabled True to enforce serving/prepared access policy.
     */
    void SetAccessAuthorityEnabled(bool enabled);

    /**
     * \brief Return whether beam-manager access-link authority is enforced.
     */
    bool IsAccessAuthorityEnabled() const;

    /**
     * \brief Set the route provider used by ComputeRoute.
     * \param provider Route provider implementation. Null restores the default Dijkstra provider.
     */
    void SetRouteProvider(Ptr<LeoSimRouteProvider> provider);

    /**
     * \brief Get the currently configured route provider.
     */
    Ptr<LeoSimRouteProvider> GetRouteProvider() const;

    /**
     * \brief Set a custom edge-cost callback used by the default Dijkstra provider.
     * \param callback Cost callback. A null callback restores built-in metric costs.
     */
    void SetEdgeCostCallback(LeoSimEdgeCostCallback callback);

    /**
     * \brief Get the custom edge-cost callback, if one is configured.
     */
    LeoSimEdgeCostCallback GetEdgeCostCallback() const;

    /**
     * \brief Get routing access state for a satellite-ground pair.
     * \param source First endpoint.
     * \param destination Second endpoint.
     * \return Explicit routing access state.
     */
    LeoSimAccessLinkState GetAccessLinkState(Ptr<Node> source, Ptr<Node> destination);

    /**
     * \brief Set operator model used for operator-aware routing.
     * \param model Pointer to operator model
     */
    void SetOperatorModel(Ptr<LeoSimOperatorModel> model);

    /**
     * \brief Set a default PathType used when none is specified in ComputeRoute
     * \param pathType Default path type constraint
     */
    void SetPathType(PathType pathType);

    /**
     * \brief Compute route between source and destination using specified metric
     * \param source Source node
     * \param destination Destination node
     * \param metric Routing metric to optimize
     * \param pathType Type of path allowed (ISL only, ground only, or mixed)
     * \return Route information (valid flag indicates success)
     */
    LeoSimRoute ComputeRoute(Ptr<Node> source,
                             Ptr<Node> destination,
                             RoutingMetric metric = LEOSIM_METRIC_HOP_COUNT,
                             PathType pathType = LEOSIM_PATH_ANY);

    /**
     * \brief Compute route with SNR constraint
     * \param source Source node
     * \param destination Destination node
     * \param minSnr Minimum required SNR in dB
     * \param metric Secondary routing metric if SNR constraint can be met
     * \return Route information
     */
    LeoSimRoute ComputeRouteWithSnrConstraint(Ptr<Node> source,
                                               Ptr<Node> destination,
                                               double minSnr,
                                               RoutingMetric metric = LEOSIM_METRIC_HOP_COUNT);

    /**
     * \brief Compute all available routes between source and destination
     * \param source Source node
     * \param destination Destination node
     * \param numRoutes Number of alternative routes to find
     * \param metric Routing metric
     * \return Vector of routes sorted by metric
     */
    std::vector<LeoSimRoute> ComputeAlternativeRoutes(Ptr<Node> source,
                                                       Ptr<Node> destination,
                                                       uint32_t numRoutes,
                                                       RoutingMetric metric = LEOSIM_METRIC_HOP_COUNT);

    /**
     * \brief Check if a direct link exists between two nodes
     * \param source Source node
     * \param destination Destination node
     * \param linkType Link type filter (LEOSIM_LINK_ISL or LEOSIM_LINK_SATELLITE_TO_GROUND)
     * \return True if link exists and is up
     */
    bool HasDirectLink(Ptr<Node> source,
                       Ptr<Node> destination,
                       LeoSimLinkType linkType = LEOSIM_LINK_ISL);

    /**
     * \brief Get all neighbors of a node using active links
     * \param node The node
     * \param linkType Link type filter (optional)
     * \return Set of neighbor nodes
     */
    std::set<Ptr<Node>> GetNeighbors(Ptr<Node> node, LeoSimLinkType linkType = LEOSIM_LINK_ISL);

    /**
     * \brief Get all ISL neighbors of a satellite
     * \param satellite Satellite node
     * \return Set of ISL-connected neighbor satellites
     */
    std::set<Ptr<Node>> GetIslNeighbors(Ptr<Node> satellite);

    /**
     * \brief Get all ground link neighbors of a node
     * \param node Node (can be satellite or ground)
     * \return Set of neighbors connected via ground links
     */
    std::set<Ptr<Node>> GetGroundNeighbors(Ptr<Node> node);

    /**
     * \brief Get the topology as an adjacency list
     * \param pathType Type of links to include in topology
     * \return Map of node to set of neighbors
     */
    std::map<Ptr<Node>, std::set<Ptr<Node>>> GetTopology(PathType pathType = LEOSIM_PATH_ANY);

    /**
     * \brief Get channel quality for a link
     * \param source Source node
     * \param destination Destination node
     * \return Channel quality metrics
     */
    LeoSimChannelQuality GetLinkQuality(Ptr<Node> source, Ptr<Node> destination);

    /**
     * \brief Check if link is available (UP state)
     * \param source Source node
     * \param destination Destination node
     * \return True if link is up
     */
    bool IsLinkAvailable(Ptr<Node> source, Ptr<Node> destination);

    /**
     * \brief Get all active ISL links
     * \return Vector of node pairs with active ISL links
     */
    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> GetActiveIslLinks();

    /**
     * \brief Get all active ground links
     * \return Vector of node pairs with active ground links
     */
    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> GetActiveGroundLinks();

    /**
     * \brief Get all active links (ISL and ground)
     * \return Vector of node pairs with active links
     */
    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> GetActiveLinks();

    /**
     * \brief Set verbose output for debugging
     * \param verbose True to enable verbose logging
     */
    void SetVerbose(bool verbose);

    /**
     * \brief Get number of active links
     * \return Count of active links
     */
    uint32_t GetNumActiveLinks();

    /**
     * \brief Get number of active ISL links
     * \return Count of active ISL links
     */
    uint32_t GetNumActiveIslLinks();

    /**
     * \brief Get number of active ground links
     * \return Count of active ground links
     */
    uint32_t GetNumActiveGroundLinks();

    /**
     * \brief Invalidate all cached routes containing a specified node
     * 
     * Removes all cached routes from m_routeCache that contain the given nodeId
     * anywhere in their path vector. Schedules an immediate recompute via
     * Simulator::ScheduleNow for affected source-destination pairs.
     * 
     * This is called during handover to clear stale paths that go through the
     * old serving satellite.
     * 
     * \param nodeId The node ID to invalidate routes through
     */
    void InvalidateRoutesForNode(uint32_t nodeId);

    /**
     * \brief Force immediate recomputation and installation of routes for a UE
     * 
     * Immediately recomputes and reinstalls all routes that start or end at ueNodeId,
     * using newSatId as the serving satellite. Calls ComputeRoute internally and
     * updates the ns-3 Ipv4StaticRouting tables with the new paths.
     * 
     * This is typically called at CHO completion to install the new handover paths
     * into the data plane immediately.
     * 
     * \param ueNodeId UE node identifier
     * \param newSatId The target satellite (new serving beam) node ID
     */
    void ForceRouteUpdate(uint32_t ueNodeId, uint32_t newSatId);

    /**
     * \brief Pre-compute and cache route for later installation
     * 
     * Runs Dijkstra's algorithm from ueNodeId via candidateSatId and caches the
     * resulting LeoSimRoute in m_routeCache without installing it into routing
     * tables yet. This is called during CHO Phase 1 (preparation) so that
     * ForceRouteUpdate at CHO completion is near-instant (cache hit).
     * 
     * The route is cached keyed by the pair (ueNodeId, candidateSatId) and can
     * be looked up later without recomputation.
     * 
     * \param ueNodeId UE node identifier
     * \param candidateSatId Candidate satellite (prospective serving beam) node ID
     */
    void PreComputeRouteForNode(uint32_t ueNodeId, uint32_t candidateSatId);

  private:
    friend class LeoSimDijkstraRoutingModel;

    LeoSimRoutingRequest BuildRoutingRequest(Ptr<Node> source,
                                             Ptr<Node> destination,
                                             RoutingMetric metric,
                                             PathType pathType,
                                             double minSnr) const;

    LeoSimRoutingContext BuildRoutingContext(PathType pathType);

    bool ValidateRoute(const LeoSimRoute& route, const LeoSimRoutingRequest& request);

    /**
     * \brief Dijkstra's algorithm implementation for route computation
     * \param source Source node
     * \param destination Destination node
     * \param metric Routing metric to use
     * \param pathType Type of path allowed
     * \param snrConstraint Optional SNR constraint (use -1 for no constraint)
     * \return Computed route
     */
    LeoSimRoute DijkstrasAlgorithm(Ptr<Node> source,
                                    Ptr<Node> destination,
                                    RoutingMetric metric,
                                    PathType pathType,
                                    double snrConstraint = -1.0);

    /**
     * \brief Get metric value for a link based on routing metric
     * \param source Source node
     * \param destination Destination node
     * \param metric Routing metric
     * \return Metric value
     */
    double GetLinkMetricValue(Ptr<Node> source,
                              Ptr<Node> destination,
                              RoutingMetric metric);

    /**
     * \brief Reconstruct path from predecessor map
     * \param source Source node
     * \param destination Destination node
     * \param previous Predecessor map
     * \param distances Distance map
     * \param metric Routing metric used
     * \return Route with detailed information
     */
    LeoSimRoute ReconstructRoute(Ptr<Node> source,
                                  Ptr<Node> destination,
                                  const std::map<Ptr<Node>, Ptr<Node>>& previous,
                                  const std::map<Ptr<Node>, double>& distances,
                                  RoutingMetric metric,
                                  PathType pathType);

    /**
     * \brief Check if link can be used given path type constraint
     * \param source Source node
     * \param destination Destination node
     * \param pathType Path type constraint
     * \return True if link can be used
     */
    bool IsLinkAllowed(Ptr<Node> source, Ptr<Node> destination, PathType pathType);

    /**
     * \brief Check beam-manager authority for a satellite-ground access link.
     * \param source First endpoint.
     * \param destination Second endpoint.
     * \return True if the access link may be used for routing.
     */
    bool IsAccessLinkAllowed(Ptr<Node> source, Ptr<Node> destination);

    /**
     * \brief Check if link meets SNR constraint
     * \param source Source node
     * \param destination Destination node
     * \param minSnr Minimum SNR requirement
     * \return True if link meets SNR requirement
     */
    bool MeetsSnrConstraint(Ptr<Node> source, Ptr<Node> destination, double minSnr);

    /**
     * \brief Compute edge weight with optional operator sharing multiplier.
     * \param nodeA First node ID
     * \param nodeB Second node ID
     * \param dir Link direction
     * \param baseCost Baseline edge cost
     * \return Effective weighted edge cost
     */
    double GetEdgeWeight(uint32_t nodeA,
               uint32_t nodeB,
               LeoSimLinkDirection dir,
               double baseCost) const;

    // Pointer to the channel model for accessing link information
    Ptr<LeoSimChannelModel> m_channelModel;

    // Pointer to the ISL channel model for inter-satellite links
    Ptr<LeoSimChannelModel> m_islChannelModel;

    // Beam manager authority for satellite-ground access links
    Ptr<LeoSimBeamManager> m_beamManager;

    // Pointer to operator model for operator-aware costs and constraints
    Ptr<LeoSimOperatorModel> m_operatorModel;

    // Replaceable route computation provider
    Ptr<LeoSimRouteProvider> m_routeProvider;

    // Optional custom edge-cost callback for the default Dijkstra provider
    LeoSimEdgeCostCallback m_edgeCostCallback;

    // Default path type (used when caller does not specify one)
    PathType m_defaultPathType;

    // Access-link policy
    AccessLinkPolicy m_accessLinkPolicy;
    uint32_t m_multiConnectivityMaxLinks;
    bool m_accessAuthorityEnabled;

    // Configuration
    bool m_verbose;  //!< Enable verbose logging

    // Cache for topology (optional optimization)
    std::map<Ptr<Node>, std::set<Ptr<Node>>> m_cachedTopology;
    Time m_lastTopologyCacheUpdate;
    Time m_topologyCacheTTL;  //!< Time-to-live for cached topology
    mutable std::mutex m_topologyCacheMutex; //!< Protects topology cache during parallel route calculations

    // Route cache: keyed by (source node ID, destination node ID) pair
    std::map<std::pair<uint32_t, uint32_t>, LeoSimRoute> m_routeCache;  //!< Cached routes
};

/**
 * \ingroup leosim
 * \brief Public route-computation request passed to custom routing providers.
 */
struct LeoSimRoutingRequest
{
    Ptr<Node> source;       //!< Source node.
    Ptr<Node> destination;  //!< Destination node.
    Time time;              //!< Simulation time at request creation.

    LeoSimRoutingCalculator::RoutingMetric metric; //!< Requested routing metric.
    LeoSimRoutingCalculator::PathType pathType;    //!< Requested path constraint.
    double minSnr;                                  //!< Minimum SNR, or < 0 for none.
};

/**
 * \ingroup leosim
 * \brief Public routing state passed to custom routing providers.
 */
struct LeoSimRoutingContext
{
    Ptr<LeoSimChannelModel> channelModel;       //!< Satellite-ground channel model.
    Ptr<LeoSimChannelModel> islChannelModel;    //!< ISL channel model, if configured.
    Ptr<LeoSimBeamManager> beamManager;         //!< Beam manager access authority.
    Ptr<LeoSimOperatorModel> operatorModel;     //!< Operator policy model.

    LeoSimRoutingCalculator::AccessLinkPolicy accessPolicy; //!< Active access-link policy.
    uint32_t multiConnectivityMaxLinks;                     //!< Max MC links per ground node.
    bool accessAuthorityEnabled;                            //!< Whether beam authority is enforced.

    std::map<Ptr<Node>, std::set<Ptr<Node>>> topology; //!< Active topology graph.

    Ptr<LeoSimRoutingCalculator> calculator; //!< Calculator exposing link-quality helpers.
};

/**
 * \ingroup leosim
 * \brief Replaceable route-computation interface.
 */
class LeoSimRouteProvider : public Object
{
  public:
    static TypeId GetTypeId();

    virtual LeoSimRoute ComputeRoute(const LeoSimRoutingRequest& request,
                                     const LeoSimRoutingContext& context) = 0;
};

/**
 * \ingroup leosim
 * \brief Default Dijkstra route provider.
 */
class LeoSimDijkstraRoutingModel : public LeoSimRouteProvider
{
  public:
    static TypeId GetTypeId();

    LeoSimRoute ComputeRoute(const LeoSimRoutingRequest& request,
                             const LeoSimRoutingContext& context) override;
};

} // namespace ns3

#endif /* LEOSIM_ROUTING_CALCULATOR_H */
