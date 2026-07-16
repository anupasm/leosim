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

#include "leosim-routing-calculator.h"
#include "leosim-beam-manager.h"
#include "leosim-mobility-model.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node-list.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

namespace
{

bool
IsGroundNode(const Ptr<Node>& node)
{
    if (!node)
    {
        return false;
    }

    Ptr<LeoSimMobilityModel> mobility = node->GetObject<LeoSimMobilityModel>();
    if (!mobility)
    {
        return false;
    }

    LeoSimNodeType type = mobility->GetNodeType();
    return (type == LEOSIM_UE || type == LEOSIM_GATEWAY);
}

} // namespace

NS_LOG_COMPONENT_DEFINE("LeoSimRoutingCalculator");
NS_OBJECT_ENSURE_REGISTERED(LeoSimRoutingCalculator);
NS_OBJECT_ENSURE_REGISTERED(LeoSimRouteProvider);
NS_OBJECT_ENSURE_REGISTERED(LeoSimDijkstraRoutingModel);

TypeId
LeoSimRouteProvider::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimRouteProvider")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim");
    return tid;
}

TypeId
LeoSimDijkstraRoutingModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimDijkstraRoutingModel")
                            .SetParent<LeoSimRouteProvider>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimDijkstraRoutingModel>();
    return tid;
}

LeoSimRoute
LeoSimDijkstraRoutingModel::ComputeRoute(const LeoSimRoutingRequest& request,
                                         const LeoSimRoutingContext& context)
{
    if (!context.calculator)
    {
        LeoSimRoute invalidRoute;
        invalidRoute.valid = false;
        return invalidRoute;
    }

    return context.calculator->DijkstrasAlgorithm(request.source,
                                                  request.destination,
                                                  request.metric,
                                                  request.pathType,
                                                  request.minSnr);
}

TypeId
LeoSimRoutingCalculator::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimRoutingCalculator")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimRoutingCalculator>();
    return tid;
}

LeoSimRoutingCalculator::LeoSimRoutingCalculator()
        : m_defaultPathType(LEOSIM_PATH_ANY),
          m_accessLinkPolicy(LEOSIM_ACCESS_SERVING_ONLY),
          m_multiConnectivityMaxLinks(1),
          m_accessAuthorityEnabled(true),
          m_verbose(false),
      m_topologyCacheTTL(Seconds(0.1))
{
    NS_LOG_FUNCTION(this);
    m_routeProvider = CreateObject<LeoSimDijkstraRoutingModel>();
}

LeoSimRoutingCalculator::~LeoSimRoutingCalculator()
{
    NS_LOG_FUNCTION(this);
}

void
LeoSimRoutingCalculator::SetChannelModel(Ptr<LeoSimChannelModel> channelModel)
{
    NS_LOG_FUNCTION(this << channelModel);
    m_channelModel = channelModel;
    m_cachedTopology.clear();
}

Ptr<LeoSimChannelModel>
LeoSimRoutingCalculator::GetChannelModel() const
{
    return m_channelModel;
}

void
LeoSimRoutingCalculator::SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel)
{
    NS_LOG_FUNCTION(this << islChannelModel);
    m_islChannelModel = islChannelModel;
    m_cachedTopology.clear();
}

Ptr<LeoSimChannelModel>
LeoSimRoutingCalculator::GetIslChannelModel() const
{
    return m_islChannelModel;
}

void
LeoSimRoutingCalculator::SetBeamManager(Ptr<LeoSimBeamManager> beamManager)
{
    NS_LOG_FUNCTION(this << beamManager);
    m_beamManager = beamManager;
    m_cachedTopology.clear();
}

void
LeoSimRoutingCalculator::SetAccessLinkPolicy(AccessLinkPolicy policy)
{
    NS_LOG_FUNCTION(this << policy);
    m_accessLinkPolicy = policy;
    m_cachedTopology.clear();
}

void
LeoSimRoutingCalculator::SetAccessAuthorityEnabled(bool enabled)
{
    NS_LOG_FUNCTION(this << enabled);
    if (m_accessAuthorityEnabled != enabled)
    {
        m_accessAuthorityEnabled = enabled;
        m_cachedTopology.clear();
    }
}

bool
LeoSimRoutingCalculator::IsAccessAuthorityEnabled() const
{
    return m_accessAuthorityEnabled;
}

void
LeoSimRoutingCalculator::SetRouteProvider(Ptr<LeoSimRouteProvider> provider)
{
    NS_LOG_FUNCTION(this << provider);
    if (provider)
    {
        m_routeProvider = provider;
    }
    else
    {
        m_routeProvider = CreateObject<LeoSimDijkstraRoutingModel>();
    }
}

Ptr<LeoSimRouteProvider>
LeoSimRoutingCalculator::GetRouteProvider() const
{
    return m_routeProvider;
}

void
LeoSimRoutingCalculator::SetEdgeCostCallback(LeoSimEdgeCostCallback callback)
{
    NS_LOG_FUNCTION(this);
    m_edgeCostCallback = callback;
}

LeoSimEdgeCostCallback
LeoSimRoutingCalculator::GetEdgeCostCallback() const
{
    return m_edgeCostCallback;
}

void
LeoSimRoutingCalculator::SetMultiConnectivityMaxLinks(uint32_t maxLinks)
{
    NS_LOG_FUNCTION(this << maxLinks);
    m_multiConnectivityMaxLinks = std::max<uint32_t>(1, maxLinks);
    m_cachedTopology.clear();
}

void
LeoSimRoutingCalculator::SetOperatorModel(Ptr<LeoSimOperatorModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_operatorModel = model;
}

void
LeoSimRoutingCalculator::SetPathType(PathType pathType)
{
    NS_LOG_FUNCTION(this << pathType);
    m_defaultPathType = pathType;
    m_cachedTopology.clear();
}

void
LeoSimRoutingCalculator::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
}

LeoSimRoutingRequest
LeoSimRoutingCalculator::BuildRoutingRequest(Ptr<Node> source,
                                             Ptr<Node> destination,
                                             RoutingMetric metric,
                                             PathType pathType,
                                             double minSnr) const
{
    LeoSimRoutingRequest request;
    request.source = source;
    request.destination = destination;
    request.time = Simulator::Now();
    request.metric = metric;
    request.pathType = pathType;
    request.minSnr = minSnr;
    return request;
}

LeoSimRoutingContext
LeoSimRoutingCalculator::BuildRoutingContext(PathType pathType)
{
    LeoSimRoutingContext context;
    context.channelModel = m_channelModel;
    context.islChannelModel = m_islChannelModel;
    context.beamManager = m_beamManager;
    context.operatorModel = m_operatorModel;
    context.accessPolicy = m_accessLinkPolicy;
    context.multiConnectivityMaxLinks = m_multiConnectivityMaxLinks;
    context.accessAuthorityEnabled = m_accessAuthorityEnabled;
    context.topology = GetTopology(pathType);
    context.calculator = GetObject<LeoSimRoutingCalculator>();
    return context;
}

bool
LeoSimRoutingCalculator::ValidateRoute(const LeoSimRoute& route,
                                       const LeoSimRoutingRequest& request)
{
    if (!route.valid || route.path.empty())
    {
        return false;
    }

    if (route.path.front() != request.source || route.path.back() != request.destination)
    {
        NS_LOG_WARN("Route provider returned a path with mismatched endpoints");
        return false;
    }

    if (route.path.size() == 1)
    {
        return request.source == request.destination;
    }

    for (size_t i = 0; i + 1 < route.path.size(); ++i)
    {
        Ptr<Node> current = route.path[i];
        Ptr<Node> next = route.path[i + 1];

        if (!current || !next)
        {
            return false;
        }

        if (request.pathType == LEOSIM_PATH_SAME_OPERATOR_ONLY && m_operatorModel &&
            !m_operatorModel->IsSameOperator(current->GetId(), next->GetId()))
        {
            NS_LOG_WARN("Route provider returned a cross-operator hop for SAME_OPERATOR_ONLY");
            return false;
        }

        if (!IsLinkAllowed(current, next, request.pathType))
        {
            NS_LOG_WARN("Route provider returned an unavailable or disallowed hop");
            return false;
        }

        if (request.minSnr >= 0 && !MeetsSnrConstraint(current, next, request.minSnr))
        {
            NS_LOG_WARN("Route provider returned a hop below the requested SNR constraint");
            return false;
        }
    }

    return true;
}

LeoSimRoute
LeoSimRoutingCalculator::ComputeRoute(Ptr<Node> source,
                                       Ptr<Node> destination,
                                       RoutingMetric metric,
                                       PathType pathType)
{
    NS_LOG_FUNCTION(this << source << destination << metric << pathType);

    // Apply the instance-level default when caller passes LEOSIM_PATH_ANY
    // but a stricter default has been configured via SetPathType.
    if (pathType == LEOSIM_PATH_ANY && m_defaultPathType != LEOSIM_PATH_ANY)
    {
        pathType = m_defaultPathType;
    }

    if (!m_channelModel)
    {
        NS_LOG_ERROR("Channel model not set");
        LeoSimRoute invalidRoute;
        invalidRoute.valid = false;
        return invalidRoute;
    }

    if (source == destination)
    {
        NS_LOG_WARN("Source and destination are the same");
        LeoSimRoute sameNodeRoute;
        sameNodeRoute.path.push_back(source);
        sameNodeRoute.hopCount = 0;
        sameNodeRoute.valid = true;
        return sameNodeRoute;
    }

    LeoSimRoutingRequest request = BuildRoutingRequest(source, destination, metric, pathType, -1.0);
    LeoSimRoutingContext context = BuildRoutingContext(pathType);
    Ptr<LeoSimRouteProvider> provider = m_routeProvider;
    if (!provider)
    {
        provider = CreateObject<LeoSimDijkstraRoutingModel>();
    }
    LeoSimRoute route = provider->ComputeRoute(request, context);
    if (!ValidateRoute(route, request))
    {
        route.valid = false;
    }
    return route;
}

LeoSimRoute
LeoSimRoutingCalculator::ComputeRouteWithSnrConstraint(Ptr<Node> source,
                                                        Ptr<Node> destination,
                                                        double minSnr,
                                                        RoutingMetric metric)
{
    NS_LOG_FUNCTION(this << source << destination << minSnr << metric);

    if (!m_channelModel)
    {
        NS_LOG_ERROR("Channel model not set");
        LeoSimRoute invalidRoute;
        invalidRoute.valid = false;
        return invalidRoute;
    }

    PathType pathType = m_defaultPathType;
    LeoSimRoutingRequest request = BuildRoutingRequest(source,
                                                       destination,
                                                       metric,
                                                       pathType,
                                                       minSnr);
    LeoSimRoutingContext context = BuildRoutingContext(pathType);
    Ptr<LeoSimRouteProvider> provider = m_routeProvider;
    if (!provider)
    {
        provider = CreateObject<LeoSimDijkstraRoutingModel>();
    }
    LeoSimRoute route = provider->ComputeRoute(request, context);
    if (!ValidateRoute(route, request))
    {
        route.valid = false;
    }
    return route;
}

std::vector<LeoSimRoute>
LeoSimRoutingCalculator::ComputeAlternativeRoutes(Ptr<Node> source,
                                                   Ptr<Node> destination,
                                                   uint32_t numRoutes,
                                                   RoutingMetric metric)
{
    NS_LOG_FUNCTION(this << source << destination << numRoutes << metric);

    std::vector<LeoSimRoute> routes;

    if (!m_channelModel)
    {
        NS_LOG_ERROR("Channel model not set");
        return routes;
    }

    // For now, return the single best route
    // TODO: Implement full K-shortest paths algorithm
    LeoSimRoute bestRoute = ComputeRoute(source, destination, metric);
    if (bestRoute.valid)
    {
        routes.push_back(bestRoute);
    }

    return routes;
}

bool
LeoSimRoutingCalculator::HasDirectLink(Ptr<Node> source,
                                        Ptr<Node> destination,
                                        LeoSimLinkType linkType)
{
    NS_LOG_FUNCTION(this << source << destination << linkType);

    if (!m_channelModel)
    {
        return false;
    }

    // Check if link is available and is of the correct type
    if (!IsLinkAvailable(source, destination))
    {
        return false;
    }

    // If any link type is acceptable, just check availability
    if (linkType == LEOSIM_LINK_ISL || linkType == LEOSIM_LINK_SATELLITE_TO_GROUND)
    {
        LeoSimChannelQuality quality = GetLinkQuality(source, destination);
        if (quality.linkType == linkType)
        {
            if (quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND)
            {
                return IsAccessLinkAllowed(source, destination);
            }
            return true;
        }
    }

    return false;
}

std::set<Ptr<Node>>
LeoSimRoutingCalculator::GetNeighbors(Ptr<Node> node, LeoSimLinkType linkType)
{
    NS_LOG_FUNCTION(this << node << linkType);

    std::set<Ptr<Node>> neighbors;

    if (!m_channelModel)
    {
        return neighbors;
    }

    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> activeLinks = GetActiveLinks();

    for (const auto& link : activeLinks)
    {
        LeoSimChannelQuality quality = GetLinkQuality(link.first, link.second);

        if (link.first == node && quality.linkType == linkType)
        {
            if (quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND &&
                !IsAccessLinkAllowed(link.first, link.second))
            {
                continue;
            }
            neighbors.insert(link.second);
        }
        else if (link.second == node && quality.linkType == linkType)
        {
            if (quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND &&
                !IsAccessLinkAllowed(link.first, link.second))
            {
                continue;
            }
            neighbors.insert(link.first);
        }
    }

    return neighbors;
}

std::set<Ptr<Node>>
LeoSimRoutingCalculator::GetIslNeighbors(Ptr<Node> satellite)
{
    NS_LOG_FUNCTION(this << satellite);
    return GetNeighbors(satellite, LEOSIM_LINK_ISL);
}

std::set<Ptr<Node>>
LeoSimRoutingCalculator::GetGroundNeighbors(Ptr<Node> node)
{
    NS_LOG_FUNCTION(this << node);
    return GetNeighbors(node, LEOSIM_LINK_SATELLITE_TO_GROUND);
}

std::map<Ptr<Node>, std::set<Ptr<Node>>>
LeoSimRoutingCalculator::GetTopology(PathType pathType)
{
    NS_LOG_FUNCTION(this << pathType);

    std::map<Ptr<Node>, std::set<Ptr<Node>>> topology;

    if (!m_channelModel)
    {
        return topology;
    }

    // Check if cache is still valid (optional optimization)
    Time now = Simulator::Now();
    {
        std::lock_guard<std::mutex> lock(m_topologyCacheMutex);
        if (m_lastTopologyCacheUpdate + m_topologyCacheTTL > now && !m_cachedTopology.empty())
        {
            return m_cachedTopology;
        }
    }

    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> activeLinks = m_channelModel->GetActiveLinks();

    for (const auto& link : activeLinks)
    {
        Ptr<Node> node1 = link.first;
        Ptr<Node> node2 = link.second;
        LeoSimChannelQuality quality = GetLinkQuality(node1, node2);

        if (quality.linkState == LEOSIM_LINK_DOWN)
        {
            continue;
        }

        if (pathType != LEOSIM_PATH_ANY)
        {
            if (pathType == LEOSIM_PATH_ISL_ONLY && quality.linkType != LEOSIM_LINK_ISL)
            {
                continue;
            }
            if (pathType == LEOSIM_PATH_GROUND_ONLY &&
                quality.linkType != LEOSIM_LINK_SATELLITE_TO_GROUND)
            {
                continue;
            }
        }

        if (quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND)
        {
            if (!IsAccessLinkAllowed(node1, node2))
            {
                continue;
            }
        }

        topology[node1].insert(node2);
        topology[node2].insert(node1);
    }

    // Get active links from ISL channel model (if set)
    if (m_islChannelModel)
    {
        std::vector<std::pair<Ptr<Node>, Ptr<Node>>> islLinks = m_islChannelModel->GetActiveLinks();

        for (const auto& link : islLinks)
        {
            Ptr<Node> node1 = link.first;
            Ptr<Node> node2 = link.second;
            LeoSimChannelQuality quality = m_islChannelModel->GetChannelQuality(node1, node2);

            if (quality.linkState == LEOSIM_LINK_DOWN)
            {
                continue;
            }

            if (pathType != LEOSIM_PATH_ANY)
            {
                if (pathType == LEOSIM_PATH_GROUND_ONLY && quality.linkType != LEOSIM_LINK_SATELLITE_TO_GROUND)
                {
                    continue;
                }
                // ISL_ONLY doesn't need special handling as we're reading from ISL model
            }

            topology[node1].insert(node2);
            topology[node2].insert(node1);
        }
    }

    {
        std::lock_guard<std::mutex> lock(m_topologyCacheMutex);
        m_cachedTopology = topology;
        m_lastTopologyCacheUpdate = now;
    }

    return topology;
}

LeoSimChannelQuality
LeoSimRoutingCalculator::GetLinkQuality(Ptr<Node> source, Ptr<Node> destination)
{
    NS_LOG_FUNCTION(this << source << destination);

    if (!m_channelModel)
    {
        LeoSimChannelQuality quality;
        quality.linkState = LEOSIM_LINK_DOWN;
        return quality;
    }

    // Check ground channel model first
    LeoSimChannelQuality quality = m_channelModel->GetChannelQuality(source, destination);
    
    // If ground link is visible, return it
    if (quality.linkState == LEOSIM_LINK_UP || quality.linkState == LEOSIM_LINK_DEGRADED)
    {
        return quality;
    }

    // Check ISL channel model if ground link is not available
    if (m_islChannelModel)
    {
        LeoSimChannelQuality islQuality = m_islChannelModel->GetChannelQuality(source, destination);
        if (islQuality.linkState == LEOSIM_LINK_UP || islQuality.linkState == LEOSIM_LINK_DEGRADED)
        {
            return islQuality;
        }
    }

    return quality;
}

bool
LeoSimRoutingCalculator::IsLinkAvailable(Ptr<Node> source, Ptr<Node> destination)
{
    NS_LOG_FUNCTION(this << source << destination);

    if (!m_channelModel)
    {
        return false;
    }

    // Check if link exists in ground channel model
    bool groundLink = m_channelModel->IsLinkUp(source, destination);
    if (groundLink)
    {
        LeoSimChannelQuality quality = m_channelModel->GetChannelQuality(source, destination);
        if (quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND)
        {
            return IsAccessLinkAllowed(source, destination);
        }
        return quality.linkType == LEOSIM_LINK_ISL;
    }

    // Check if link exists in ISL channel model (if set)
    if (m_islChannelModel && m_islChannelModel->IsLinkUp(source, destination))
    {
        return true;
    }

    return false;
}

bool
LeoSimRoutingCalculator::IsAccessLinkAllowed(Ptr<Node> source, Ptr<Node> destination)
{
    if (!m_accessAuthorityEnabled)
    {
        return true;
    }

    LeoSimAccessLinkState state = GetAccessLinkState(source, destination);
    return state == LEOSIM_ACCESS_NOT_APPLICABLE ||
           state == LEOSIM_ACCESS_SERVING ||
           (m_accessLinkPolicy == LEOSIM_ACCESS_SERVING_AND_CHO &&
            state == LEOSIM_ACCESS_PREPARED) ||
           (m_accessLinkPolicy == LEOSIM_ACCESS_MULTI_CONNECTIVITY &&
            state == LEOSIM_ACCESS_MULTI_CONNECTIVITY_CANDIDATE);
}

LeoSimRoutingCalculator::LeoSimAccessLinkState
LeoSimRoutingCalculator::GetAccessLinkState(Ptr<Node> source, Ptr<Node> destination)
{
    if (!source || !destination)
    {
        return LEOSIM_ACCESS_CHANNEL_DOWN;
    }

    Ptr<Node> groundNode = nullptr;
    Ptr<Node> satNode = nullptr;

    if (IsGroundNode(source))
    {
        groundNode = source;
        satNode = destination;
    }
    else if (IsGroundNode(destination))
    {
        groundNode = destination;
        satNode = source;
    }

    if (!groundNode || !satNode)
    {
        return LEOSIM_ACCESS_NOT_APPLICABLE;
    }

    LeoSimChannelQuality quality = GetLinkQuality(source, destination);
    if (quality.linkState != LEOSIM_LINK_UP && quality.linkState != LEOSIM_LINK_DEGRADED)
    {
        return LEOSIM_ACCESS_CHANNEL_DOWN;
    }

    if (!m_beamManager)
    {
        return LEOSIM_ACCESS_SERVING;
    }

    const uint32_t groundId = groundNode->GetId();
    const uint32_t satId = satNode->GetId();

    if (m_beamManager->IsServingAccessLink(groundId, satId))
    {
        return LEOSIM_ACCESS_SERVING;
    }

    LeoSimBeamRecord current = m_beamManager->GetCurrentBeam(groundId);
    if (current.satelliteNodeId == satId)
    {
        return LEOSIM_ACCESS_BEAM_DARK;
    }

    const auto prepared = m_beamManager->GetPreparedCandidateBeams(groundId);
    auto preparedIt = std::find_if(prepared.begin(),
                                   prepared.end(),
                                   [satId](const LeoSimBeamRecord& rec) {
                                       return rec.satelliteNodeId == satId;
                                   });
    if (preparedIt != prepared.end())
    {
        return preparedIt->beamActive ? LEOSIM_ACCESS_PREPARED : LEOSIM_ACCESS_BEAM_DARK;
    }

    const auto ranked = m_beamManager->GetRankedCandidates(groundId);
    uint32_t accepted = 0;
    bool hasEligibleBeam = false;
    for (const auto& candidate : ranked)
    {
        const LeoSimBeamRecord& rec = candidate.beamRecord;
        if (!rec.beamActive)
        {
            if (rec.satelliteNodeId == satId)
            {
                return LEOSIM_ACCESS_BEAM_DARK;
            }
            continue;
        }

        const bool withinMultiConnectivityLimit = accepted < m_multiConnectivityMaxLinks;
        ++accepted;

        if (rec.satelliteNodeId != satId)
        {
            continue;
        }

        hasEligibleBeam = true;
        if (m_accessLinkPolicy == LEOSIM_ACCESS_MULTI_CONNECTIVITY &&
            withinMultiConnectivityLimit)
        {
            return LEOSIM_ACCESS_MULTI_CONNECTIVITY_CANDIDATE;
        }

        break;
    }

    return hasEligibleBeam ? LEOSIM_ACCESS_NOT_SELECTED : LEOSIM_ACCESS_NO_ELIGIBLE_BEAM;
}

std::vector<std::pair<Ptr<Node>, Ptr<Node>>>
LeoSimRoutingCalculator::GetActiveIslLinks()
{
    NS_LOG_FUNCTION(this);

    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> islLinks;

    if (!m_channelModel)
    {
        return islLinks;
    }

    Ptr<LeoSimChannelModel> islModel = m_islChannelModel ? m_islChannelModel : m_channelModel;
    std::vector<LeoSimChannelModel::LinkSnapshot> links =
        islModel->GetLinksByType(LEOSIM_LINK_ISL, false);

    for (const auto& link : links)
    {
        islLinks.push_back(std::make_pair(link.node1, link.node2));
    }

    return islLinks;
}

std::vector<std::pair<Ptr<Node>, Ptr<Node>>>
LeoSimRoutingCalculator::GetActiveGroundLinks()
{
    NS_LOG_FUNCTION(this);

    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> groundLinks;

    if (!m_channelModel)
    {
        return groundLinks;
    }

    std::vector<LeoSimChannelModel::LinkSnapshot> links =
        m_channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, false);

    for (const auto& link : links)
    {
        if (IsAccessLinkAllowed(link.node1, link.node2))
        {
            groundLinks.push_back(std::make_pair(link.node1, link.node2));
        }
    }

    return groundLinks;
}

std::vector<std::pair<Ptr<Node>, Ptr<Node>>>
LeoSimRoutingCalculator::GetActiveLinks()
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        return std::vector<std::pair<Ptr<Node>, Ptr<Node>>>();
    }

    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> filteredLinks;
    std::set<std::pair<uint32_t, uint32_t>> seenLinks;
    auto appendIfNew = [&filteredLinks, &seenLinks](Ptr<Node> a, Ptr<Node> b) {
        if (!a || !b)
        {
            return;
        }
        auto key = std::make_pair(std::min(a->GetId(), b->GetId()),
                                  std::max(a->GetId(), b->GetId()));
        if (seenLinks.insert(key).second)
        {
            filteredLinks.push_back(std::make_pair(a, b));
        }
    };

    for (const auto& link : m_channelModel->GetActiveLinks())
    {
        LeoSimChannelQuality quality = GetLinkQuality(link.first, link.second);
        if (quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND &&
            !IsAccessLinkAllowed(link.first, link.second))
        {
            continue;
        }
        appendIfNew(link.first, link.second);
    }

    if (m_islChannelModel)
    {
        for (const auto& link : m_islChannelModel->GetActiveLinks())
        {
            appendIfNew(link.first, link.second);
        }
    }

    return filteredLinks;
}

uint32_t
LeoSimRoutingCalculator::GetNumActiveLinks()
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        return 0;
    }

    return GetActiveLinks().size();
}

uint32_t
LeoSimRoutingCalculator::GetNumActiveIslLinks()
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        return 0;
    }

    Ptr<LeoSimChannelModel> islModel = m_islChannelModel ? m_islChannelModel : m_channelModel;
    std::vector<LeoSimChannelModel::LinkSnapshot> links =
        islModel->GetLinksByType(LEOSIM_LINK_ISL, false);
    return links.size();
}

uint32_t
LeoSimRoutingCalculator::GetNumActiveGroundLinks()
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        return 0;
    }

    return GetActiveGroundLinks().size();
}

double
LeoSimRoutingCalculator::GetLinkMetricValue(Ptr<Node> source,
                                             Ptr<Node> destination,
                                             RoutingMetric metric)
{
    LeoSimChannelQuality quality = GetLinkQuality(source, destination);

    if (!m_edgeCostCallback.IsNull())
    {
        return m_edgeCostCallback(source, destination, quality);
    }

    switch (metric)
    {
    case LEOSIM_METRIC_HOP_COUNT:
        return 1.0; // Each hop has weight 1
    case LEOSIM_METRIC_PATH_LOSS:
        return quality.pathLoss; // Lower is better
    case LEOSIM_METRIC_SNR:
        return -quality.snr; // Negative SNR so we minimize it (maximize SNR)
    case LEOSIM_METRIC_DISTANCE:
        return quality.distance; // Lower is better
    case LEOSIM_METRIC_SIGNAL_STRENGTH:
        return -quality.signalStrength; // Negative signal strength so we minimize it (maximize
                                         // signal strength)
    default:
        return 1.0;
    }
}

bool
LeoSimRoutingCalculator::IsLinkAllowed(Ptr<Node> source, Ptr<Node> destination, PathType pathType)
{
    if (pathType == LEOSIM_PATH_ANY)
    {
        return IsLinkAvailable(source, destination);
    }

    LeoSimChannelQuality quality = GetLinkQuality(source, destination);

    if (!IsLinkAvailable(source, destination))
    {
        return false;
    }

    if (pathType == LEOSIM_PATH_ISL_ONLY)
    {
        return quality.linkType == LEOSIM_LINK_ISL;
    }

    if (pathType == LEOSIM_PATH_GROUND_ONLY)
    {
        return quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND;
    }

    // LEOSIM_PATH_SAME_OPERATOR_ONLY: link type is unrestricted; operator check is
    // handled separately in DijkstrasAlgorithm, so allow the link here.
    if (pathType == LEOSIM_PATH_SAME_OPERATOR_ONLY)
    {
        return true;
    }

    return false;
}

bool
LeoSimRoutingCalculator::MeetsSnrConstraint(Ptr<Node> source, Ptr<Node> destination, double minSnr)
{
    if (minSnr < 0)
    {
        return true; // No constraint
    }

    LeoSimChannelQuality quality = GetLinkQuality(source, destination);
    return quality.snr >= minSnr;
}

double
LeoSimRoutingCalculator::GetEdgeWeight(uint32_t nodeA,
                                       uint32_t nodeB,
                                       LeoSimLinkDirection dir,
                                       double baseCost) const
{
    double effectiveCost = baseCost;

    if (m_operatorModel)
    {
        const double multiplier = m_operatorModel->GetRoutingCostMultiplier(nodeA, nodeB, dir);
        if (multiplier > 1e5)
        {
            return std::numeric_limits<double>::infinity();
        }
        effectiveCost *= multiplier;
    }

    // Apply weather-induced routing penalty when attenuation is available.
    if (m_channelModel)
    {
        // Attenuation cache is keyed as (groundNodeId, satNodeId). Check both
        // orders to support calls where endpoint order is unknown at this layer.
        auto attenForward = m_channelModel->GetLastAttenuation(nodeA, nodeB);
        auto attenReverse = m_channelModel->GetLastAttenuation(nodeB, nodeA);
        double totalAttenuation = std::max(attenForward.totalAttenuation_dB,
                                           attenReverse.totalAttenuation_dB);

        if (totalAttenuation > 0.0)
        {
            // Map attenuation to cost multiplier: 1.0 (0 dB) -> 10.0 (30 dB).
            double weatherMultiplier = 1.0 +
                std::pow(totalAttenuation / 30.0, 2.0) * 9.0;
            effectiveCost *= weatherMultiplier;
        }
    }

    return effectiveCost;
}

LeoSimRoute
LeoSimRoutingCalculator::DijkstrasAlgorithm(Ptr<Node> source,
                                             Ptr<Node> destination,
                                             RoutingMetric metric,
                                             PathType pathType,
                                             double snrConstraint)
{
    NS_LOG_FUNCTION(this << source << destination << metric << pathType << snrConstraint);

    std::map<Ptr<Node>, std::set<Ptr<Node>>> topology = GetTopology(pathType);

    // Dijkstra's algorithm
    std::map<Ptr<Node>, double> distances;
    std::map<Ptr<Node>, Ptr<Node>> previous;
    std::set<Ptr<Node>> settled;
    using QueueEntry = std::pair<double, Ptr<Node>>;
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;

    // Initialize distances
    for (const auto& node : topology)
    {
        distances[node.first] = std::numeric_limits<double>::max();
    }

    // Also check if source and destination are in topology
    if (distances.find(source) == distances.end())
    {
        distances[source] = std::numeric_limits<double>::max();
    }
    if (distances.find(destination) == distances.end())
    {
        distances[destination] = std::numeric_limits<double>::max();
    }

    distances[source] = 0;
    queue.push(std::make_pair(0.0, source));

    while (!queue.empty())
    {
        const QueueEntry entry = queue.top();
        queue.pop();

        Ptr<Node> current = entry.second;
        const double currentDistance = entry.first;
        if (current == nullptr)
        {
            continue;
        }
        if (settled.find(current) != settled.end())
        {
            continue;
        }
        if (currentDistance > distances[current])
        {
            continue;
        }

        if (current == destination)
        {
            // Found path to destination
            break;
        }

        settled.insert(current);

        // Check neighbors
        if (topology.find(current) != topology.end())
        {
            for (const auto& neighbor : topology[current])
            {
                if (settled.find(neighbor) != settled.end())
                {
                    continue;
                }

                if (pathType == LEOSIM_PATH_SAME_OPERATOR_ONLY && m_operatorModel &&
                    !m_operatorModel->IsSameOperator(current->GetId(), neighbor->GetId()))
                {
                    continue;
                }

                if (!IsLinkAllowed(current, neighbor, pathType))
                {
                    continue;
                }

                if (snrConstraint >= 0 && !MeetsSnrConstraint(current, neighbor, snrConstraint))
                {
                    continue;
                }

                const double baseLinkCost = GetLinkMetricValue(current, neighbor, metric);
                LeoSimLinkDirection dir = LEOSIM_DIR_DOWNLINK;

                if (m_operatorModel)
                {
                    const LeoSimNodeRole currentRole = m_operatorModel->GetRole(current->GetId());
                    const LeoSimNodeRole neighborRole = m_operatorModel->GetRole(neighbor->GetId());
                    if (currentRole == LEOSIM_ROLE_SATELLITE &&
                        neighborRole == LEOSIM_ROLE_SATELLITE)
                    {
                        dir = LEOSIM_DIR_ISL;
                    }
                }
                else
                {
                    LeoSimChannelQuality quality = GetLinkQuality(current, neighbor);
                    if (quality.linkType == LEOSIM_LINK_ISL)
                    {
                        dir = LEOSIM_DIR_ISL;
                    }
                }

                const double linkMetric =
                    GetEdgeWeight(current->GetId(), neighbor->GetId(), dir, baseLinkCost);
                if (std::isinf(linkMetric))
                {
                    continue;
                }

                double newDistance = distances[current] + linkMetric;

                if (newDistance < distances[neighbor])
                {
                    distances[neighbor] = newDistance;
                    previous[neighbor] = current;
                    queue.push(std::make_pair(newDistance, neighbor));
                }
            }
        }
    }

    return ReconstructRoute(source, destination, previous, distances, metric, pathType);
}

LeoSimRoute
LeoSimRoutingCalculator::ReconstructRoute(Ptr<Node> source,
                                           Ptr<Node> destination,
                                           const std::map<Ptr<Node>, Ptr<Node>>& previous,
                                           const std::map<Ptr<Node>, double>& distances,
                                           RoutingMetric metric,
                                           PathType pathType)
{
    LeoSimRoute route;

    // Check if path exists
    if (previous.find(destination) == previous.end() && source != destination)
    {
        NS_LOG_WARN("No path found from " << source->GetId() << " to " << destination->GetId());
        route.valid = false;
        return route;
    }

    // Reconstruct path
    std::vector<Ptr<Node>> path;
    Ptr<Node> current = destination;

    while (current != nullptr)
    {
        path.push_back(current);

        if (current == source)
        {
            break;
        }

        auto it = previous.find(current);
        if (it == previous.end())
        {
            // Path doesn't exist
            route.valid = false;
            return route;
        }

        current = it->second;
    }

    // Reverse to get source-to-destination order
    std::reverse(path.begin(), path.end());

    // Calculate route metrics
    route.path = path;
    route.hopCount = path.size() - 1;
    route.totalPathLoss = 0.0;
    route.totalDistance = 0.0;
    route.minSignalStrength = std::numeric_limits<double>::max();
    route.minSnr = std::numeric_limits<double>::max();
    route.valid = true;

    // Calculate metrics for each hop
    for (size_t i = 0; i < path.size() - 1; i++)
    {
        Ptr<Node> hop_source = path[i];
        Ptr<Node> hop_dest = path[i + 1];

        LeoSimChannelQuality quality = GetLinkQuality(hop_source, hop_dest);

        route.totalPathLoss += quality.pathLoss;
        route.totalDistance += quality.distance;
        route.minSignalStrength = std::min(route.minSignalStrength, quality.signalStrength);
        route.minSnr = std::min(route.minSnr, quality.snr);

        if (quality.linkType == LEOSIM_LINK_ISL)
        {
            route.hasIslLinks = true;
        }
        if (quality.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND)
        {
            route.hasGroundLinks = true;
        }

        route.linkTypes.push_back(quality.linkType);
    }

    if (m_verbose)
    {
        NS_LOG_DEBUG("Route from " << source->GetId() << " to " << destination->GetId() << ": "
                    << route.hopCount << " hops, "
                    << "PathLoss=" << route.totalPathLoss << "dB, "
                    << "Distance=" << route.totalDistance << "m, "
                    << "MinSNR=" << route.minSnr << "dB");
    }

    return route;
}

void
LeoSimRoutingCalculator::InvalidateRoutesForNode(uint32_t nodeId)
{
    NS_LOG_FUNCTION(this << nodeId);

    std::vector<std::pair<uint32_t, uint32_t>> routesToInvalidate;

    // Find all routes that contain nodeId in their path
    for (auto& entry : m_routeCache)
    {
        const std::pair<uint32_t, uint32_t>& routeKey = entry.first;
        LeoSimRoute& route = entry.second;

        // Check if nodeId appears in the path
        for (const auto& node : route.path)
        {
            if (node && node->GetId() == nodeId)
            {
                routesToInvalidate.push_back(routeKey);
                break;
            }
        }
    }

    // Remove the invalid routes from the cache
    for (const auto& routeKey : routesToInvalidate)
    {
        m_routeCache.erase(routeKey);
        NS_LOG_DEBUG("Invalidated cached route from node " << routeKey.first << " to "
                                                           << routeKey.second
                                                           << " due to node " << nodeId);

        // Schedule immediate recompute for this pair via Simulator::ScheduleNow
        uint32_t sourceId = routeKey.first;
        uint32_t destId = routeKey.second;

        // Lambda to trigger recompute of the route
        Simulator::ScheduleNow([this, sourceId, destId]() {
            const uint32_t nodeCount = NodeList::GetNNodes();
            Ptr<Node> sourceNode = (sourceId < nodeCount) ? NodeList::GetNode(sourceId) : nullptr;
            Ptr<Node> destNode = (destId < nodeCount) ? NodeList::GetNode(destId) : nullptr;

            if (sourceNode && destNode)
            {
                LeoSimRoute newRoute = ComputeRoute(sourceNode, destNode);
                if (newRoute.valid)
                {
                    m_routeCache[std::make_pair(sourceId, destId)] = newRoute;
                    NS_LOG_DEBUG("Recomputed route from " << sourceId << " to " << destId);
                }
            }
        });
    }

    NS_LOG_DEBUG("Invalidated " << routesToInvalidate.size() << " routes containing node "
                               << nodeId);
}

void
LeoSimRoutingCalculator::ForceRouteUpdate(uint32_t ueNodeId, uint32_t newSatId)
{
    NS_LOG_FUNCTION(this << ueNodeId << newSatId);

    Ptr<Node> ueNode = NodeList::GetNode(ueNodeId);
    Ptr<Node> satNode = NodeList::GetNode(newSatId);

    if (!ueNode || !satNode)
    {
        NS_LOG_ERROR("Invalid node IDs: UE=" << ueNodeId << ", SAT=" << newSatId);
        return;
    }

    // Recompute the route from UE to SAT
    LeoSimRoute ueToSat = ComputeRoute(ueNode, satNode);
    if (ueToSat.valid)
    {
        // Update cache
        m_routeCache[std::make_pair(ueNodeId, newSatId)] = ueToSat;
        NS_LOG_DEBUG("Force-updated route cache: UE " << ueNodeId << " -> SAT " << newSatId);
    }

    // Recompute the route from SAT to UE (reverse direction)
    LeoSimRoute satToUe = ComputeRoute(satNode, ueNode);
    if (satToUe.valid)
    {
        // Update cache
        m_routeCache[std::make_pair(newSatId, ueNodeId)] = satToUe;
        NS_LOG_DEBUG("Force-updated route cache: SAT " << newSatId << " -> UE " << ueNodeId);
    }

    // TODO: Update ns-3 Ipv4StaticRouting tables with new routes
    // This would require access to routing protocol helpers and is application-specific
    NS_LOG_DEBUG("Force-updated bidirectional routes for UE " << ueNodeId << " via satellite "
                                                             << newSatId);
}

void
LeoSimRoutingCalculator::PreComputeRouteForNode(uint32_t ueNodeId, uint32_t candidateSatId)
{
    NS_LOG_FUNCTION(this << ueNodeId << candidateSatId);

    Ptr<Node> ueNode = NodeList::GetNode(ueNodeId);
    Ptr<Node> satNode = NodeList::GetNode(candidateSatId);

    if (!ueNode || !satNode)
    {
        NS_LOG_ERROR("Invalid node IDs: UE=" << ueNodeId << ", Candidate SAT=" << candidateSatId);
        return;
    }

    // Route from UE to candidate satellite
    LeoSimRoute ueToSat = ComputeRoute(ueNode, satNode);
    if (ueToSat.valid)
    {
        m_routeCache[std::make_pair(ueNodeId, candidateSatId)] = ueToSat;
        NS_LOG_DEBUG("Pre-computed route cache: UE " << ueNodeId << " -> Candidate SAT "
                                                     << candidateSatId << " (" << ueToSat.hopCount
                                                     << " hops)");
    }
    else
    {
        NS_LOG_WARN("Failed to pre-compute route from UE " << ueNodeId << " to candidate SAT "
                                                           << candidateSatId);
    }

    // Route from candidate satellite to UE
    LeoSimRoute satToUe = ComputeRoute(satNode, ueNode);
    if (satToUe.valid)
    {
        m_routeCache[std::make_pair(candidateSatId, ueNodeId)] = satToUe;
        NS_LOG_DEBUG("Pre-computed route cache: Candidate SAT " << candidateSatId << " -> UE "
                                                                << ueNodeId << " ("
                                                                << satToUe.hopCount << " hops)");
    }
    else
    {
        NS_LOG_WARN("Failed to pre-compute route from candidate SAT " << candidateSatId
                                                                      << " to UE " << ueNodeId);
    }

    NS_LOG_DEBUG("Pre-computed bidirectional routes for UE " << ueNodeId
                                                            << " via candidate satellite "
                                                            << candidateSatId);
}

} // namespace ns3
