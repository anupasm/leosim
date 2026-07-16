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

#include "leosim-routing-calculator-helper.h"

#include "ns3/leosim-routing-calculator.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-task-profiler.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/event-id.h"

#include <algorithm>
#include <vector>

NS_LOG_COMPONENT_DEFINE("LeoSimRoutingCalculatorHelper");

namespace ns3
{

LeoSimRoutingCalculatorHelper::LeoSimRoutingCalculatorHelper()
    : m_reactiveDebounceInterval(MilliSeconds(200))
{
}

LeoSimRoutingCalculatorHelper::~LeoSimRoutingCalculatorHelper()
{
}

Ptr<LeoSimRoutingCalculator>
LeoSimRoutingCalculatorHelper::CreateRoutingCalculator(Ptr<LeoSimChannelModel> channelModel)
{
    return CreateRoutingCalculator(channelModel, false);
}

Ptr<LeoSimRoutingCalculator>
LeoSimRoutingCalculatorHelper::CreateRoutingCalculator(Ptr<LeoSimChannelModel> channelModel,
                                                       bool verbose)
{
    Ptr<LeoSimRoutingCalculator> calculator = CreateObject<LeoSimRoutingCalculator>();
    calculator->SetChannelModel(channelModel);
    calculator->SetVerbose(verbose);
    return calculator;
}

Ptr<LeoSimRoutingCalculator>
LeoSimRoutingCalculatorHelper::CreateUnifiedRoutingCalculator(Ptr<LeoSimChannelModel> groundChannelModel,
                                                               Ptr<LeoSimChannelModel> islChannelModel,
                                                               bool verbose)
{
    Ptr<LeoSimRoutingCalculator> calculator = CreateObject<LeoSimRoutingCalculator>();
    calculator->SetChannelModel(groundChannelModel);
    calculator->SetIslChannelModel(islChannelModel);
    calculator->SetVerbose(verbose);
    return calculator;
}

bool
LeoSimRoutingCalculatorHelper::InstallRoute(const LeoSimRoute& route, bool verbose)
{
    if (!route.valid || route.path.size() < 2)
    {
        return false;
    }

    Ptr<Node> srcNode = route.path.front();
    Ptr<Node> dstNode = route.path.back();
    Ptr<Ipv4> srcIpv4 = srcNode->GetObject<Ipv4>();
    Ptr<Ipv4> dstIpv4 = dstNode->GetObject<Ipv4>();

    if (!srcIpv4 || !dstIpv4)
    {
        if (verbose)
        {
            NS_LOG_DEBUG("Cannot install route: missing IPv4 stack on source or destination");
        }
        return false;
    }

    Ptr<Ipv4StaticRouting> srcStaticRouting =
        Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(srcIpv4->GetRoutingProtocol());
    if (!srcStaticRouting)
    {
        if (verbose)
        {
            NS_LOG_WARN("Source node " << srcNode->GetId() << " has no static routing");
        }
        return false;
    }

    std::vector<Ipv4Address> dstIpAddrs;
    for (uint32_t dstIfIdx = 1; dstIfIdx < dstIpv4->GetNInterfaces(); ++dstIfIdx)
    {
        for (uint32_t dstAddrIdx = 0; dstAddrIdx < dstIpv4->GetNAddresses(dstIfIdx); ++dstAddrIdx)
        {
            Ipv4InterfaceAddress dstAddr = dstIpv4->GetAddress(dstIfIdx, dstAddrIdx);
            Ipv4Address addr = dstAddr.GetLocal();
            if (addr != Ipv4Address::GetZero() && addr != Ipv4Address("127.0.0.1"))
            {
                dstIpAddrs.push_back(addr);
            }
        }
    }

    if (dstIpAddrs.empty())
    {
        if (verbose)
        {
            NS_LOG_DEBUG("Could not find any address for destination node " << dstNode->GetId());
        }
        return false;
    }

    Ptr<Node> nextHopNode = route.path[1];
    Ptr<Ipv4> nextHopIpv4 = nextHopNode->GetObject<Ipv4>();
    if (!nextHopIpv4)
    {
        if (verbose)
        {
            NS_LOG_DEBUG("Next-hop node " << nextHopNode->GetId() << " has no IPv4");
        }
        return false;
    }

    Ipv4Address nextHopAddr = Ipv4Address::GetZero();
    uint32_t srcInterface = 1;
    bool foundRoute = false;

    for (uint32_t srcIfIdx = 1; srcIfIdx < srcIpv4->GetNInterfaces() && !foundRoute; ++srcIfIdx)
    {
        if (srcIpv4->GetNAddresses(srcIfIdx) == 0)
        {
            continue;
        }

        Ipv4InterfaceAddress srcAddrObj = srcIpv4->GetAddress(srcIfIdx, 0);
        Ipv4Address srcNetwork = srcAddrObj.GetLocal().CombineMask(srcAddrObj.GetMask());

        for (uint32_t nhIfIdx = 1; nhIfIdx < nextHopIpv4->GetNInterfaces(); ++nhIfIdx)
        {
            for (uint32_t nhAddrIdx = 0; nhAddrIdx < nextHopIpv4->GetNAddresses(nhIfIdx); ++nhAddrIdx)
            {
                Ipv4InterfaceAddress nhAddr = nextHopIpv4->GetAddress(nhIfIdx, nhAddrIdx);
                Ipv4Address nhNetwork = nhAddr.GetLocal().CombineMask(nhAddr.GetMask());
                if (srcNetwork == nhNetwork && nhAddr.GetLocal() != srcAddrObj.GetLocal())
                {
                    nextHopAddr = nhAddr.GetLocal();
                    srcInterface = srcIfIdx;
                    foundRoute = true;
                    break;
                }
            }
            if (foundRoute)
            {
                break;
            }
        }
    }

    if (!foundRoute || nextHopAddr == Ipv4Address::GetZero())
    {
        if (verbose)
        {
            NS_LOG_DEBUG("ERROR: Could not find reachable next-hop address! Source Node: "
                         << srcNode->GetId() << ", Destination Node: " << dstNode->GetId()
                         << " (NextHop=" << nextHopNode->GetId()
                         << ", DstAddr=" << dstIpAddrs.front() << ")");
        }
        return false;
    }

    Ipv4Mask hostMask = Ipv4Mask("255.255.255.255");
    const bool isUpdate = Simulator::Now().GetSeconds() > 0.0;
    for (const auto& dstIpAddr : dstIpAddrs)
    {
        srcStaticRouting->AddNetworkRouteTo(dstIpAddr, hostMask, nextHopAddr, srcInterface, 100);
        if (verbose && isUpdate)
        {
            NS_LOG_DEBUG("    [UPDATE] Installing route on Node " << srcNode->GetId()
                         << ": Dest=" << dstIpAddr << " via " << nextHopAddr
                         << " (iface=" << srcInterface << ")");
        }
    }

    return true;
}

bool
LeoSimRoutingCalculatorHelper::InstallRoutes(const std::vector<LeoSimRoute>& routes, bool verbose)
{
    bool allInstalled = true;
    for (const auto& route : routes)
    {
        if (route.valid)
        {
            allInstalled = InstallRoute(route, verbose) && allInstalled;
        }
    }
    return allInstalled;
}

void
LeoSimRoutingCalculatorHelper::SetStaticRoutes(Ptr<LeoSimRoutingCalculator> calculator,
                                                   const NodeContainer& sources,
                                                   const NodeContainer& destinations,
                                                   bool verbose)
{
    double currentTime = Simulator::Now().GetSeconds();
    bool isUpdate = (currentTime > 0.0);  // True if this is a dynamic update (not initial)
    
    if (isUpdate && verbose)
    {
        NS_LOG_DEBUG("Recalculating routes based on current topology...");
    }
    else if (!isUpdate && verbose)
    {
        NS_LOG_DEBUG("Setting static routes between " << sources.GetN() << " sources and "
                                                       << destinations.GetN() << " destinations");
    }
    if (!calculator)
    {
        NS_LOG_ERROR("Routing calculator is null");
        return;
    }

    uint32_t routesInstalled = 0;
    uint32_t routesAttempted = 0;
    uint32_t routesFailed = 0;

    // For each source node
    for (uint32_t i = 0; i < sources.GetN(); ++i)
    {
        Ptr<Node> srcNode = sources.Get(i);
        Ptr<Ipv4> srcIpv4 = srcNode->GetObject<Ipv4>();

        if (!srcIpv4)
        {
            if (verbose)
                NS_LOG_WARN("Source node " << srcNode->GetId() << " has no IPv4 stack");
            continue;
        }

        Ptr<Ipv4StaticRouting> srcStaticRouting =
            Ipv4RoutingHelper::GetRouting<Ipv4StaticRouting>(srcIpv4->GetRoutingProtocol());

        if (!srcStaticRouting)
        {
            if (verbose)
                NS_LOG_WARN("Source node " << srcNode->GetId() << " has no static routing");
            continue;
        }

        // IMPORTANT: Remove all computed destination-specific routes (/32 routes) before reinstalling
        // This ensures that dynamic updates completely replace old routes instead of adding duplicates
        // Keep only the on-link routes (gateway=0.0.0.0) which are automatically managed
        
        uint32_t routesToRemove = 0;
        // We need to be careful here because removing routes changes indices
        // Work backwards to avoid index shifting issues
        std::vector<uint32_t> indicesToRemove;
        for (uint32_t routeIdx = 0; routeIdx < srcStaticRouting->GetNRoutes(); ++routeIdx)
        {
            Ipv4RoutingTableEntry entry = srcStaticRouting->GetRoute(routeIdx);
            // Remove /32 host routes that have actual gateways (our computed routes)
            // Keep on-link routes (gateway=0.0.0.0) and loopback
            if (entry.GetDestNetworkMask() == Ipv4Mask("255.255.255.255") &&
                entry.GetGateway() != Ipv4Address::GetZero())
            {
                indicesToRemove.push_back(routeIdx);
            }
        }
        
        // Remove in reverse order to avoid index shifting
        for (int idx = (int)indicesToRemove.size() - 1; idx >= 0; --idx)
        {
            uint32_t routeIdx = indicesToRemove[idx];
            if (verbose)
            {
                Ipv4RoutingTableEntry entry = srcStaticRouting->GetRoute(routeIdx);
                NS_LOG_DEBUG("    Removing old route: Dest=" << entry.GetDest() << " via "
                                                             << entry.GetGateway());
            }
            srcStaticRouting->RemoveRoute(routeIdx);
            routesToRemove++;
        }
        
        if (verbose && routesToRemove > 0)
        {
            NS_LOG_DEBUG("  Removed " << routesToRemove << " old computed routes for Node "
                                       << srcNode->GetId());
        }

        std::vector<LeoSimRoute> computedRoutes(destinations.GetN());
        std::vector<bool> routeComputed(destinations.GetN(), false);

        for (uint32_t j = 0; j < destinations.GetN(); ++j)
        {
            Ptr<Node> dstNode = destinations.Get(j);
            if (srcNode == dstNode)
            {
                continue;
            }

            computedRoutes[j] = calculator->ComputeRoute(
                srcNode,
                dstNode,
                LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT);
            routeComputed[j] = true;
        }

        // For each destination node
        for (uint32_t j = 0; j < destinations.GetN(); ++j)
        {
            Ptr<Node> dstNode = destinations.Get(j);

            if (srcNode == dstNode)
                continue; // Skip same node

            routesAttempted++;

            const LeoSimRoute& route = computedRoutes[j];

            if (!routeComputed[j] || !route.valid || route.path.size() < 2)
            {
                routesFailed++;
                if (verbose)
                {
                    NS_LOG_DEBUG("  No route: Node " << srcNode->GetId() << " to " << dstNode->GetId());
                    NS_LOG_DEBUG("No valid route from node " << srcNode->GetId() << " to node "
                                                            << dstNode->GetId());
                }
                continue;
            }

            if (InstallRoute(route, verbose))
            {
                routesInstalled++;
            }
            else
            {
                routesFailed++;
            }
        }
    }

    if (verbose)
    {
        NS_LOG_DEBUG("\nRoute installation summary:");
        NS_LOG_DEBUG("  Routes attempted: " << routesAttempted);
        NS_LOG_DEBUG("  Routes failed: " << routesFailed);
        NS_LOG_DEBUG("  Routes installed: " << routesInstalled);
        NS_LOG_DEBUG("Total routes installed: " << routesInstalled);
        
        if (isUpdate && routesInstalled > 0)
        {
            NS_LOG_DEBUG("  [UPDATE] " << routesInstalled << " routes recalculated and updated");
        }
    }
}

void
LeoSimRoutingCalculatorHelper::EnableDynamicRouting(Ptr<LeoSimRoutingCalculator> calculator,
                                                     const NodeContainer& sources,
                                                     const NodeContainer& destinations,
                                                     Time updateInterval,
                                                     double stopTime,
                                                     bool verbose)
{
    if (!calculator)
    {
        NS_LOG_ERROR("Routing calculator is null");
        return;
    }

    if (verbose)
    {
        NS_LOG_DEBUG("\n=== Enabling Dynamic Routing ===");
        NS_LOG_DEBUG("Update interval: " << updateInterval.GetSeconds() << " seconds");
        if (stopTime > 0.0)
            NS_LOG_DEBUG("Stop time: " << stopTime << " seconds");
        NS_LOG_DEBUG("Number of sources: " << sources.GetN());
        NS_LOG_DEBUG("Number of destinations: " << destinations.GetN());
    }

    // Perform initial route calculation
    SetStaticRoutes(calculator, sources, destinations, verbose);

    // Schedule periodic updates
    m_dynamicRoutingUpdate =
        Simulator::Schedule(updateInterval,
                            &LeoSimRoutingCalculatorHelper::UpdateRoutesAndReschedule,
                            this,
                            calculator,
                            sources,
                            destinations,
                            updateInterval,
                            stopTime,
                            verbose);
}

void
LeoSimRoutingCalculatorHelper::UpdateRoutesAndReschedule(Ptr<LeoSimRoutingCalculator> calculator,
                                                          const NodeContainer& sources,
                                                          const NodeContainer& destinations,
                                                          Time updateInterval,
                                                          double stopTime,
                                                          bool verbose)
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.dynamic_routing_update");
    double currentTime = Simulator::Now().GetSeconds();
    
    if (verbose)
    {
        NS_LOG_DEBUG("\n========================================");
        NS_LOG_DEBUG("[" << currentTime << "s] DYNAMIC ROUTING UPDATE");
        NS_LOG_DEBUG("========================================");
    }

    // Get active link count before update
    uint32_t activeLinksBeforeUpdate = 0;
    if (verbose && calculator)
    {
        activeLinksBeforeUpdate = calculator->GetNumActiveLinks();
        NS_LOG_DEBUG("Active links in network: " << activeLinksBeforeUpdate);
    }

    // Reinstall routes with fresh calculations
    SetStaticRoutes(calculator, sources, destinations, verbose);

    // Get active link count after update
    if (verbose && calculator)
    {
        uint32_t activeLinksAfterUpdate = calculator->GetNumActiveLinks();
        NS_LOG_DEBUG("Updated network status - Active links: " << activeLinksAfterUpdate);
        NS_LOG_DEBUG("========================================\n");
    }

    // Check if we should continue scheduling updates
    if (stopTime > 0.0 && currentTime + updateInterval.GetSeconds() >= stopTime)
    {
        if (verbose)
        {
            NS_LOG_DEBUG("[" << currentTime << "s] Stopping dynamic routing updates (reached stop time)");
        }
        m_dynamicRoutingUpdate = EventId();
        return;
    }

    // Schedule the next update
    m_dynamicRoutingUpdate =
        Simulator::Schedule(updateInterval,
                            &LeoSimRoutingCalculatorHelper::UpdateRoutesAndReschedule,
                            this,
                            calculator,
                            sources,
                            destinations,
                            updateInterval,
                            stopTime,
                            verbose);
}

bool
LeoSimRoutingCalculatorHelper::HasPendingDynamicRoutingUpdate() const
{
    return m_dynamicRoutingUpdate.IsPending();
}

void
LeoSimRoutingCalculatorHelper::EnableReactiveLinkTriggeredRouting(
    Ptr<LeoSimRoutingCalculator> calculator,
    const NodeContainer& sources,
    const NodeContainer& destinations,
    Ptr<LeoSimChannelModel> groundChannelModel,
    Ptr<LeoSimChannelModel> islChannelModel,
    Time debounceInterval,
    bool verbose)
{
    if (!calculator || !groundChannelModel)
    {
        NS_LOG_ERROR("Null calculator or ground channel model passed to EnableReactiveLinkTriggeredRouting");
        return;
    }

    if (verbose)
    {
        NS_LOG_DEBUG("\n=== Enabling Reactive Link-Triggered Routing ===");
        NS_LOG_DEBUG("Debounce interval: " << debounceInterval.GetMilliSeconds() << " ms");
    }

    m_reactiveCalculator = calculator;
    m_reactiveSources = sources;
    m_reactiveDestinations = destinations;
    m_reactiveDebounceInterval = debounceInterval;
    m_reactiveVerbose = verbose;

    auto cb = MakeCallback(&LeoSimRoutingCalculatorHelper::OnLinkStateChanged, this);

    groundChannelModel->TraceConnectWithoutContext("LinkStateChange", cb);

    if (islChannelModel)
    {
        islChannelModel->TraceConnectWithoutContext("LinkStateChange", cb);
    }
}

void
LeoSimRoutingCalculatorHelper::OnLinkStateChanged(Ptr<Node> nodeA,
                                                  Ptr<Node> nodeB,
                                                  LeoSimLinkState newState)
{
    // Only react to links going DOWN — newly UP links will be picked up by the
    // next periodic update or the already-scheduled debounced event.
    if (newState != LEOSIM_LINK_DOWN)
    {
        return;
    }

    NS_LOG_DEBUG("Reactive routing: link between node " << nodeA->GetId()
                << " and " << nodeB->GetId() << " went DOWN — scheduling route update");

    if (m_reactiveVerbose)
    {
        NS_LOG_DEBUG("[" << Simulator::Now().GetSeconds()
                         << "s] Reactive routing: link " << nodeA->GetId()
                         << "<->" << nodeB->GetId()
                         << " DOWN — scheduling route refresh in "
                         << m_reactiveDebounceInterval.GetMilliSeconds() << " ms");
    }

    // Cancel any already-pending debounced update and schedule a fresh one so
    // that a burst of simultaneous link-down events only causes one recompute.
    if (m_pendingReactiveUpdate.IsPending())
    {
        m_pendingReactiveUpdate.Cancel();
    }

    m_pendingReactiveUpdate =
        Simulator::Schedule(m_reactiveDebounceInterval,
                            &LeoSimRoutingCalculatorHelper::DoReactiveUpdate,
                            this);
}

void
LeoSimRoutingCalculatorHelper::RequestRouteRefresh()
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
                            &LeoSimRoutingCalculatorHelper::DoReactiveUpdate,
                            this);
}

void
LeoSimRoutingCalculatorHelper::DoReactiveUpdate()
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.reactive_routing_update");

    if (m_reactiveVerbose)
    {
        NS_LOG_DEBUG("\n[" << Simulator::Now().GetSeconds()
                            << "s] REACTIVE ROUTING UPDATE (link-state triggered)");
    }

    if (!m_reactiveCalculator)
    {
        NS_LOG_WARN("Reactive routing update skipped: calculator is null");
        return;
    }

    SetStaticRoutes(m_reactiveCalculator,
                    m_reactiveSources,
                    m_reactiveDestinations,
                    m_reactiveVerbose);
}

} // namespace ns3
