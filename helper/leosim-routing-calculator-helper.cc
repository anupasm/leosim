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
#include "ns3/ipv4-static-routing.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-routing-helper.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <iostream>

NS_LOG_COMPONENT_DEFINE("LeoSimRoutingCalculatorHelper");

namespace ns3
{

LeoSimRoutingCalculatorHelper::LeoSimRoutingCalculatorHelper()
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
        std::cout << "Recalculating routes based on current topology..." << std::endl;
    }
    else if (!isUpdate && verbose)
    {
        std::cout << "Setting static routes between " << sources.GetN() << " sources and "
                                                     << destinations.GetN() << " destinations" << std::endl;
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
                std::cout << "    Removing old route: Dest=" << entry.GetDest() << " via " 
                          << entry.GetGateway() << std::endl;
            }
            srcStaticRouting->RemoveRoute(routeIdx);
            routesToRemove++;
        }
        
        if (verbose && routesToRemove > 0)
        {
            std::cout << "  Removed " << routesToRemove << " old computed routes for Node " 
                      << srcNode->GetId() << std::endl;
        }

        // For each destination node
        for (uint32_t j = 0; j < destinations.GetN(); ++j)
        {
            Ptr<Node> dstNode = destinations.Get(j);

            if (srcNode == dstNode)
                continue; // Skip same node

            routesAttempted++;

            // Compute route from source to destination
            LeoSimRoute route = calculator->ComputeRoute(
                srcNode,
                dstNode,
                LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT);

            if (!route.valid || route.path.size() < 2)
            {
                routesFailed++;
                if (verbose)
                {
                    std::cout << "  No route: Node " << srcNode->GetId() << " to " << dstNode->GetId() << std::endl;
                    NS_LOG_INFO("No valid route from node " << srcNode->GetId() << " to node "
                                                            << dstNode->GetId());
                }
                continue;
            }

            // Get destination node's IPv4 addresses
            Ptr<Ipv4> dstIpv4 = dstNode->GetObject<Ipv4>();
            if (!dstIpv4)
            {
                if (verbose)
                    std::cout << "  No IPv4 on destination node " << dstNode->GetId() << std::endl;
                continue;
            }

            // Collect all destination IPv4 addresses (excluding loopback).
            // Nodes are multi-homed in LeoSim (multiple satellite links), and TCP may
            // pick a source address that isn't the first interface. If we only install
            // routes to one address per node, return traffic can fail.
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
                    std::cout << "    Could not find any address for destination node " << dstNode->GetId() << std::endl;
                }
                routesFailed++;
                continue;
            }

            // Next hop is the second node in the path (first intermediate node)
            Ptr<Node> nextHopNode = route.path[1];
            Ptr<Ipv4> nextHopIpv4 = nextHopNode->GetObject<Ipv4>();

            if (!nextHopIpv4)
            {
                if (verbose)
                {
                    std::cout << "      Error: NextHop node " << nextHopNode->GetId() << " has no IPv4" << std::endl;
                }
                routesFailed++;
                continue;
            }

            // Find the best address on the next hop node to use as gateway
            // Strategy:
            // 1. First try to find an address on the same subnet as source's interface (direct link)
            // 2. If that fails, use any available address on next hop
            Ipv4Address nextHopAddr = Ipv4Address::GetZero();
            uint32_t srcInterface = 1;
            bool foundRoute = false;

                // For each interface on source, try to find a matching downstream neighbor
                for (uint32_t srcIfIdx = 1; (srcIfIdx < srcIpv4->GetNInterfaces()) && !foundRoute; ++srcIfIdx)
                {
                    Ipv4Address srcNetwork = Ipv4Address::GetZero();
                    Ipv4Mask srcMask = Ipv4Mask::GetZero();
                    Ipv4InterfaceAddress srcAddrObj = Ipv4InterfaceAddress();
                    
                    // Get the network address and mask for this source interface
                    if (srcIpv4->GetNAddresses(srcIfIdx) > 0)
                    {
                        srcAddrObj = srcIpv4->GetAddress(srcIfIdx, 0);
                        srcNetwork = srcAddrObj.GetLocal();
                        srcMask = srcAddrObj.GetMask();
                        Ipv4Address temp = srcNetwork;
                        srcNetwork = temp.CombineMask(srcMask);
                    }
                    else
                    {
                        continue;  // Skip interfaces with no addresses
                    }

                    // Find an address on next hop that's on the same subnet as this source interface
                    for (uint32_t nhIfIdx = 1; nhIfIdx < nextHopIpv4->GetNInterfaces(); ++nhIfIdx)
                    {
                        for (uint32_t nhAddrIdx = 0; nhAddrIdx < nextHopIpv4->GetNAddresses(nhIfIdx); ++nhAddrIdx)
                        {
                            Ipv4InterfaceAddress nhAddr = nextHopIpv4->GetAddress(nhIfIdx, nhAddrIdx);
                            Ipv4Address nhNetwork = nhAddr.GetLocal();
                            nhNetwork = nhNetwork.CombineMask(nhAddr.GetMask());
                            
                            // Check if they're on the same subnet
                            if (srcNetwork == nhNetwork && nhAddr.GetLocal() != srcAddrObj.GetLocal())
                            {
                                nextHopAddr = nhAddr.GetLocal();
                                srcInterface = srcIfIdx;
                                foundRoute = true;
                                
                                break;
                            }
                        }
                        if (foundRoute)
                            break;
                    }
                }

                // If we couldn't find a matching subnet, look for any address on next hop
                // BUT only if it's actually reachable from one of source's interfaces
                if (!foundRoute)
                {
                    // Try to find any pair of (src_subnet, nh_address) that match
                    for (uint32_t srcIfIdx = 1; srcIfIdx < srcIpv4->GetNInterfaces() && !foundRoute; ++srcIfIdx)
                    {
                        if (srcIpv4->GetNAddresses(srcIfIdx) > 0)
                        {
                            Ipv4InterfaceAddress srcAddrObj = srcIpv4->GetAddress(srcIfIdx, 0);
                            Ipv4Address srcNetwork = srcAddrObj.GetLocal().CombineMask(srcAddrObj.GetMask());
                            
                            // Look for ANY interface on next hop that's on the same subnet as this source interface
                            for (uint32_t nhIfIdx = 1; nhIfIdx < nextHopIpv4->GetNInterfaces(); ++nhIfIdx)
                            {
                                if (nextHopIpv4->GetNAddresses(nhIfIdx) > 0)
                                {
                                    Ipv4InterfaceAddress nhAddr = nextHopIpv4->GetAddress(nhIfIdx, 0);
                                    Ipv4Address nhNetwork = nhAddr.GetLocal().CombineMask(nhAddr.GetMask());
                                    
                                    // Check if they're on the same subnet
                                    if (srcNetwork == nhNetwork && nhAddr.GetLocal() != srcAddrObj.GetLocal())
                                    {
                                        nextHopAddr = nhAddr.GetLocal();
                                        srcInterface = srcIfIdx;
                                        foundRoute = true;
                                        break;
                                    }
                                }
                            }
                        }
                    }
                }

                if (foundRoute && nextHopAddr != Ipv4Address::GetZero())
                {
                    Ipv4Mask hostMask = Ipv4Mask("255.255.255.255"); // /32 host route

                    for (const auto& dstIpAddr : dstIpAddrs)
                    {
                        srcStaticRouting->AddNetworkRouteTo(dstIpAddr, hostMask,
                                                            nextHopAddr, srcInterface, 100);

                        if (verbose && isUpdate)
                        {
                            std::cout << "    [UPDATE] Installing route on Node " << srcNode->GetId()
                                      << ": Dest=" << dstIpAddr << " via " << nextHopAddr
                                      << " (iface=" << srcInterface << ")" << std::endl;
                        }
                        routesInstalled++;
                    }
                }
                else
                {
                    if (verbose)
                    {
                        std::cout << "ERROR: Could not find reachable next-hop address! Source Node: " << srcNode->GetId() 
                                  << ", Destination Node: " << dstNode->GetId() 
                                  << " (NextHop=" << nextHopNode->GetId() 
                                  << ", DstAddr=" << dstIpAddrs.front() << ")" << std::endl;
                    }
                    routesFailed++;
                }
        }
    }

    if (verbose)
    {
        std::cout << "\nRoute installation summary:" << std::endl;
        std::cout << "  Routes attempted: " << routesAttempted << std::endl;
        std::cout << "  Routes failed: " << routesFailed << std::endl;
        std::cout << "  Routes installed: " << routesInstalled << std::endl;
        NS_LOG_INFO("Total routes installed: " << routesInstalled);
        
        if (isUpdate && routesInstalled > 0)
        {
            std::cout << "  [UPDATE] " << routesInstalled << " routes recalculated and updated" << std::endl;
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
        std::cout << "\n=== Enabling Dynamic Routing ===" << std::endl;
        std::cout << "Update interval: " << updateInterval.GetSeconds() << " seconds" << std::endl;
        if (stopTime > 0.0)
            std::cout << "Stop time: " << stopTime << " seconds" << std::endl;
        std::cout << "Number of sources: " << sources.GetN() << std::endl;
        std::cout << "Number of destinations: " << destinations.GetN() << std::endl;
    }

    // Perform initial route calculation
    SetStaticRoutes(calculator, sources, destinations, verbose);

    // Schedule periodic updates
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
    double currentTime = Simulator::Now().GetSeconds();
    
    if (verbose)
    {
        std::cout << "\n========================================" << std::endl;
        std::cout << "[" << currentTime << "s] DYNAMIC ROUTING UPDATE" << std::endl;
        std::cout << "========================================" << std::endl;
    }

    // Get active link count before update
    uint32_t activeLinksBeforeUpdate = 0;
    if (verbose && calculator)
    {
        activeLinksBeforeUpdate = calculator->GetNumActiveLinks();
        std::cout << "Active links in network: " << activeLinksBeforeUpdate << std::endl;
    }

    // Reinstall routes with fresh calculations
    SetStaticRoutes(calculator, sources, destinations, verbose);

    // Get active link count after update
    if (verbose && calculator)
    {
        uint32_t activeLinksAfterUpdate = calculator->GetNumActiveLinks();
        std::cout << "Updated network status - Active links: " << activeLinksAfterUpdate << std::endl;
        std::cout << "========================================\n" << std::endl;
    }

    // Check if we should continue scheduling updates
    if (stopTime > 0.0 && currentTime + updateInterval.GetSeconds() >= stopTime)
    {
        if (verbose)
        {
            std::cout << "[" << currentTime << "s] Stopping dynamic routing updates (reached stop time)"
                      << std::endl;
        }
        return;
    }

    // Schedule the next update
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

} // namespace ns3