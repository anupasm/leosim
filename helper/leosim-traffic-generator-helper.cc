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

#include "leosim-traffic-generator-helper.h"

#include "ns3/icmpv4-l4-protocol.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/node.h"
#include "ns3/ping-helper.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimTrafficGeneratorHelper");

LeoSimTrafficGeneratorHelper::LeoSimTrafficGeneratorHelper()
    : m_pingInterval(Seconds(1.0)),
      m_pingDataSize(56),
      m_verbose(false)
{
    NS_LOG_FUNCTION(this);
}

LeoSimTrafficGeneratorHelper::~LeoSimTrafficGeneratorHelper()
{
    NS_LOG_FUNCTION(this);
}

void
LeoSimTrafficGeneratorHelper::SetPingInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_pingInterval = interval;
}

Time
LeoSimTrafficGeneratorHelper::GetPingInterval() const
{
    NS_LOG_FUNCTION(this);
    return m_pingInterval;
}

void
LeoSimTrafficGeneratorHelper::SetPingDataSize(uint32_t size)
{
    NS_LOG_FUNCTION(this << size);
    m_pingDataSize = size;
}

uint32_t
LeoSimTrafficGeneratorHelper::GetPingDataSize() const
{
    NS_LOG_FUNCTION(this);
    return m_pingDataSize;
}

void
LeoSimTrafficGeneratorHelper::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
}

ApplicationContainer
LeoSimTrafficGeneratorHelper::InstallPingClient(Ptr<Node> sourceNode,
                                                Ipv4Address destAddress,
                                                Time startTime,
                                                Time stopTime)
{
    NS_LOG_FUNCTION(this << sourceNode << destAddress << startTime << stopTime);

    ApplicationContainer apps;

    PingHelper pingHelper(destAddress);
    pingHelper.SetAttribute("Interval", TimeValue(m_pingInterval));
    pingHelper.SetAttribute("Size", UintegerValue(m_pingDataSize));
    pingHelper.SetAttribute("Count", UintegerValue(0)); // 0 = unlimited

    apps = pingHelper.Install(sourceNode);
    apps.Start(startTime);
    apps.Stop(stopTime);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installed ping client on Node " << sourceNode->GetId() << " -> " << destAddress
                                                     << " (interval: " << m_pingInterval.As(Time::S)
                                                     << "s, payload: " << m_pingDataSize << " bytes)");
    }

    return apps;
}

ApplicationContainer
LeoSimTrafficGeneratorHelper::InstallPingClients(NodeContainer sourceNodes,
                                                 Ipv4Address destAddress,
                                                 Time startTime,
                                                 Time stopTime)
{
    NS_LOG_FUNCTION(this << sourceNodes.GetN() << destAddress << startTime << stopTime);

    ApplicationContainer apps;

    for (uint32_t i = 0; i < sourceNodes.GetN(); i++)
    {
        ApplicationContainer clientApps =
            InstallPingClient(sourceNodes.Get(i), destAddress, startTime, stopTime);
        apps.Add(clientApps);
    }

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installed ping clients on " << sourceNodes.GetN() << " nodes -> "
                                                 << destAddress);
    }

    return apps;
}

ApplicationContainer
LeoSimTrafficGeneratorHelper::InstallBidirectionalPing(Ptr<Node> node1,
                                                       Ptr<Node> node2,
                                                       Ipv4Address addr1,
                                                       Ipv4Address addr2,
                                                       Time startTime,
                                                       Time stopTime)
{
    NS_LOG_FUNCTION(this << node1 << node2 << addr1 << addr2 << startTime << stopTime);

    ApplicationContainer apps;

    // Install ping from node1 to node2
    ApplicationContainer apps1 = InstallPingClient(node1, addr2, startTime, stopTime);
    apps.Add(apps1);

    // Install ping from node2 to node1
    ApplicationContainer apps2 = InstallPingClient(node2, addr1, startTime, stopTime);
    apps.Add(apps2);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installed bidirectional ping between Node " << node1->GetId() << " and Node "
                                                                 << node2->GetId());
    }

    return apps;
}

ApplicationContainer
LeoSimTrafficGeneratorHelper::InstallUeToServerPing(NodeContainer ueNodes,
                                                    Ptr<Node> serverNode,
                                                    Ipv4Address serverAddress,
                                                    Time startTime,
                                                    Time stopTime)
{
    NS_LOG_FUNCTION(this << ueNodes.GetN() << serverNode << serverAddress << startTime << stopTime);

    ApplicationContainer apps;

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        ApplicationContainer clientApps = InstallPingClient(ueNodes.Get(i), serverAddress, startTime, stopTime);
        apps.Add(clientApps);

        if (m_verbose)
        {
            NS_LOG_DEBUG("UE Node " << ueNodes.Get(i)->GetId() << " -> Server Node "
                                   << serverNode->GetId() << " (" << serverAddress << ")");
        }
    }

    return apps;
}

ApplicationContainer
LeoSimTrafficGeneratorHelper::InstallBidirectionalUeToServerPing(NodeContainer ueNodes,
                                                                 Ptr<Node> serverNode,
                                                                 std::vector<Ipv4Address> ueAddresses,
                                                                 Ipv4Address serverAddress,
                                                                 Time startTime,
                                                                 Time stopTime)
{
    NS_LOG_FUNCTION(this << ueNodes.GetN() << serverNode << serverAddress << startTime << stopTime);

    NS_ASSERT_MSG(ueNodes.GetN() == ueAddresses.size(),
                  "Number of UE nodes must match number of addresses");

    ApplicationContainer apps;

    for (uint32_t i = 0; i < ueNodes.GetN(); i++)
    {
        ApplicationContainer bidirectionalApps =
            InstallBidirectionalPing(ueNodes.Get(i), serverNode, ueAddresses[i], serverAddress,
                                     startTime, stopTime);
        apps.Add(bidirectionalApps);

        if (m_verbose)
        {
            NS_LOG_DEBUG("Bidirectional ping between UE Node " << ueNodes.Get(i)->GetId() << " ("
                                                              << ueAddresses[i] << ") and Server "
                                                              << serverNode->GetId() << " ("
                                                              << serverAddress << ")");
        }
    }

    return apps;
}

} // namespace ns3
