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

#include "leosim-mobility-helper.h"

#include "ns3/log.h"
#include "ns3/names.h"

#include <iostream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimMobilityHelper");

LeoSimMobilityHelper::LeoSimMobilityHelper()
    : m_calculateVelocity(true),
      m_verbose(false)
{
    NS_LOG_FUNCTION(this);
}

LeoSimMobilityHelper::~LeoSimMobilityHelper()
{
    NS_LOG_FUNCTION(this);
}

void
LeoSimMobilityHelper::SetLoader(Ptr<LeoSimLoader> loader)
{
    NS_LOG_FUNCTION(this << loader);
    m_loader = loader;
}

std::vector<LeoSimWaypoint>
LeoSimMobilityHelper::ConvertToWaypoints(uint32_t satId)
{
    std::vector<LeoSimWaypoint> waypoints;

    if (!m_loader)
    {
        NS_LOG_ERROR("No loader set");
        return waypoints;
    }

    auto positions = m_loader->GetSatellitePositions(satId);

    for (const auto& pos : positions)
    {
        LeoSimWaypoint wp;
        wp.time = Seconds(pos.time);
        wp.position = pos.position;
        wp.velocity = Vector(0, 0, 0); // Will be calculated automatically
        NS_LOG_DEBUG("Added waypoint for satellite " << satId << " at time " << wp.time.GetSeconds()
                                                     << "s, position=" << wp.position);
        waypoints.push_back(wp);
    }

    NS_LOG_INFO("Converted " << waypoints.size() << " waypoints for satellite " << satId);
    return waypoints;
}

uint32_t
LeoSimMobilityHelper::InstallSatellites(NodeContainer& nodes)
{
    NS_LOG_FUNCTION(this);

    if (!m_loader)
    {
        NS_LOG_ERROR("No loader set. Call SetLoader() first.");
        return 0;
    }

    uint32_t numSatellites = std::min(nodes.GetN(), m_loader->GetNumSatellites());

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        Ptr<Node> node = nodes.Get(i);
        std::string satName = m_loader->GetSatelliteName(i);
        
        InstallSatellite(node, i, satName);

        if (m_verbose)
        {
            std::cout << "Installed satellite mobility on node " << i << " (" << satName << ")"
                      << std::endl;
        }
    }

    NS_LOG_INFO("Installed satellite mobility on " << numSatellites << " nodes");
    return numSatellites;
}

uint32_t
LeoSimMobilityHelper::InstallGateways(NodeContainer& nodes)
{
    NS_LOG_FUNCTION(this);

    if (!m_loader)
    {
        NS_LOG_ERROR("No loader set. Call SetLoader() first.");
        return 0;
    }

    uint32_t numDevices = std::min(nodes.GetN(), m_loader->GetNumGroundDevices());

    for (uint32_t i = 0; i < numDevices; ++i)
    {
        Ptr<Node> node = nodes.Get(i);
        std::string deviceName = m_loader->GetGroundDeviceName(i);
        Vector position = m_loader->GetGroundDevicePosition(i);
        
        InstallGateway(node, i, deviceName, position);

        if (m_verbose)
        {
            std::cout << "Installed gateway mobility on node " << i << " (" << deviceName << ")"
                      << std::endl;
        }
    }

    NS_LOG_INFO("Installed gateway mobility on " << numDevices << " nodes");
    return numDevices;
}

uint32_t
LeoSimMobilityHelper::InstallUEs(NodeContainer& nodes)
{
    NS_LOG_FUNCTION(this);

    if (!m_loader)
    {
        NS_LOG_ERROR("No loader set. Call SetLoader() first.");
        return 0;
    }

    uint32_t numDevices = std::min(nodes.GetN(), m_loader->GetNumGroundDevices());

    for (uint32_t i = 0; i < numDevices; ++i)
    {
        Ptr<Node> node = nodes.Get(i);
        std::string deviceName = m_loader->GetGroundDeviceName(i);
        Vector position = m_loader->GetGroundDevicePosition(i);
        
        InstallUE(node, i, deviceName, position);

        if (m_verbose)
        {
            std::cout << "Installed UE mobility on node " << i << " (" << deviceName << ")"
                      << std::endl;
        }
    }

    NS_LOG_INFO("Installed UE mobility on " << numDevices << " nodes");
    return numDevices;
}

Ptr<LeoSimMobilityModel>
LeoSimMobilityHelper::InstallSatellite(Ptr<Node> node,
                                        uint32_t satId,
                                        const std::string& satName)
{
    NS_LOG_FUNCTION(this << node << satId << satName);

    // Create mobility model
    Ptr<LeoSimMobilityModel> mobility = CreateObject<LeoSimMobilityModel>();
    mobility->SetNodeType(LEOSIM_SATELLITE);
    mobility->SetNodeId(satId);
    mobility->SetNodeName(satName);
    mobility->SetVelocityCalculation(m_calculateVelocity);

    // Convert position data to waypoints
    auto waypoints = ConvertToWaypoints(satId);
    mobility->SetWaypoints(waypoints);

    // Aggregate to node
    node->AggregateObject(mobility);

    // Store for later control
    m_mobilityModels.push_back(mobility);

    NS_LOG_DEBUG("Installed satellite mobility: " << satName << " with " << waypoints.size()
                                                   << " waypoints");

    return mobility;
}

Ptr<LeoSimMobilityModel>
LeoSimMobilityHelper::InstallGateway(Ptr<Node> node,
                                      uint32_t deviceId,
                                      const std::string& deviceName,
                                      const Vector& position)
{
    NS_LOG_FUNCTION(this << node << deviceId << deviceName << position);

    // Create mobility model
    Ptr<LeoSimMobilityModel> mobility = CreateObject<LeoSimMobilityModel>();
    mobility->SetNodeType(LEOSIM_GATEWAY);
    mobility->SetNodeId(deviceId);
    mobility->SetNodeName(deviceName);
    mobility->SetPosition(position);

    // Add single waypoint for fixed position
    LeoSimWaypoint wp;
    wp.time = Seconds(0);
    wp.position = position;
    wp.velocity = Vector(0, 0, 0);
    mobility->AddWaypoint(wp);

    // Aggregate to node
    node->AggregateObject(mobility);

    // Store for later control
    m_mobilityModels.push_back(mobility);

    NS_LOG_DEBUG("Installed gateway mobility: " << deviceName << " at " << position);

    return mobility;
}

Ptr<LeoSimMobilityModel>
LeoSimMobilityHelper::InstallUE(Ptr<Node> node,
                                 uint32_t deviceId,
                                 const std::string& deviceName,
                                 const Vector& position)
{
    NS_LOG_FUNCTION(this << node << deviceId << deviceName << position);

    // Create mobility model
    Ptr<LeoSimMobilityModel> mobility = CreateObject<LeoSimMobilityModel>();
    mobility->SetNodeType(LEOSIM_UE);
    mobility->SetNodeId(deviceId);
    mobility->SetNodeName(deviceName);
    mobility->SetPosition(position);

    // Add single waypoint for fixed position
    LeoSimWaypoint wp;
    wp.time = Seconds(0);
    wp.position = position;
    wp.velocity = Vector(0, 0, 0);
    mobility->AddWaypoint(wp);

    // Aggregate to node
    node->AggregateObject(mobility);

    // Store for later control
    m_mobilityModels.push_back(mobility);

    NS_LOG_DEBUG("Installed UE mobility: " << deviceName << " at " << position);

    return mobility;
}

void
LeoSimMobilityHelper::StartAll()
{
    NS_LOG_FUNCTION(this);

    for (auto& mobility : m_mobilityModels)
    {
        mobility->Start();
    }

    NS_LOG_INFO("Started mobility for " << m_mobilityModels.size() << " nodes");
}

void
LeoSimMobilityHelper::StopAll()
{
    NS_LOG_FUNCTION(this);

    for (auto& mobility : m_mobilityModels)
    {
        mobility->Stop();
    }

    NS_LOG_INFO("Stopped mobility for " << m_mobilityModels.size() << " nodes");
}

void
LeoSimMobilityHelper::SetVelocityCalculation(bool enable)
{
    m_calculateVelocity = enable;
}

void
LeoSimMobilityHelper::SetVerbose(bool verbose)
{
    m_verbose = verbose;
}

} // namespace ns3
