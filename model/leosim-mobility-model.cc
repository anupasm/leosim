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

#include "leosim-mobility-model.h"

#include "ns3/boolean.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/uinteger.h"

#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimMobilityModel");

NS_OBJECT_ENSURE_REGISTERED(LeoSimMobilityModel);

TypeId
LeoSimMobilityModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LeoSimMobilityModel")
            .SetParent<MobilityModel>()
            .SetGroupName("LeoSim")
            .AddConstructor<LeoSimMobilityModel>()
            .AddAttribute("NodeType",
                          "Type of node (0=Satellite, 1=Gateway, 2=UE)",
                          UintegerValue(LEOSIM_SATELLITE),
                          MakeUintegerAccessor(&LeoSimMobilityModel::m_nodeType),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("NodeId",
                          "Node ID",
                          UintegerValue(0),
                          MakeUintegerAccessor(&LeoSimMobilityModel::m_nodeId),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("CalculateVelocity",
                          "Whether to automatically calculate velocity from position changes",
                          BooleanValue(true),
                          MakeBooleanAccessor(&LeoSimMobilityModel::m_calculateVelocity),
                          MakeBooleanChecker());
    return tid;
}

LeoSimMobilityModel::LeoSimMobilityModel()
    : m_nodeType(LEOSIM_SATELLITE),
      m_nodeId(0),
      m_nodeName(""),
      m_position(0, 0, 0),
      m_velocity(0, 0, 0),
      m_currentWaypoint(0),
      m_active(false),
      m_calculateVelocity(true)
{
    NS_LOG_FUNCTION(this);
}

LeoSimMobilityModel::~LeoSimMobilityModel()
{
    NS_LOG_FUNCTION(this);
}

void
LeoSimMobilityModel::SetNodeType(LeoSimNodeType nodeType)
{
    NS_LOG_FUNCTION(this << nodeType);
    m_nodeType = nodeType;
}

LeoSimNodeType
LeoSimMobilityModel::GetNodeType() const
{
    return m_nodeType;
}

void
LeoSimMobilityModel::SetNodeId(uint32_t id)
{
    NS_LOG_FUNCTION(this << id);
    m_nodeId = id;
}

uint32_t
LeoSimMobilityModel::GetNodeId() const
{
    return m_nodeId;
}

void
LeoSimMobilityModel::SetNodeName(const std::string& name)
{
    NS_LOG_FUNCTION(this << name);
    m_nodeName = name;
}

std::string
LeoSimMobilityModel::GetNodeName() const
{
    return m_nodeName;
}

void
LeoSimMobilityModel::AddWaypoint(const LeoSimWaypoint& waypoint)
{
    NS_LOG_FUNCTION(this << waypoint.time);
    m_waypoints.push_back(waypoint);
}

void
LeoSimMobilityModel::SetWaypoints(const std::vector<LeoSimWaypoint>& waypoints)
{
    NS_LOG_FUNCTION(this << waypoints.size());
    m_waypoints = waypoints;
}

std::vector<LeoSimWaypoint>
LeoSimMobilityModel::GetWaypoints() const
{
    return m_waypoints;
}

void
LeoSimMobilityModel::Start()
{
    NS_LOG_FUNCTION(this);

    if (m_waypoints.empty())
    {
        NS_LOG_WARN("No waypoints set for node " << m_nodeId);
        return;
    }

    m_active = true;
    m_currentWaypoint = 0;

    // Set initial position
    m_position = m_waypoints[0].position;
    NotifyCourseChange();

    // For stationary nodes (gateways), don't schedule updates
    if (m_nodeType == LEOSIM_GATEWAY)
    {
        NS_LOG_DEBUG("Gateway node " << m_nodeId << " at fixed position");
        return;
    }

    // Schedule first update
    if (m_waypoints.size() > 1)
    {
        Time nextUpdate = m_waypoints[1].time;
        m_event = Simulator::Schedule(nextUpdate, &LeoSimMobilityModel::UpdatePosition, this);
        NS_LOG_DEBUG("Scheduled " << m_waypoints.size() << " waypoints for node " << m_nodeId);
    }
}

void
LeoSimMobilityModel::Stop()
{
    NS_LOG_FUNCTION(this);
    m_active = false;
    if (m_event.IsPending())
    {
        Simulator::Cancel(m_event);
    }
}

bool
LeoSimMobilityModel::IsActive() const
{
    return m_active;
}

void
LeoSimMobilityModel::SetVelocityCalculation(bool enable)
{
    m_calculateVelocity = enable;
}

void
LeoSimMobilityModel::UpdatePosition()
{
    NS_LOG_FUNCTION(this);

    if (!m_active || m_currentWaypoint >= m_waypoints.size())
    {
        return;
    }

    // Get current waypoint
    const LeoSimWaypoint& waypoint = m_waypoints[m_currentWaypoint];

    // Calculate velocity if enabled and we have a previous waypoint
    if (m_calculateVelocity && m_currentWaypoint > 0)
    {
        const LeoSimWaypoint& prevWaypoint = m_waypoints[m_currentWaypoint - 1];
        double dt = (waypoint.time - prevWaypoint.time).GetSeconds();
        if (dt > 0)
        {
            m_velocity = CalculateVelocity(prevWaypoint.position, waypoint.position, dt);
        }
    }
    else if (waypoint.velocity.x != 0 || waypoint.velocity.y != 0 || waypoint.velocity.z != 0)
    {
        // Use provided velocity
        m_velocity = waypoint.velocity;
    }

    // Update position
    Vector oldPosition = m_position;
    m_position = waypoint.position;

    NS_LOG_DEBUG("Node " << m_nodeId << " updated position at t=" << Simulator::Now().GetSeconds()
                         << "s: " << m_position);

    // Notify of course change if position changed significantly
    if (CalculateDistance(oldPosition, m_position) > 0.01)
    {
        NotifyCourseChange();
    }

    // Schedule next waypoint
    m_currentWaypoint++;
    if (m_currentWaypoint < m_waypoints.size())
    {
        Time nextUpdate = m_waypoints[m_currentWaypoint].time;
        Time now = Simulator::Now();
        Time delay = nextUpdate - now;
        
        NS_LOG_DEBUG("Node " << m_nodeId << " scheduling next update: nextUpdate=" << nextUpdate.GetSeconds()
                             << "s, now=" << now.GetSeconds() << "s, delay=" << delay.GetSeconds() << "s");
        
        if (delay.IsStrictlyPositive())
        {
            m_event = Simulator::Schedule(delay, &LeoSimMobilityModel::UpdatePosition, this);
        }
        else
        {
            NS_LOG_WARN("Waypoint time is in the past (nextUpdate=" << nextUpdate.GetSeconds() << "s, now="
                                                                     << now.GetSeconds() << "s), updating immediately");
            UpdatePosition();
        }
    }
    else
    {
        NS_LOG_DEBUG("Node " << m_nodeId << " completed all waypoints");
        m_active = false;
    }
}

Vector
LeoSimMobilityModel::CalculateVelocity(const Vector& pos1, const Vector& pos2, double dt) const
{
    Vector velocity;
    velocity.x = (pos2.x - pos1.x) / dt;
    velocity.y = (pos2.y - pos1.y) / dt;
    velocity.z = (pos2.z - pos1.z) / dt;
    return velocity;
}

Vector
LeoSimMobilityModel::InterpolatePosition(const LeoSimWaypoint& wp1,
                                          const LeoSimWaypoint& wp2,
                                          Time currentTime) const
{
    double totalTime = (wp2.time - wp1.time).GetSeconds();
    double elapsedTime = (currentTime - wp1.time).GetSeconds();
    
    if (totalTime <= 0)
    {
        return wp2.position;
    }

    double fraction = elapsedTime / totalTime;
    fraction = std::max(0.0, std::min(1.0, fraction));

    Vector position;
    position.x = wp1.position.x + fraction * (wp2.position.x - wp1.position.x);
    position.y = wp1.position.y + fraction * (wp2.position.y - wp1.position.y);
    position.z = wp1.position.z + fraction * (wp2.position.z - wp1.position.z);

    return position;
}

Vector
LeoSimMobilityModel::DoGetPosition() const
{
    // For real-time position queries, interpolate if between waypoints
    if (m_active && m_calculateVelocity && m_currentWaypoint > 0 &&
        m_currentWaypoint < m_waypoints.size())
    {
        Time now = Simulator::Now();
        const LeoSimWaypoint& prevWp = m_waypoints[m_currentWaypoint - 1];
        const LeoSimWaypoint& nextWp = m_waypoints[m_currentWaypoint];

        if (now >= prevWp.time && now < nextWp.time)
        {
            return InterpolatePosition(prevWp, nextWp, now);
        }
    }

    return m_position;
}

void
LeoSimMobilityModel::DoSetPosition(const Vector& position)
{
    NS_LOG_FUNCTION(this << position);
    m_position = position;
    NotifyCourseChange();
}

Vector
LeoSimMobilityModel::DoGetVelocity() const
{
    return m_velocity;
}

} // namespace ns3
