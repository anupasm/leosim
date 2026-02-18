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

#ifndef LEOSIM_MOBILITY_MODEL_H
#define LEOSIM_MOBILITY_MODEL_H

#include "ns3/mobility-model.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Node types for LeoSim
 */
enum LeoSimNodeType
{
    LEOSIM_SATELLITE,  //!< Satellite node
    LEOSIM_GATEWAY,    //!< Ground gateway node
    LEOSIM_UE          //!< User equipment (ground terminal)
};

/**
 * \ingroup leosim
 * \brief Position waypoint for mobility
 */
struct LeoSimWaypoint
{
    Time time;         //!< Time of waypoint
    Vector position;   //!< Position at waypoint
    Vector velocity;   //!< Velocity at waypoint (optional)
};

/**
 * \ingroup leosim
 * \brief Mobility model for LEO satellite simulations
 *
 * This mobility model supports satellites, gateways, and UEs.
 * - Satellites: Moving along orbital paths with scheduled waypoints
 * - Gateways: Fixed ground stations
 * - UEs: Ground user equipment (can be mobile or stationary)
 */
class LeoSimMobilityModel : public MobilityModel
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief Constructor
     */
    LeoSimMobilityModel();

    /**
     * \brief Destructor
     */
    ~LeoSimMobilityModel() override;

    /**
     * \brief Set the node type
     * \param nodeType Type of node (SATELLITE, GATEWAY, or UE)
     */
    void SetNodeType(LeoSimNodeType nodeType);

    /**
     * \brief Get the node type
     * \return Node type
     */
    LeoSimNodeType GetNodeType() const;

    /**
     * \brief Set node ID
     * \param id Node ID
     */
    void SetNodeId(uint32_t id);

    /**
     * \brief Get node ID
     * \return Node ID
     */
    uint32_t GetNodeId() const;

    /**
     * \brief Set node name
     * \param name Node name
     */
    void SetNodeName(const std::string& name);

    /**
     * \brief Get node name
     * \return Node name
     */
    std::string GetNodeName() const;

    /**
     * \brief Add a waypoint to the mobility path
     * \param waypoint Waypoint to add
     */
    void AddWaypoint(const LeoSimWaypoint& waypoint);

    /**
     * \brief Set all waypoints at once
     * \param waypoints Vector of waypoints
     */
    void SetWaypoints(const std::vector<LeoSimWaypoint>& waypoints);

    /**
     * \brief Get all waypoints
     * \return Vector of waypoints
     */
    std::vector<LeoSimWaypoint> GetWaypoints() const;

    /**
     * \brief Start the mobility (begin following waypoints)
     */
    void Start();

    /**
     * \brief Stop the mobility
     */
    void Stop();

    /**
     * \brief Check if mobility is active
     * \return True if active
     */
    bool IsActive() const;

    /**
     * \brief Set whether to calculate velocity automatically
     * \param enable True to enable velocity calculation
     */
    void SetVelocityCalculation(bool enable);

  protected:
    Vector DoGetPosition() const override;
    void DoSetPosition(const Vector& position) override;
    Vector DoGetVelocity() const override;

  private:
    /**
     * \brief Update position to next waypoint
     */
    void UpdatePosition();

    /**
     * \brief Calculate velocity between two positions
     * \param pos1 First position
     * \param pos2 Second position
     * \param dt Time difference
     * \return Velocity vector
     */
    Vector CalculateVelocity(const Vector& pos1, const Vector& pos2, double dt) const;

    /**
     * \brief Interpolate position between two waypoints
     * \param wp1 First waypoint
     * \param wp2 Second waypoint
     * \param currentTime Current simulation time
     * \return Interpolated position
     */
    Vector InterpolatePosition(const LeoSimWaypoint& wp1,
                               const LeoSimWaypoint& wp2,
                               Time currentTime) const;

    LeoSimNodeType m_nodeType;              //!< Type of node
    uint32_t m_nodeId;                      //!< Node ID
    std::string m_nodeName;                 //!< Node name
    Vector m_position;                      //!< Current position
    Vector m_velocity;                      //!< Current velocity
    std::vector<LeoSimWaypoint> m_waypoints; //!< Mobility waypoints
    uint32_t m_currentWaypoint;             //!< Current waypoint index
    bool m_active;                          //!< Whether mobility is active
    bool m_calculateVelocity;               //!< Whether to calculate velocity
    EventId m_event;                        //!< Next position update event
};

} // namespace ns3

#endif /* LEOSIM_MOBILITY_MODEL_H */
