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

#ifndef LEOSIM_MOBILITY_HELPER_H
#define LEOSIM_MOBILITY_HELPER_H

#include "ns3/leosim-loader.h"
#include "ns3/leosim-mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"

#include <string>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Helper for installing LeoSim mobility models
 *
 * This helper simplifies the installation of LeoSimMobilityModel on nodes,
 * supporting satellites, gateways, and UEs with position data from LeoSimLoader.
 */
class LeoSimMobilityHelper
{
  public:
    /**
     * \brief Constructor
     */
    LeoSimMobilityHelper();

    /**
     * \brief Destructor
     */
    ~LeoSimMobilityHelper();

    /**
     * \brief Set the LeoSimLoader to use for position data
     * \param loader Pointer to LeoSimLoader
     */
    void SetLoader(Ptr<LeoSimLoader> loader);

    /**
     * \brief Install satellite mobility on nodes
     * \param nodes NodeContainer for satellites
     * \return Number of nodes configured
     */
    uint32_t InstallSatellites(NodeContainer& nodes);

    /**
     * \brief Install gateway mobility on nodes
     * \param nodes NodeContainer for gateways
     * \return Number of nodes configured
     */
    uint32_t InstallGateways(NodeContainer& nodes);

    /**
     * \brief Install UE mobility on nodes
     * \param nodes NodeContainer for UEs
     * \return Number of nodes configured
     */
    uint32_t InstallUEs(NodeContainer& nodes);

    /**
     * \brief Install mobility on a single satellite node
     * \param node Node pointer
     * \param satId Satellite ID
     * \param satName Satellite name
     * \return Pointer to installed mobility model
     */
    Ptr<LeoSimMobilityModel> InstallSatellite(Ptr<Node> node,
                                               uint32_t satId,
                                               const std::string& satName);

    /**
     * \brief Install mobility on a single gateway node
     * \param node Node pointer
     * \param deviceId Device ID
     * \param deviceName Device name
     * \param position Position vector
     * \return Pointer to installed mobility model
     */
    Ptr<LeoSimMobilityModel> InstallGateway(Ptr<Node> node,
                                             uint32_t deviceId,
                                             const std::string& deviceName,
                                             const Vector& position);

    /**
     * \brief Install mobility on a single UE node
     * \param node Node pointer
     * \param deviceId Device ID
     * \param deviceName Device name
     * \param position Position vector
     * \return Pointer to installed mobility model
     */
    Ptr<LeoSimMobilityModel> InstallUE(Ptr<Node> node,
                                        uint32_t deviceId,
                                        const std::string& deviceName,
                                        const Vector& position);

    /**
     * \brief Start mobility for all configured nodes
     */
    void StartAll();

    /**
     * \brief Stop mobility for all configured nodes
     */
    void StopAll();

    /**
     * \brief Set whether to calculate velocity automatically
     * \param enable True to enable velocity calculation
     */
    void SetVelocityCalculation(bool enable);

    /**
     * \brief Enable verbose logging
     * \param verbose True to enable
     */
    void SetVerbose(bool verbose);

  private:
    /**
     * \brief Convert satellite position data to waypoints
     * \param satId Satellite ID
     * \return Vector of waypoints
     */
    std::vector<LeoSimWaypoint> ConvertToWaypoints(uint32_t satId);

    Ptr<LeoSimLoader> m_loader;                           //!< LeoSim loader
    std::vector<Ptr<LeoSimMobilityModel>> m_mobilityModels; //!< Installed mobility models
    bool m_calculateVelocity;                             //!< Calculate velocity flag
    bool m_verbose;                                       //!< Verbose logging flag
};

} // namespace ns3

#endif /* LEOSIM_MOBILITY_HELPER_H */
