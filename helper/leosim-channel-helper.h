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

#ifndef LEOSIM_CHANNEL_HELPER_H
#define LEOSIM_CHANNEL_HELPER_H

#include "ns3/leosim-channel-model.h"
#include "ns3/node-container.h"
#include "ns3/ptr.h"
#include "ns3/vector.h"

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Helper for creating and managing dynamic satellite-to-ground channels
 *
 * This helper simplifies the setup of dynamic channels between satellites and
 * ground nodes (gateways and UEs). It automatically creates and configures
 * LeoSimChannelModel instances with appropriate parameters.
 */
class LeoSimChannelHelper
{
  public:
    /**
     * \brief Constructor
     */
    LeoSimChannelHelper();

    /**
     * \brief Destructor
     */
    ~LeoSimChannelHelper();

    /**
     * \brief Create bounded channels between satellites and ground nodes
     * \param satellites Satellite nodes
     * \param groundNodes Ground nodes (gateways and/or UEs)
     * \return Pointer to created channel model
     */
    Ptr<LeoSimChannelModel> CreateChannels(NodeContainer satellites, NodeContainer groundNodes);

    /**
     * \brief Create channels between satellites and gateways
     * \param satellites Satellite nodes
     * \param gateways Gateway nodes
     * \return Pointer to created channel model
     */
    Ptr<LeoSimChannelModel> CreateSatelliteToGatewayChannels(NodeContainer satellites,
                                                               NodeContainer gateways);

    /**
     * \brief Create channels between satellites and UEs
     * \param satellites Satellite nodes
     * \param ues UE nodes
     * \return Pointer to created channel model
     */
    Ptr<LeoSimChannelModel> CreateSatelliteToUeChannels(NodeContainer satellites,
                                                          NodeContainer ues);

    /**
     * \brief Create ISL (Inter-Satellite Link) mesh between all satellites
     * \param satellites Satellite nodes
     * \return Pointer to created channel model with ISL mesh
     */
    Ptr<LeoSimChannelModel> CreateIslMesh(NodeContainer satellites);

    /**
     * \brief Create bounded nearest-neighbor ISL candidates.
     *
     * This avoids the O(N^2) link count of a full ISL mesh. A bounded spatial
     * candidate set is used to construct a degree-limited spanning forest,
     * followed by shortest-edge capacity filling. Connectivity is audited.
     *
     * \param satellites Satellite nodes
     * \param maxNeighbors Maximum nearest neighbors per satellite
     * \return Pointer to created channel model with bounded ISL links
     */
    Ptr<LeoSimChannelModel> CreateIslNearestNeighborMesh(NodeContainer satellites,
                                                         uint32_t maxNeighbors);

    /**
     * \brief Create realistic bounded ISL topology candidates.
     *
     * Satellites are interpreted as a plane-major constellation grid:
     * index = plane * satellitesPerPlane + slot. Each satellite gets candidate
     * ISLs to the forward/backward satellites in the same plane and to the same
     * slot in the two adjacent planes. Physical availability is still governed
     * by the channel model's ISL distance and quality constraints.
     *
     * \param satellites Satellite nodes in plane-major order
     * \param satellitesPerPlane Number of slots in each orbital plane
     * \param wrapPlanes Whether first/last planes are adjacent
     * \return Pointer to created channel model with bounded ISL links
     */
    Ptr<LeoSimChannelModel> CreateIslGridTopology(NodeContainer satellites,
                                                  uint32_t satellitesPerPlane,
                                                  bool wrapPlanes = true);

    /**
     * \brief Add ISL links to existing channel model
     * \param channelModel Existing channel model
     * \param satellites Satellite nodes
     * \return Number of ISL links added
     */
    uint32_t AddIslLinks(Ptr<LeoSimChannelModel> channelModel, NodeContainer satellites);

    /**
     * \brief Add bounded nearest-neighbor ISL links to an existing channel model.
     * \param channelModel Existing channel model
     * \param satellites Satellite nodes
     * \param maxNeighbors Maximum nearest neighbors per satellite
     * \return Number of ISL links added
     */
    uint32_t AddNearestNeighborIslLinks(Ptr<LeoSimChannelModel> channelModel,
                                        NodeContainer satellites,
                                        uint32_t maxNeighbors);

    /**
     * \brief Add realistic bounded ISL topology candidates to an existing model.
     * \param channelModel Existing channel model
     * \param satellites Satellite nodes in plane-major order
     * \param satellitesPerPlane Number of slots in each orbital plane
     * \param wrapPlanes Whether first/last planes are adjacent
     * Candidate pairs outside the configured ISL range at creation time are skipped.
     * \return Number of initially in-range ISL links added
     */
    uint32_t AddGridIslLinks(Ptr<LeoSimChannelModel> channelModel,
                             NodeContainer satellites,
                             uint32_t satellitesPerPlane,
                             bool wrapPlanes = true);

    /**
     * \brief Add single ISL link to existing channel model
     * \param channelModel Existing channel model
     * \param sat1 First satellite
     * \param sat2 Second satellite
     * \return Link ID
     */
    uint32_t AddIslLink(Ptr<LeoSimChannelModel> channelModel, Ptr<Node> sat1, Ptr<Node> sat2);

    /**
     * \brief Update ISL topology dynamically based on satellite positions
     * \param channelModel Existing channel model
     * \param satellites Satellite nodes
     * \return Number of active ISL links
     * 
     * Creates ISL links between satellites in contact and removes out-of-range links
     */
    uint32_t UpdateIslTopology(Ptr<LeoSimChannelModel> channelModel, NodeContainer satellites);

    /**
     * \brief Add links to existing channel model
     * \param channelModel Existing channel model
     * \param satellites Satellite nodes
     * \param groundNodes Ground nodes
     */
    void AddLinks(Ptr<LeoSimChannelModel> channelModel,
                  NodeContainer satellites,
                  NodeContainer groundNodes);

    /**
     * \brief Set maximum satellite access-link candidates per ground node
     * \param maxLinks Maximum links to create per ground node, minimum 1
     */
    void SetMaxGroundLinksPerNode(uint32_t maxLinks);

    /**
     * \brief Set minimum elevation angle
     * \param angle Minimum elevation angle in degrees
     */
    void SetMinElevationAngle(double angle);

    /**
     * \brief Set maximum link distance
     * \param distance Maximum distance in meters
     */
    void SetMaxLinkDistance(double distance);

    /**
     * \brief Set carrier frequency
     * \param frequency Frequency in Hz
     */
    void SetFrequency(double frequency);

    /**
     * \brief Set transmit power
     * \param power Power in dBm
     */
    void SetTransmitPower(double power);

    /**
     * \brief Set transmit antenna gain
     * \param gain Gain in dB
     */
    void SetTxAntennaGain(double gain);

    /**
     * \brief Set receive antenna gain
     * \param gain Gain in dB
     */
    void SetRxAntennaGain(double gain);

    /**
     * \brief Set system noise temperature
     * \param temperature Temperature in Kelvin
     */
    void SetNoiseTemperature(double temperature);

    /**
     * \brief Set noise bandwidth
     * \param bandwidth Noise bandwidth in Hz
     */
    void SetNoiseBandwidth(double bandwidth);

    /**
     * \brief Enable/disable atmospheric attenuation
     * \param enable True to enable
     */
    void SetAtmosphericAttenuationEnabled(bool enable);

    /**
     * \brief Set update interval for channel calculations
     * \param interval Update interval
     */
    void SetUpdateInterval(Time interval);

    /**
     * \brief Enable verbose output
     * \param verbose True to enable verbose logging
     */
    void SetVerbose(bool verbose);

    /**
     * \brief Set ISL maximum distance
     * \param distance Maximum ISL distance in meters
     */
    void SetIslMaxDistance(double distance);

    /**
     * \brief Set ISL transmit power
     * \param power ISL transmit power in dBm
     */
    void SetIslTransmitPower(double power);

    /**
     * \brief Set ISL antenna gain
     * \param gain ISL antenna gain in dB
     */
    void SetIslAntennaGain(double gain);

    /**
     * \brief Set ISL frequency
     * \param frequency ISL frequency in Hz (e.g., 26 GHz Ka-band or optical)
     */
    void SetIslFrequency(double frequency);

    /**
     * \brief Install channel callback for link state changes
     * \param channelModel Channel model
     * \param callback Callback function
     */
    void InstallLinkStateChangeCallback(
        Ptr<LeoSimChannelModel> channelModel,
        Callback<void, Ptr<Node>, Ptr<Node>, LeoSimLinkState> callback);

    /**
     * \brief Install channel callback for path loss updates
     * \param channelModel Channel model
     * \param callback Callback function
     */
    void InstallPathLossCallback(Ptr<LeoSimChannelModel> channelModel,
                                  Callback<void, Ptr<Node>, Ptr<Node>, double> callback);

    /**
     * \brief Get channel model with configured attributes
     * \return New channel model instance
     */
    Ptr<LeoSimChannelModel> GetChannelModel();

    /**
     * \brief Print channel statistics
     * \param channelModel Channel model
     */
    void PrintChannelStatistics(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Log all active links
     * \param channelModel Channel model
     * \param prefix Log message prefix
     */
    void LogActiveLinks(Ptr<LeoSimChannelModel> channelModel, const std::string& prefix = "");

  private:
    /**
     * \brief Apply configured attributes to channel model
     * \param channelModel Channel model to configure
     */
    void ConfigureChannelModel(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Check if a link between satellite and ground node is feasible
     * \param satellite Satellite node
     * \param groundNode Ground node
     * \return True if link is feasible based on elevation angle and distance constraints
     */
    bool IsLinkFeasible(Ptr<Node> satellite, Ptr<Node> groundNode) const;

    /**
     * \brief Add bounded satellite-ground links to a channel model
     * \param channelModel Existing channel model
     * \param satellites Satellite nodes
     * \param groundNodes Ground nodes
     * \return Number of links added
     */
    uint32_t AddGroundAccessLinks(Ptr<LeoSimChannelModel> channelModel,
                                  NodeContainer satellites,
                                  NodeContainer groundNodes);

    /**
     * \brief Calculate elevation angle from ground node to satellite
     * \param groundPos Ground-node ECEF position
     * \param satPos Satellite ECEF position
     * \return Elevation angle in degrees
     */
    double CalculateElevationAngle(const Vector& groundPos, const Vector& satPos) const;

    // Channel parameters
    double m_minElevationAngle;     //!< Minimum elevation angle (degrees)
    double m_maxLinkDistance;       //!< Maximum link distance (meters)
    double m_frequency;             //!< Carrier frequency (Hz)
    double m_transmitPower;         //!< Transmit power (dBm)
    double m_txAntennaGain;         //!< Transmit antenna gain (dB)
    double m_rxAntennaGain;         //!< Receive antenna gain (dB)
    double m_noiseTemperature;      //!< System noise temperature (K)
    double m_noiseBandwidth;        //!< Noise bandwidth (Hz)
    bool m_atmosphericEnabled;      //!< Enable atmospheric attenuation
    Time m_updateInterval;          //!< Update interval
    bool m_verbose;                 //!< Verbose logging
    uint32_t m_maxGroundLinksPerNode; //!< Maximum satellite candidates per ground node

    // ISL-specific parameters
    double m_islMaxDistance;        //!< Maximum ISL distance (meters)
    double m_islTransmitPower;      //!< ISL transmit power (dBm)
    double m_islAntennaGain;        //!< ISL antenna gain (dB)
    double m_islFrequency;          //!< ISL frequency (Hz)
};

} // namespace ns3

#endif /* LEOSIM_CHANNEL_HELPER_H */
