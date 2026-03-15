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

#ifndef LEOSIM_CHANNEL_MODEL_H
#define LEOSIM_CHANNEL_MODEL_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"
#include "ns3/traced-callback.h"

#include <map>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Link type classification
 */
enum LeoSimLinkType
{
    LEOSIM_LINK_SATELLITE_TO_GROUND,  //!< Satellite to ground station/UE link
    LEOSIM_LINK_ISL                    //!< Inter-Satellite Link (ISL)
};

/**
 * \ingroup leosim
 * \brief Link state between two nodes
 */
enum LeoSimLinkState
{
    LEOSIM_LINK_UP,        //!< Link is active and available
    LEOSIM_LINK_DOWN,      //!< Link is down (out of range or blocked)
    LEOSIM_LINK_DEGRADED   //!< Link is active but with reduced quality
};

/**
 * \ingroup leosim
 * \brief Channel quality metrics
 */
struct LeoSimChannelQuality
{
    double distance;           //!< Distance between nodes (meters)
    double pathLoss;           //!< Path loss (dB)
    double elevationAngle;     //!< Elevation angle from ground node (degrees)
    double signalStrength;     //!< Received signal strength (dBm)
    double snr;                //!< Signal-to-noise ratio (dB)
    LeoSimLinkState linkState; //!< Current link state
    LeoSimLinkType linkType;   //!< Link type (ground or ISL)
    Time lastUpdate;           //!< Time of last update
};

/**
 * \ingroup leosim
 * \brief Dynamic channel model for LEO satellite communications
 *
 * This model computes channel quality between satellites and ground nodes
 * based on:
 * - Distance-based path loss
 * - Elevation angle constraints
 * - Atmospheric attenuation
 * - Line-of-sight availability
 *
 * The model supports dynamic link updates as nodes move.
 */
class LeoSimChannelModel : public Object
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
    LeoSimChannelModel();

    /**
     * \brief Destructor
     */
    ~LeoSimChannelModel() override;

    /**
     * \brief Add a link between two nodes
     * \param node1 First node (typically satellite)
     * \param node2 Second node (typically ground station)
     * \param linkType Type of link (ground or ISL)
     * \return Link ID for this connection
     */
    uint32_t AddLink(Ptr<Node> node1, Ptr<Node> node2, LeoSimLinkType linkType = LEOSIM_LINK_SATELLITE_TO_GROUND);

    /**
     * \brief Add ISL (Inter-Satellite Link) between two satellites
     * \param sat1 First satellite node
     * \param sat2 Second satellite node
     * \return Link ID for this connection
     */
    uint32_t AddIslLink(Ptr<Node> sat1, Ptr<Node> sat2);

    /**
     * \brief Create ISL mesh between all satellites
     * \param satellites Container of satellite nodes
     * \return Number of ISL links created
     */
    uint32_t CreateIslMesh(NodeContainer satellites);

    /**
     * \brief Update ISL topology based on current satellite positions
     * \param satellites Container of satellite nodes
     * \param maxDistance Maximum distance for ISL contact (meters)
     * \return Number of active ISL links
     * 
     * This method dynamically creates ISL links between satellites that are
     * within contact range and removes links that are out of range.
     */
    uint32_t UpdateIslTopology(NodeContainer satellites, double maxDistance);

    /**
     * \brief Remove a link between two nodes
     * \param node1 First node
     * \param node2 Second node
     */
    void RemoveLink(Ptr<Node> node1, Ptr<Node> node2);

    /**
     * \brief Update all links
     *
     * Recalculates channel quality for all links
     */
    void UpdateAllLinks();

    /**
     * \brief Get channel quality for a specific link
     * \param node1 First node
     * \param node2 Second node
     * \return Channel quality metrics
     */
    LeoSimChannelQuality GetChannelQuality(Ptr<Node> node1, Ptr<Node> node2);

    /**
     * \brief Check if link is available
     * \param node1 First node
     * \param node2 Second node
     * \return True if link is up
     */
    bool IsLinkUp(Ptr<Node> node1, Ptr<Node> node2);

    /**
     * \brief Set minimum elevation angle for ground-to-satellite links
     * \param angle Minimum elevation angle in degrees
     */
    void SetMinElevationAngle(double angle);

    /**
     * \brief Set ISL maximum distance
     * \param distance Maximum ISL distance in meters
     */
    void SetIslMaxDistance(double distance);

    /**
     * \brief Get ISL maximum distance
     * \return Maximum ISL distance in meters
     */
    double GetIslMaxDistance() const;

    /**
     * \brief Set ISL transmit power
     * \param power ISL transmit power in dBm
     */
    void SetIslTransmitPower(double power);

    /**
     * \brief Get ISL transmit power
     * \return ISL transmit power in dBm
     */
    double GetIslTransmitPower() const;

    /**
     * \brief Set ISL antenna gain
     * \param gain ISL antenna gain in dB
     */
    void SetIslAntennaGain(double gain);

    /**
     * \brief Get ISL antenna gain
     * \return ISL antenna gain in dB
     */
    double GetIslAntennaGain() const;

    /**
     * \brief Set ISL frequency
     * \param frequency ISL frequency in Hz (e.g., Ka-band 26 GHz or optical)
     */
    void SetIslFrequency(double frequency);

    /**
     * \brief Get ISL frequency
     * \return ISL frequency in Hz
     */
    double GetIslFrequency() const;

    /**
     * \brief Get minimum elevation angle
     * \return Minimum elevation angle in degrees
     */
    double GetMinElevationAngle() const;

    /**
     * \brief Set maximum link distance
     * \param distance Maximum distance in meters
     */
    void SetMaxLinkDistance(double distance);

    /**
     * \brief Get maximum link distance
     * \return Maximum distance in meters
     */
    double GetMaxLinkDistance() const;

    /**
     * \brief Set carrier frequency
     * \param frequency Frequency in Hz
     */
    void SetFrequency(double frequency);

    /**
     * \brief Get carrier frequency
     * \return Frequency in Hz
     */
    double GetFrequency() const;

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
     * \brief Get transmit power
     * \return Power in dBm
     */
    double GetTransmitPower() const;

    /**
     * \brief Get transmit antenna gain
     * \return Gain in dB
     */
    double GetTxAntennaGain() const;

    /**
     * \brief Set receive antenna gain
     * \param gain Gain in dB
     */
    void SetRxAntennaGain(double gain);

    /**
     * \brief Get receive antenna gain
     * \return Gain in dB
     */
    double GetRxAntennaGain() const;

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
     * \brief Get system noise temperature
     * \return Temperature in Kelvin
     */
    double GetNoiseTemperature() const;

    /**
     * \brief Get noise bandwidth
     * \return Noise bandwidth in Hz
     */
    double GetNoiseBandwidth() const;

    /**
     * \brief Enable/disable atmospheric attenuation model
     * \param enable True to enable
     */
    void SetAtmosphericAttenuationEnabled(bool enable);

    /**
     * \brief Check if atmospheric attenuation is enabled
     * \return True if enabled
     */
    bool IsAtmosphericAttenuationEnabled() const;

    /**
     * \brief Set update interval for channel calculations
     * \param interval Update interval
     */
    void SetUpdateInterval(Time interval);

    /**
     * \brief Get update interval
     * \return Update interval
     */
    Time GetUpdateInterval() const;

    /**
     * \brief Enable verbose output
     * \param verbose True to enable verbose logging
     */
    void SetVerbose(bool verbose);

    /**
     * \brief Start periodic updates
     */
    void StartUpdates();

    /**
     * \brief Stop periodic updates
     */
    void StopUpdates();

    /**
     * \brief Get all active links
     * \return Vector of node pairs representing active links
     */
    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> GetActiveLinks() const;

    /**
     * \brief Lightweight snapshot of link state
     */
    struct LinkSnapshot
    {
        Ptr<Node> node1;
        Ptr<Node> node2;
        uint32_t linkId;
        LeoSimLinkType linkType;
        LeoSimLinkState linkState;
    };

    /**
     * \brief Get links of a specific type, optionally including DOWN links
     * \param linkType Link type to filter
     * \param includeDown Whether to include DOWN links
     * \return Vector of link snapshots
     */
    std::vector<LinkSnapshot> GetLinksByType(LeoSimLinkType linkType, bool includeDown) const;

    /**
     * \brief Get all link quality records for a node with UP or DEGRADED state
     * \param nodeId Node identifier
     * \return Vector of LeoSimChannelQuality records for links connected to this node
     *         where link state is UP or DEGRADED
     */
    std::vector<LeoSimChannelQuality> GetLinksForNode(uint32_t nodeId) const;

    /**
     * \brief Get link quality information for a specific node pair
     * \param nodeA First node identifier
     * \param nodeB Second node identifier
     * \return LeoSimChannelQuality struct for this link pair, or default-constructed
     *         quality with state=DOWN if link does not exist
     */
    LeoSimChannelQuality GetLinkQuality(uint32_t nodeA, uint32_t nodeB) const;

    /**
     * \brief Get link state for a specific node pair
     * \param nodeA First node identifier
     * \param nodeB Second node identifier
     * \return Current LeoSimLinkState for this node pair, or LEOSIM_LINK_DOWN if not found
     */
    LeoSimLinkState GetLinkState(uint32_t nodeA, uint32_t nodeB) const;

  private:
    /**
     * \brief Calculate free space path loss
     * \param distance Distance in meters
     * \return Path loss in dB
     */
    double CalculateFreeSpacePathLoss(double distance) const;

    /**
     * \brief Calculate atmospheric attenuation
     * \param distance Distance in meters
     * \param elevationAngle Elevation angle in degrees
     * \return Attenuation in dB
     */
    double CalculateAtmosphericAttenuation(double distance, double elevationAngle) const;

    /**
     * \brief Calculate elevation angle from ground node to satellite
     * \param groundPos Ground node position
     * \param satPos Satellite position
     * \return Elevation angle in degrees
     */
    double CalculateElevationAngle(const Vector& groundPos, const Vector& satPos) const;

    /**
     * \brief Update a specific link
     * \param linkId Link ID
     */
    void UpdateLink(uint32_t linkId);

    /**
     * \brief Periodic update callback
     */
    void PeriodicUpdate();

    /**
     * \brief Get or create link ID for node pair
     * \param node1 First node
     * \param node2 Second node
     * \return Link ID
     */
    uint32_t GetLinkId(Ptr<Node> node1, Ptr<Node> node2);

    // Link tracking
    struct LinkInfo
    {
        Ptr<Node> node1;
        Ptr<Node> node2;
        LeoSimChannelQuality quality;
        uint32_t linkId;
        LeoSimLinkType linkType;
    };

    std::map<uint32_t, LinkInfo> m_links;              //!< Map of link ID to link info
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> m_nodePairToLink; //!< Node pair to link ID
    uint32_t m_nextLinkId;                             //!< Next available link ID

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
    Time m_updateInterval;          //!< Update interval for periodic updates
    bool m_verbose;                 //!< Verbose logging

    // ISL-specific parameters
    double m_islMaxDistance;        //!< Maximum ISL distance (meters)
    double m_islTransmitPower;      //!< ISL transmit power (dBm)
    double m_islAntennaGain;        //!< ISL antenna gain (dB)
    double m_islFrequency;          //!< ISL frequency (Hz)

    // Update scheduling
    EventId m_updateEvent;          //!< Scheduled update event

    // Traced callbacks
    TracedCallback<Ptr<Node>, Ptr<Node>, LeoSimLinkState> m_linkStateChangeTrace; //!< Link state change
    TracedCallback<Ptr<Node>, Ptr<Node>, double> m_pathLossTrace; //!< Path loss update
};

} // namespace ns3

#endif /* LEOSIM_CHANNEL_MODEL_H */
