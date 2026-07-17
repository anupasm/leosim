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

#ifndef LEOSIM_DEVICE_INSTALLER_H
#define LEOSIM_DEVICE_INSTALLER_H

#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-operator-model.h"
#include "ns3/error-model.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/ptr.h"
#include "ns3/simple-channel.h"

#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Dynamic device installer for LEO satellite networks
 *
 * This helper dynamically installs network devices on satellites and ground nodes
 * based on the links defined in LeoSimChannelModel's m_links. It supports both
 * Inter-Satellite Links (ISLs) and satellite-to-ground links.
 *
 * The installer creates point-to-point devices for each link, connecting satellites
 * and ground nodes dynamically based on the active links in the channel model.
 *
 * Usage example:
 * \code
 * Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
 * // ... add links to channel model ...
 *
 * LeoSimDeviceInstaller deviceInstaller;
 * deviceInstaller.SetChannelModel(channelModel);
 * deviceInstaller.SetDeviceDataRate("100Mbps");
 * deviceInstaller.SetDeviceDelay("1ms");
 *
 * NetDeviceContainer devices = deviceInstaller.Install(satellites, groundNodes);
 * \endcode
 */
class LeoSimDeviceInstaller
{
  public:
    /**
     * \brief Constructor
     */
    LeoSimDeviceInstaller();

    /**
     * \brief Destructor
     */
    ~LeoSimDeviceInstaller();

    /**
     * \brief Set the channel model containing the link topology
     * \param channelModel Pointer to LeoSimChannelModel
     */
    void SetChannelModel(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Set the operator model used for sharing-aware rate enforcement.
     * \param model Pointer to LeoSimOperatorModel
     */
    void SetOperatorModel(Ptr<LeoSimOperatorModel> model);

    /**
     * \brief Set data rate for installed devices
     * \param dataRate Data rate as string (e.g., "100Mbps", "1Gbps")
     */
    void SetDeviceDataRate(std::string dataRate);

    /**
     * \brief Set propagation delay for installed devices
     * \param delay Delay as string (e.g., "1ms", "10us")
     */
    void SetDeviceDelay(std::string delay);

    /**
     * \brief Set MTU size for installed devices
     * \param mtu Maximum transmission unit in bytes
     */
    void SetDeviceMtu(uint32_t mtu);

    /**
     * \brief Enable verbose output
     * \param verbose True to enable verbose logging
     */
    void SetVerbose(bool verbose);

    /**
     * \brief Install devices on all nodes based on channel model links
     *
     * Iterates through all links in the channel model's m_links and installs
     * point-to-point network devices on the connected nodes.
     *
     * \param satellites Container of satellite nodes
     * \param groundNodes Container of ground nodes (gateways and/or UEs)
     * \return Container of all installed net devices
     */
    NetDeviceContainer Install(NodeContainer satellites, NodeContainer groundNodes);

    /**
     * \brief Install devices on nodes based on specific link type
     *
     * \param satellites Container of satellite nodes
     * \param groundNodes Container of ground nodes
     * \param linkType Install devices only for links of this type
     * \return Container of installed net devices
     */
    NetDeviceContainer Install(NodeContainer satellites,
                               NodeContainer groundNodes,
                               LeoSimLinkType linkType);

    /**
     * \brief Install devices dynamically for active links
     *
     * This method installs devices and tracks them. It can be called periodically
     * to update device installation based on changing link topology.
     *
     * \param satellites Container of satellite nodes
     * \param groundNodes Container of ground nodes
     * \return Number of new devices installed
     */
    uint32_t InstallDynamic(NodeContainer satellites, NodeContainer groundNodes);

    /**
     * \brief Update device installation based on link changes
     *
     * Checks for new links in the channel model and installs devices for them,
     * while also removing devices for links that no longer exist.
     *
     * \return Number of devices added or removed
     */
    uint32_t Update();

    /**
     * \brief Get installed devices for a specific node
     * \param node Node pointer
     * \return Container of devices installed on this node
     */
    NetDeviceContainer GetDevicesForNode(Ptr<Node> node);

    /**
     * \brief Get devices for a specific link
     * \param node1 First node of the link
     * \param node2 Second node of the link
     * \return Container of devices for this link (at most 2 devices)
     */
    NetDeviceContainer GetDevicesForLink(Ptr<Node> node1, Ptr<Node> node2);

    /**
     * \brief Get all installed devices
     * \return Container of all net devices
     */
    NetDeviceContainer GetAllDevices() const;

    /**
     * \brief Get the number of installed devices
     * \return Total number of installed devices
     */
    uint32_t GetNumDevices() const;

    /**
     * \brief Clear all installed devices
     *
     * Removes all devices from nodes and clears tracking data.
     */
    void Clear();

    /**
     * \brief Set the number of devices to allocate per node in the pool
     * \param numDevices Number of devices each node should allocate
     */
    void SetDevicesPerNode(uint32_t numDevices);

    /**
     * \brief Get the number of devices per node
     */
    uint32_t GetDevicesPerNode() const { return m_devicesPerNode; }

    /**
     * \brief Enable link state change callbacks for pooled device reuse
     * \param channelModel Pointer to the channel model
     */
    void EnableLinkStateCallbacks(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Apply operator-sharing adjusted data rates to installed devices.
     *
     * Iterates over point-to-point device pairs and applies alpha-weighted
     * rates using the configured operator sharing model.
     *
     * \param devices Container of devices to update
     */
    void ApplySharingRates(NetDeviceContainer& devices);

    /**
     * \brief Re-apply sharing-aware rates without reinstalling devices.
     *
     * This can be used for dynamic renegotiation at runtime.
     *
     * \param devices Container of devices to update
     */
    void UpdateSharingRates(NetDeviceContainer& devices);

  private:
    /**
     * \brief Install point-to-point devices for a single link
     * \param node1 First node
     * \param node2 Second node
     * \return Pair of net devices (one for each node)
     */
    std::pair<Ptr<NetDevice>, Ptr<NetDevice>> InstallLink(Ptr<Node> node1, Ptr<Node> node2);

    /**
     * \brief Check if a link is already installed
     * \param node1 First node
     * \param node2 Second node
     * \return True if devices exist for this link
     */
    bool IsLinkInstalled(Ptr<Node> node1, Ptr<Node> node2);

    /**
     * \brief Callback for link state changes
     *
     * When using pooled devices, this callback is triggered when link states change.
     * It reuses available pool devices for new UP links and frees them for DOWN links.
     *
     * \param node1 First node of the link
     * \param node2 Second node of the link
     * \param newState New state of the link
     */
    void OnLinkStateChange(Ptr<Node> node1, Ptr<Node> node2, LeoSimLinkState newState);

    /**
     * \brief Infer logical link direction from node roles.
     * \param nodeA First node
     * \param nodeB Second node
     * \return Inferred LeoSimLinkDirection
     */
    LeoSimLinkDirection InferDirection(Ptr<Node> nodeA, Ptr<Node> nodeB) const;

  private:
    /**
     * \brief Create a unique key for a node pair
     * \param node1 First node
     * \param node2 Second node
     * \return String key for the node pair
     */
    std::string GetNodePairKey(Ptr<Node> node1, Ptr<Node> node2);

    // Channel model
    Ptr<LeoSimChannelModel> m_channelModel; //!< Pointer to channel model
    Ptr<LeoSimOperatorModel> m_operatorModel; //!< Operator model for sharing-aware rates

    // Device configuration
    std::string m_dataRate;  //!< Data rate for devices
    std::string m_delay;     //!< Propagation delay for devices
    uint32_t m_mtu;          //!< MTU size for devices
    uint64_t m_baseGroundRateBps = 100000000; //!< Baseline ground-link rate (100 Mbps)
    uint64_t m_baseIslRateBps = 10000000000; //!< Baseline ISL rate (10 Gbps)
    bool m_verbose;          //!< Verbose logging
    uint32_t m_numInstalledDevices;                         //!< Count of installed devices
    uint32_t m_devicesPerNode; //!< Number of devices to allocate per node

    // Track installed devices
    std::map<std::string, std::pair<Ptr<NetDevice>, Ptr<NetDevice>>>
        m_installedLinks; //!< Map of node pair keys to installed devices
    std::map<Ptr<Node>, NetDeviceContainer> m_nodeDevices; //!< Devices per node
    std::map<std::string, std::pair<Ptr<RateErrorModel>, Ptr<RateErrorModel>>>
        m_linkErrorModels; //!< Receive-side gates enforcing logical link state
    bool m_linkStateCallbacksEnabled{false};

    // Shared channel for ISL links
    Ptr<SimpleChannel> m_islChannel; //!< Channel for ISL devices
};

} // namespace ns3

#endif /* LEOSIM_DEVICE_INSTALLER_H */
