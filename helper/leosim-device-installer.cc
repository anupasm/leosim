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

#include "leosim-device-installer.h"

#include "ns3/attribute.h"
#include "ns3/data-rate.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/point-to-point-channel.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/simulator.h"
#include "ns3/simple-channel.h"
#include "ns3/string.h"
#include "ns3/nstime.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimDeviceInstaller");

LeoSimDeviceInstaller::LeoSimDeviceInstaller()
    : m_dataRate("100Mbps"),
      m_delay("1ms"),
      m_delayMode(LeoSimDelayMode::GEOMETRY),
      m_delayUpdateInterval(Seconds(1)),
      m_propagationSpeed(299792458.0),
      m_mtu(1500),
      m_verbose(false),
      m_numInstalledDevices(0),
      m_devicesPerNode(10)
{
    NS_LOG_FUNCTION(this);
    // Create a shared channel for ISL devices
    m_islChannel = CreateObject<SimpleChannel>();
}

LeoSimDeviceInstaller::~LeoSimDeviceInstaller()
{
    NS_LOG_FUNCTION(this);
    Clear();
}

void
LeoSimDeviceInstaller::SetChannelModel(Ptr<LeoSimChannelModel> channelModel)
{
    NS_LOG_FUNCTION(this << channelModel);
    m_channelModel = channelModel;
}

void
LeoSimDeviceInstaller::SetOperatorModel(Ptr<LeoSimOperatorModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_operatorModel = model;
}

void
LeoSimDeviceInstaller::SetDeviceDataRate(std::string dataRate)
{
    NS_LOG_FUNCTION(this << dataRate);
    m_dataRate = dataRate;

    // Parse the configured data-rate string and use it as baseline rates.
    std::string normalized = dataRate;
    normalized.erase(std::remove_if(normalized.begin(), normalized.end(),
                                    [](unsigned char c) { return std::isspace(c) != 0; }),
                     normalized.end());
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    uint64_t multiplier = 1;
    if (normalized.size() >= 4 && normalized.substr(normalized.size() - 4) == "gbps")
    {
        multiplier = 1000000000ULL;
        normalized = normalized.substr(0, normalized.size() - 4);
    }
    else if (normalized.size() >= 4 && normalized.substr(normalized.size() - 4) == "mbps")
    {
        multiplier = 1000000ULL;
        normalized = normalized.substr(0, normalized.size() - 4);
    }
    else if (normalized.size() >= 4 && normalized.substr(normalized.size() - 4) == "kbps")
    {
        multiplier = 1000ULL;
        normalized = normalized.substr(0, normalized.size() - 4);
    }
    else if (normalized.size() >= 3 && normalized.substr(normalized.size() - 3) == "bps")
    {
        multiplier = 1ULL;
        normalized = normalized.substr(0, normalized.size() - 3);
    }

    try
    {
        const uint64_t value = static_cast<uint64_t>(std::stoull(normalized));
        const uint64_t bps = value * multiplier;
        m_baseGroundRateBps = bps;
        m_baseIslRateBps = bps;
    }
    catch (const std::exception&)
    {
        NS_LOG_WARN("Failed to parse data rate for baseline bps: " << dataRate);
    }
}

void
LeoSimDeviceInstaller::SetDeviceDelay(std::string delay)
{
    NS_LOG_FUNCTION(this << delay);
    m_delay = delay;
}

void
LeoSimDeviceInstaller::SetDelayMode(std::string mode)
{
    std::transform(mode.begin(),
                   mode.end(),
                   mode.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (mode == "constant")
    {
        m_delayMode = LeoSimDelayMode::CONSTANT;
    }
    else if (mode == "geometry")
    {
        m_delayMode = LeoSimDelayMode::GEOMETRY;
    }
    else
    {
        throw std::invalid_argument("Unknown LeoSim delay mode '" + mode +
                                    "'; expected constant or geometry");
    }
}

void
LeoSimDeviceInstaller::SetDelayUpdateInterval(Time interval)
{
    if (!interval.IsStrictlyPositive())
    {
        throw std::invalid_argument("Delay update interval must be positive");
    }
    m_delayUpdateInterval = interval;
}

void
LeoSimDeviceInstaller::SetPropagationSpeed(double metersPerSecond)
{
    if (!std::isfinite(metersPerSecond) || metersPerSecond <= 0.0)
    {
        throw std::invalid_argument("Propagation speed must be finite and positive");
    }
    m_propagationSpeed = metersPerSecond;
}

Time
LeoSimDeviceInstaller::CalculateLinkDelay(Ptr<Node> node1, Ptr<Node> node2) const
{
    if (m_delayMode == LeoSimDelayMode::CONSTANT)
    {
        return Time(m_delay);
    }

    Ptr<MobilityModel> mobility1 = node1 ? node1->GetObject<MobilityModel>() : nullptr;
    Ptr<MobilityModel> mobility2 = node2 ? node2->GetObject<MobilityModel>() : nullptr;
    if (!mobility1 || !mobility2)
    {
        NS_LOG_WARN("Geometry delay requested for link without mobility; using constant fallback");
        return Time(m_delay);
    }

    const Vector p1 = mobility1->GetPosition();
    const Vector p2 = mobility2->GetPosition();
    const double dx = p1.x - p2.x;
    const double dy = p1.y - p2.y;
    const double dz = p1.z - p2.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
    return Seconds(distance / m_propagationSpeed);
}

void
LeoSimDeviceInstaller::UpdatePropagationDelays()
{
    if (m_delayMode != LeoSimDelayMode::GEOMETRY)
    {
        return;
    }

    for (const auto& [key, devices] : m_installedLinks)
    {
        Ptr<PointToPointNetDevice> device =
            DynamicCast<PointToPointNetDevice>(devices.first);
        Ptr<PointToPointNetDevice> peer =
            DynamicCast<PointToPointNetDevice>(devices.second);
        if (!device || !peer)
        {
            continue;
        }
        Ptr<PointToPointChannel> channel =
            DynamicCast<PointToPointChannel>(device->GetChannel());
        if (!channel)
        {
            continue;
        }
        const Time delay = CalculateLinkDelay(device->GetNode(), peer->GetNode());
        channel->SetAttribute("Delay", TimeValue(delay));
    }

    ScheduleDelayUpdate();
}

void
LeoSimDeviceInstaller::ScheduleDelayUpdate()
{
    if (m_delayMode == LeoSimDelayMode::GEOMETRY && !m_delayUpdateEvent.IsPending())
    {
        m_delayUpdateEvent =
            Simulator::Schedule(m_delayUpdateInterval,
                                &LeoSimDeviceInstaller::UpdatePropagationDelays,
                                this);
    }
}

void
LeoSimDeviceInstaller::SetDeviceMtu(uint32_t mtu)
{
    NS_LOG_FUNCTION(this << mtu);
    m_mtu = mtu;
}

void
LeoSimDeviceInstaller::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
}

std::string
LeoSimDeviceInstaller::GetNodePairKey(Ptr<Node> node1, Ptr<Node> node2)
{
    NS_LOG_FUNCTION(this);
    // Create a canonical key where lower ID comes first
    uint32_t id1 = node1->GetId();
    uint32_t id2 = node2->GetId();

    if (id1 > id2)
    {
        std::swap(id1, id2);
    }

    return std::to_string(id1) + "-" + std::to_string(id2);
}

bool
LeoSimDeviceInstaller::IsLinkInstalled(Ptr<Node> node1, Ptr<Node> node2)
{
    NS_LOG_FUNCTION(this);
    std::string key = GetNodePairKey(node1, node2);
    return m_installedLinks.find(key) != m_installedLinks.end();
}

std::pair<Ptr<NetDevice>, Ptr<NetDevice>>
LeoSimDeviceInstaller::InstallLink(Ptr<Node> node1, Ptr<Node> node2)
{
    NS_LOG_FUNCTION(this << node1 << node2);

    // Check if link is already installed
    if (IsLinkInstalled(node1, node2))
    {
        auto it = m_installedLinks.find(GetNodePairKey(node1, node2));
        return it->second;
    }

    // Create point-to-point devices between the two nodes
    PointToPointHelper p2pHelper;
    p2pHelper.SetDeviceAttribute("DataRate", ns3::StringValue(m_dataRate));
    p2pHelper.SetChannelAttribute("Delay",
                                  TimeValue(CalculateLinkDelay(node1, node2)));

    NetDeviceContainer devices = p2pHelper.Install(node1, node2);

    // Set MTU on devices
    for (uint32_t i = 0; i < devices.GetN(); ++i)
    {
        devices.Get(i)->SetAttribute("Mtu", ns3::UintegerValue(m_mtu));
    }

    // Track the installation
    std::pair<Ptr<NetDevice>, Ptr<NetDevice>> devPair(devices.Get(0), devices.Get(1));
    std::string key = GetNodePairKey(node1, node2);
    m_installedLinks[key] = devPair;

    // Track devices per node
    m_nodeDevices[node1].Add(devices.Get(0));
    m_nodeDevices[node2].Add(devices.Get(1));

    m_numInstalledDevices += 2;

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installed P2P devices between nodes " << node1->GetId() << " and "
                                                           << node2->GetId());
    }

    return devPair;
}

NetDeviceContainer
LeoSimDeviceInstaller::Install(NodeContainer satellites, NodeContainer groundNodes)
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        NS_LOG_ERROR("Channel model not set");
        return NetDeviceContainer();
    }

    NetDeviceContainer allDevices;

    // Get ALL links from the channel model (both UP and DOWN)
    // This creates a full pool of devices that can be reused as links change state
    std::vector<LeoSimChannelModel::LinkSnapshot> allLinks =
        m_channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, true);
    std::vector<LeoSimChannelModel::LinkSnapshot> islLinks =
        m_channelModel->GetLinksByType(LEOSIM_LINK_ISL, true);

    // Combine all links
    allLinks.insert(allLinks.end(), islLinks.begin(), islLinks.end());

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installing devices for " << allLinks.size() << " links");
    }

    // Install devices for each link
    for (const auto& link : allLinks)
    {
        auto devPair = InstallLink(link.node1, link.node2);
        allDevices.Add(devPair.first);
        allDevices.Add(devPair.second);
    }

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installed " << allDevices.GetN() << " total devices");
    }

    ScheduleDelayUpdate();
    return allDevices;
}

NetDeviceContainer
LeoSimDeviceInstaller::Install(NodeContainer satellites,
                               NodeContainer groundNodes,
                               LeoSimLinkType linkType)
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        NS_LOG_ERROR("Channel model not set");
        return NetDeviceContainer();
    }

    NetDeviceContainer allDevices;

    // Get links of specific type
    std::vector<LeoSimChannelModel::LinkSnapshot> links =
        m_channelModel->GetLinksByType(linkType, false);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installing devices for " << links.size() << " links of type " << linkType);
    }

    // Install devices for each link
    for (const auto& link : links)
    {
        auto devPair = InstallLink(link.node1, link.node2);
        allDevices.Add(devPair.first);
        allDevices.Add(devPair.second);
    }

    if (m_verbose)
    {
        NS_LOG_DEBUG("Installed " << allDevices.GetN() << " devices for link type " << linkType);
    }

    return allDevices;
}

uint32_t
LeoSimDeviceInstaller::InstallDynamic(NodeContainer satellites, NodeContainer groundNodes)
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        NS_LOG_ERROR("Channel model not set");
        return 0;
    }

    uint32_t devicesAdded = 0;

    // Get all active links
    std::vector<LeoSimChannelModel::LinkSnapshot> allLinks =
        m_channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, false);
    std::vector<LeoSimChannelModel::LinkSnapshot> islLinks =
        m_channelModel->GetLinksByType(LEOSIM_LINK_ISL, false);

    allLinks.insert(allLinks.end(), islLinks.begin(), islLinks.end());

    // Install devices for new links
    for (const auto& link : allLinks)
    {
        if (!IsLinkInstalled(link.node1, link.node2))
        {
            InstallLink(link.node1, link.node2);
            devicesAdded += 2;
        }
    }

    if (m_verbose)
    {
        NS_LOG_DEBUG("Dynamic installation added " << devicesAdded << " devices");
    }

    return devicesAdded;
}

uint32_t
LeoSimDeviceInstaller::Update()
{
    NS_LOG_FUNCTION(this);

    if (!m_channelModel)
    {
        NS_LOG_ERROR("Channel model not set");
        return 0;
    }

    uint32_t changes = 0;

    // Get all active links
    std::vector<LeoSimChannelModel::LinkSnapshot> allLinks =
        m_channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, false);
    std::vector<LeoSimChannelModel::LinkSnapshot> islLinks =
        m_channelModel->GetLinksByType(LEOSIM_LINK_ISL, false);

    allLinks.insert(allLinks.end(), islLinks.begin(), islLinks.end());

    // Check for new links
    for (const auto& link : allLinks)
    {
        if (!IsLinkInstalled(link.node1, link.node2))
        {
            InstallLink(link.node1, link.node2);
            changes += 2;
        }
    }

    // Check for removed links (links that are not in channel model anymore)
    std::vector<std::string> linksToRemove;
    for (const auto& [key, devPair] : m_installedLinks)
    {
        // Extract node IDs from key
        size_t dashPos = key.find('-');
        uint32_t id1 = std::stoul(key.substr(0, dashPos));
        uint32_t id2 = std::stoul(key.substr(dashPos + 1));

        // Check if this link still exists in channel model
        bool linkExists = false;
        for (const auto& link : allLinks)
        {
            if ((link.node1->GetId() == id1 && link.node2->GetId() == id2) ||
                (link.node1->GetId() == id2 && link.node2->GetId() == id1))
            {
                linkExists = true;
                break;
            }
        }

        if (!linkExists)
        {
            linksToRemove.push_back(key);
        }
    }

    // Remove devices for non-existent links
    for (const auto& key : linksToRemove)
    {
        auto it = m_installedLinks.find(key);
        if (it != m_installedLinks.end())
        {
            // Note: In a real implementation, you would uninstall the devices
            // from the nodes, but ns-3 doesn't provide a direct way to do this
            m_installedLinks.erase(it);
            changes += 2;
        }
    }

    if (m_verbose)
    {
        NS_LOG_DEBUG("Update resulted in " << changes << " changes");
    }

    return changes;
}

NetDeviceContainer
LeoSimDeviceInstaller::GetDevicesForNode(Ptr<Node> node)
{
    NS_LOG_FUNCTION(this << node);
    auto it = m_nodeDevices.find(node);
    if (it != m_nodeDevices.end())
    {
        return it->second;
    }
    return NetDeviceContainer();
}

NetDeviceContainer
LeoSimDeviceInstaller::GetDevicesForLink(Ptr<Node> node1, Ptr<Node> node2)
{
    NS_LOG_FUNCTION(this << node1 << node2);
    std::string key = GetNodePairKey(node1, node2);
    auto it = m_installedLinks.find(key);
    if (it != m_installedLinks.end())
    {
        NetDeviceContainer devices;
        devices.Add(it->second.first);
        devices.Add(it->second.second);
        return devices;
    }
    return NetDeviceContainer();
}

NetDeviceContainer
LeoSimDeviceInstaller::GetAllDevices() const
{
    NS_LOG_FUNCTION(this);
    NetDeviceContainer allDevices;
    for (const auto& [key, devPair] : m_installedLinks)
    {
        allDevices.Add(devPair.first);
        allDevices.Add(devPair.second);
    }
    return allDevices;
}

uint32_t
LeoSimDeviceInstaller::GetNumDevices() const
{
    NS_LOG_FUNCTION(this);
    return m_numInstalledDevices;
}

void
LeoSimDeviceInstaller::Clear()
{
    NS_LOG_FUNCTION(this);
    if (m_delayUpdateEvent.IsPending())
    {
        Simulator::Cancel(m_delayUpdateEvent);
    }
    m_installedLinks.clear();
    m_linkErrorModels.clear();
    m_nodeDevices.clear();
    m_numInstalledDevices = 0;
    if (m_verbose)
    {
        NS_LOG_DEBUG("Cleared all installed devices");
    }
}

void
LeoSimDeviceInstaller::SetDevicesPerNode(uint32_t numDevices)
{
    NS_LOG_FUNCTION(this << numDevices);
    m_devicesPerNode = numDevices;
    if (m_verbose)
    {
        NS_LOG_DEBUG("Set devices per node to " << numDevices);
    }
}

void
LeoSimDeviceInstaller::EnableLinkStateCallbacks(Ptr<LeoSimChannelModel> channelModel)
{
    NS_LOG_FUNCTION(this << channelModel);
    if (!channelModel)
    {
        NS_LOG_ERROR("Channel model is null");
        return;
    }
    
    m_channelModel = channelModel;

    if (m_linkStateCallbacksEnabled)
    {
        return;
    }

    // Gate both receive directions of each point-to-point link. DOWN links
    // corrupt every received frame; UP and DEGRADED links pass every frame.
    for (const auto& installed : m_installedLinks)
    {
        Ptr<PointToPointNetDevice> devA =
            DynamicCast<PointToPointNetDevice>(installed.second.first);
        Ptr<PointToPointNetDevice> devB =
            DynamicCast<PointToPointNetDevice>(installed.second.second);
        if (!devA || !devB)
        {
            continue;
        }

        Ptr<RateErrorModel> gateA = CreateObject<RateErrorModel>();
        Ptr<RateErrorModel> gateB = CreateObject<RateErrorModel>();
        devA->SetReceiveErrorModel(gateA);
        devB->SetReceiveErrorModel(gateB);
        m_linkErrorModels.emplace(installed.first, std::make_pair(gateA, gateB));

        const LeoSimLinkState state =
            channelModel->GetLinkState(devA->GetNode()->GetId(), devB->GetNode()->GetId());
        const double errorRate = state == LEOSIM_LINK_DOWN ? 1.0 : 0.0;
        gateA->SetRate(errorRate);
        gateB->SetRate(errorRate);
    }
    
    // Connect to the channel model's link state change trace
    // This lets us track which devices are actually being used
    channelModel->TraceConnectWithoutContext(
        "LinkStateChange",
        MakeCallback(&LeoSimDeviceInstaller::OnLinkStateChange, this));
    m_linkStateCallbacksEnabled = true;
    
    if (m_verbose)
    {
        NS_LOG_DEBUG("Link state callbacks enabled");
    }
}

void
LeoSimDeviceInstaller::OnLinkStateChange(Ptr<Node> node1, Ptr<Node> node2, LeoSimLinkState newState)
{
    // Early exit if not enabled
    if (!m_channelModel)
    {
        return;
    }
    
    // Defensive check for null parameters
    if (!node1 || !node2)
    {
        return;
    }
    
    NS_LOG_FUNCTION(this << node1->GetId() << node2->GetId() << newState);
    
    auto errorModels = m_linkErrorModels.find(GetNodePairKey(node1, node2));
    if (errorModels == m_linkErrorModels.end())
    {
        NS_LOG_WARN("No receive gate found for link " << node1->GetId() << "<->"
                                                       << node2->GetId());
        return;
    }

    const double errorRate = newState == LEOSIM_LINK_DOWN ? 1.0 : 0.0;
    errorModels->second.first->SetRate(errorRate);
    errorModels->second.second->SetRate(errorRate);
}

LeoSimLinkDirection
LeoSimDeviceInstaller::InferDirection(Ptr<Node> nodeA, Ptr<Node> nodeB) const
{
    if (!m_operatorModel || !nodeA || !nodeB)
    {
        return LEOSIM_DIR_DOWNLINK;
    }

    const LeoSimNodeRole roleA = m_operatorModel->GetRole(nodeA->GetId());
    const LeoSimNodeRole roleB = m_operatorModel->GetRole(nodeB->GetId());

    if (roleA == LEOSIM_ROLE_SATELLITE && roleB == LEOSIM_ROLE_SATELLITE)
    {
        return LEOSIM_DIR_ISL;
    }

    if ((roleA == LEOSIM_ROLE_SATELLITE && roleB == LEOSIM_ROLE_UE) ||
        (roleB == LEOSIM_ROLE_SATELLITE && roleA == LEOSIM_ROLE_UE))
    {
        return LEOSIM_DIR_DOWNLINK;
    }

    if ((roleA == LEOSIM_ROLE_SATELLITE && roleB == LEOSIM_ROLE_SERVER) ||
        (roleB == LEOSIM_ROLE_SATELLITE && roleA == LEOSIM_ROLE_SERVER))
    {
        return LEOSIM_DIR_DOWNLINK;
    }

    return LEOSIM_DIR_DOWNLINK;
}

void
LeoSimDeviceInstaller::ApplySharingRates(NetDeviceContainer& devices)
{
    NS_LOG_FUNCTION(this << devices.GetN());

    if (!m_operatorModel)
    {
        NS_LOG_WARN("Operator model not set; skipping ApplySharingRates");
        return;
    }

    for (uint32_t i = 0; i + 1 < devices.GetN(); i += 2)
    {
        Ptr<NetDevice> devA = devices.Get(i);
        Ptr<NetDevice> devB = devices.Get(i + 1);
        if (!devA || !devB)
        {
            continue;
        }

        Ptr<Node> nodeA = devA->GetNode();
        Ptr<Node> nodeB = devB->GetNode();
        if (!nodeA || !nodeB)
        {
            continue;
        }

        const LeoSimLinkDirection dir = InferDirection(nodeA, nodeB);

        uint64_t effectiveRateBps = 1;
        double alphaUsed = 1.0;

        if (dir == LEOSIM_DIR_ISL)
        {
            alphaUsed = m_operatorModel->GetAlpha(nodeA->GetId(), nodeB->GetId(), LEOSIM_DIR_ISL);
            effectiveRateBps = m_operatorModel->GetEffectiveDataRateBps(
                nodeA->GetId(), nodeB->GetId(), m_baseIslRateBps, LEOSIM_DIR_ISL);
        }
        else
        {
            const double alphaDl =
                m_operatorModel->GetAlpha(nodeA->GetId(), nodeB->GetId(), LEOSIM_DIR_DOWNLINK);
            const double alphaUl =
                m_operatorModel->GetAlpha(nodeA->GetId(), nodeB->GetId(), LEOSIM_DIR_UPLINK);

            alphaUsed = std::min(alphaDl, alphaUl);

            const uint64_t effDl = m_operatorModel->GetEffectiveDataRateBps(
                nodeA->GetId(), nodeB->GetId(), m_baseGroundRateBps, LEOSIM_DIR_DOWNLINK);
            const uint64_t effUl = m_operatorModel->GetEffectiveDataRateBps(
                nodeA->GetId(), nodeB->GetId(), m_baseGroundRateBps, LEOSIM_DIR_UPLINK);
            effectiveRateBps = std::min(effDl, effUl);
        }

        Ptr<PointToPointNetDevice> p2pA = DynamicCast<PointToPointNetDevice>(devA);
        Ptr<PointToPointNetDevice> p2pB = DynamicCast<PointToPointNetDevice>(devB);
        if (p2pA)
        {
            p2pA->SetDataRate(DataRate(effectiveRateBps));
        }
        if (p2pB)
        {
            p2pB->SetDataRate(DataRate(effectiveRateBps));
        }

        if (m_verbose)
        {
            NS_LOG_DEBUG("Sharing rate applied: nodes " << nodeA->GetId() << "(" << m_operatorModel->GetOperatorId(nodeA->GetId())
                                                       << ") <-> " << nodeB->GetId() << "(" << m_operatorModel->GetOperatorId(nodeB->GetId())
                                                       << "), alpha=" << alphaUsed
                                                       << ", effectiveRate=" << (static_cast<double>(effectiveRateBps) / 1e6) << " Mbps");
        }
    }
}

void
LeoSimDeviceInstaller::UpdateSharingRates(NetDeviceContainer& devices)
{
    NS_LOG_FUNCTION(this << devices.GetN());
    NS_LOG_DEBUG("Updating sharing rates at t=" << Simulator::Now().GetSeconds());
    ApplySharingRates(devices);
}

} // namespace ns3
