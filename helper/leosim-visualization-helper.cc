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

#include "leosim-visualization-helper.h"

#include "ns3/channel.h"
#include "ns3/config.h"
#include "ns3/mobility-model.h"
#include "ns3/names.h"
#include "ns3/net-device.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

#include <iomanip>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimVisualizationHelper");

LeoSimVisualizationHelper::LeoSimVisualizationHelper()
    : m_outputFile("leosim_positions.csv"),
      m_linkFile("leosim_links.csv"),
    m_packetFile("leosim_packets.csv"),
      m_loaderHelper(nullptr),
    m_channelModel(nullptr),
    m_islChannelModel(nullptr),
    m_enablePacketLogging(false),
    m_packetLoggingInstalled(false)
{
}

LeoSimVisualizationHelper::~LeoSimVisualizationHelper()
{
    if (m_posFile.is_open())
    {
        m_posFile.close();
    }
    if (m_linkFileStream.is_open())
    {
        m_linkFileStream.close();
    }
    if (m_packetFileStream.is_open())
    {
        m_packetFileStream.close();
    }
}

void
LeoSimVisualizationHelper::SetOutputFile(std::string outputFile)
{
    m_outputFile = outputFile;
}

void
LeoSimVisualizationHelper::SetLinkFile(std::string linkFile)
{
    m_linkFile = linkFile;
}

void
LeoSimVisualizationHelper::SetPacketFile(std::string packetFile)
{
    m_packetFile = packetFile;
}

void
LeoSimVisualizationHelper::EnablePacketLogging(bool enable)
{
    m_enablePacketLogging = enable;
}

void
LeoSimVisualizationHelper::SetLoaderHelper(LeoSimLoaderHelper& loaderHelper)
{
    m_loaderHelper = &loaderHelper;
}

void
LeoSimVisualizationHelper::SetChannelModel(Ptr<LeoSimChannelModel> channelModel)
{
    m_channelModel = channelModel;
}

void
LeoSimVisualizationHelper::SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel)
{
    m_islChannelModel = islChannelModel;
}

void
LeoSimVisualizationHelper::Initialize()
{
    m_posFile.open(m_outputFile);
    if (!m_posFile.is_open())
    {
        NS_LOG_ERROR("Could not open output file: " << m_outputFile);
        return;
    }

    // Write CSV header for positions
    m_posFile << "time,type,id,name,x,y,z" << std::endl;

    m_linkFileStream.open(m_linkFile);
    if (!m_linkFileStream.is_open())
    {
        NS_LOG_ERROR("Could not open link file: " << m_linkFile);
    }
    else
    {
        // Write CSV header for links
        m_linkFileStream << "time,sat_id,ground_id,ground_type" << std::endl;
    }

    if (m_enablePacketLogging)
    {
        m_packetFileStream.open(m_packetFile);
        if (!m_packetFileStream.is_open())
        {
            NS_LOG_ERROR("Could not open packet file: " << m_packetFile);
        }
        else
        {
            m_packetFileStream << "time,event,node_id,device_id,peer_node_id,link_type,size_bytes,snr_db,doppler_hz"
                               << std::endl;
        }
    }
}

void
LeoSimVisualizationHelper::Finalize()
{
    if (m_posFile.is_open())
    {
        m_posFile.close();
    }
    if (m_linkFileStream.is_open())
    {
        m_linkFileStream.close();
    }
    if (m_packetFileStream.is_open())
    {
        m_packetFileStream.close();
    }
}

void
LeoSimVisualizationHelper::LogNodePosition(Ptr<Node> node,
                                           std::string nodeType,
                                           uint32_t nodeId,
                                           std::string nodeName)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();

    if (!mobility)
    {
        return;
    }

    Vector pos = mobility->GetPosition();
    double time = Simulator::Now().GetSeconds();

    std::string resolvedName = Names::FindName(node);
    if (resolvedName.empty())
    {
        resolvedName = nodeName;
    }
    // Write to file: time, type, id, name, x, y, z
    m_posFile << std::fixed << std::setprecision(3);
    m_posFile << time << "," << nodeType << "," << nodeId << "," << resolvedName << "," << pos.x << ","
              << pos.y << "," << pos.z << std::endl;
}

void
LeoSimVisualizationHelper::LogIslConnections()
{
    if (!m_islChannelModel || !m_linkFileStream.is_open())
    {
        return;
    }

    double time = Simulator::Now().GetSeconds();

    auto islLinks = m_islChannelModel->GetLinksByType(LEOSIM_LINK_ISL, false);
    for (const auto& link : islLinks)
    {
        m_linkFileStream << std::fixed << std::setprecision(3);
        m_linkFileStream << time << "," << link.node1->GetId() << ","
                        << link.node2->GetId() << ",ISL" << std::endl;
    }
}

void
LeoSimVisualizationHelper::LogGroundConnections()
{
    if (!m_channelModel || !m_linkFileStream.is_open())
    {
        return;
    }

    double time = Simulator::Now().GetSeconds();

    auto groundLinks = m_channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, false);

    for (const auto& link : groundLinks)
    {
        Ptr<Node> node1 = link.node1;
        Ptr<Node> node2 = link.node2;

        Ptr<Node> sat = nullptr;
        Ptr<Node> ground = nullptr;
        std::string groundType;

        // Check if node1 is a satellite
        bool node1IsSat = false;
        for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
        {
            if (m_satellites.Get(i) == node1)
            {
                node1IsSat = true;
                sat = node1;
                ground = node2;
                break;
            }
        }

        if (!node1IsSat)
        {
            for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
            {
                if (m_satellites.Get(i) == node2)
                {
                    sat = node2;
                    ground = node1;
                    break;
                }
            }
        }

        if (!sat || !ground)
        {
            continue;
        }

        // Find ground station type
        for (uint32_t i = 0; i < m_servers.GetN(); ++i)
        {
            if (m_servers.Get(i) == ground)
            {
                groundType = "SERVER";
                break;
            }
        }

        if (groundType.empty())
        {
            for (uint32_t i = 0; i < m_ues.GetN(); ++i)
            {
                if (m_ues.Get(i) == ground)
                {
                    groundType = "UE";
                    break;
                }
            }
        }

        if (!groundType.empty())
        {
            m_linkFileStream << std::fixed << std::setprecision(3);
            m_linkFileStream << time << "," << sat->GetId() << ","
                            << ground->GetId() << "," << groundType << std::endl;
        }
    }
}

void
LeoSimVisualizationHelper::SchedulePositionLogging(NodeContainer satellites,
                                                   NodeContainer servers,
                                                   NodeContainer ues,
                                                   double interval,
                                                   double simTime)
{
    if (!m_loaderHelper)
    {
        NS_LOG_ERROR("LoaderHelper not set!");
        return;
    }

    // Store node containers for link logging
    m_satellites = satellites;
    m_servers = servers;
    m_ues = ues;

    for (double t = 0; t <= simTime; t += interval)
    {
        // Log satellites
        for (uint32_t i = 0; i < satellites.GetN(); ++i)
        {
            Ptr<Node> node = satellites.Get(i);
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogNodePosition,
                                this,
                                node,
                                "SATELLITE",
                                node->GetId(),
                                m_loaderHelper->GetSatelliteName(i));
        }

        // Log servers
        for (uint32_t i = 0; i < servers.GetN(); ++i)
        {
            Ptr<Node> node = servers.Get(i);
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogNodePosition,
                                this,
                                node,
                                "SERVER",
                                node->GetId(),
                                m_loaderHelper->GetGroundDeviceName(i));
        }

        // Log UEs
        for (uint32_t i = 0; i < ues.GetN(); ++i)
        {
            Ptr<Node> node = ues.Get(i);
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogNodePosition,
                                this,
                                node,
                                "UE",
                                node->GetId(),
                                m_loaderHelper->GetGroundDeviceName(servers.GetN() + i));
        }

        // Log ground link connections if channel model is set
        if (m_channelModel)
        {
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogGroundConnections,
                                this);
        }

        // Log ISL connections if ISL channel model is set
        if (m_islChannelModel)
        {
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogIslConnections,
                                this);
        }
    }
}

bool
LeoSimVisualizationHelper::ParseContextIds(const std::string& context,
                                           int& nodeId,
                                           int& deviceId) const
{
    nodeId = -1;
    deviceId = -1;

    std::size_t nodePos = context.find("/NodeList/");
    if (nodePos == std::string::npos)
    {
        return false;
    }
    nodePos += std::string("/NodeList/").size();
    std::size_t nodeEnd = context.find("/", nodePos);
    if (nodeEnd == std::string::npos)
    {
        return false;
    }

    std::size_t devPos = context.find("/DeviceList/", nodeEnd);
    if (devPos == std::string::npos)
    {
        return false;
    }
    devPos += std::string("/DeviceList/").size();
    std::size_t devEnd = context.find("/", devPos);
    if (devEnd == std::string::npos)
    {
        return false;
    }

    try
    {
        nodeId = std::stoi(context.substr(nodePos, nodeEnd - nodePos));
        deviceId = std::stoi(context.substr(devPos, devEnd - devPos));
    }
    catch (...)
    {
        return false;
    }

    return true;
}

bool
LeoSimVisualizationHelper::IsSatelliteNode(Ptr<Node> node) const
{
    for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
    {
        if (m_satellites.Get(i) == node)
        {
            return true;
        }
    }
    return false;
}

Ptr<Node>
LeoSimVisualizationHelper::GetDevicePeerNode(int nodeId, int deviceId) const
{
    Ptr<Node> node = NodeList::GetNode(nodeId);
    if (!node || deviceId >= (int)node->GetNDevices())
    {
        return nullptr;
    }

    Ptr<NetDevice> netDevice = node->GetDevice(deviceId);
    if (!netDevice)
    {
        return nullptr;
    }

    Ptr<Channel> channel = netDevice->GetChannel();
    if (!channel || channel->GetNDevices() < 2)
    {
        return nullptr;
    }

    // Find the peer device on the other end of the channel
    for (std::size_t j = 0; j < channel->GetNDevices(); ++j)
    {
        Ptr<NetDevice> peerDevice = channel->GetDevice(j);
        if (peerDevice != netDevice)
        {
            return peerDevice->GetNode();
        }
    }

    return nullptr;
}

std::string
LeoSimVisualizationHelper::GetLinkType(Ptr<Node> node, Ptr<Node> peerNode) const
{
    if (!node || !peerNode)
    {
        return "UNKNOWN";
    }

    const bool nodeIsSat = IsSatelliteNode(node);
    const bool peerIsSat = IsSatelliteNode(peerNode);
    return (nodeIsSat && peerIsSat) ? "ISL" : "GROUND";
}



void
LeoSimVisualizationHelper::OnPhyTx(std::string context, Ptr<const Packet> packet)
{
    if (!m_packetFileStream.is_open())
    {
        return;
    }

    int nodeId = -1;
    int deviceId = -1;
    if (!ParseContextIds(context, nodeId, deviceId))
    {
        return;
    }

    int peerNodeId = -1;
    std::string linkType = "UNKNOWN";
    
    // Get peer node dynamically from channel
    Ptr<Node> peerNode = GetDevicePeerNode(nodeId, deviceId);
    if (peerNode)
    {
        peerNodeId = peerNode->GetId();
        Ptr<Node> currNode = NodeList::GetNode(nodeId);
        if (currNode)
        {
            linkType = GetLinkType(currNode, peerNode);
        }
    }

    m_packetFileStream << std::fixed << std::setprecision(3);
    m_packetFileStream << Simulator::Now().GetSeconds() << ",TX," << nodeId << "," << deviceId
                       << "," << peerNodeId << "," << linkType << "," << packet->GetSize()
                       << "," << 0.0 << "," << 0.0 << std::endl;
}

void
LeoSimVisualizationHelper::OnPhyRx(std::string context,
                                   Ptr<const Packet> packet,
                                   double snrDb,
                                   double dopplerHz)
{
    if (!m_packetFileStream.is_open())
    {
        return;
    }

    int nodeId = -1;
    int deviceId = -1;
    if (!ParseContextIds(context, nodeId, deviceId))
    {
        return;
    }

    int peerNodeId = -1;
    std::string linkType = "UNKNOWN";
    
    // Get peer node dynamically from channel
    Ptr<Node> peerNode = GetDevicePeerNode(nodeId, deviceId);
    if (peerNode)
    {
        peerNodeId = peerNode->GetId();
        Ptr<Node> currNode = NodeList::GetNode(nodeId);
        if (currNode)
        {
            linkType = GetLinkType(currNode, peerNode);
        }
    }

    m_packetFileStream << std::fixed << std::setprecision(3);
    m_packetFileStream << Simulator::Now().GetSeconds() << ",RX," << nodeId << "," << deviceId
                       << "," << peerNodeId << "," << linkType << "," << packet->GetSize()
                       << "," << snrDb << "," << dopplerHz << std::endl;
}

void
LeoSimVisualizationHelper::OnPhyRxBasic(std::string context, Ptr<const Packet> packet)
{
    // Wrapper for OnPhyRx when SNR/Doppler are not available
    OnPhyRx(context, packet, 0.0, 0.0);
}


void
LeoSimVisualizationHelper::OnPhyRxDrop(std::string context, Ptr<const Packet> packet)
{
    if (!m_packetFileStream.is_open())
    {
        return;
    }

    int nodeId = -1;
    int deviceId = -1;
    if (!ParseContextIds(context, nodeId, deviceId))
    {
        return;
    }

    int peerNodeId = -1;
    std::string linkType = "UNKNOWN";
    
    // Get peer node dynamically from channel
    Ptr<Node> peerNode = GetDevicePeerNode(nodeId, deviceId);
    if (peerNode)
    {
        peerNodeId = peerNode->GetId();
        Ptr<Node> currNode = NodeList::GetNode(nodeId);
        if (currNode)
        {
            linkType = GetLinkType(currNode, peerNode);
        }
    }

    m_packetFileStream << std::fixed << std::setprecision(3);
    m_packetFileStream << Simulator::Now().GetSeconds() << ",DROP," << nodeId << "," << deviceId
                       << "," << peerNodeId << "," << linkType << "," << packet->GetSize()
                       << "," << 0.0 << "," << 0.0 << std::endl;
}

void
LeoSimVisualizationHelper::InstallPacketLogging(NodeContainer satellites,
                                                   NodeContainer servers,
                                                   NodeContainer ues)
{
    if (!m_enablePacketLogging || m_packetLoggingInstalled)
    {
        return;
    }

    if (!m_packetFileStream.is_open())
    {
        NS_LOG_ERROR("Packet file not open; call Initialize() after setting packet file.");
        return;
    }

    m_satellites = satellites;
    m_servers = servers;
    m_ues = ues;

    // Install packet tracing on all nodes by connecting to device traces
    for (uint32_t i = 0; i < satellites.GetN(); i++)
    {
        Ptr<Node> node = satellites.Get(i);
        for (uint32_t j = 0; j < node->GetNDevices(); j++)
        {
            Ptr<NetDevice> device = node->GetDevice(j);
            if (!device)
                continue;
            // Only install on PointToPointNetDevices
            if (device->GetInstanceTypeId().GetName() != "ns3::PointToPointNetDevice")
                continue;

            std::ostringstream pathTx, pathRx, pathDrop;
            pathTx << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyTxBegin";
            pathRx << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyRxEnd";
            pathDrop << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyRxDrop";
            
            Config::Connect(pathTx.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyTx, this));
            Config::Connect(pathRx.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyRxBasic, this));
            Config::Connect(pathDrop.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyRxDrop, this));
        }
    }

    // Install tracing on server nodes
    for (uint32_t i = 0; i < servers.GetN(); i++)
    {
        Ptr<Node> node = servers.Get(i);
        for (uint32_t j = 0; j < node->GetNDevices(); j++)
        {
            Ptr<NetDevice> device = node->GetDevice(j);
            if (!device)
                continue;
            // Only install on PointToPointNetDevices
            if (device->GetInstanceTypeId().GetName() != "ns3::PointToPointNetDevice")
                continue;

            std::ostringstream pathTx, pathRx, pathDrop;
            pathTx << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyTxBegin";
            pathRx << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyRxEnd";
            pathDrop << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyRxDrop";
            
            Config::Connect(pathTx.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyTx, this));
            Config::Connect(pathRx.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyRxBasic, this));
            Config::Connect(pathDrop.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyRxDrop, this));
        }
    }

    // Install tracing on UE nodes
    for (uint32_t i = 0; i < ues.GetN(); i++)
    {
        Ptr<Node> node = ues.Get(i);
        for (uint32_t j = 0; j < node->GetNDevices(); j++)
        {
            Ptr<NetDevice> device = node->GetDevice(j);
            if (!device)
                continue;
            // Only install on PointToPointNetDevices
            if (device->GetInstanceTypeId().GetName() != "ns3::PointToPointNetDevice")
                continue;

            std::ostringstream pathTx, pathRx, pathDrop;
            pathTx << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyTxBegin";
            pathRx << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyRxEnd";
            pathDrop << "/NodeList/" << node->GetId() << "/DeviceList/" << j << "/$ns3::PointToPointNetDevice/PhyRxDrop";
            
            Config::Connect(pathTx.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyTx, this));
            Config::Connect(pathRx.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyRxBasic, this));
            Config::Connect(pathDrop.str(),
                          MakeCallback(&LeoSimVisualizationHelper::OnPhyRxDrop, this));
        }
    }

    NS_LOG_INFO("Packet logging installed for " << (satellites.GetN() + servers.GetN() + ues.GetN())
                                                  << " nodes");
    m_packetLoggingInstalled = true;
}

} // namespace ns3
