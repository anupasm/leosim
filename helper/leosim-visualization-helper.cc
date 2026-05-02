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
#include <limits>
#include <ios>
#include <sstream>

namespace ns3
{

namespace
{
std::string
RoleToString(LeoSimNodeRole role)
{
    switch (role)
    {
    case LEOSIM_ROLE_SATELLITE:
        return "SATELLITE";
    case LEOSIM_ROLE_UE:
        return "UE";
    case LEOSIM_ROLE_SERVER:
        return "SERVER";
    case LEOSIM_ROLE_GATEWAY:
        return "GATEWAY";
    default:
        return "UNKNOWN";
    }
}

void
AppendSharingRows(std::ofstream& stream,
                  Ptr<LeoSimOperatorModel> model,
                  const std::vector<LeoSimChannelModel::LinkSnapshot>& links,
                  const std::string& defaultLinkType,
                  LeoSimLinkDirection direction,
                  double simTime,
                  double baseRateMbps)
{
    for (const auto& link : links)
    {
        uint32_t nodeA = link.node1->GetId();
        uint32_t nodeB = link.node2->GetId();
        if (model->IsSameOperator(nodeA, nodeB))
        {
            continue;
        }

        std::string opA = model->GetOperatorId(nodeA);
        std::string opB = model->GetOperatorId(nodeB);
        LeoSimNodeRole roleA = model->GetRole(nodeA);
        LeoSimNodeRole roleB = model->GetRole(nodeB);
        std::string linkType = defaultLinkType;
        LeoSimLinkDirection effectiveDirection = direction;

        if (direction != LEOSIM_DIR_ISL)
        {
            const bool nodeASat = (roleA == LEOSIM_ROLE_SATELLITE);
            const bool nodeBSat = (roleB == LEOSIM_ROLE_SATELLITE);
            if (nodeASat && !nodeBSat)
            {
                linkType = "DOWNLINK";
                effectiveDirection = LEOSIM_DIR_DOWNLINK;
            }
            else if (!nodeASat && nodeBSat)
            {
                linkType = "UPLINK";
                effectiveDirection = LEOSIM_DIR_UPLINK;
            }
        }

        double alphaDl = model->GetAlpha(nodeA, nodeB, LEOSIM_DIR_DOWNLINK);
        double alphaUl = model->GetAlpha(nodeA, nodeB, LEOSIM_DIR_UPLINK);
        double alphaIsl = model->GetAlpha(nodeA, nodeB, LEOSIM_DIR_ISL);
        double effRateMbps = model->GetAlpha(nodeA, nodeB, effectiveDirection) * baseRateMbps;

        stream << std::fixed << std::setprecision(3);
        stream << simTime << ',' << nodeA << ',' << nodeB << ',' << opA << ',' << opB << ','
               << linkType << ',' << alphaDl << ',' << alphaUl << ',' << alphaIsl << ','
               << effRateMbps << std::endl;
    }
}
} // namespace

NS_LOG_COMPONENT_DEFINE("LeoSimVisualizationHelper");

LeoSimVisualizationHelper::LeoSimVisualizationHelper()
    : m_outputFile("leosim_positions.csv"),
      m_linkFile("leosim_links.csv"),
      m_packetFile("leosim_packets.csv"),
      m_beamFile("leosim_beams_multibeam.csv"),
      m_handoverFile("leosim_handovers.csv"),
      m_choFile("leosim_cho.csv"),
    m_operatorFile("leosim_operators.csv"),
    m_sharingFile("leosim_sharing.csv"),
      m_loaderHelper(nullptr),
      m_channelModel(nullptr),
      m_islChannelModel(nullptr),
      m_beamManager(nullptr),
      m_enablePacketLogging(false),
      m_packetLoggingInstalled(false),
      m_enableBeamLogging(false),
      m_beamLoggingInitialized(false),
      m_beamCallbacksInstalled(false),
      m_maxUesPerBeam(20)
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
    if (m_beamFileStream.is_open())
    {
        m_beamFileStream.close();
    }
    if (m_handoverFileStream.is_open())
    {
        m_handoverFileStream.close();
    }
    if (m_choFileStream.is_open())
    {
        m_choFileStream.close();
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
LeoSimVisualizationHelper::EnableBeamLogging(bool enable)
{
    m_enableBeamLogging = enable;
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
LeoSimVisualizationHelper::SetBeamManager(Ptr<LeoSimBeamManager> beamManager)
{
    m_beamManager = beamManager;
    InstallBeamManagerCallbacks();
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

    if (m_enableBeamLogging && !m_beamLoggingInitialized)
    {
        InitBeamLogging();
        m_beamLoggingInitialized = true;
        InstallBeamManagerCallbacks();
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

    FinalizeBeamLogging();
}

void
LeoSimVisualizationHelper::InstallBeamManagerCallbacks()
{
    if (!m_enableBeamLogging)
    {
        return;
    }
    if (m_beamCallbacksInstalled)
    {
        return;
    }
    if (!m_beamManager)
    {
        return;
    }

    // Beam manager fires these callbacks during the simulation.
    m_beamManager->SetHandoverCallback(MakeCallback(&LeoSimVisualizationHelper::OnHandoverEvent,
                                                    this));
    m_beamManager->SetBeamStateCallback(MakeCallback(&LeoSimVisualizationHelper::OnBeamState, this));
    m_beamManager->SetChoConfigCallback(MakeCallback(&LeoSimVisualizationHelper::OnChoConfig, this));

    m_beamCallbacksInstalled = true;
    NS_LOG_INFO("Beam manager callbacks installed for visualization logging");
}

void
LeoSimVisualizationHelper::OnBeamState(uint32_t ueId, LeoSimBeamRecord rec, double topsisScore)
{
    LogBeamState(ueId, rec, topsisScore);
}

void
LeoSimVisualizationHelper::OnHandoverEvent(LeoSimHandoverEvent evt)
{
    LogHandoverEvent(evt);
}

void
LeoSimVisualizationHelper::OnChoConfig(uint32_t ueId,
                                       uint32_t servingSatId,
                                       std::vector<LeoSimTopsisCandidate> candidates)
{
    LogChoConfig(ueId, servingSatId, candidates);
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

        // Log serving beam state snapshot (if beam logging is enabled and a beam manager is set)
        if (m_enableBeamLogging && m_beamManager)
        {
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogServingBeamSnapshot,
                                this);
        }
    }
}

void
LeoSimVisualizationHelper::LogServingBeamSnapshot()
{
    if (!m_enableBeamLogging || !m_beamManager)
    {
        return;
    }

    for (uint32_t i = 0; i < m_ues.GetN(); ++i)
    {
        Ptr<Node> ueNode = m_ues.Get(i);
        if (!ueNode)
        {
            continue;
        }

        uint32_t ueId = ueNode->GetId();
        LeoSimBeamRecord rec = m_beamManager->GetCurrentBeam(ueId);
        if (rec.satelliteNodeId == std::numeric_limits<uint32_t>::max())
        {
            continue;
        }

        LogBeamState(ueId, rec, 0.0);
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

void
LeoSimVisualizationHelper::SetBeamFile(const std::string& filename)
{
    m_beamFile = filename;
}

void
LeoSimVisualizationHelper::SetHandoverFile(const std::string& filename)
{
    m_handoverFile = filename;
}

void
LeoSimVisualizationHelper::SetChoFile(const std::string& filename)
{
    m_choFile = filename;
}

void
LeoSimVisualizationHelper::SetOperatorFile(const std::string& filename)
{
    m_operatorFile = filename;
}

void
LeoSimVisualizationHelper::SetSharingFile(const std::string& filename)
{
    m_sharingFile = filename;
}

void
LeoSimVisualizationHelper::InitOperatorLogging(Ptr<LeoSimOperatorModel> model,
                                               const NodeContainer& allNodes)
{
    if (!model)
    {
        NS_LOG_WARN("Operator model not set; skipping operator CSV initialization");
        return;
    }

    std::ofstream operatorStream(m_operatorFile, std::ios::out | std::ios::trunc);
    if (!operatorStream.is_open())
    {
        NS_LOG_ERROR("Could not open operator file: " << m_operatorFile);
        return;
    }

    operatorStream << "node_id,node_name,role,operator_id" << std::endl;
    for (uint32_t i = 0; i < allNodes.GetN(); ++i)
    {
        Ptr<Node> node = allNodes.Get(i);
        if (!node)
        {
            continue;
        }

        uint32_t nodeId = node->GetId();
        std::string nodeName = Names::FindName(node);
        LeoSimNodeRole role = model->GetRole(nodeId);
        operatorStream << nodeId << ',' << nodeName << ',' << RoleToString(role) << ','
                       << model->GetOperatorId(nodeId) << std::endl;
    }
}

void
LeoSimVisualizationHelper::LogSharingState(Ptr<LeoSimOperatorModel> model,
                                           Ptr<LeoSimChannelModel> channelModel,
                                           Ptr<LeoSimChannelModel> islChannelModel,
                                           double simTime)
{
    if (!model)
    {
        NS_LOG_WARN("Operator model not set; skipping sharing-state logging");
        return;
    }

    std::ifstream existingStream(m_sharingFile);
    const bool needsHeader = !existingStream.good() || existingStream.peek() == std::ifstream::traits_type::eof();
    existingStream.close();

    std::ofstream sharingStream(m_sharingFile, std::ios::out | std::ios::app);
    if (!sharingStream.is_open())
    {
        NS_LOG_ERROR("Could not open sharing file: " << m_sharingFile);
        return;
    }

    if (needsHeader)
    {
        sharingStream << "time,node_a,node_b,op_a,op_b,link_type,alpha_dl,alpha_ul,alpha_isl,eff_rate_mbps"
                      << std::endl;
    }

    if (channelModel)
    {
        auto groundLinks = channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, true);
        AppendSharingRows(sharingStream,
                          model,
                          groundLinks,
                          "DOWNLINK",
                          LEOSIM_DIR_DOWNLINK,
                          simTime,
                          100.0);
    }

    if (islChannelModel)
    {
        auto islLinks = islChannelModel->GetLinksByType(LEOSIM_LINK_ISL, false);
        AppendSharingRows(sharingStream,
                          model,
                          islLinks,
                          "ISL",
                          LEOSIM_DIR_ISL,
                          simTime,
                          10000.0);
    }
}

void
LeoSimVisualizationHelper::SetMaxUesPerBeam(uint32_t maxUes)
{
    m_maxUesPerBeam = maxUes;
}

void
LeoSimVisualizationHelper::InitBeamLogging()
{
    NS_LOG_FUNCTION(this);

    // Open beam state file and write header
    m_beamFileStream.open(m_beamFile, std::ios::out | std::ios::trunc);
    if (m_beamFileStream.is_open())
    {
        m_beamFileStream << "time,ue_id,sat_id,beam_id,cell_id,color_group,rsrp_dbm,sinr_db,"
                         << "intra_ici_dbm,inter_ici_dbm,elevation_deg,tte_sec,beam_load,"
                         << "beam_util,beam_active,topsis_score,state\n";
        m_beamFileStream.flush();
        NS_LOG_INFO("Opened beam state file: " << m_beamFile);
    }
    else
    {
        NS_LOG_WARN("Could not open beam state file: " << m_beamFile);
    }

    // Open handover event file and write header
    m_handoverFileStream.open(m_handoverFile, std::ios::out | std::ios::trunc);
    if (m_handoverFileStream.is_open())
    {
        m_handoverFileStream << "time_ms,ue_id,src_sat,tgt_sat,src_beam,src_cell,tgt_beam,tgt_cell,"
                             << "mode,type,trigger,latency_ms,buff_pkts,drop_pkts,success,"
                             << "route_change,sinr_before,sinr_after\n";
        m_handoverFileStream.flush();
        NS_LOG_INFO("Opened handover event file: " << m_handoverFile);
    }
    else
    {
        NS_LOG_WARN("Could not open handover event file: " << m_handoverFile);
    }

    // Open CHO config file and write header
    m_choFileStream.open(m_choFile, std::ios::out | std::ios::trunc);
    if (m_choFileStream.is_open())
    {
        m_choFileStream << "time,ue_id,serving_sat,candidate_sat,topsis_rank,topsis_score,tte_sec,config_expiry\n";
        m_choFileStream.flush();
        NS_LOG_INFO("Opened CHO config file: " << m_choFile);
    }
    else
    {
        NS_LOG_WARN("Could not open CHO config file: " << m_choFile);
    }
}

void
LeoSimVisualizationHelper::LogBeamState(uint32_t ueId, const LeoSimBeamRecord& rec, double topsisScore)
{
    NS_LOG_FUNCTION(this << ueId << topsisScore);

    if (!m_beamFileStream.is_open())
    {
        return;
    }

    Time now = Simulator::Now();
    std::string state;
    switch (rec.state)
    {
        case LEOSIM_BEAM_CONNECTED:
            state = "CONNECTED";
            break;
        case LEOSIM_BEAM_MEASURING:
            state = "MEASURING";
            break;
        case LEOSIM_BEAM_PREPARING:
            state = "PREPARING";
            break;
        case LEOSIM_BEAM_EVALUATING:
            state = "EVALUATING";
            break;
        case LEOSIM_BEAM_EXECUTING:
            state = "EXECUTING";
            break;
        case LEOSIM_BEAM_SEARCHING:
            state = "SEARCHING";
            break;
        default:
            state = "UNKNOWN";
    }

    // Compute beam utilization
    double beamUtil = (m_maxUesPerBeam > 0) ? (rec.beamLoad / (double)m_maxUesPerBeam) : 0.0;

    m_beamFileStream << std::fixed << std::setprecision(3)
                     << now.GetSeconds() << ","
                     << ueId << ","
                     << rec.satelliteNodeId << ","
                     << rec.beamId << ","
                     << rec.cellId << ","
                     << rec.colorGroup << ","
                     << rec.rsrp << ","
                     << rec.sinr << ","
                     << rec.intraBeamInterference_dBm << ","
                     << rec.interSatInterference_dBm << ","
                     << rec.elevationAngle << ","
                     << rec.remainingServiceTime << ","
                     << rec.beamLoad << ","
                     << beamUtil << ","
                     << (rec.beamActive ? "1" : "0") << ","
                     << topsisScore << ","
                     << state << "\n";
    m_beamFileStream.flush();
}

void
LeoSimVisualizationHelper::LogHandoverEvent(const LeoSimHandoverEvent& evt)
{
    NS_LOG_FUNCTION(this << evt.ueNodeId);

    if (!m_handoverFileStream.is_open())
    {
        return;
    }

    double timeMs = evt.initiatedAt.GetMilliSeconds();
    std::string mode = (evt.mode == LEOSIM_HO_MODE_CHO) ? "CHO" : "BHO";
    
    std::string type;
    switch (evt.type)
    {
        case LEOSIM_HO_INTRA_BEAM:
            type = "INTRA_BEAM";
            break;
        case LEOSIM_HO_INTER_SATELLITE:
            type = "INTER_SAT";
            break;
        case LEOSIM_HO_INTER_ORBIT:
            type = "INTER_ORBIT";
            break;
        case LEOSIM_HO_INTRA_BEAM_ADJACENT:
            type = "INTRA_BEAM_ADJ";
            break;
        case LEOSIM_HO_INTRA_BEAM_DISTANT:
            type = "INTRA_BEAM_DIST";
            break;
        case LEOSIM_HO_BEAM_HOPPING_DARK:
            type = "BEAM_HOP_DARK";
            break;
        default:
            type = "UNKNOWN";
    }

    std::string trigger;
    switch (evt.trigger)
    {
        case LEOSIM_HO_A3:
            trigger = "A3_EVENT";
            break;
        case LEOSIM_HO_A4:
            trigger = "A4_EVENT";
            break;
        case LEOSIM_HO_TIME_BASED:
            trigger = "TIME_BASED";
            break;
        case LEOSIM_HO_LOCATION_BASED:
            trigger = "LOCATION_BASED";
            break;
        case LEOSIM_HO_ELEVATION:
            trigger = "ELEVATION";
            break;
        case LEOSIM_HO_RLF:
            trigger = "RLF";
            break;
        case LEOSIM_HO_LOAD_BALANCE:
            trigger = "LOAD_BALANCE";
            break;
        default:
            trigger = "UNKNOWN";
    }

    // Route change: true if inter-satellite, false if intra-beam
    std::string routeChange = (evt.type == LEOSIM_HO_INTER_SATELLITE || evt.type == LEOSIM_HO_INTER_ORBIT)
                                  ? "true"
                                  : "false";

    m_handoverFileStream << std::fixed << std::setprecision(1)
                         << timeMs << ","
                         << evt.ueNodeId << ","
                         << evt.sourceSatId << ","
                         << evt.targetSatId << ","
                         << evt.sourceBeamId << ","
                         << evt.sourceCellId << ","
                         << evt.targetBeamId << ","
                         << evt.targetCellId << ","
                         << mode << ","
                         << type << ","
                         << trigger << ","
                         << evt.handoverLatencyMs << ","
                         << evt.packetsBuffered << ","
                         << evt.packetsDropped << ","
                         << (evt.success ? "1" : "0") << ","
                         << routeChange << ","
                         << evt.sinrBefore << ","
                         << evt.sinrAfter << "\n";
    m_handoverFileStream.flush();
}

void
LeoSimVisualizationHelper::LogChoConfig(uint32_t ueId,
                                        uint32_t servingSatId,
                                        const std::vector<LeoSimTopsisCandidate>& candidates)
{
    NS_LOG_FUNCTION(this << ueId << servingSatId << candidates.size());

    if (!m_choFileStream.is_open())
    {
        return;
    }

    Time now = Simulator::Now();
    Time configExpiry = now + Seconds(30.0);  // Default 30 second CHO config validity

    for (uint32_t rank = 0; rank < candidates.size(); ++rank)
    {
        const LeoSimTopsisCandidate& candidate = candidates[rank];
        m_choFileStream << std::fixed << std::setprecision(3)
                        << now.GetSeconds() << ","
                        << ueId << ","
                        << servingSatId << ","
                        << candidate.beamRecord.satelliteNodeId << ","
                        << rank << ","
                        << candidate.topsisScore << ","
                        << candidate.beamRecord.remainingServiceTime << ","
                        << configExpiry.GetSeconds() << "\n";
    }
    m_choFileStream.flush();
}

void
LeoSimVisualizationHelper::FinalizeBeamLogging()
{
    NS_LOG_FUNCTION(this);

    if (m_beamFileStream.is_open())
    {
        m_beamFileStream.close();
        NS_LOG_INFO("Closed beam state file");
    }
    if (m_handoverFileStream.is_open())
    {
        m_handoverFileStream.close();
        NS_LOG_INFO("Closed handover event file");
    }
    if (m_choFileStream.is_open())
    {
        m_choFileStream.close();
        NS_LOG_INFO("Closed CHO config file");
    }
}

} // namespace ns3

