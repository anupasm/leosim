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
#include "ns3/leosim-beam-layout-engine.h"
#include "ns3/leosim-task-profiler.h"

#include "ns3/channel.h"
#include "ns3/config.h"
#include "ns3/mobility-model.h"
#include "ns3/names.h"
#include "ns3/net-device.h"
#include "ns3/node-list.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

#include <cmath>
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

std::string
WeatherStateToString(LeoSimWeatherState state)
{
    switch (state)
    {
    case LEOSIM_WX_CLEAR:
        return "CLEAR";
    case LEOSIM_WX_CLOUDY:
        return "CLOUDY";
    case LEOSIM_WX_LIGHT_RAIN:
        return "LIGHT_RAIN";
    case LEOSIM_WX_HEAVY_RAIN:
        return "HEAVY_RAIN";
    default:
        return "UNKNOWN";
    }
}

std::string
LinkStateToString(LeoSimLinkState state)
{
    switch (state)
    {
    case LEOSIM_LINK_UP:
        return "UP";
    case LEOSIM_LINK_DOWN:
        return "DOWN";
    case LEOSIM_LINK_DEGRADED:
        return "DEGRADED";
    default:
        return "UNKNOWN";
    }
}

std::string
BeamStateToString(LeoSimBeamState state)
{
    switch (state)
    {
    case LEOSIM_BEAM_CONNECTED:
        return "CONNECTED";
    case LEOSIM_BEAM_MEASURING:
        return "MEASURING";
    case LEOSIM_BEAM_PREPARING:
        return "PREPARING";
    case LEOSIM_BEAM_EVALUATING:
        return "EVALUATING";
    case LEOSIM_BEAM_EXECUTING:
        return "EXECUTING";
    case LEOSIM_BEAM_SEARCHING:
        return "SEARCHING";
    default:
        return "UNKNOWN";
    }
}

int64_t
CsvId(uint32_t id)
{
    return id == std::numeric_limits<uint32_t>::max() ? -1 : static_cast<int64_t>(id);
}

double
SanitizePathLoss(double pathLoss)
{
    return (!std::isfinite(pathLoss) || pathLoss < 50.0 || pathLoss > 250.0) ? 0.0 : pathLoss;
}

std::string
DegradationReason(const LeoSimChannelQuality& quality, Ptr<LeoSimChannelModel> channelModel)
{
    if (quality.linkState == LEOSIM_LINK_UP)
    {
        return "OPERATIONAL";
    }
    if (quality.linkState == LEOSIM_LINK_DEGRADED)
    {
        return "SNR_DEGRADED";
    }
    if (channelModel && quality.elevationAngle < channelModel->GetMinElevationAngle())
    {
        return "LOW_ELEVATION_ANGLE";
    }
    if (channelModel &&
        quality.distance > (quality.linkType == LEOSIM_LINK_ISL
                                ? channelModel->GetIslMaxDistance()
                                : channelModel->GetMaxLinkDistance()))
    {
        return "DISTANCE_EXCEEDED";
    }
    if (quality.snr <= 0.0)
    {
        return "SNR_FLOOR_EXCEEDED";
    }
    return "UNKNOWN";
}
} // namespace

NS_LOG_COMPONENT_DEFINE("LeoSimVisualizationHelper");

LeoSimVisualizationHelper::LeoSimVisualizationHelper()
    : m_outputFile("leosim_positions.csv"),
      m_linkFile(""),
      m_packetFile("leosim_packets.csv"),
      m_beamFile(""),
      m_handoverFile("leosim_handovers.csv"),
      m_choFile("leosim_cho.csv"),
      m_operatorFile("leosim_operators.csv"),
      m_sharingFile("leosim_sharing.csv"),
      m_coverageFile("leosim_coverage.csv"),
      m_linkQualityFile(""),
      m_unifiedLinkStateFile("leosim_link_state.csv"),
      m_beamAssociationFile("leosim_beam_associations.csv"),
      m_loaderHelper(nullptr),
      m_channelModel(nullptr),
      m_islChannelModel(nullptr),
      m_beamManager(nullptr),
      m_enablePacketLogging(false),
      m_enablePacketGeolocationLogging(false),
      m_enablePositionGeolocationLogging(false),
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
    if (m_coverageFileStream.is_open())
    {
        m_coverageFileStream.close();
    }
    if (m_linkQualityFileStream.is_open())
    {
        m_linkQualityFileStream.close();
    }
    if (m_unifiedLinkStateFileStream.is_open())
    {
        m_unifiedLinkStateFileStream.close();
    }
    if (m_beamAssociationFileStream.is_open())
    {
        m_beamAssociationFileStream.close();
    }
}

void
LeoSimVisualizationHelper::SetOutputFile(std::string outputFile)
{
    m_outputFile = outputFile;
}

void
LeoSimVisualizationHelper::EnablePositionGeolocationLogging(bool enable)
{
    m_enablePositionGeolocationLogging = enable;
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
LeoSimVisualizationHelper::EnablePacketGeolocationLogging(bool enable)
{
    m_enablePacketGeolocationLogging = enable;
}

void
LeoSimVisualizationHelper::EnableBeamLogging(bool enable)
{
    m_enableBeamLogging = enable;
}

void
LeoSimVisualizationHelper::InitializeBeamLogging()
{
    if (!m_enableBeamLogging || m_beamLoggingInitialized)
    {
        return;
    }

    InitBeamLogging();
    m_beamLoggingInitialized = true;
    InstallBeamManagerCallbacks();
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
    // Automatically extract multi-beam model from beam manager if available
    if (beamManager)
    {
        m_multiBeamModel = beamManager->GetMultiBeamModel();
    }
    InstallBeamManagerCallbacks();
}

void
LeoSimVisualizationHelper::SetMultiBeamModel(Ptr<LeoSimMultiBeamModel> multiBeamModel)
{
    m_multiBeamModel = multiBeamModel;
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
    m_posFile << "time,type,id,name,x,y,z";
    if (m_enablePositionGeolocationLogging)
    {
        m_posFile << ",latitude_deg,longitude_deg,altitude_m";
    }
    m_posFile << std::endl;

    if (!m_linkFile.empty())
    {
        m_linkFileStream.open(m_linkFile);
        if (!m_linkFileStream.is_open())
        {
            NS_LOG_ERROR("Could not open link file: " << m_linkFile);
        }
        else
        {
            // Write CSV header for links
            m_linkFileStream << "time,sat_id,ground_id,ground_type,beam_id" << std::endl;
        }
    }

    if (!m_unifiedLinkStateFile.empty())
    {
        m_unifiedLinkStateFileStream.open(m_unifiedLinkStateFile, std::ios::out | std::ios::trunc);
        if (!m_unifiedLinkStateFileStream.is_open())
        {
            NS_LOG_ERROR("Could not open unified link-state file: " << m_unifiedLinkStateFile);
        }
        else
        {
            m_unifiedLinkStateFileStream
                << "time,node1_id,node2_id,link_type,sat_id,ground_id,ground_type,"
                << "channel_state,snr_db,doppler_hz,distance_m,elevation_deg,path_loss_db,"
                << "signal_strength_dbm,quality_last_update_s,rain_attenuation_db,"
                << "cloud_attenuation_db,gaseous_attenuation_db,scintillation_amplitude_db,"
                << "scintillation_sample_db,total_weather_attenuation_db,"
                << "weather_elevation_deg,weather_state,weather_computed_at_s,"
                << "degradation_reason,beam_id,cell_id,color_group,"
                << "beam_state,beam_active,beam_covered,is_prepared_candidate,"
                << "is_serving_access,access_state"
                << std::endl;
        }
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
            m_packetFileStream << "time,event,node_id,device_id,peer_node_id,link_type,size_bytes";
            if (m_enablePacketGeolocationLogging)
            {
                m_packetFileStream << ",x,y,z";
            }
            m_packetFileStream << std::endl;
        }
    }

    InitializeBeamLogging();

    // Initialize coverage and link quality logging
    InitCoverageAndLinkQualityLogging();
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
    if (m_unifiedLinkStateFileStream.is_open())
    {
        m_unifiedLinkStateFileStream.close();
    }
    if (m_beamAssociationFileStream.is_open())
    {
        m_beamAssociationFileStream.close();
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
    NS_LOG_DEBUG("Beam manager callbacks installed for visualization logging");
}

void
LeoSimVisualizationHelper::OnBeamState(uint32_t nodeId, LeoSimBeamRecord rec, double topsisScore)
{
    // Determine node type (UE or SERVER)
    std::string nodeType = "UNKNOWN";
    for (uint32_t i = 0; i < m_ues.GetN(); ++i)
    {
        if (m_ues.Get(i)->GetId() == nodeId)
        {
            nodeType = "UE";
            break;
        }
    }
    
    if (nodeType == "UNKNOWN")
    {
        for (uint32_t i = 0; i < m_servers.GetN(); ++i)
        {
            if (m_servers.Get(i)->GetId() == nodeId)
            {
                nodeType = "SERVER";
                break;
            }
        }
    }
    
    LogBeamState(nodeId, nodeType, rec, topsisScore);
}

void
LeoSimVisualizationHelper::SetGroundNodeContainers(const NodeContainer& servers,
                                                    const NodeContainer& ues)
{
    m_servers = servers;
    m_ues = ues;
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
    // Write Cartesian ECEF coordinates, with optional WGS84 geolocation.
    m_posFile << std::fixed << std::setprecision(3);
    m_posFile << time << "," << nodeType << "," << nodeId << "," << resolvedName << "," << pos.x << ","
              << pos.y << "," << pos.z;
    if (m_enablePositionGeolocationLogging)
    {
        const Vector geodetic = LeoSimLoader::CartesianToGeodetic(pos);
        m_posFile << "," << geodetic.x << "," << geodetic.y << "," << geodetic.z;
    }
    m_posFile << std::endl;
}

void
LeoSimVisualizationHelper::LogIslConnections()
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.log_isl_connections");

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
                        << link.node2->GetId() << ",ISL,-1" << std::endl;
    }
}

void
LeoSimVisualizationHelper::LogGroundConnections()
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.log_ground_connections");

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
            int32_t beamId = -1;
            if (m_beamManager)
            {
                LeoSimBeamRecord rec = m_beamManager->GetCurrentBeam(ground->GetId());
                if (rec.satelliteNodeId == sat->GetId())
                {
                    beamId = static_cast<int32_t>(rec.beamId);
                }
            }

            if (beamId < 0 && m_multiBeamModel)
            {
                const auto& satBeams = m_multiBeamModel->GetBeamsForSatellite(sat->GetId());
                if (!satBeams.empty())
                {
                    Ptr<MobilityModel> groundMobility = ground->GetObject<MobilityModel>();
                    if (groundMobility)
                    {
                        const Vector groundPos = groundMobility->GetPosition();
                        const double r = std::sqrt(groundPos.x * groundPos.x + groundPos.y * groundPos.y +
                                                   groundPos.z * groundPos.z);
                        if (r > 0.0)
                        {
                            constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
                            const double groundLatDeg = std::asin(groundPos.z / r) * kRadToDeg;
                            const double groundLonDeg = std::atan2(groundPos.y, groundPos.x) * kRadToDeg;
                            const int32_t inferredBeamId = LeoSimBeamLayoutEngine::FindBeamForPosition(
                                satBeams,
                                groundLatDeg,
                                groundLonDeg);
                            if (inferredBeamId >= 0)
                            {
                                beamId = inferredBeamId;
                            }
                        }
                    }
                }
            }

            // Keep SAT-ground link logs consistent with beam coverage geometry:
            // if no beam footprint contains this ground node, skip logging this link.
            if (beamId < 0)
            {
                NS_LOG_DEBUG("Skipping SAT-ground link without valid beam mapping: sat="
                             << sat->GetId() << " ground=" << ground->GetId());
                continue;
            }

            m_linkFileStream << std::fixed << std::setprecision(3);
            m_linkFileStream << time << "," << sat->GetId() << ","
                            << ground->GetId() << "," << groundType << "," << beamId << std::endl;
        }
    }
}

void
LeoSimVisualizationHelper::LogUnifiedLinkState()
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.log_unified_link_state");

    if (!m_unifiedLinkStateFileStream.is_open())
    {
        return;
    }

    const double time = Simulator::Now().GetSeconds();

    auto writeLink = [&](Ptr<LeoSimChannelModel> channelModel,
                         const LeoSimChannelModel::LinkSnapshot& link) {
        if (!channelModel || !link.node1 || !link.node2)
        {
            return;
        }

        Ptr<Node> node1 = link.node1;
        Ptr<Node> node2 = link.node2;
        LeoSimChannelQuality quality = channelModel->GetChannelQuality(node1, node2);

        const bool node1IsSat = IsSatelliteNode(node1);
        const bool node2IsSat = IsSatelliteNode(node2);
        Ptr<Node> sat = nullptr;
        Ptr<Node> ground = nullptr;
        std::string linkType = (link.linkType == LEOSIM_LINK_ISL) ? "ISL" : "SAT_GROUND";
        std::string groundType;

        if (link.linkType == LEOSIM_LINK_SATELLITE_TO_GROUND)
        {
            if (node1IsSat && !node2IsSat)
            {
                sat = node1;
                ground = node2;
                linkType = "DOWNLINK";
            }
            else if (!node1IsSat && node2IsSat)
            {
                sat = node2;
                ground = node1;
                linkType = "UPLINK";
            }

            if (ground)
            {
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
            }
        }

        int64_t satId = sat ? static_cast<int64_t>(sat->GetId()) : -1;
        int64_t groundId = ground ? static_cast<int64_t>(ground->GetId()) : -1;
        int64_t beamId = -1;
        int64_t cellId = -1;
        int32_t colorGroup = -1;
        std::string beamState = "";
        bool beamActive = false;
        bool beamCovered = false;
        bool isPreparedCandidate = false;
        bool isServingAccess = false;
        std::string accessState;

        if (sat && ground)
        {
            LeoSimBeamRecord rec;
            bool hasServingRecordForSat = false;
            if (m_beamManager)
            {
                rec = m_beamManager->GetCurrentBeam(ground->GetId());
                hasServingRecordForSat = (rec.satelliteNodeId == sat->GetId());
                isServingAccess = m_beamManager->IsServingAccessLink(ground->GetId(), sat->GetId());
                for (const auto& candidate : m_beamManager->GetPreparedCandidateBeams(ground->GetId()))
                {
                    if (candidate.satelliteNodeId == sat->GetId())
                    {
                        isPreparedCandidate = true;
                        break;
                    }
                }
                if (hasServingRecordForSat)
                {
                    beamId = CsvId(rec.beamId);
                    cellId = CsvId(rec.cellId);
                    colorGroup = rec.colorGroup;
                    beamState = BeamStateToString(rec.state);
                    beamActive = rec.beamActive;
                    beamCovered = beamId >= 0;
                }
            }

            if (beamId < 0 && m_multiBeamModel)
            {
                const auto& satBeams = m_multiBeamModel->GetBeamsForSatellite(sat->GetId());
                Ptr<MobilityModel> groundMobility = ground->GetObject<MobilityModel>();
                if (!satBeams.empty() && groundMobility)
                {
                    const Vector groundPos = groundMobility->GetPosition();
                    const double r = std::sqrt(groundPos.x * groundPos.x +
                                               groundPos.y * groundPos.y +
                                               groundPos.z * groundPos.z);
                    if (r > 0.0)
                    {
                        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
                        const double groundLatDeg = std::asin(groundPos.z / r) * kRadToDeg;
                        const double groundLonDeg = std::atan2(groundPos.y, groundPos.x) * kRadToDeg;
                        const int32_t inferredBeamId =
                            LeoSimBeamLayoutEngine::FindBeamForPosition(satBeams,
                                                                         groundLatDeg,
                                                                         groundLonDeg);
                        if (inferredBeamId >= 0)
                        {
                            beamId = inferredBeamId;
                            for (const auto& beam : satBeams)
                            {
                                if (beam.beamId == static_cast<uint32_t>(inferredBeamId))
                                {
                                    cellId = CsvId(beam.cellId);
                                    colorGroup = beam.colorGroup;
                                    beamActive = beam.activeInCurrentSlot;
                                    beamCovered = true;
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            if (beamState.empty())
            {
                beamState = isServingAccess ? "CONNECTED" : "NOT_SERVING";
            }

            const bool channelUsable = quality.linkState == LEOSIM_LINK_UP ||
                                       quality.linkState == LEOSIM_LINK_DEGRADED;
            if (!channelUsable)
            {
                accessState = "CHANNEL_DOWN";
            }
            else if (!beamCovered)
            {
                accessState = "NO_BEAM_COVERAGE";
            }
            else if (!beamActive)
            {
                accessState = "BEAM_INACTIVE";
            }
            else if (isServingAccess)
            {
                accessState = "SERVING";
            }
            else if (isPreparedCandidate)
            {
                accessState = "PREPARED_CANDIDATE";
            }
            else
            {
                accessState = "BEAM_COVERED";
            }
        }
        else if (link.linkType == LEOSIM_LINK_ISL)
        {
            accessState = LinkStateToString(quality.linkState);
        }

        m_unifiedLinkStateFileStream << std::fixed << std::setprecision(3)
                                     << time << ","
                                     << node1->GetId() << ","
                                     << node2->GetId() << ","
                                     << linkType << ","
                                     << satId << ","
                                     << groundId << ","
                                     << groundType << ","
                                     << LinkStateToString(quality.linkState) << ","
                                     << quality.snr << ","
                                     << quality.dopplerHz << ","
                                     << quality.distance << ","
                                     << quality.elevationAngle << ","
                                     << SanitizePathLoss(quality.pathLoss) << ","
                                     << quality.signalStrength << ","
                                     << quality.lastUpdate.GetSeconds() << ","
                                     << quality.weatherAtten.rainAttenuation_dB << ","
                                     << quality.weatherAtten.cloudAttenuation_dB << ","
                                     << quality.weatherAtten.gaseousAttenuation_dB << ","
                                     << quality.weatherAtten.scintillationAmplitude_dB << ","
                                     << quality.weatherAtten.scintillationSample_dB << ","
                                     << quality.weatherAtten.totalAttenuation_dB << ","
                                     << quality.weatherAtten.elevationAngle_deg << ","
                                     << WeatherStateToString(quality.weatherAtten.groundState) << ","
                                     << quality.weatherAtten.computedAt.GetSeconds() << ","
                                     << DegradationReason(quality, channelModel) << ","
                                     << beamId << ","
                                     << cellId << ","
                                     << colorGroup << ","
                                     << beamState << ","
                                     << (beamActive ? "1" : "0") << ","
                                     << (beamCovered ? "1" : "0") << ","
                                     << (isPreparedCandidate ? "1" : "0") << ","
                                     << (isServingAccess ? "1" : "0") << ","
                                     << accessState
                                     << std::endl;
    };

    if (m_channelModel)
    {
        auto groundLinks = m_channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, true);
        for (const auto& link : groundLinks)
        {
            writeLink(m_channelModel, link);
        }
    }

    if (m_islChannelModel)
    {
        auto islLinks = m_islChannelModel->GetLinksByType(LEOSIM_LINK_ISL, true);
        for (const auto& link : islLinks)
        {
            writeLink(m_islChannelModel, link);
        }
    }

    m_unifiedLinkStateFileStream.flush();
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

        if (m_unifiedLinkStateFileStream.is_open())
        {
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogUnifiedLinkState,
                                this);
        }

        // Log satellite ground coverage
        if (m_channelModel && m_coverageFileStream.is_open())
        {
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogSatelliteGroundCoverage,
                                this);
        }

        // Log out-of-threshold links
        if (m_channelModel && m_linkQualityFileStream.is_open())
        {
            Simulator::Schedule(Seconds(t),
                                &LeoSimVisualizationHelper::LogOutOfThresholdLinks,
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
        devEnd = context.size();
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
LeoSimVisualizationHelper::AppendPacketGeolocation(int nodeId)
{
    if (!m_enablePacketGeolocationLogging)
    {
        return;
    }

    Ptr<Node> node = NodeList::GetNode(nodeId);
    Ptr<MobilityModel> mobility = node ? node->GetObject<MobilityModel>() : nullptr;
    if (mobility)
    {
        const Vector position = mobility->GetPosition();
        m_packetFileStream << ',' << position.x << ',' << position.y << ',' << position.z;
    }
    else
    {
        m_packetFileStream << ",nan,nan,nan";
    }
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
                       << "," << peerNodeId << "," << linkType << "," << packet->GetSize();
    AppendPacketGeolocation(nodeId);
    m_packetFileStream << std::endl;
}

void
LeoSimVisualizationHelper::OnPhyRx(std::string context,
                                   Ptr<const Packet> packet,
                                   double snrDb,
                                   double dopplerHz)
{
    (void)snrDb;
    (void)dopplerHz;
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
                       << "," << peerNodeId << "," << linkType << "," << packet->GetSize();
    AppendPacketGeolocation(nodeId);
    m_packetFileStream << std::endl;
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
                       << "," << peerNodeId << "," << linkType << "," << packet->GetSize();
    AppendPacketGeolocation(nodeId);
    m_packetFileStream << std::endl;
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

    uint64_t connectedTraceCount = 0;
    auto connectDevice = [this, &connectedTraceCount](Ptr<Node> node, uint32_t deviceIndex) {
        Ptr<NetDevice> device = node->GetDevice(deviceIndex);
        if (!device || device->GetInstanceTypeId().GetName() != "ns3::PointToPointNetDevice")
        {
            return;
        }

        // Attach directly to the known device. Config::Connect performs a
        // global namespace lookup for every path and becomes effectively
        // quadratic when tens of thousands of ISL devices are present.
        std::ostringstream context;
        context << "/NodeList/" << node->GetId() << "/DeviceList/" << deviceIndex;
        connectedTraceCount += device->TraceConnect(
            "PhyTxBegin", context.str(), MakeCallback(&LeoSimVisualizationHelper::OnPhyTx, this));
        connectedTraceCount += device->TraceConnect(
            "PhyRxEnd", context.str(), MakeCallback(&LeoSimVisualizationHelper::OnPhyRxBasic, this));
        connectedTraceCount += device->TraceConnect(
            "PhyRxDrop", context.str(), MakeCallback(&LeoSimVisualizationHelper::OnPhyRxDrop, this));
    };

    // Install packet tracing on all nodes by connecting to device traces
    for (uint32_t i = 0; i < satellites.GetN(); i++)
    {
        Ptr<Node> node = satellites.Get(i);
        for (uint32_t j = 0; j < node->GetNDevices(); j++)
        {
            connectDevice(node, j);
        }
    }

    // Install tracing on server nodes
    for (uint32_t i = 0; i < servers.GetN(); i++)
    {
        Ptr<Node> node = servers.Get(i);
        for (uint32_t j = 0; j < node->GetNDevices(); j++)
        {
            connectDevice(node, j);
        }
    }

    // Install tracing on UE nodes
    for (uint32_t i = 0; i < ues.GetN(); i++)
    {
        Ptr<Node> node = ues.Get(i);
        for (uint32_t j = 0; j < node->GetNDevices(); j++)
        {
            connectDevice(node, j);
        }
    }

    NS_LOG_DEBUG("Packet logging installed for " << (satellites.GetN() + servers.GetN() + ues.GetN())
                                                  << " nodes using " << connectedTraceCount
                                                  << " trace connections");
    if (connectedTraceCount == 0)
    {
        NS_LOG_ERROR("Packet logging enabled but no compatible device traces were connected");
    }
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
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.log_operator_sharing_state");

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

    if (!m_beamFile.empty())
    {
        // Open legacy beam state file and write header, when explicitly configured.
        m_beamFileStream.open(m_beamFile, std::ios::out | std::ios::trunc);
        if (m_beamFileStream.is_open())
        {
            m_beamFileStream << "time,ground_id,ground_type,sat_id,beam_id,cell_id,color_group,rsrp_dbm,sinr_db,"
                             << "intra_ici_dbm,inter_ici_dbm,elevation_deg,tte_sec,beam_load,"
                             << "beam_util,beam_active,topsis_score,state\n";
            m_beamFileStream.flush();
            NS_LOG_DEBUG("Opened beam state file: " << m_beamFile);
        }
        else
        {
            NS_LOG_WARN("Could not open beam state file: " << m_beamFile);
        }
    }

    // Open handover event file and write header when explicitly configured.
    if (!m_handoverFile.empty())
    {
        m_handoverFileStream.open(m_handoverFile, std::ios::out | std::ios::trunc);
        if (m_handoverFileStream.is_open())
        {
            m_handoverFileStream << "time_ms,ue_id,src_sat,tgt_sat,src_beam,src_cell,tgt_beam,tgt_cell,"
                                 << "mode,type,trigger,latency_ms,buff_pkts,drop_pkts,success,"
                                 << "failure_reason,route_change,sinr_before,sinr_after\n";
            m_handoverFileStream.flush();
            NS_LOG_DEBUG("Opened handover event file: " << m_handoverFile);
        }
        else
        {
            NS_LOG_WARN("Could not open handover event file: " << m_handoverFile);
        }
    }

    // Open CHO config file and write header when explicitly configured.
    if (!m_choFile.empty())
    {
        m_choFileStream.open(m_choFile, std::ios::out | std::ios::trunc);
        if (m_choFileStream.is_open())
        {
            m_choFileStream
                << "time,ground_id,serving_sat,candidate_count,candidate_sat,topsis_rank,"
                   "topsis_score,rsrp_dbm,sinr_db,elevation_deg,tte_sec,config_expiry\n";
            m_choFileStream.flush();
            NS_LOG_DEBUG("Opened CHO config file: " << m_choFile);
        }
        else
        {
            NS_LOG_WARN("Could not open CHO config file: " << m_choFile);
        }
    }

    if (!m_beamAssociationFile.empty())
    {
        m_beamAssociationFileStream.open(m_beamAssociationFile, std::ios::out | std::ios::trunc);
        if (m_beamAssociationFileStream.is_open())
        {
            m_beamAssociationFileStream
                << "time,ground_id,ground_type,sat_id,beam_id,cell_id,color_group,state,"
                << "beam_active,is_serving,rsrp_dbm,sinr_db,elevation_deg,tte_sec,beam_load,"
                << "beam_util,topsis_score"
                << std::endl;
            m_beamAssociationFileStream.flush();
            NS_LOG_DEBUG("Opened beam association file: " << m_beamAssociationFile);
        }
        else
        {
            NS_LOG_WARN("Could not open beam association file: " << m_beamAssociationFile);
        }
    }
}

void
LeoSimVisualizationHelper::LogBeamState(uint32_t groundId,
                                        const std::string& groundType,
                                        const LeoSimBeamRecord& rec,
                                        double topsisScore)
{
    NS_LOG_FUNCTION(this << groundId << groundType << topsisScore);

    Time now = Simulator::Now();
    std::string state = BeamStateToString(rec.state);

    // Compute beam utilization
    double beamUtil = (m_maxUesPerBeam > 0) ? (rec.beamLoad / (double)m_maxUesPerBeam) : 0.0;
    const bool isServing =
        m_beamManager &&
        rec.satelliteNodeId != std::numeric_limits<uint32_t>::max() &&
        rec.beamId != std::numeric_limits<uint32_t>::max() &&
        m_beamManager->IsServingAccessLink(groundId, rec.satelliteNodeId);

    if (m_beamFileStream.is_open())
    {
        m_beamFileStream << std::fixed << std::setprecision(3)
                         << now.GetSeconds() << ","
                         << groundId << ","
                         << groundType << ","
                         << CsvId(rec.satelliteNodeId) << ","
                         << CsvId(rec.beamId) << ","
                         << CsvId(rec.cellId) << ","
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

    if (m_beamAssociationFileStream.is_open())
    {
        m_beamAssociationFileStream << std::fixed << std::setprecision(3)
                                    << now.GetSeconds() << ","
                                    << groundId << ","
                                    << groundType << ","
                                    << CsvId(rec.satelliteNodeId) << ","
                                    << CsvId(rec.beamId) << ","
                                    << CsvId(rec.cellId) << ","
                                    << rec.colorGroup << ","
                                    << state << ","
                                    << (rec.beamActive ? "1" : "0") << ","
                                    << (isServing ? "1" : "0") << ","
                                    << rec.rsrp << ","
                                    << rec.sinr << ","
                                    << rec.elevationAngle << ","
                                    << rec.remainingServiceTime << ","
                                    << rec.beamLoad << ","
                                    << beamUtil << ","
                                    << topsisScore << "\n";
        m_beamAssociationFileStream.flush();
    }
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
    std::cout << "Logging handover event: time=" << timeMs << "ms, ue=" << evt.ueNodeId
              << ", srcSat=" << evt.sourceSatId << ", tgtSat=" << evt.targetSatId
              << ", srcBeam=" << evt.sourceBeamId << ", tgtBeam=" << evt.targetBeamId
              << ", mode=" << mode << ", type=" << type << ", trigger=" << trigger
              << ", latency=" << evt.handoverLatencyMs << "ms, success=" << evt.success
              << std::endl;
    m_handoverFileStream << std::fixed << std::setprecision(1)
                         << timeMs << ","
                         << evt.ueNodeId << ","
                         << CsvId(evt.sourceSatId) << ","
                         << CsvId(evt.targetSatId) << ","
                         << CsvId(evt.sourceBeamId) << ","
                         << CsvId(evt.sourceCellId) << ","
                         << CsvId(evt.targetBeamId) << ","
                         << CsvId(evt.targetCellId) << ","
                         << mode << ","
                         << type << ","
                         << trigger << ","
                         << evt.handoverLatencyMs << ","
                         << evt.packetsBuffered << ","
                         << evt.packetsDropped << ","
                         << (evt.success ? "1" : "0") << ","
                         << evt.failureReason << ","
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
    for (uint32_t rank = 0; rank < candidates.size(); ++rank)
    {
        const LeoSimTopsisCandidate& candidate = candidates[rank];
        const Time configExpiry =
            now + Seconds(std::max(0.0, candidate.beamRecord.remainingServiceTime));
        m_choFileStream << std::fixed << std::setprecision(3)
                        << now.GetSeconds() << ","
                        << ueId << ","
                        << servingSatId << ","
                        << candidates.size() << ","
                        << candidate.beamRecord.satelliteNodeId << ","
                        << rank << ","
                        << candidate.topsisScore << ","
                        << candidate.beamRecord.rsrp << ","
                        << candidate.beamRecord.sinr << ","
                        << candidate.beamRecord.elevationAngle << ","
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
        NS_LOG_DEBUG("Closed beam state file");
    }
    if (m_handoverFileStream.is_open())
    {
        m_handoverFileStream.close();
        NS_LOG_DEBUG("Closed handover event file");
    }
    if (m_choFileStream.is_open())
    {
        m_choFileStream.close();
        NS_LOG_DEBUG("Closed CHO config file");
    }
    if (m_beamAssociationFileStream.is_open())
    {
        m_beamAssociationFileStream.close();
        NS_LOG_DEBUG("Closed beam association file");
    }
}

void
LeoSimVisualizationHelper::SetWeatherFile(const std::string& filename)
{
    m_weatherFile = filename;
}

void
LeoSimVisualizationHelper::SetAttenuationFile(const std::string& filename)
{
    m_attenuationFile = filename;
}

void
LeoSimVisualizationHelper::InitWeatherLogging()
{
    if (!m_weatherFile.empty())
    {
        std::ofstream wf(m_weatherFile, std::ios::out | std::ios::trunc);
        if (wf.is_open())
        {
            wf << "time,node_id,state,rain_rate_mmh,cloud_lwc,temp_c,pressure_hpa,humidity"
               << std::endl;
        }
        else
        {
            NS_LOG_ERROR("Could not open weather file: " << m_weatherFile);
        }
    }

    if (!m_attenuationFile.empty())
    {
        std::ofstream af(m_attenuationFile, std::ios::out | std::ios::trunc);
        if (af.is_open())
        {
            af << "time,ue_id,sat_id,elevation_deg,rain_dB,cloud_dB,gas_dB,scint_dB,total_dB,link_state"
               << std::endl;
        }
        else
        {
            NS_LOG_ERROR("Could not open attenuation file: " << m_attenuationFile);
        }
    }
}

void
LeoSimVisualizationHelper::LogWeatherState(Ptr<LeoSimWeatherModel> model,
                                           const NodeContainer& groundNodes,
                                           double simTime)
{
    if (!model)
    {
        NS_LOG_WARN("Weather model not set; skipping weather-state logging");
        return;
    }
    if (m_weatherFile.empty())
    {
        return;
    }

    std::ofstream wf(m_weatherFile, std::ios::out | std::ios::app);
    if (!wf.is_open())
    {
        NS_LOG_ERROR("Could not open weather file for appending: " << m_weatherFile);
        return;
    }

    wf << std::fixed << std::setprecision(4);
    for (uint32_t i = 0; i < groundNodes.GetN(); ++i)
    {
        Ptr<Node> node = groundNodes.Get(i);
        if (!node)
        {
            continue;
        }
        uint32_t nodeId = node->GetId();
        LeoSimWeatherState state = model->GetWeatherState(nodeId);
        LeoSimWeatherParams p = model->GetWeatherParams(nodeId);

        wf << simTime << ',' << nodeId << ',' << WeatherStateToString(state) << ','
           << p.rainRateMmh << ',' << p.cloudLiquidWater << ',' << p.temperatureCelsius << ','
           << p.pressureHPa << ',' << p.humidity << std::endl;
    }
}

void
LeoSimVisualizationHelper::LogAttenuationState(Ptr<LeoSimChannelModel> channelModel,
                                               const NodeContainer& ueNodes,
                                               const NodeContainer& satNodes,
                                               double simTime)
{
    (void)(satNodes); // reserved for future use

    if (!channelModel)
    {
        NS_LOG_WARN("Channel model not set; skipping attenuation-state logging");
        return;
    }
    if (m_attenuationFile.empty())
    {
        return;
    }

    std::ofstream af(m_attenuationFile, std::ios::out | std::ios::app);
    if (!af.is_open())
    {
        NS_LOG_ERROR("Could not open attenuation file for appending: " << m_attenuationFile);
        return;
    }

    af << std::fixed << std::setprecision(4);
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> ue = ueNodes.Get(i);
        if (!ue)
        {
            continue;
        }
        uint32_t ueId = ue->GetId();
        auto links = channelModel->GetLinksForNode(ueId);
        for (const auto& lq : links)
        {
            // Skip links that are not UP or DEGRADED
            if (lq.linkState != LEOSIM_LINK_UP && lq.linkState != LEOSIM_LINK_DEGRADED)
            {
                continue;
            }
            const LeoSimAttenuationResult& a = lq.weatherAtten;
            af << simTime << ',' << ueId << ',' << lq.peerNodeId << ','
               << lq.elevationAngle << ',' << a.rainAttenuation_dB << ','
               << a.cloudAttenuation_dB << ',' << a.gaseousAttenuation_dB << ','
               << a.scintillationSample_dB << ',' << a.totalAttenuation_dB << ','
               << LinkStateToString(lq.linkState) << std::endl;
        }
    }
}

void
LeoSimVisualizationHelper::SetCoverageFile(const std::string& filename)
{
    m_coverageFile = filename;
}

void
LeoSimVisualizationHelper::SetLinkQualityFile(const std::string& filename)
{
    m_linkQualityFile = filename;
}

void
LeoSimVisualizationHelper::SetUnifiedLinkStateFile(const std::string& filename)
{
    m_unifiedLinkStateFile = filename;
}

void
LeoSimVisualizationHelper::SetBeamAssociationFile(const std::string& filename)
{
    m_beamAssociationFile = filename;
}

void
LeoSimVisualizationHelper::InitCoverageAndLinkQualityLogging()
{
    // Initialize coverage file
    if (!m_coverageFile.empty())
    {
        m_coverageFileStream.open(m_coverageFile, std::ios::out | std::ios::trunc);
        if (m_coverageFileStream.is_open())
        {
            // CSV header: Coverage footprint data for satellite beams
            // time: simulation time (seconds)
            // satellite_id: satellite node ID
            // beam_id: unique beam identifier
            // cell_id: logical cell ID for the beam
            // color_group: frequency reuse color index
            // center_lat: beam center latitude (degrees)
            // center_lon: beam center longitude (degrees)
            // radius_km: beam footprint radius (kilometers)
            // active_in_slot: whether beam is active in current hopping slot
            m_coverageFileStream << "time,satellite_id,beam_id,cell_id,color_group,center_lat,center_lon,radius_km,active_in_slot"
                                 << std::endl;
            NS_LOG_DEBUG("Opened ground coverage file (beam footprints): " << m_coverageFile);
        }
        else
        {
            NS_LOG_ERROR("Could not open coverage file: " << m_coverageFile);
        }
    }

    // Initialize link quality file
    if (!m_linkQualityFile.empty())
    {
        m_linkQualityFileStream.open(m_linkQualityFile, std::ios::out | std::ios::trunc);
        if (m_linkQualityFileStream.is_open())
        {
            // CSV header: time, node1_id, node2_id, link_type, snr_db, distance_m, elevation_deg, 
            //             path_loss_db, signal_strength_dbm, link_state, out_of_threshold_reason
            m_linkQualityFileStream << "time,node1_id,node2_id,link_type,snr_db,distance_m,elevation_deg,"
                                    << "path_loss_db,signal_strength_dbm,link_state,degradation_reason"
                                    << std::endl;
            NS_LOG_DEBUG("Opened link quality file: " << m_linkQualityFile);
        }
        else
        {
            NS_LOG_ERROR("Could not open link quality file: " << m_linkQualityFile);
        }
    }
}

void
LeoSimVisualizationHelper::LogSatelliteGroundCoverage()
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.log_satellite_ground_coverage");

    if (!m_coverageFileStream.is_open())
    {
        return;
    }

    // Log only real beam footprints from the multi-beam model.
    if (m_multiBeamModel && !m_multiBeamModel->GetAllBeams().empty())
    {
        double time = Simulator::Now().GetSeconds();
        const auto& allBeams = m_multiBeamModel->GetAllBeams();

        // Log each beam's footprint
        for (const auto& satBeamPair : allBeams)
        {
            uint32_t satId = satBeamPair.first;
            const auto& beams = satBeamPair.second;

            for (const auto& beam : beams)
            {
                m_coverageFileStream << std::fixed << std::setprecision(6);
                m_coverageFileStream << time << ","
                                    << satId << ","
                                    << beam.beamId << ","
                                    << beam.cellId << ","
                                    << beam.colorGroup << ","
                                    << beam.centerLat << ","
                                    << beam.centerLon << ","
                                    << std::setprecision(3)
                                    << beam.radiusKm << ","
                                    << (beam.activeInCurrentSlot ? "1" : "0") << std::endl;
            }
        }
    }
    else
    {
        NS_LOG_DEBUG("Coverage logging skipped: multi-beam model is not set or has no beams.");
    }
}

void
LeoSimVisualizationHelper::LogOutOfThresholdLinks()
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.log_out_of_threshold_links");

    if (!m_channelModel || !m_linkQualityFileStream.is_open())
    {
        return;
    }

    double time = Simulator::Now().GetSeconds();

    // Log all ground links that are out of threshold (DOWN or DEGRADED)
    auto groundLinks = m_channelModel->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, true);

    for (const auto& link : groundLinks)
    {
        Ptr<Node> node1 = link.node1;
        Ptr<Node> node2 = link.node2;

        // Get detailed channel quality
        LeoSimChannelQuality quality = m_channelModel->GetChannelQuality(node1, node2);

        // Determine link type
        std::string linkType = "UNKNOWN";
        bool node1IsSat = false;
        for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
        {
            if (m_satellites.Get(i) == node1)
            {
                node1IsSat = true;
                break;
            }
        }
        bool node2IsSat = false;
        for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
        {
            if (m_satellites.Get(i) == node2)
            {
                node2IsSat = true;
                break;
            }
        }

        if (node1IsSat && !node2IsSat)
        {
            linkType = "DOWNLINK";
        }
        else if (!node1IsSat && node2IsSat)
        {
            linkType = "UPLINK";
        }
        else if (node1IsSat && node2IsSat)
        {
            linkType = "ISL";
        }

        // Determine reason for degradation or status
        std::string degradationReason;
        if (quality.linkState == LEOSIM_LINK_UP)
        {
            degradationReason = "OPERATIONAL";
        }
        else if (quality.linkState == LEOSIM_LINK_DOWN)
        {
            if (quality.snr <= 0.0)
            {
                degradationReason = "SNR_FLOOR_EXCEEDED";
            }
            else if (quality.elevationAngle < m_channelModel->GetMinElevationAngle())
            {
                degradationReason = "LOW_ELEVATION_ANGLE";
            }
            else if (quality.distance > m_channelModel->GetMaxLinkDistance())
            {
                degradationReason = "DISTANCE_EXCEEDED";
            }
            else
            {
                degradationReason = "UNKNOWN";
            }
        }
        else if (quality.linkState == LEOSIM_LINK_DEGRADED)
        {
            degradationReason = "SNR_DEGRADED";
        }

        m_linkQualityFileStream << std::fixed << std::setprecision(3);
        
        // Sanitize pathLoss: clamp to reasonable range [50, 250] dB
        // This prevents garbage values from uninitialized variables in early returns
        double pathLossToLog = quality.pathLoss;
        if (!std::isfinite(pathLossToLog) || pathLossToLog < 50.0 || pathLossToLog > 250.0)
        {
            pathLossToLog = 0.0;  // Log as 0.0 if out of reasonable range
        }
        
        m_linkQualityFileStream << time << ","
                               << node1->GetId() << ","
                               << node2->GetId() << ","
                               << linkType << ","
                               << quality.snr << ","
                               << quality.distance << ","
                               << quality.elevationAngle << ","
                               << pathLossToLog << ","
                               << quality.signalStrength << ","
                               << LinkStateToString(quality.linkState) << ","
                               << degradationReason << std::endl;
    }
}

} // namespace ns3
