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

#include "leosim-channel-model.h"
#include "leosim-mobility-model.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/boolean.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimChannelModel");
NS_OBJECT_ENSURE_REGISTERED(LeoSimChannelModel);

// Physical constants
static const double SPEED_OF_LIGHT = 299792458.0; // m/s
static const double BOLTZMANN_CONSTANT = 1.380649e-23; // J/K
static const double EARTH_RADIUS = 6371000.0; // meters

TypeId
LeoSimChannelModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LeoSimChannelModel")
            .SetParent<Object>()
            .SetGroupName("LeoSim")
            .AddConstructor<LeoSimChannelModel>()
            .AddAttribute("MinElevationAngle",
                          "Minimum elevation angle for ground-to-satellite links (degrees)",
                          DoubleValue(10.0),
                          MakeDoubleAccessor(&LeoSimChannelModel::m_minElevationAngle),
                          MakeDoubleChecker<double>(0.0, 90.0))
            .AddAttribute("MaxLinkDistance",
                          "Maximum link distance (meters)",
                          DoubleValue(2500000.0), // 2500 km
                          MakeDoubleAccessor(&LeoSimChannelModel::m_maxLinkDistance),
                          MakeDoubleChecker<double>())
            .AddAttribute("Frequency",
                          "Carrier frequency (Hz)",
                          DoubleValue(12.0e9), // 12 GHz (Ku-band)
                          MakeDoubleAccessor(&LeoSimChannelModel::m_frequency),
                          MakeDoubleChecker<double>())
            .AddAttribute("TransmitPower",
                          "Transmit power (dBm)",
                          DoubleValue(40.0),
                          MakeDoubleAccessor(&LeoSimChannelModel::m_transmitPower),
                          MakeDoubleChecker<double>())
            .AddAttribute("TxAntennaGain",
                          "Transmit antenna gain (dB)",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&LeoSimChannelModel::m_txAntennaGain),
                          MakeDoubleChecker<double>())
            .AddAttribute("RxAntennaGain",
                          "Receive antenna gain (dB)",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&LeoSimChannelModel::m_rxAntennaGain),
                          MakeDoubleChecker<double>())
            .AddAttribute("NoiseTemperature",
                          "System noise temperature (Kelvin)",
                          DoubleValue(290.0),
                          MakeDoubleAccessor(&LeoSimChannelModel::m_noiseTemperature),
                          MakeDoubleChecker<double>())
            .AddAttribute("NoiseBandwidth",
                          "Noise bandwidth (Hz)",
                          DoubleValue(1.0e6),
                          MakeDoubleAccessor(&LeoSimChannelModel::m_noiseBandwidth),
                          MakeDoubleChecker<double>())
            .AddAttribute("AtmosphericAttenuationEnabled",
                          "Enable atmospheric attenuation model",
                          BooleanValue(true),
                          MakeBooleanAccessor(&LeoSimChannelModel::m_atmosphericEnabled),
                          MakeBooleanChecker())
            .AddAttribute("UpdateInterval",
                          "Interval for periodic channel updates",
                          TimeValue(Seconds(1.0)),
                          MakeTimeAccessor(&LeoSimChannelModel::m_updateInterval),
                          MakeTimeChecker())
            .AddAttribute("IslMaxDistance",
                          "Maximum ISL distance (meters)",
                          DoubleValue(5000000.0), // 5000 km for ISL
                          MakeDoubleAccessor(&LeoSimChannelModel::m_islMaxDistance),
                          MakeDoubleChecker<double>())
            .AddAttribute("IslTransmitPower",
                          "ISL transmit power (dBm)",
                          DoubleValue(30.0), // Lower power for ISL
                          MakeDoubleAccessor(&LeoSimChannelModel::m_islTransmitPower),
                          MakeDoubleChecker<double>())
            .AddAttribute("IslAntennaGain",
                          "ISL antenna gain (dB)",
                          DoubleValue(30.0), // Higher gain for ISL
                          MakeDoubleAccessor(&LeoSimChannelModel::m_islAntennaGain),
                          MakeDoubleChecker<double>())
            .AddAttribute("IslFrequency",
                          "ISL frequency (Hz)",
                          DoubleValue(26.0e9), // 26 GHz Ka-band for ISL
                          MakeDoubleAccessor(&LeoSimChannelModel::m_islFrequency),
                          MakeDoubleChecker<double>())
            .AddTraceSource("LinkStateChange",
                            "Trace link state changes",
                            MakeTraceSourceAccessor(&LeoSimChannelModel::m_linkStateChangeTrace),
                            "ns3::LeoSimChannelModel::LinkStateChangeCallback")
            .AddTraceSource("PathLoss",
                            "Trace path loss updates",
                            MakeTraceSourceAccessor(&LeoSimChannelModel::m_pathLossTrace),
                            "ns3::LeoSimChannelModel::PathLossCallback");
    return tid;
}

LeoSimChannelModel::LeoSimChannelModel()
    : m_nextLinkId(1),
      m_minElevationAngle(10.0),
      m_maxLinkDistance(2500000.0),
      m_frequency(12.0e9),
      m_transmitPower(40.0),
    m_txAntennaGain(20.0),
    m_rxAntennaGain(20.0),
      m_noiseTemperature(290.0),
    m_noiseBandwidth(1.0e6),
      m_atmosphericEnabled(true),
      m_updateInterval(Seconds(1.0)),
      m_verbose(false),
      m_islMaxDistance(5000000.0),
      m_islTransmitPower(30.0),
      m_islAntennaGain(30.0),
      m_islFrequency(26.0e9)
{
    NS_LOG_FUNCTION(this);
}

LeoSimChannelModel::~LeoSimChannelModel()
{
    NS_LOG_FUNCTION(this);
}

uint32_t
LeoSimChannelModel::AddLink(Ptr<Node> node1, Ptr<Node> node2, LeoSimLinkType linkType)
{
    NS_LOG_FUNCTION(this << node1->GetId() << node2->GetId() << linkType);

    uint32_t id1 = node1->GetId();
    uint32_t id2 = node2->GetId();
    auto nodePair = std::make_pair(std::min(id1, id2), std::max(id1, id2));

    // Check if link already exists
    auto it = m_nodePairToLink.find(nodePair);
    if (it != m_nodePairToLink.end())
    {
        NS_LOG_INFO("Link already exists between nodes " << id1 << " and " << id2);
        return it->second;
    }

    uint32_t linkId = m_nextLinkId++;

    LinkInfo info;
    info.node1 = node1;
    info.node2 = node2;
    info.linkId = linkId;
    info.linkType = linkType;
    info.quality.linkState = LEOSIM_LINK_DOWN;
    info.quality.linkType = linkType;
    info.quality.lastUpdate = Simulator::Now();

    m_links[linkId] = info;
    m_nodePairToLink[nodePair] = linkId;

    // Initial update
    UpdateLink(linkId);

    if (m_verbose)
    {
        const char* typeStr = (linkType == LEOSIM_LINK_ISL) ? "ISL" : "Ground";
        NS_LOG_INFO("Added " << typeStr << " link " << linkId << " between nodes " << id1 << " and " << id2);
    }

    return linkId;
}

void
LeoSimChannelModel::RemoveLink(Ptr<Node> node1, Ptr<Node> node2)
{
    NS_LOG_FUNCTION(this << node1->GetId() << node2->GetId());

    uint32_t id1 = node1->GetId();
    uint32_t id2 = node2->GetId();
    auto nodePair = std::make_pair(std::min(id1, id2), std::max(id1, id2));

    auto it = m_nodePairToLink.find(nodePair);
    if (it != m_nodePairToLink.end())
    {
        uint32_t linkId = it->second;
        m_links.erase(linkId);
        m_nodePairToLink.erase(it);

        if (m_verbose)
        {
            NS_LOG_INFO("Removed link " << linkId << " between nodes " << id1 << " and " << id2);
        }
    }
}

void
LeoSimChannelModel::UpdateAllLinks()
{
    NS_LOG_FUNCTION(this);

    for (auto& pair : m_links)
    {
        UpdateLink(pair.first);
        NS_LOG_DEBUG("\nlink:" <<pair.second.linkType<<" "<< pair.first<<" "<<pair.second.quality.linkState);

    }
}

void
LeoSimChannelModel::UpdateLink(uint32_t linkId)
{
    NS_LOG_FUNCTION(this << linkId);
    NS_LOG_DEBUG("\nUpdateLink called for linkId=" << linkId);

    auto it = m_links.find(linkId);
    if (it == m_links.end())
    {
        NS_LOG_DEBUG("Link " << linkId << " not found in m_links");
        return;
    }

    LinkInfo& info = it->second;
    Ptr<Node> node1 = info.node1;
    Ptr<Node> node2 = info.node2;

    NS_LOG_DEBUG("Updating link between nodes " << node1->GetId() << " and " << node2->GetId());

    // Get mobility models
    Ptr<MobilityModel> mob1 = node1->GetObject<MobilityModel>();
    Ptr<MobilityModel> mob2 = node2->GetObject<MobilityModel>();

    if (!mob1 || !mob2)
    {
        if (m_verbose)
        {
            NS_LOG_WARN("Link " << linkId << ": Nodes " << node1->GetId() << "/" << node2->GetId() 
                       << " do not have mobility models");
        }
        NS_LOG_DEBUG("Missing mobility models: mob1=" << mob1 << ", mob2=" << mob2);
        info.quality.linkState = LEOSIM_LINK_DOWN;
        return;
    }

    Vector pos1 = mob1->GetPosition();
    Vector pos2 = mob2->GetPosition();

    NS_LOG_DEBUG("Node " << node1->GetId() << " position: (" << pos1.x << ", " << pos1.y << ", " << pos1.z << ")");
    NS_LOG_DEBUG("Node " << node2->GetId() << " position: (" << pos2.x << ", " << pos2.y << ", " << pos2.z << ")");

    // Calculate distance
    double dx = pos2.x - pos1.x;
    double dy = pos2.y - pos1.y;
    double dz = pos2.z - pos1.z;
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    NS_LOG_DEBUG("Calculated distance: " << distance << " meters (" << distance/1000.0 << " km)");

    info.quality.distance = distance;
    info.quality.lastUpdate = Simulator::Now();

    // Use ISL-specific parameters for ISL links
    bool isIsl = (info.linkType == LEOSIM_LINK_ISL);
    double maxDistance = isIsl ? m_islMaxDistance : m_maxLinkDistance;
    double txPower = isIsl ? m_islTransmitPower : m_transmitPower;
    double txGain = isIsl ? m_islAntennaGain : m_txAntennaGain;
    double rxGain = isIsl ? m_islAntennaGain : m_rxAntennaGain;
    double frequency = isIsl ? m_islFrequency : m_frequency;

    NS_LOG_DEBUG("Link type: " << (isIsl ? "ISL" : "Ground"));
    NS_LOG_DEBUG("Parameters: maxDistance=" << maxDistance/1000.0 << " km, txPower=" << txPower 
                 << " dBm, txGain=" << txGain << " dB, rxGain=" << rxGain << " dB, frequency=" 
                 << frequency/1e9 << " GHz");

    // Check distance constraint
    if (distance > maxDistance)
    {
        LeoSimLinkState oldState = info.quality.linkState;
        info.quality.linkState = LEOSIM_LINK_DOWN;
        info.quality.signalStrength = -200.0; // Very weak signal
        info.quality.snr = -100.0;

        NS_LOG_DEBUG("Distance exceeds maximum: " << distance/1000.0 << " km > " << maxDistance/1000.0 << " km");

        if (oldState != LEOSIM_LINK_DOWN)
        {
            m_linkStateChangeTrace(node1, node2, LEOSIM_LINK_DOWN);
            if (m_verbose)
            {
                const char* typeStr = isIsl ? "ISL" : "Ground";
                NS_LOG_INFO(typeStr << " link " << linkId << " DOWN: distance " << distance/1000.0 
                           << " km exceeds maximum " << maxDistance/1000.0 << " km");
            }
        }
        return;
    }

    // Calculate elevation angle (only for ground-to-satellite links)
    double elevationAngle = 0.0;
    if (!isIsl)
    {
        bool elevationComputed = false;

        Ptr<LeoSimMobilityModel> leoMob1 = node1->GetObject<LeoSimMobilityModel>();
        Ptr<LeoSimMobilityModel> leoMob2 = node2->GetObject<LeoSimMobilityModel>();

        NS_LOG_DEBUG("Checking for LeoSimMobilityModel: leoMob1=" << leoMob1 << ", leoMob2=" << leoMob2);

        if (leoMob1 && leoMob2)
        {
            LeoSimNodeType type1 = leoMob1->GetNodeType();
            LeoSimNodeType type2 = leoMob2->GetNodeType();

            NS_LOG_DEBUG("Node types: node1=" << type1 << ", node2=" << type2);

            if (type1 == LEOSIM_SATELLITE && (type2 == LEOSIM_GATEWAY || type2 == LEOSIM_UE))
            {
                elevationAngle = CalculateElevationAngle(pos2, pos1);
                elevationComputed = true;
                NS_LOG_DEBUG("Computed elevation angle (ground->sat): " << elevationAngle << "°");
            }
            else if (type2 == LEOSIM_SATELLITE && (type1 == LEOSIM_GATEWAY || type1 == LEOSIM_UE))
            {
                elevationAngle = CalculateElevationAngle(pos1, pos2);
                elevationComputed = true;
                NS_LOG_DEBUG("Computed elevation angle (sat->ground): " << elevationAngle << "°");
            }
        }

        if (!elevationComputed)
        {
            elevationAngle = CalculateElevationAngle(pos1, pos2);
            NS_LOG_DEBUG("Computed default elevation angle: " << elevationAngle << "°");
        }
        info.quality.elevationAngle = elevationAngle;

        // Check elevation angle constraint (only for ground links)
        if (elevationAngle < m_minElevationAngle)
        {
            LeoSimLinkState oldState = info.quality.linkState;
            info.quality.linkState = LEOSIM_LINK_DOWN;
            info.quality.signalStrength = -200.0;
            info.quality.snr = -100.0;

            NS_LOG_DEBUG("Elevation angle below minimum: " << elevationAngle << "° < " << m_minElevationAngle << "°");

            if (oldState != LEOSIM_LINK_DOWN)
            {
                m_linkStateChangeTrace(node1, node2, LEOSIM_LINK_DOWN);
                if (m_verbose)
                {
                    NS_LOG_INFO("Ground link " << linkId << " DOWN: elevation angle " << elevationAngle 
                               << "° below minimum " << m_minElevationAngle << "°");
                }
            }
            return;
        }
    }
    else
    {
        // ISL has no elevation angle constraint
        info.quality.elevationAngle = 90.0; // Set to zenith for ISL
        NS_LOG_DEBUG("ISL link - elevation angle set to 90°");
    }

    // Calculate path loss using appropriate frequency
    double pathLoss = 20.0 * std::log10(distance) + 20.0 * std::log10(frequency) 
                     - 20.0 * std::log10(SPEED_OF_LIGHT) + 20.0 * std::log10(4.0 * M_PI);

    NS_LOG_DEBUG("Free space path loss: " << pathLoss << " dB");

    // Add atmospheric attenuation only for ground links (not ISL)
    if (!isIsl && m_atmosphericEnabled)
    {
        double atmosphericLoss = CalculateAtmosphericAttenuation(distance, elevationAngle);
        NS_LOG_DEBUG("Atmospheric attenuation: " << atmosphericLoss << " dB");
        pathLoss += atmosphericLoss;
        NS_LOG_DEBUG("Total path loss (with atmosphere): " << pathLoss << " dB");
    }

    info.quality.pathLoss = pathLoss;

    // Calculate received signal strength using appropriate parameters
    // RSS = TX_Power + Gains - PathLoss
    double signalStrength = txPower + txGain + rxGain - pathLoss;
    info.quality.signalStrength = signalStrength;

    NS_LOG_DEBUG("Signal strength calculation: " << txPower << " + " << txGain << " + " << rxGain 
                 << " - " << pathLoss << " = " << signalStrength << " dBm");

    // Calculate noise power (simple model)
    // Noise = 10*log10(k*T*B)
    double noisePower = 10.0 * std::log10(BOLTZMANN_CONSTANT * m_noiseTemperature * m_noiseBandwidth) + 30.0; // Convert to dBm
    
    NS_LOG_DEBUG("Noise power: " << noisePower << " dBm");

    // Calculate SNR
    double snr = signalStrength - noisePower;
    info.quality.snr = snr;

    NS_LOG_DEBUG("SNR: " << snr << " dB");

    // Determine link state based on SNR
    LeoSimLinkState oldState = info.quality.linkState;
    LeoSimLinkState newState;

    if (snr > 10.0)
    {
        newState = LEOSIM_LINK_UP;
        NS_LOG_DEBUG("Link state: UP (SNR > 10 dB)");
    }
    else if (snr > 0.0)
    {
        newState = LEOSIM_LINK_DEGRADED;
        NS_LOG_DEBUG("Link state: DEGRADED (0 < SNR <= 10 dB)");
    }
    else
    {
        newState = LEOSIM_LINK_DOWN;
        NS_LOG_DEBUG("Link state: DOWN (SNR <= 0 dB)");
    }

    info.quality.linkState = newState;

    // Fire trace callbacks if state changed
    if (oldState != newState)
    {
        m_linkStateChangeTrace(node1, node2, newState);
        if (m_verbose)
        {
            const char* stateStr[] = {"UP", "DOWN", "DEGRADED"};
            const char* typeStr = isIsl ? "ISL" : "Ground";
            NS_LOG_INFO(typeStr << " link " << linkId << " state changed to " << stateStr[newState] 
                       << " (SNR: " << snr << " dB, distance: " << distance/1000.0 << " km");
            if (!isIsl)
            {
                NS_LOG_INFO(", elevation: " << elevationAngle << "°");
            }
            NS_LOG_INFO(")");
        }
        NS_LOG_DEBUG("Link state changed from " << oldState << " to " << newState);
    }
    else if (m_verbose && newState == LEOSIM_LINK_DOWN)
    {
        // Log why link stays DOWN
        const char* typeStr = isIsl ? "ISL" : "Ground";
        NS_LOG_INFO(typeStr << " link " << linkId << " remains DOWN (SNR: " << snr 
                   << " dB, distance: " << distance/1000.0 << " km, elev: " << elevationAngle << "°)");
    }

    m_pathLossTrace(node1, node2, pathLoss);
}

LeoSimChannelQuality
LeoSimChannelModel::GetChannelQuality(Ptr<Node> node1, Ptr<Node> node2)
{
    NS_LOG_FUNCTION(this << node1->GetId() << node2->GetId());

    uint32_t id1 = node1->GetId();
    uint32_t id2 = node2->GetId();
    auto nodePair = std::make_pair(std::min(id1, id2), std::max(id1, id2));

    auto it = m_nodePairToLink.find(nodePair);
    if (it != m_nodePairToLink.end())
    {
        return m_links[it->second].quality;
    }

    // Return empty quality if link doesn't exist
    LeoSimChannelQuality quality;
    quality.linkState = LEOSIM_LINK_DOWN;
    quality.distance = 0.0;
    quality.pathLoss = 0.0;
    quality.elevationAngle = 0.0;
    quality.signalStrength = -200.0;
    quality.snr = -100.0;
    quality.peerNodeId = 0;
    quality.lastUpdate = Simulator::Now();
    return quality;
}

bool
LeoSimChannelModel::IsLinkUp(Ptr<Node> node1, Ptr<Node> node2)
{
    LeoSimChannelQuality quality = GetChannelQuality(node1, node2);
    return quality.linkState == LEOSIM_LINK_UP || quality.linkState == LEOSIM_LINK_DEGRADED;
}

double
LeoSimChannelModel::CalculateFreeSpacePathLoss(double distance) const
{
    NS_LOG_FUNCTION(this << distance);

    // Free space path loss: FSPL = 20*log10(d) + 20*log10(f) + 20*log10(4π/c)
    // where d is distance in meters, f is frequency in Hz, c is speed of light
    
    if (distance <= 0.0)
    {
        return 0.0;
    }

    double pathLoss = 20.0 * std::log10(distance) + 20.0 * std::log10(m_frequency) - 20.0 * std::log10(SPEED_OF_LIGHT) + 20.0 * std::log10(4.0 * M_PI);

    return pathLoss;
}

double
LeoSimChannelModel::CalculateAtmosphericAttenuation(double distance, double elevationAngle) const
{
    NS_LOG_FUNCTION(this << distance << elevationAngle);

    // Simplified atmospheric attenuation model
    // Real models would use ITU-R P.676 for gaseous absorption
    // and ITU-R P.618 for rain attenuation

    if (elevationAngle >= 90.0)
    {
        elevationAngle = 90.0;
    }

    // Slant path through atmosphere (approximate)
    double atmosphereHeight = 20000.0; // 20 km effective atmosphere height
    double elevationRad = elevationAngle * M_PI / 180.0;
    
    double slantPath;
    if (elevationAngle > 10.0)
    {
        slantPath = atmosphereHeight / std::sin(elevationRad);
    }
    else
    {
        // More complex calculation for low elevation angles
        slantPath = atmosphereHeight / std::sin(10.0 * M_PI / 180.0);
    }

    // Frequency-dependent attenuation (simplified)
    // At 12 GHz (Ku-band), typical clear-sky attenuation is ~0.5 dB at zenith
    double zenithAttenuation = 0.5; // dB
    double frequencyFactor = m_frequency / 12.0e9;
    
    // Scale by slant path and frequency
    double attenuation = zenithAttenuation * (slantPath / atmosphereHeight) * std::sqrt(frequencyFactor);

    return attenuation;
}

double
LeoSimChannelModel::CalculateElevationAngle(const Vector& groundPos, const Vector& satPos) const
{
    NS_LOG_FUNCTION(this << groundPos << satPos);

    // Calculate vector from ground to satellite
    Vector los;
    los.x = satPos.x - groundPos.x;
    los.y = satPos.y - groundPos.y;
    los.z = satPos.z - groundPos.z;

    double losDistance = std::sqrt(los.x*los.x + los.y*los.y + los.z*los.z);
    
    if (losDistance == 0.0)
    {
        return 0.0;
    }

    // Calculate local horizontal plane
    // Assume ground position is the reference point
    double groundDistance = std::sqrt(groundPos.x*groundPos.x + groundPos.y*groundPos.y + groundPos.z*groundPos.z);
    
    if (groundDistance == 0.0)
    {
        // If ground is at origin, use simple angle calculation
        double elevationRad = std::asin(los.z / losDistance);
        return elevationRad * 180.0 / M_PI;
    }

    // Unit vector from Earth center to ground position (local vertical)
    Vector up;
    up.x = groundPos.x / groundDistance;
    up.y = groundPos.y / groundDistance;
    up.z = groundPos.z / groundDistance;

    // Normalize LOS vector
    Vector losNorm;
    losNorm.x = los.x / losDistance;
    losNorm.y = los.y / losDistance;
    losNorm.z = los.z / losDistance;

    // Dot product gives cos(angle from zenith)
    double cosZenith = losNorm.x*up.x + losNorm.y*up.y + losNorm.z*up.z;
    
    // Elevation angle is 90° - zenith angle
    double zenithAngle = std::acos(std::min(1.0, std::max(-1.0, cosZenith)));
    double elevationAngle = (M_PI / 2.0 - zenithAngle) * 180.0 / M_PI;

    return elevationAngle;
}

void
LeoSimChannelModel::PeriodicUpdate()
{
    NS_LOG_FUNCTION(this);

    UpdateAllLinks();

    // Schedule next update
    if (m_updateEvent.IsExpired())
    {
        m_updateEvent = Simulator::Schedule(m_updateInterval, &LeoSimChannelModel::PeriodicUpdate, this);
    }
}

void
LeoSimChannelModel::StartUpdates()
{
    NS_LOG_FUNCTION(this);

    if (!m_updateEvent.IsPending())
    {
        m_updateEvent = Simulator::Schedule(m_updateInterval, &LeoSimChannelModel::PeriodicUpdate, this);
        if (m_verbose)
        {
            NS_LOG_INFO("Started periodic channel updates (interval: " << m_updateInterval.GetSeconds() << "s)");
        }
    }
}

void
LeoSimChannelModel::StopUpdates()
{
    NS_LOG_FUNCTION(this);

    if (m_updateEvent.IsPending())
    {
        Simulator::Cancel(m_updateEvent);
        if (m_verbose)
        {
            NS_LOG_INFO("Stopped periodic channel updates");
        }
    }
}

std::vector<std::pair<Ptr<Node>, Ptr<Node>>>
LeoSimChannelModel::GetActiveLinks() const
{
    std::vector<std::pair<Ptr<Node>, Ptr<Node>>> activeLinks;
    
    if (m_verbose)
    {
        NS_LOG_INFO("GetActiveLinks: Checking " << m_links.size() << " total links");
    }
    
    for (const auto& pair : m_links)
    {
        const LinkInfo& info = pair.second;
        
        if (m_verbose)
        {
            const char* stateStr[] = {"UP", "DOWN", "DEGRADED"};
            const char* typeStr = (info.linkType == LEOSIM_LINK_ISL) ? "ISL" : "Ground";
            NS_LOG_INFO("  Link " << info.linkId << " (" << typeStr << "): " 
                       << info.node1->GetId() << "<->" << info.node2->GetId() 
                       << " state=" << stateStr[info.quality.linkState]);
        }

        if (info.quality.linkState == LEOSIM_LINK_UP || info.quality.linkState == LEOSIM_LINK_DEGRADED)
        {
            activeLinks.push_back(std::make_pair(info.node1, info.node2));
        }
    }
    
    if (m_verbose)
    {
        NS_LOG_INFO("GetActiveLinks: Returning " << activeLinks.size() << " active links");
    }
    
    return activeLinks;
}

std::vector<LeoSimChannelModel::LinkSnapshot>
LeoSimChannelModel::GetLinksByType(LeoSimLinkType linkType, bool includeDown) const
{
    std::vector<LinkSnapshot> links;

    for (const auto& pair : m_links)
    {
        const LinkInfo& info = pair.second;
        if (info.linkType != linkType)
        {
            continue;
        }

        if (!includeDown && info.quality.linkState == LEOSIM_LINK_DOWN)
        {
            continue;
        }

        LinkSnapshot snapshot;
        snapshot.node1 = info.node1;
        snapshot.node2 = info.node2;
        snapshot.linkId = info.linkId;
        snapshot.linkType = info.linkType;
        snapshot.linkState = info.quality.linkState;
        links.push_back(snapshot);
    }

    return links;
}

uint32_t
LeoSimChannelModel::GetLinkId(Ptr<Node> node1, Ptr<Node> node2)
{
    uint32_t id1 = node1->GetId();
    uint32_t id2 = node2->GetId();
    auto nodePair = std::make_pair(std::min(id1, id2), std::max(id1, id2));

    auto it = m_nodePairToLink.find(nodePair);
    if (it != m_nodePairToLink.end())
    {
        return it->second;
    }
    
    return 0; // Invalid link ID
}

void
LeoSimChannelModel::SetMinElevationAngle(double angle)
{
    NS_LOG_FUNCTION(this << angle);
    m_minElevationAngle = angle;
}

double
LeoSimChannelModel::GetMinElevationAngle() const
{
    return m_minElevationAngle;
}

void
LeoSimChannelModel::SetMaxLinkDistance(double distance)
{
    NS_LOG_FUNCTION(this << distance);
    m_maxLinkDistance = distance;
}

double
LeoSimChannelModel::GetMaxLinkDistance() const
{
    return m_maxLinkDistance;
}

void
LeoSimChannelModel::SetFrequency(double frequency)
{
    NS_LOG_FUNCTION(this << frequency);
    m_frequency = frequency;
}

double
LeoSimChannelModel::GetFrequency() const
{
    return m_frequency;
}

void
LeoSimChannelModel::SetTransmitPower(double power)
{
    NS_LOG_FUNCTION(this << power);
    m_transmitPower = power;
}

void
LeoSimChannelModel::SetTxAntennaGain(double gain)
{
    NS_LOG_FUNCTION(this << gain);
    m_txAntennaGain = gain;
}

double
LeoSimChannelModel::GetTransmitPower() const
{
    return m_transmitPower;
}

double
LeoSimChannelModel::GetTxAntennaGain() const
{
    return m_txAntennaGain;
}

void
LeoSimChannelModel::SetRxAntennaGain(double gain)
{
    NS_LOG_FUNCTION(this << gain);
    m_rxAntennaGain = gain;
}

double
LeoSimChannelModel::GetRxAntennaGain() const
{
    return m_rxAntennaGain;
}

void
LeoSimChannelModel::SetNoiseTemperature(double temperature)
{
    NS_LOG_FUNCTION(this << temperature);
    m_noiseTemperature = temperature;
}

void
LeoSimChannelModel::SetNoiseBandwidth(double bandwidth)
{
    NS_LOG_FUNCTION(this << bandwidth);
    m_noiseBandwidth = bandwidth;
}

double
LeoSimChannelModel::GetNoiseTemperature() const
{
    return m_noiseTemperature;
}

double
LeoSimChannelModel::GetNoiseBandwidth() const
{
    return m_noiseBandwidth;
}

void
LeoSimChannelModel::SetAtmosphericAttenuationEnabled(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_atmosphericEnabled = enable;
}

bool
LeoSimChannelModel::IsAtmosphericAttenuationEnabled() const
{
    return m_atmosphericEnabled;
}

void
LeoSimChannelModel::SetUpdateInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_updateInterval = interval;
}

Time
LeoSimChannelModel::GetUpdateInterval() const
{
    return m_updateInterval;
}

void
LeoSimChannelModel::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
}

uint32_t
LeoSimChannelModel::AddIslLink(Ptr<Node> sat1, Ptr<Node> sat2)
{
    NS_LOG_FUNCTION(this << sat1->GetId() << sat2->GetId());
    return AddLink(sat1, sat2, LEOSIM_LINK_ISL);
}

uint32_t
LeoSimChannelModel::CreateIslMesh(NodeContainer satellites)
{
    NS_LOG_FUNCTION(this << satellites.GetN());
    
    uint32_t linkCount = 0;
    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        for (uint32_t j = i + 1; j < satellites.GetN(); ++j)
        {
            AddIslLink(satellites.Get(i), satellites.Get(j));
            linkCount++;
        }
    }
    
    if (m_verbose)
    {
        NS_LOG_INFO("Created ISL mesh with " << linkCount << " links between " 
                   << satellites.GetN() << " satellites");
    }
    
    return linkCount;
}

uint32_t
LeoSimChannelModel::UpdateIslTopology(NodeContainer satellites, double maxDistance)
{
    NS_LOG_FUNCTION(this << satellites.GetN() << maxDistance);
    
    uint32_t activeLinks = 0;
    
    // Check all satellite pairs
    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        for (uint32_t j = i + 1; j < satellites.GetN(); ++j)
        {
            Ptr<Node> sat1 = satellites.Get(i);
            Ptr<Node> sat2 = satellites.Get(j);
            
            // Get current positions
            Ptr<MobilityModel> mob1 = sat1->GetObject<MobilityModel>();
            Ptr<MobilityModel> mob2 = sat2->GetObject<MobilityModel>();
            
            if (!mob1 || !mob2)
            {
                continue;
            }
            
            Vector pos1 = mob1->GetPosition();
            Vector pos2 = mob2->GetPosition();
            
            // Calculate distance
            double dx = pos2.x - pos1.x;
            double dy = pos2.y - pos1.y;
            double dz = pos2.z - pos1.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Check if link should exist
            uint32_t id1 = sat1->GetId();
            uint32_t id2 = sat2->GetId();
            auto nodePair = std::make_pair(std::min(id1, id2), std::max(id1, id2));
            bool linkExists = (m_nodePairToLink.find(nodePair) != m_nodePairToLink.end());
            
            if (distance <= maxDistance)
            {
                // Satellites are in contact
                if (!linkExists)
                {
                    // Create new ISL link
                    AddIslLink(sat1, sat2);
                    if (m_verbose)
                    {
                        NS_LOG_INFO("Created ISL: Sat " << id1 << " <-> Sat " << id2 
                                   << " (distance: " << distance/1000.0 << " km)");
                    }
                }
                activeLinks++;
            }
            else if (linkExists)
            {
                // Satellites out of contact, remove link
                RemoveLink(sat1, sat2);
                if (m_verbose)
                {
                    NS_LOG_INFO("Removed ISL: Sat " << id1 << " <-> Sat " << id2 
                               << " (distance: " << distance/1000.0 << " km > " 
                               << maxDistance/1000.0 << " km)");
                }
            }
        }
    }
    
    return activeLinks;
}

void
LeoSimChannelModel::SetIslMaxDistance(double distance)
{
    NS_LOG_FUNCTION(this << distance);
    m_islMaxDistance = distance;
}

double
LeoSimChannelModel::GetIslMaxDistance() const
{
    return m_islMaxDistance;
}

void
LeoSimChannelModel::SetIslTransmitPower(double power)
{
    NS_LOG_FUNCTION(this << power);
    m_islTransmitPower = power;
}

double
LeoSimChannelModel::GetIslTransmitPower() const
{
    return m_islTransmitPower;
}

void
LeoSimChannelModel::SetIslAntennaGain(double gain)
{
    NS_LOG_FUNCTION(this << gain);
    m_islAntennaGain = gain;
}

double
LeoSimChannelModel::GetIslAntennaGain() const
{
    return m_islAntennaGain;
}

void
LeoSimChannelModel::SetIslFrequency(double frequency)
{
    NS_LOG_FUNCTION(this << frequency);
    m_islFrequency = frequency;
}

double
LeoSimChannelModel::GetIslFrequency() const
{
    return m_islFrequency;
}

std::vector<LeoSimChannelQuality>
LeoSimChannelModel::GetLinksForNode(uint32_t nodeId, bool filterBySharing) const
{
    NS_LOG_FUNCTION(this << nodeId << filterBySharing);
    std::vector<LeoSimChannelQuality> result;

    // Iterate through all links to find those connected to this node
    for (const auto& linkEntry : m_links)
    {
        const LinkInfo& linkInfo = linkEntry.second;
        uint32_t id1 = linkInfo.node1->GetId();
        uint32_t id2 = linkInfo.node2->GetId();

        // Check if this link is connected to the requested node
        if (id1 == nodeId || id2 == nodeId)
        {
            // Only include links with UP or DEGRADED state
            if (linkInfo.quality.linkState == LEOSIM_LINK_UP ||
                linkInfo.quality.linkState == LEOSIM_LINK_DEGRADED)
            {
                LeoSimChannelQuality quality = linkInfo.quality;
                quality.peerNodeId = (id1 == nodeId) ? id2 : id1;
                result.push_back(quality);
            }
        }
    }

    if (filterBySharing)
    {
        result.erase(std::remove_if(result.begin(),
                                    result.end(),
                                    [&](const LeoSimChannelQuality& lq) {
                                        return GetAlpha(nodeId,
                                                        lq.peerNodeId,
                                                        LEOSIM_DIR_DOWNLINK) == 0.0;
                                    }),
                     result.end());
    }

    NS_LOG_DEBUG("Found " << result.size() << " UP/DEGRADED links for node " << nodeId);
    return result;
}

LeoSimChannelQuality
LeoSimChannelModel::GetLinkQuality(uint32_t nodeA, uint32_t nodeB) const
{
    NS_LOG_FUNCTION(this << nodeA << nodeB);

    // Create normalized node pair (ordered)
    auto nodePair = std::make_pair(std::min(nodeA, nodeB), std::max(nodeA, nodeB));

    // Look up the link ID for this node pair
    auto it = m_nodePairToLink.find(nodePair);
    if (it != m_nodePairToLink.end())
    {
        uint32_t linkId = it->second;
        auto linkIt = m_links.find(linkId);
        if (linkIt != m_links.end())
        {
            NS_LOG_DEBUG("Found link quality for nodes " << nodeA << " and " << nodeB);
            return linkIt->second.quality;
        }
    }

    // Return default-constructed quality with DOWN state if link not found
    LeoSimChannelQuality defaultQuality;
    defaultQuality.linkState = LEOSIM_LINK_DOWN;
    defaultQuality.distance = 0.0;
    defaultQuality.pathLoss = 0.0;
    defaultQuality.elevationAngle = 0.0;
    defaultQuality.signalStrength = -200.0;
    defaultQuality.snr = -100.0;
    defaultQuality.peerNodeId = 0;
    defaultQuality.lastUpdate = Simulator::Now();
    defaultQuality.linkType = LEOSIM_LINK_SATELLITE_TO_GROUND; // Default type

    NS_LOG_DEBUG("Link not found for nodes " << nodeA << " and " << nodeB 
                 << ", returning default DOWN quality");
    return defaultQuality;
}

LeoSimLinkState
LeoSimChannelModel::GetLinkState(uint32_t nodeA, uint32_t nodeB) const
{
    NS_LOG_FUNCTION(this << nodeA << nodeB);

    // Create normalized node pair (ordered)
    auto nodePair = std::make_pair(std::min(nodeA, nodeB), std::max(nodeA, nodeB));

    // Look up the link ID for this node pair
    auto it = m_nodePairToLink.find(nodePair);
    if (it != m_nodePairToLink.end())
    {
        uint32_t linkId = it->second;
        auto linkIt = m_links.find(linkId);
        if (linkIt != m_links.end())
        {
            NS_LOG_DEBUG("Found link state for nodes " << nodeA << " and " << nodeB 
                         << ": " << linkIt->second.quality.linkState);
            return linkIt->second.quality.linkState;
        }
    }

    NS_LOG_DEBUG("Link not found for nodes " << nodeA << " and " << nodeB 
                 << ", returning DOWN state");
    return LEOSIM_LINK_DOWN;
}

void
LeoSimChannelModel::SetOperatorModel(Ptr<LeoSimOperatorModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_operatorModel = model;
}

LeoSimOperatorId
LeoSimChannelModel::GetOperatorId(uint32_t nodeId) const
{
    NS_LOG_FUNCTION(this << nodeId);
    if (!m_operatorModel)
    {
        return "unknown";
    }
    return m_operatorModel->GetOperatorId(nodeId);
}

double
LeoSimChannelModel::GetAlpha(uint32_t nodeA, uint32_t nodeB, LeoSimLinkDirection dir) const
{
    NS_LOG_FUNCTION(this << nodeA << nodeB << dir);
    if (!m_operatorModel)
    {
        return 1.0;
    }
    return m_operatorModel->GetAlpha(nodeA, nodeB, dir);
}

} // namespace ns3

