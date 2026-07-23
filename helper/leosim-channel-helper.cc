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

#include "leosim-channel-helper.h"

#include "ns3/leosim-mobility-model.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <queue>
#include <set>
#include <tuple>
#include <vector>
#include "ns3/names.h"
namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimChannelHelper");

LeoSimChannelHelper::LeoSimChannelHelper()
    : m_minElevationAngle(10.0),
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
      m_maxGroundLinksPerNode(1),
      m_islMaxDistance(5000000.0),
      m_islTransmitPower(30.0),
      m_islAntennaGain(30.0),
      m_islFrequency(26.0e9)
{
    NS_LOG_FUNCTION(this);
}

LeoSimChannelHelper::~LeoSimChannelHelper()
{
    NS_LOG_FUNCTION(this);
}

Ptr<LeoSimChannelModel>
LeoSimChannelHelper::CreateChannels(NodeContainer satellites, NodeContainer groundNodes)
{
    NS_LOG_FUNCTION(this << satellites.GetN() << groundNodes.GetN());

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    ConfigureChannelModel(channelModel);

    uint32_t linkCount = AddGroundAccessLinks(channelModel, satellites, groundNodes);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Created " << linkCount << " bounded satellite-to-ground candidate links");
    }

    // Start periodic updates
    channelModel->StartUpdates();

    return channelModel;
}

Ptr<LeoSimChannelModel>
LeoSimChannelHelper::CreateSatelliteToGatewayChannels(NodeContainer satellites,
                                                      NodeContainer gateways)
{
    NS_LOG_FUNCTION(this << satellites.GetN() << gateways.GetN());

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    ConfigureChannelModel(channelModel);

    uint32_t linkCount = AddGroundAccessLinks(channelModel, satellites, gateways);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Created " << linkCount << " bounded satellite-to-gateway candidate links");
    }

    // Start periodic updates
    channelModel->StartUpdates();

    return channelModel;
}

Ptr<LeoSimChannelModel>
LeoSimChannelHelper::CreateSatelliteToUeChannels(NodeContainer satellites, NodeContainer ues)
{
    NS_LOG_FUNCTION(this << satellites.GetN() << ues.GetN());

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    ConfigureChannelModel(channelModel);

    uint32_t linkCount = AddGroundAccessLinks(channelModel, satellites, ues);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Created " << linkCount << " bounded satellite-to-UE candidate links");
    }

    // Start periodic updates
    channelModel->StartUpdates();

    return channelModel;
}

void
LeoSimChannelHelper::AddLinks(Ptr<LeoSimChannelModel> channelModel,
                              NodeContainer satellites,
                              NodeContainer groundNodes)
{
    NS_LOG_FUNCTION(this << channelModel << satellites.GetN() << groundNodes.GetN());

    uint32_t linkCount = AddGroundAccessLinks(channelModel, satellites, groundNodes);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Added " << linkCount << " bounded access links to existing channel model");
    }
}

uint32_t
LeoSimChannelHelper::AddGroundAccessLinks(Ptr<LeoSimChannelModel> channelModel,
                                          NodeContainer satellites,
                                          NodeContainer groundNodes)
{
    NS_LOG_FUNCTION(this << channelModel << satellites.GetN() << groundNodes.GetN());

    if (!channelModel || satellites.GetN() == 0 || groundNodes.GetN() == 0)
    {
        return 0;
    }

    uint32_t linkCount = 0;
    const uint32_t maxLinks = std::max<uint32_t>(1, m_maxGroundLinksPerNode);

    for (uint32_t g = 0; g < groundNodes.GetN(); ++g)
    {
        Ptr<Node> ground = groundNodes.Get(g);
        Ptr<MobilityModel> groundMobility = ground->GetObject<MobilityModel>();
        if (!groundMobility)
        {
            continue;
        }

        const Vector groundPos = groundMobility->GetPosition();
        std::vector<std::vector<Vector>> satellitePositions(satellites.GetN());
        std::size_t epochs = 1;

        for (uint32_t s = 0; s < satellites.GetN(); ++s)
        {
            Ptr<Node> satellite = satellites.Get(s);
            Ptr<MobilityModel> satMobility = satellite->GetObject<MobilityModel>();
            if (!satMobility)
            {
                continue;
            }

            Ptr<LeoSimMobilityModel> leoMobility =
                satellite->GetObject<LeoSimMobilityModel>();
            if (leoMobility)
            {
                const auto waypoints = leoMobility->GetWaypoints();
                satellitePositions[s].reserve(waypoints.size());
                for (const auto& waypoint : waypoints)
                {
                    satellitePositions[s].push_back(waypoint.position);
                }
            }
            if (satellitePositions[s].empty())
            {
                satellitePositions[s].push_back(satMobility->GetPosition());
            }
            epochs = std::max(epochs, satellitePositions[s].size());
        }

        // Provision the union of the nearest visible satellites at every loaded
        // trajectory epoch. Net devices and addresses must exist before the
        // simulation starts; the channel model subsequently controls which of
        // these candidate links is active at each instant.
        std::set<uint32_t> selectedSatelliteIndices;
        std::vector<std::pair<double, uint32_t>> initialFallback;
        for (std::size_t epoch = 0; epoch < epochs; ++epoch)
        {
            std::vector<std::pair<double, uint32_t>> feasible;
            for (uint32_t s = 0; s < satellites.GetN(); ++s)
            {
                if (satellitePositions[s].empty())
                {
                    continue;
                }
                const Vector& satPos = satellitePositions[s][
                    std::min(epoch, satellitePositions[s].size() - 1)];
                const double dx = satPos.x - groundPos.x;
                const double dy = satPos.y - groundPos.y;
                const double dz = satPos.z - groundPos.z;
                const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (epoch == 0)
                {
                    initialFallback.emplace_back(distance, s);
                }
                if (distance <= m_maxLinkDistance &&
                    CalculateElevationAngle(groundPos, satPos) >= m_minElevationAngle)
                {
                    feasible.emplace_back(distance, s);
                }
            }
            std::sort(feasible.begin(), feasible.end());
            const uint32_t count = std::min<uint32_t>(maxLinks, feasible.size());
            for (uint32_t n = 0; n < count; ++n)
            {
                selectedSatelliteIndices.insert(feasible[n].second);
            }
        }

        // Retain the previous nearest-satellite fallback for datasets with no
        // geometrically feasible access link at any sampled epoch.
        if (selectedSatelliteIndices.empty() && !initialFallback.empty())
        {
            std::sort(initialFallback.begin(), initialFallback.end());
            const uint32_t count = std::min<uint32_t>(maxLinks, initialFallback.size());
            for (uint32_t n = 0; n < count; ++n)
            {
                selectedSatelliteIndices.insert(initialFallback[n].second);
            }
        }

        for (uint32_t satelliteIndex : selectedSatelliteIndices)
        {
            channelModel->AddLink(satellites.Get(satelliteIndex), ground);
            linkCount++;
        }
    }

    return linkCount;
}

void
LeoSimChannelHelper::ConfigureChannelModel(Ptr<LeoSimChannelModel> channelModel)
{
    NS_LOG_FUNCTION(this << channelModel);

    channelModel->SetMinElevationAngle(m_minElevationAngle);
    channelModel->SetMaxLinkDistance(m_maxLinkDistance);
    channelModel->SetFrequency(m_frequency);
    channelModel->SetTransmitPower(m_transmitPower);
    channelModel->SetTxAntennaGain(m_txAntennaGain);
    channelModel->SetRxAntennaGain(m_rxAntennaGain);
    channelModel->SetNoiseTemperature(m_noiseTemperature);
    channelModel->SetNoiseBandwidth(m_noiseBandwidth);
    channelModel->SetAtmosphericAttenuationEnabled(m_atmosphericEnabled);
    channelModel->SetUpdateInterval(m_updateInterval);
    channelModel->SetVerbose(m_verbose);

    // Configure ISL parameters
    channelModel->SetIslMaxDistance(m_islMaxDistance);
    channelModel->SetIslTransmitPower(m_islTransmitPower);
    channelModel->SetIslAntennaGain(m_islAntennaGain);
    channelModel->SetIslFrequency(m_islFrequency);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Configured channel model: " << "MinElev=" << m_minElevationAngle << "°, "
                                                 << "MaxDist=" << m_maxLinkDistance / 1000.0
                                                 << "km, "
                                                 << "Freq=" << m_frequency / 1e9 << "GHz, "
                                                 << "TxPower=" << m_transmitPower << "dBm, "
                                                 << "TxGain=" << m_txAntennaGain << "dB, "
                                                 << "RxGain=" << m_rxAntennaGain << "dB, "
                                                 << "NoiseBW=" << m_noiseBandwidth / 1e6 << "MHz");
        NS_LOG_DEBUG("ISL parameters: MaxDist=" << m_islMaxDistance / 1000.0 << "km, "
                                               << "TxPower=" << m_islTransmitPower << "dBm, "
                                               << "Gain=" << m_islAntennaGain << "dB, "
                                               << "Freq=" << m_islFrequency / 1e9 << "GHz");
    }
}

Ptr<LeoSimChannelModel>
LeoSimChannelHelper::GetChannelModel()
{
    NS_LOG_FUNCTION(this);

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    ConfigureChannelModel(channelModel);
    return channelModel;
}

void
LeoSimChannelHelper::SetMinElevationAngle(double angle)
{
    NS_LOG_FUNCTION(this << angle);
    m_minElevationAngle = angle;
}

void
LeoSimChannelHelper::SetMaxLinkDistance(double distance)
{
    NS_LOG_FUNCTION(this << distance);
    m_maxLinkDistance = distance;
}

void
LeoSimChannelHelper::SetFrequency(double frequency)
{
    NS_LOG_FUNCTION(this << frequency);
    m_frequency = frequency;
}

void
LeoSimChannelHelper::SetTransmitPower(double power)
{
    NS_LOG_FUNCTION(this << power);
    m_transmitPower = power;
}

void
LeoSimChannelHelper::SetTxAntennaGain(double gain)
{
    NS_LOG_FUNCTION(this << gain);
    m_txAntennaGain = gain;
}

void
LeoSimChannelHelper::SetRxAntennaGain(double gain)
{
    NS_LOG_FUNCTION(this << gain);
    m_rxAntennaGain = gain;
}

void
LeoSimChannelHelper::SetNoiseTemperature(double temperature)
{
    NS_LOG_FUNCTION(this << temperature);
    m_noiseTemperature = temperature;
}

void
LeoSimChannelHelper::SetNoiseBandwidth(double bandwidth)
{
    NS_LOG_FUNCTION(this << bandwidth);
    m_noiseBandwidth = bandwidth;
}

void
LeoSimChannelHelper::SetAtmosphericAttenuationEnabled(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_atmosphericEnabled = enable;
}

void
LeoSimChannelHelper::SetUpdateInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_updateInterval = interval;
}

void
LeoSimChannelHelper::SetMaxGroundLinksPerNode(uint32_t maxLinks)
{
    NS_LOG_FUNCTION(this << maxLinks);
    m_maxGroundLinksPerNode = std::max<uint32_t>(1, maxLinks);
}

void
LeoSimChannelHelper::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
}

bool
LeoSimChannelHelper::IsLinkFeasible(Ptr<Node> satellite, Ptr<Node> groundNode) const
{
    NS_LOG_FUNCTION(this << satellite << groundNode);

    if (!satellite || !groundNode)
    {
        return false;
    }

    Ptr<MobilityModel> satMobility = satellite->GetObject<MobilityModel>();
    Ptr<MobilityModel> groundMobility = groundNode->GetObject<MobilityModel>();
    if (!satMobility || !groundMobility)
    {
        return false;
    }

    const Vector satPos = satMobility->GetPosition();
    const Vector groundPos = groundMobility->GetPosition();
    const double dx = satPos.x - groundPos.x;
    const double dy = satPos.y - groundPos.y;
    const double dz = satPos.z - groundPos.z;
    const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

    return distance <= m_maxLinkDistance &&
           CalculateElevationAngle(groundPos, satPos) >= m_minElevationAngle;
}

double
LeoSimChannelHelper::CalculateElevationAngle(const Vector& groundPos, const Vector& satPos) const
{
    Vector los;
    los.x = satPos.x - groundPos.x;
    los.y = satPos.y - groundPos.y;
    los.z = satPos.z - groundPos.z;

    const double losDistance = std::sqrt(los.x * los.x + los.y * los.y + los.z * los.z);
    if (losDistance == 0.0)
    {
        return 0.0;
    }

    const double groundDistance =
        std::sqrt(groundPos.x * groundPos.x + groundPos.y * groundPos.y + groundPos.z * groundPos.z);
    if (groundDistance == 0.0)
    {
        return std::asin(los.z / losDistance) * 180.0 / M_PI;
    }

    Vector up;
    up.x = groundPos.x / groundDistance;
    up.y = groundPos.y / groundDistance;
    up.z = groundPos.z / groundDistance;

    Vector losNorm;
    losNorm.x = los.x / losDistance;
    losNorm.y = los.y / losDistance;
    losNorm.z = los.z / losDistance;

    const double cosZenith =
        std::min(1.0, std::max(-1.0, losNorm.x * up.x + losNorm.y * up.y + losNorm.z * up.z));
    const double zenithAngle = std::acos(cosZenith);
    return (M_PI / 2.0 - zenithAngle) * 180.0 / M_PI;
}

void
LeoSimChannelHelper::InstallLinkStateChangeCallback(
    Ptr<LeoSimChannelModel> channelModel,
    Callback<void, Ptr<Node>, Ptr<Node>, LeoSimLinkState> callback)
{
    NS_LOG_FUNCTION(this << channelModel);
    channelModel->TraceConnectWithoutContext("LinkStateChange", callback);
}

void
LeoSimChannelHelper::InstallPathLossCallback(Ptr<LeoSimChannelModel> channelModel,
                                             Callback<void, Ptr<Node>, Ptr<Node>, double> callback)
{
    NS_LOG_FUNCTION(this << channelModel);
    channelModel->TraceConnectWithoutContext("PathLoss", callback);
}

void
LeoSimChannelHelper::PrintChannelStatistics(Ptr<LeoSimChannelModel> channelModel)
{
    NS_LOG_FUNCTION(this << channelModel);

    auto activeLinks = channelModel->GetActiveLinks();

    std::cout << "\n=== Channel Statistics at t=" << Simulator::Now().GetSeconds()
              << "s ===" << std::endl;
    std::cout << "Total active links: " << activeLinks.size() << std::endl;

    if (activeLinks.empty())
    {
        std::cout << "No active links" << std::endl;
        return;
    }

    std::cout << "\nActive Links:" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << std::setw(10) << "Node1" << " | " << std::setw(10) << "Node2" << " | "
              << std::setw(10) << "Dist(km)" << " | " << std::setw(10) << "Elev(°)" << " | "
              << std::setw(10) << "Loss(dB)" << " | " << std::setw(10) << "SNR(dB)" << " | "
              << std::setw(10) << "State" << std::endl;
    std::cout << std::string(90, '-') << std::endl;

    for (const auto& link : activeLinks)
    {
        LeoSimChannelQuality quality = channelModel->GetChannelQuality(link.first, link.second);

        const char* stateStr[] = {"UP", "DOWN", "DEGRADED"};
        std::string node1Name = Names::FindName(link.first);
        std::string node2Name = Names::FindName(link.second);
        if (node1Name.empty()) node1Name = std::to_string(link.first->GetId());
        if (node2Name.empty()) node2Name = std::to_string(link.second->GetId());

        std::cout << std::setw(10) << node1Name << " | " << std::setw(10)
              << node2Name << " | " << std::setw(10) << quality.distance / 1000.0
              << " | " << std::setw(10) << quality.elevationAngle << " | " << std::setw(10)
              << quality.pathLoss << " | " << std::setw(10) << quality.snr << " | "
              << std::setw(10) << stateStr[quality.linkState] << std::endl;
    }
    std::cout << std::endl;
}

void
LeoSimChannelHelper::LogActiveLinks(Ptr<LeoSimChannelModel> channelModel, const std::string& prefix)
{
    NS_LOG_FUNCTION(this << channelModel << prefix);

    auto activeLinks = channelModel->GetActiveLinks();

    std::string msg = prefix;
    if (!msg.empty())
    {
        msg += " - ";
    }
    msg += "Active links: " + std::to_string(activeLinks.size());

    NS_LOG_DEBUG(msg);

    for (const auto& link : activeLinks)
    {
        LeoSimChannelQuality quality = channelModel->GetChannelQuality(link.first, link.second);
        NS_LOG_DEBUG("  Link " << link.first->GetId() << "->" << link.second->GetId()
                              << ": dist=" << quality.distance / 1000.0 << "km, "
                              << "elev=" << quality.elevationAngle << "°, "
                              << "SNR=" << quality.snr << "dB");
    }
}

Ptr<LeoSimChannelModel>
LeoSimChannelHelper::CreateIslMesh(NodeContainer satellites)
{
    NS_LOG_FUNCTION(this << satellites.GetN());

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    ConfigureChannelModel(channelModel);
    // Create ISL mesh
    uint32_t linkCount = channelModel->CreateIslMesh(satellites);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Created ISL mesh with " << linkCount << " links between " 
                   << satellites.GetN() << " satellites");
    }

    return channelModel;
}

Ptr<LeoSimChannelModel>
LeoSimChannelHelper::CreateIslNearestNeighborMesh(NodeContainer satellites, uint32_t maxNeighbors)
{
    NS_LOG_FUNCTION(this << satellites.GetN() << maxNeighbors);

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    ConfigureChannelModel(channelModel);

    uint32_t linkCount = AddNearestNeighborIslLinks(channelModel, satellites, maxNeighbors);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Created bounded ISL mesh with " << linkCount << " links between "
                     << satellites.GetN() << " satellites");
    }

    channelModel->StartUpdates();
    return channelModel;
}

Ptr<LeoSimChannelModel>
LeoSimChannelHelper::CreateIslGridTopology(NodeContainer satellites,
                                           uint32_t satellitesPerPlane,
                                           bool wrapPlanes)
{
    NS_LOG_FUNCTION(this << satellites.GetN() << satellitesPerPlane << wrapPlanes);

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    ConfigureChannelModel(channelModel);

    uint32_t linkCount =
        AddGridIslLinks(channelModel, satellites, satellitesPerPlane, wrapPlanes);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Created grid ISL topology with " << linkCount << " links between "
                                                       << satellites.GetN() << " satellites");
    }

    channelModel->StartUpdates();
    return channelModel;
}

uint32_t
LeoSimChannelHelper::AddIslLinks(Ptr<LeoSimChannelModel> channelModel, NodeContainer satellites)
{
    NS_LOG_FUNCTION(this << channelModel << satellites.GetN());

    uint32_t linkCount = channelModel->CreateIslMesh(satellites);

    if (m_verbose)
    {
        NS_LOG_DEBUG("Added " << linkCount << " ISL links to existing channel model");
    }

    return linkCount;
}

uint32_t
LeoSimChannelHelper::AddNearestNeighborIslLinks(Ptr<LeoSimChannelModel> channelModel,
                                                NodeContainer satellites,
                                                uint32_t maxNeighbors)
{
    NS_LOG_FUNCTION(this << channelModel << satellites.GetN() << maxNeighbors);

    if (!channelModel || maxNeighbors == 0 || satellites.GetN() < 2)
    {
        return 0;
    }

    struct Candidate
    {
        double distance;
        uint32_t first;
        uint32_t second;
    };

    const uint32_t numSatellites = satellites.GetN();
    const uint32_t candidateLimit = std::max<uint32_t>(16, maxNeighbors * 4);
    const double maxDistanceSquared = m_islMaxDistance * m_islMaxDistance;
    std::map<std::pair<uint32_t, uint32_t>, double> uniqueCandidates;

    (void)candidateLimit;
    (void)maxDistanceSquared;

    // GetObject() is comparatively expensive and this loop examines O(N^2)
    // pairs. Cache mobility positions once rather than doing two object lookups
    // for every pair.
    std::vector<Vector> positions(numSatellites);
    std::vector<bool> hasMobility(numSatellites, false);
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        Ptr<MobilityModel> mobility = satellites.Get(i)->GetObject<MobilityModel>();
        if (mobility)
        {
            positions[i] = mobility->GetPosition();
            hasMobility[i] = true;
        }
    }

    // Keep only a small nearest-neighbor candidate set per satellite. This uses
    // O(N*k) storage rather than materializing the O(N^2) full mesh.
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        if (!hasMobility[i])
        {
            continue;
        }

        const Vector& satPos = positions[i];
        // The largest retained candidate is at the top, so a closer candidate
        // can replace it in O(log k) without allocating an N-element vector.
        std::priority_queue<std::pair<double, uint32_t>> nearest;

        for (uint32_t j = 0; j < numSatellites; ++j)
        {
            if (i == j || !hasMobility[j])
            {
                continue;
            }

            const Vector& peerPos = positions[j];
            const double dx = peerPos.x - satPos.x;
            const double dy = peerPos.y - satPos.y;
            const double dz = peerPos.z - satPos.z;
            const double distanceSquared = dx * dx + dy * dy + dz * dz;
            if (distanceSquared <= maxDistanceSquared)
            {
                const auto candidate = std::make_pair(distanceSquared, j);
                if (nearest.size() < candidateLimit)
                {
                    nearest.push(candidate);
                }
                else if (candidate < nearest.top())
                {
                    nearest.pop();
                    nearest.push(candidate);
                }
            }
        }

        (void)nearest;

        while (!nearest.empty())
        {
            const double distance = std::sqrt(nearest.top().first);
            const uint32_t j = nearest.top().second;
            nearest.pop();
            const auto pair = std::make_pair(std::min(i, j), std::max(i, j));
            auto [it, inserted] = uniqueCandidates.emplace(pair, distance);
            if (!inserted)
            {
                it->second = std::min(it->second, distance);
            }
        }
    }

    std::vector<Candidate> candidates;
    candidates.reserve(uniqueCandidates.size());
    for (const auto& [pair, distance] : uniqueCandidates)
    {
        candidates.push_back({distance, pair.first, pair.second});
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.distance != b.distance)
        {
            return a.distance < b.distance;
        }
        return std::tie(a.first, a.second) < std::tie(b.first, b.second);
    });
    (void)candidates;

    std::vector<uint32_t> parent(numSatellites);
    std::vector<uint32_t> rank(numSatellites, 0);
    std::vector<uint32_t> degree(numSatellites, 0);
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        parent[i] = i;
    }
    auto findRoot = [&parent](uint32_t node) {
        uint32_t root = node;
        while (parent[root] != root)
        {
            root = parent[root];
        }
        while (parent[node] != node)
        {
            const uint32_t next = parent[node];
            parent[node] = root;
            node = next;
        }
        return root;
    };
    auto unite = [&parent, &rank, &findRoot](uint32_t a, uint32_t b) {
        a = findRoot(a);
        b = findRoot(b);
        if (a == b)
        {
            return;
        }
        if (rank[a] < rank[b])
        {
            std::swap(a, b);
        }
        parent[b] = a;
        if (rank[a] == rank[b])
        {
            rank[a]++;
        }
    };

    std::set<std::pair<uint32_t, uint32_t>> selected;
    auto select = [&](const Candidate& edge) {
        if (degree[edge.first] >= maxNeighbors || degree[edge.second] >= maxNeighbors)
        {
            return false;
        }
        const auto pair = std::make_pair(edge.first, edge.second);
        if (!selected.insert(pair).second)
        {
            return false;
        }
        degree[edge.first]++;
        degree[edge.second]++;
        unite(edge.first, edge.second);
        return true;
    };

    // Connectivity pass: prefer short edges that join different components.
    for (const Candidate& edge : candidates)
    {
        if (findRoot(edge.first) != findRoot(edge.second))
        {
            select(edge);
        }
    }

    // Capacity pass: fill unused terminals with the shortest remaining edges.
    for (const Candidate& edge : candidates)
    {
        select(edge);
    }

    uint32_t linkCount = 0;
    for (const auto& [first, second] : selected)
    {
        channelModel->AddIslLink(satellites.Get(first), satellites.Get(second));
        linkCount++;
    }

    std::set<uint32_t> components;
    uint32_t isolated = 0;
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        components.insert(findRoot(i));
        isolated += degree[i] == 0 ? 1 : 0;
    }

    if (m_verbose || components.size() > 1 || isolated > 0)
    {
        std::cout << "ISL spatial topology: " << linkCount << " links, max degree "
                  << maxNeighbors << ", " << components.size() << " connected components, "
                  << isolated << " isolated satellites" << std::endl;
    }

    return linkCount;
}

uint32_t
LeoSimChannelHelper::AddGridIslLinks(Ptr<LeoSimChannelModel> channelModel,
                                     NodeContainer satellites,
                                     uint32_t satellitesPerPlane,
                                     bool wrapPlanes)
{
    NS_LOG_FUNCTION(this << channelModel << satellites.GetN() << satellitesPerPlane
                         << wrapPlanes);

    const uint32_t numSatellites = satellites.GetN();
    if (!channelModel || satellitesPerPlane == 0 || numSatellites < 2)
    {
        return 0;
    }

    const uint32_t numPlanes = (numSatellites + satellitesPerPlane - 1) / satellitesPerPlane;
    std::set<std::pair<uint32_t, uint32_t>> addedPairs;
    uint32_t linkCount = 0;

    auto addCandidate = [&](uint32_t i, uint32_t j) {
        if (i >= numSatellites || j >= numSatellites || i == j)
        {
            return;
        }

        const uint32_t id1 = satellites.Get(i)->GetId();
        const uint32_t id2 = satellites.Get(j)->GetId();
        const auto pair = std::make_pair(std::min(id1, id2), std::max(id1, id2));
        if (addedPairs.find(pair) != addedPairs.end())
        {
            return;
        }

        Ptr<MobilityModel> mobility1 = satellites.Get(i)->GetObject<MobilityModel>();
        Ptr<MobilityModel> mobility2 = satellites.Get(j)->GetObject<MobilityModel>();
        if (!mobility1 || !mobility2 ||
            mobility1->GetDistanceFrom(mobility2) > m_islMaxDistance)
        {
            return;
        }

        addedPairs.insert(pair);
        channelModel->AddIslLink(satellites.Get(i), satellites.Get(j));
        linkCount++;
    };

    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        const uint32_t plane = i / satellitesPerPlane;
        const uint32_t slot = i % satellitesPerPlane;
        const uint32_t planeStart = plane * satellitesPerPlane;
        const uint32_t planeEnd = std::min(planeStart + satellitesPerPlane, numSatellites);
        const uint32_t satsInPlane = planeEnd - planeStart;

        if (satsInPlane > 1)
        {
            const uint32_t forwardSlot = (slot + 1) % satsInPlane;
            const uint32_t backwardSlot = (slot + satsInPlane - 1) % satsInPlane;
            addCandidate(i, planeStart + forwardSlot);
            addCandidate(i, planeStart + backwardSlot);
        }

        if (numPlanes > 1)
        {
            if (plane > 0)
            {
                addCandidate(i, (plane - 1) * satellitesPerPlane + slot);
            }
            else if (wrapPlanes)
            {
                addCandidate(i, (numPlanes - 1) * satellitesPerPlane + slot);
            }

            if (plane + 1 < numPlanes)
            {
                addCandidate(i, (plane + 1) * satellitesPerPlane + slot);
            }
            else if (wrapPlanes)
            {
                addCandidate(i, slot);
            }
        }
    }

    if (m_verbose)
    {
        NS_LOG_DEBUG("Added " << linkCount
                              << " initially in-range grid ISL links (same-plane forward/backward "
                                 "plus adjacent planes)");
    }

    return linkCount;
}

uint32_t
LeoSimChannelHelper::AddIslLink(Ptr<LeoSimChannelModel> channelModel, Ptr<Node> sat1, Ptr<Node> sat2)
{
    NS_LOG_FUNCTION(this << channelModel << sat1->GetId() << sat2->GetId());
    return channelModel->AddIslLink(sat1, sat2);
}

uint32_t
LeoSimChannelHelper::UpdateIslTopology(Ptr<LeoSimChannelModel> channelModel, NodeContainer satellites)
{
    NS_LOG_FUNCTION(this << channelModel << satellites.GetN());
    return channelModel->UpdateIslTopology(satellites, m_islMaxDistance);
}

void
LeoSimChannelHelper::SetIslMaxDistance(double distance)
{
    NS_LOG_FUNCTION(this << distance);
    m_islMaxDistance = distance;
}

void
LeoSimChannelHelper::SetIslTransmitPower(double power)
{
    NS_LOG_FUNCTION(this << power);
    m_islTransmitPower = power;
}

void
LeoSimChannelHelper::SetIslAntennaGain(double gain)
{
    NS_LOG_FUNCTION(this << gain);
    m_islAntennaGain = gain;
}

void
LeoSimChannelHelper::SetIslFrequency(double frequency)
{
    NS_LOG_FUNCTION(this << frequency);
    m_islFrequency = frequency;
}

} // namespace ns3
