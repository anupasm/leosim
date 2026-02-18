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

#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/simulator.h"

#include <cmath>
#include <iomanip>
#include <iostream>
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

    // Create links between satellites and ground nodes only when in contact
    uint32_t linkCount = 0;
    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        for (uint32_t j = 0; j < groundNodes.GetN(); ++j)
        {
            Ptr<Node> sat = satellites.Get(i);
            Ptr<Node> ground = groundNodes.Get(j);

            channelModel->AddLink(sat, ground);
            linkCount++;
        }
    }

    if (m_verbose)
    {
        NS_LOG_INFO("Created " << linkCount << " satellite-to-ground links (in contact only)");
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

    // Create links between all satellites and all gateways
    uint32_t linkCount = 0;
    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        for (uint32_t j = 0; j < gateways.GetN(); ++j)
        {
            channelModel->AddLink(satellites.Get(i), gateways.Get(j));
            linkCount++;
        }
    }

    if (m_verbose)
    {
        NS_LOG_INFO("Created " << linkCount << " satellite-to-gateway links");
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

    // Create links between all satellites and all UEs
    uint32_t linkCount = 0;
    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        for (uint32_t j = 0; j < ues.GetN(); ++j)
        {
            channelModel->AddLink(satellites.Get(i), ues.Get(j));
            linkCount++;
        }
    }

    if (m_verbose)
    {
        NS_LOG_INFO("Created " << linkCount << " satellite-to-UE links");
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

    uint32_t linkCount = 0;
    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        for (uint32_t j = 0; j < groundNodes.GetN(); ++j)
        {
            channelModel->AddLink(satellites.Get(i), groundNodes.Get(j));
            linkCount++;
        }
    }

    if (m_verbose)
    {
        NS_LOG_INFO("Added " << linkCount << " links to existing channel model");
    }
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
        NS_LOG_INFO("Configured channel model: " << "MinElev=" << m_minElevationAngle << "°, "
                                                 << "MaxDist=" << m_maxLinkDistance / 1000.0
                                                 << "km, "
                                                 << "Freq=" << m_frequency / 1e9 << "GHz, "
                                                 << "TxPower=" << m_transmitPower << "dBm, "
                                                 << "TxGain=" << m_txAntennaGain << "dB, "
                                                 << "RxGain=" << m_rxAntennaGain << "dB, "
                                                 << "NoiseBW=" << m_noiseBandwidth / 1e6 << "MHz");
        NS_LOG_INFO("ISL parameters: MaxDist=" << m_islMaxDistance / 1000.0 << "km, "
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
LeoSimChannelHelper::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
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

    NS_LOG_INFO(msg);

    for (const auto& link : activeLinks)
    {
        LeoSimChannelQuality quality = channelModel->GetChannelQuality(link.first, link.second);
        NS_LOG_INFO("  Link " << link.first->GetId() << "->" << link.second->GetId()
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
        NS_LOG_INFO("Created ISL mesh with " << linkCount << " links between " 
                   << satellites.GetN() << " satellites");
    }

    return channelModel;
}

uint32_t
LeoSimChannelHelper::AddIslLinks(Ptr<LeoSimChannelModel> channelModel, NodeContainer satellites)
{
    NS_LOG_FUNCTION(this << channelModel << satellites.GetN());

    uint32_t linkCount = channelModel->CreateIslMesh(satellites);

    if (m_verbose)
    {
        NS_LOG_INFO("Added " << linkCount << " ISL links to existing channel model");
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
