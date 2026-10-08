/*
 * Copyright (c) 2026 Anupa De Silva
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

#include "leosim-beam-manager-helper.h"

#include "ns3/log.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimBeamManagerHelper");

LeoSimBeamManagerHelper::LeoSimBeamManagerHelper()
    : m_hoMode(LEOSIM_HO_MODE_CHO),
      m_earthFixedBeam(false),
      m_ttt(Seconds(1.0)),
      m_t310(Seconds(1.0)),
      m_n310(3),
      m_n311(3),
      m_a3Offset(3.0),
      m_a4Threshold(-110.0),
      m_elevationThreshold(10.0),
      m_tteThreshold(Seconds(30.0)),
      m_topsisWeights({0.18, 0.22, 0.14, 0.10, 0.08, 0.14, 0.00, 0.10, 0.14}),
      m_maxCandidates(3),
      m_choPreparationDelay(MilliSeconds(100)),
      m_choExecutionDelay(MilliSeconds(150)),
      m_updateInterval(MilliSeconds(100)),
      m_loadBalancingEnabled(false),
      m_handoverBufferingEnabled(true),
      m_maxHandoverBufferSize(1024),
      m_phasedArraySteeringInterval(MilliSeconds(100)),
      m_beamGeometryUpdateInterval(MilliSeconds(100)),
      m_sinrThresholdDb(-10.0),
      m_singleBestLinkMode(false),
      m_verbose(false)
{
    NS_LOG_FUNCTION(this);
    m_manager = CreateObject<LeoSimBeamManager>();
}

LeoSimBeamManagerHelper::~LeoSimBeamManagerHelper()
{
    NS_LOG_FUNCTION(this);
}

void
LeoSimBeamManagerHelper::SetChannelModel(Ptr<LeoSimChannelModel> channelModel)
{
    NS_LOG_FUNCTION(this << channelModel);
    m_channelModel = channelModel;
}

void
LeoSimBeamManagerHelper::SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel)
{
    NS_LOG_FUNCTION(this << islChannelModel);
    m_islChannelModel = islChannelModel;
}

void
LeoSimBeamManagerHelper::SetRoutingCalculator(Ptr<LeoSimRoutingCalculator> routingCalculator)
{
    NS_LOG_FUNCTION(this << routingCalculator);
    m_routingCalculator = routingCalculator;
}

void
LeoSimBeamManagerHelper::SetLoader(Ptr<LeoSimLoader> loader)
{
    NS_LOG_FUNCTION(this << loader);
    m_loader = loader;
}

void
LeoSimBeamManagerHelper::SetHandoverMode(LeoSimHandoverMode mode)
{
    NS_LOG_FUNCTION(this << mode);
    m_hoMode = mode;
}

void
LeoSimBeamManagerHelper::SetEarthFixedBeamMode(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_earthFixedBeam = enable;
}

void
LeoSimBeamManagerHelper::SetTtt(Time ttt)
{
    NS_LOG_FUNCTION(this << ttt);
    m_ttt = ttt;
}

void
LeoSimBeamManagerHelper::SetT310(Time t310)
{
    NS_LOG_FUNCTION(this << t310);
    m_t310 = t310;
}

void
LeoSimBeamManagerHelper::SetN310(uint32_t n310)
{
    NS_LOG_FUNCTION(this << n310);
    m_n310 = n310;
}

void
LeoSimBeamManagerHelper::SetN311(uint32_t n311)
{
    NS_LOG_FUNCTION(this << n311);
    m_n311 = n311;
}

void
LeoSimBeamManagerHelper::SetA3Offset(double offsetDb)
{
    NS_LOG_FUNCTION(this << offsetDb);
    m_a3Offset = offsetDb;
}

void
LeoSimBeamManagerHelper::SetA4Threshold(double thresholdDbm)
{
    NS_LOG_FUNCTION(this << thresholdDbm);
    m_a4Threshold = thresholdDbm;
}

void
LeoSimBeamManagerHelper::SetElevationThreshold(double elevationDeg)
{
    NS_LOG_FUNCTION(this << elevationDeg);
    m_elevationThreshold = elevationDeg;
}

void
LeoSimBeamManagerHelper::SetGroundNodeElevationThresholds(
    const std::map<uint32_t, double>& thresholds)
{
    m_groundNodeElevationThresholds = thresholds;
}

void
LeoSimBeamManagerHelper::SetTteThreshold(Time tte)
{
    NS_LOG_FUNCTION(this << tte);
    m_tteThreshold = tte;
}

void
LeoSimBeamManagerHelper::SetSinrThreshold(double sinrDb)
{
    NS_LOG_FUNCTION(this << sinrDb);
    m_sinrThresholdDb = sinrDb;
}

void
LeoSimBeamManagerHelper::SetTopsisWeights(double w1,
                                          double w2,
                                          double w3,
                                          double w4,
                                          double w5,
                                          double w6,
                                          double w7,
                                          double w8,
                                          double w9)
{
    NS_LOG_FUNCTION(this << w1 << w2 << w3 << w4 << w5 << w6 << w7 << w8 << w9);
    m_topsisWeights[0] = w1;
    m_topsisWeights[1] = w2;
    m_topsisWeights[2] = w3;
    m_topsisWeights[3] = w4;
    m_topsisWeights[4] = w5;
    m_topsisWeights[5] = w6;
    m_topsisWeights[6] = w7;
    m_topsisWeights[7] = w8;
    m_topsisWeights[8] = w9;
}

void
LeoSimBeamManagerHelper::SetMaxCandidates(uint32_t maxCandidates)
{
    NS_LOG_FUNCTION(this << maxCandidates);
    m_maxCandidates = maxCandidates;
}

void LeoSimBeamManagerHelper::SetSingleBestLinkMode(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_singleBestLinkMode = enable;
}


void
LeoSimBeamManagerHelper::SetChoPreparationDelay(Time delay)
{
    NS_LOG_FUNCTION(this << delay);
    m_choPreparationDelay = delay;
}

void
LeoSimBeamManagerHelper::SetChoExecutionDelay(Time delay)
{
    NS_LOG_FUNCTION(this << delay);
    m_choExecutionDelay = delay;
}

void
LeoSimBeamManagerHelper::SetUpdateInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_updateInterval = interval;
}

void
LeoSimBeamManagerHelper::SetGroundNodeActivationTimes(
    const std::map<uint32_t, Time>& activationTimes)
{
    m_groundNodeActivationTimes = activationTimes;
}

void
LeoSimBeamManagerHelper::EnableGroundNodeLifecycleLogging(const std::string& filename)
{
    m_groundNodeLifecycleFile = filename;
}

void
LeoSimBeamManagerHelper::EnableLoadBalancing(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_loadBalancingEnabled = enable;
}

void
LeoSimBeamManagerHelper::EnableHandoverBuffering(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_handoverBufferingEnabled = enable;
}

void
LeoSimBeamManagerHelper::SetMaxHandoverBufferSize(uint32_t maxPackets)
{
    NS_ABORT_MSG_IF(maxPackets == 0, "Handover buffer size must be positive");
    m_maxHandoverBufferSize = maxPackets;
}

void
LeoSimBeamManagerHelper::EnableFlowMonitorIntegration(Ptr<FlowMonitor> monitor,
                                                      Ptr<Ipv4FlowClassifier> classifier)
{
    NS_LOG_FUNCTION(this << monitor << classifier);
    m_flowMonitor = monitor;
    m_flowClassifier = classifier;
}

void
LeoSimBeamManagerHelper::EnableBeamLogging(std::string beamFile,
                                           std::string handoverFile,
                                           std::string choFile)
{
    NS_LOG_FUNCTION(this << beamFile << handoverFile << choFile);
    m_beamFile = beamFile;
    m_handoverFile = handoverFile;
    m_choFile = choFile;
}

void
LeoSimBeamManagerHelper::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
}

void
LeoSimBeamManagerHelper::SetMultiBeamModel(Ptr<LeoSimMultiBeamModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_multiBeamModel = model;
}

void
LeoSimBeamManagerHelper::SetPhasedArraySteeringInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_phasedArraySteeringInterval = interval;
}

void
LeoSimBeamManagerHelper::SetBeamGeometryUpdateInterval(Time interval)
{
    NS_LOG_FUNCTION(this << interval);
    m_beamGeometryUpdateInterval = interval;
}

Ptr<LeoSimBeamManager>
LeoSimBeamManagerHelper::Install(NodeContainer groundNodes,
                                  NodeContainer satNodes,
                                  Time simTime)
{
    NS_LOG_FUNCTION(this << groundNodes.GetN() << " ground nodes, " << satNodes.GetN()
                         << " satellites");

    // Create a fresh beam manager instance
    m_manager = CreateObject<LeoSimBeamManager>();

    // Apply all configured parameters to the manager
    if (m_channelModel)
    {
        m_manager->SetChannelModel(m_channelModel);
    }

    if (m_islChannelModel)
    {
        m_manager->SetIslChannelModel(m_islChannelModel);
    }

    if (m_routingCalculator)
    {
        m_manager->SetRoutingCalculator(m_routingCalculator);
    }

    if (m_loader)
    {
        m_manager->SetLoader(m_loader);
    }

    m_manager->SetHandoverMode(m_hoMode);
    m_manager->SetBeamMode(m_earthFixedBeam);

    m_manager->SetTttDuration(m_ttt);
    m_manager->SetT310Duration(m_t310);
    m_manager->SetN310Count(m_n310);
    m_manager->SetN311Count(m_n311);

    m_manager->SetA3Offset(m_a3Offset);
    m_manager->SetA4Threshold(m_a4Threshold);
    m_manager->SetElevationThreshold(m_elevationThreshold);
    for (const auto& [groundNodeId, threshold] : m_groundNodeElevationThresholds)
    {
        m_manager->SetGroundNodeElevationThreshold(groundNodeId, threshold);
    }
    m_manager->SetTteThreshold(m_tteThreshold);
    m_manager->SetSinrThresholdDb(m_sinrThresholdDb);

    m_manager->SetTopsisWeights(m_topsisWeights[0],
                                 m_topsisWeights[1],
                                 m_topsisWeights[2],
                                 m_topsisWeights[3],
                                 m_topsisWeights[4],
                                 m_topsisWeights[5],
                                 m_topsisWeights[6],
                                 m_topsisWeights[7],
                                 m_topsisWeights[8]);
    m_manager->SetMaxCandidates(m_maxCandidates);
    m_manager->SetSingleBestLinkMode(m_singleBestLinkMode);
    m_manager->SetUpdateInterval(m_updateInterval);
    m_manager->SetGroundNodeActivationTimes(m_groundNodeActivationTimes);
    if (!m_groundNodeLifecycleFile.empty())
    {
        m_manager->EnableGroundNodeLifecycleLogging(m_groundNodeLifecycleFile);
    }

    m_manager->SetChoPreparationDelay(m_choPreparationDelay);
    m_manager->SetChoExecutionDelay(m_choExecutionDelay);

    m_manager->EnableLoadBalancing(m_loadBalancingEnabled);
    m_manager->EnableHandoverBuffering(m_handoverBufferingEnabled);
    m_manager->SetMaxBufferSize(m_maxHandoverBufferSize);

    if (m_flowMonitor && m_flowClassifier)
    {
        m_manager->SetFlowMonitor(m_flowMonitor);
        m_manager->SetFlowClassifier(m_flowClassifier);
    }

    m_manager->SetVerbose(m_verbose);

    // Phased array / multi-beam model
    if (m_multiBeamModel)
    {
        m_manager->SetMultiBeamModel(m_multiBeamModel);
        m_manager->SetPhasedArraySteeringInterval(m_phasedArraySteeringInterval);
        m_manager->SetBeamGeometryUpdateInterval(m_beamGeometryUpdateInterval);
    }

    // Start the beam manager with the provided nodes and simulation time
    m_manager->Start(groundNodes, satNodes, Seconds(0.0), simTime);

    NS_LOG_DEBUG("LeoSimBeamManager installed with "
                << groundNodes.GetN() << " ground nodes and " << satNodes.GetN()
                << " satellites, "
                << "simulation duration: " << simTime.GetSeconds() << "s");

    return m_manager;
}

} // namespace ns3
