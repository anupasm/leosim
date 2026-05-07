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

#include "leosim-beam-manager.h"

#include "leosim-channel-model.h"
#include "leosim-beam-layout-engine.h"
#include "leosim-loader.h"
#include "leosim-routing-calculator.h"

#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node.h"
#include "ns3/node-list.h"
#include "ns3/ipv4.h"
#include "ns3/ipv4-interface.h"

#include "leosim-mobility-model.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <numeric>

namespace ns3
{

namespace
{
uint32_t
GetLeoSimIdFromNodeId(uint32_t nodeId)
{
    Ptr<Node> node = NodeList::GetNode(nodeId);
    if (!node)
    {
        return nodeId;
    }

    Ptr<LeoSimMobilityModel> mobility = node->GetObject<LeoSimMobilityModel>();
    if (!mobility)
    {
        return nodeId;
    }

    return mobility->GetNodeId();
}
} // namespace

NS_LOG_COMPONENT_DEFINE("LeoSimBeamManager");
NS_OBJECT_ENSURE_REGISTERED(LeoSimBeamManager);

TypeId
LeoSimBeamManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimBeamManager")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimBeamManager>();
    return tid;
}

void
LeoSimBeamManager::SetHandoverCallback(Callback<void, LeoSimHandoverEvent> callback)
{
    NS_LOG_FUNCTION(this);
    m_handoverCallback = callback;
}

void
LeoSimBeamManager::SetBeamStateCallback(Callback<void, uint32_t, LeoSimBeamRecord, double> callback)
{
    NS_LOG_FUNCTION(this);
    m_beamStateCallback = callback;

    // If the manager already started and has current serving beams,
    // immediately publish them so late subscribers (e.g., visualization helper)
    // can log an initial state.
    if (!m_beamStateCallback.IsNull() && !m_currentBeams.empty())
    {
        for (const auto& [ueNodeId, beam] : m_currentBeams)
        {
            m_beamStateCallback(ueNodeId, beam, 0.0);
        }
    }
}

void
LeoSimBeamManager::SetChoConfigCallback(
    Callback<void, uint32_t, uint32_t, std::vector<LeoSimTopsisCandidate>> callback)
{
    NS_LOG_FUNCTION(this);
    m_choConfigCallback = callback;
}

// ============================================================================
// Dependency Injection Setters
// ============================================================================

void
LeoSimBeamManager::SetChannelModel(Ptr<LeoSimChannelModel> channelModel)
{
    NS_LOG_FUNCTION(this << channelModel);
    m_channelModel = channelModel;
}

void
LeoSimBeamManager::SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel)
{
    NS_LOG_FUNCTION(this << islChannelModel);
    m_islChannelModel = islChannelModel;
}

void
LeoSimBeamManager::SetRoutingCalculator(Ptr<LeoSimRoutingCalculator> routingCalculator)
{
    NS_LOG_FUNCTION(this << routingCalculator);
    m_routingCalculator = routingCalculator;
}

void
LeoSimBeamManager::SetLoader(Ptr<LeoSimLoader> loader)
{
    NS_LOG_FUNCTION(this << loader);
    m_loader = loader;
}

void
LeoSimBeamManager::SetOperatorModel(Ptr<LeoSimOperatorModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_operatorModel = model;
}

void
LeoSimBeamManager::SetWeatherModel(Ptr<LeoSimWeatherModel> model)
{
    NS_LOG_FUNCTION(this << model);
    m_weatherModel = model;
}

void
LeoSimBeamManager::SetVerbose(bool verbose)
{
    NS_LOG_FUNCTION(this << verbose);
    m_verbose = verbose;
}

// ============================================================================
// Handover Mode Configuration
// ============================================================================

void
LeoSimBeamManager::SetHandoverMode(LeoSimHandoverMode mode)
{
    NS_LOG_FUNCTION(this << mode);
    m_hoMode = mode;
    NS_LOG_DEBUG("Handover mode set to "
                 << (m_hoMode == LEOSIM_HO_MODE_BHO ? "BHO (reactive)" : "CHO (conditional)"));
}

void
LeoSimBeamManager::SetBeamMode(bool earthFixed)
{
    NS_LOG_FUNCTION(this << earthFixed);
    m_earthFixedBeam = earthFixed;
    NS_LOG_DEBUG("Beam mode set to " << (earthFixed ? "earth-fixed" : "body-fixed"));
}

// ============================================================================
// 3GPP Timer Configuration
// ============================================================================

void
LeoSimBeamManager::SetTttDuration(Time ttt)
{
    NS_LOG_FUNCTION(this << ttt);
    m_ttt = ttt;
    NS_LOG_DEBUG("TTT duration set to " << ttt.GetSeconds() << " seconds");
}

void
LeoSimBeamManager::SetT310Duration(Time t310)
{
    NS_LOG_FUNCTION(this << t310);
    m_t310 = t310;
    NS_LOG_DEBUG("T310 duration set to " << t310.GetSeconds() << " seconds");
}

void
LeoSimBeamManager::SetN310Count(uint32_t n310)
{
    NS_LOG_FUNCTION(this << n310);
    m_n310 = n310;
    NS_LOG_DEBUG("N310 counter set to " << n310);
}

void
LeoSimBeamManager::SetN311Count(uint32_t n311)
{
    NS_LOG_FUNCTION(this << n311);
    m_n311 = n311;
    NS_LOG_DEBUG("N311 counter set to " << n311);
}

// ============================================================================
// Handover Threshold Configuration
// ============================================================================

void
LeoSimBeamManager::SetA3Offset(double offset)
{
    NS_LOG_FUNCTION(this << offset);
    m_a3Offset = offset;
    NS_LOG_DEBUG("A3 offset set to " << offset << " dB");
}

void
LeoSimBeamManager::SetA4Threshold(double threshold)
{
    NS_LOG_FUNCTION(this << threshold);
    m_a4Threshold = threshold;
    NS_LOG_DEBUG("A4 threshold set to " << threshold << " dBm");
}

void
LeoSimBeamManager::SetElevationThreshold(double threshold)
{
    NS_LOG_FUNCTION(this << threshold);
    m_elevationThreshold = threshold;
    NS_LOG_DEBUG("Elevation threshold set to " << threshold << " degrees");
}

void
LeoSimBeamManager::SetTteThreshold(Time threshold)
{
    NS_LOG_FUNCTION(this << threshold);
    m_tteThreshold = threshold;
    NS_LOG_DEBUG("TTE threshold set to " << threshold.GetSeconds() << " seconds");
}

void
LeoSimBeamManager::SetWeatherFadeThresholdDb(double thresholdDb)
{
    NS_LOG_FUNCTION(this << thresholdDb);
    m_weatherFadeThresholdDb = thresholdDb;
    NS_LOG_DEBUG("Weather fade threshold set to " << thresholdDb << " dB");
}

// ============================================================================
// TOPSIS Multi-Criteria Configuration
// ============================================================================

void
LeoSimBeamManager::SetTopsisWeights(double wRsrp,
                                     double wSinr,
                                     double wTte,
                                     double wLoad,
                                     double wLatency,
                                     double wElevation,
                                     double wActive,
                                     double wOperatorCompat,
                                     double wWeather)
{
    NS_LOG_FUNCTION(this << wRsrp << wSinr << wTte << wLoad << wLatency << wElevation
                         << wActive << wOperatorCompat << wWeather);
    m_topsisWeights[0] = wRsrp;
    m_topsisWeights[1] = wSinr;
    m_topsisWeights[2] = wTte;
    m_topsisWeights[3] = wLoad;
    m_topsisWeights[4] = wLatency;
    m_topsisWeights[5] = wElevation;
    m_topsisWeights[6] = wActive;
    m_topsisWeights[7] = wOperatorCompat;
    m_topsisWeights[8] = wWeather;
    
    NS_LOG_DEBUG("TOPSIS weights: RSRP=" << wRsrp << " SINR=" << wSinr << " TTE=" << wTte
                                         << " Load=" << wLoad << " Latency=" << wLatency
                                         << " Elevation=" << wElevation << " Active="
                                         << wActive << " OperatorCompat=" << wOperatorCompat
                                         << " Weather=" << wWeather);
}

void
LeoSimBeamManager::SetMaxCandidates(uint32_t maxCandidates)
{
    NS_LOG_FUNCTION(this << maxCandidates);
    m_maxCandidates = maxCandidates;
    NS_LOG_DEBUG("Maximum CHO candidates set to " << maxCandidates);
}

// ============================================================================
// CHO Phase Timing
// ============================================================================

void
LeoSimBeamManager::SetChoPreparationDelay(Time delay)
{
    NS_LOG_FUNCTION(this << delay);
    m_prepDelay = delay;
    NS_LOG_DEBUG("CHO preparation delay set to " << delay.GetMilliSeconds() << " ms");
}

void
LeoSimBeamManager::SetChoExecutionDelay(Time delay)
{
    NS_LOG_FUNCTION(this << delay);
    m_execDelay = delay;
    NS_LOG_DEBUG("CHO execution delay set to " << delay.GetMilliSeconds() << " ms");
}

// ============================================================================
// Load Balancing and Buffering
// ============================================================================

void
LeoSimBeamManager::EnableLoadBalancing(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_loadBalancingEnabled = enable;
    NS_LOG_DEBUG("Load balancing " << (enable ? "enabled" : "disabled"));
}

void
LeoSimBeamManager::SetLoadImbalanceThreshold(uint32_t threshold)
{
    NS_LOG_FUNCTION(this << threshold);
    m_loadImbalanceThreshold = threshold;
    NS_LOG_DEBUG("Load imbalance threshold set to " << threshold << " UEs");
}

void
LeoSimBeamManager::EnableHandoverBuffering(bool enable)
{
    NS_LOG_FUNCTION(this << enable);
    m_handoverBufferingEnabled = enable;
    NS_LOG_DEBUG("Handover buffering " << (enable ? "enabled" : "disabled"));
}

void
LeoSimBeamManager::SetMaxBufferSize(uint32_t maxSize)
{
    NS_LOG_FUNCTION(this << maxSize);
    m_maxBufferSize = maxSize;
    NS_LOG_DEBUG("Maximum buffer size set to " << maxSize << " packets");
}

void
LeoSimBeamManager::RunT310Evaluation(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);

    // Retrieve the current beam record for this UE
    auto beamIt = m_currentBeams.find(ueNodeId);
    if (beamIt == m_currentBeams.end())
    {
        NS_LOG_WARN("No current beam record for UE " << ueNodeId << ", skipping T310 evaluation");
        return;
    }

    const LeoSimBeamRecord& rec = beamIt->second;

    // Get the current link state from the channel model
    LeoSimLinkState linkState = m_channelModel->GetLinkState(ueNodeId, rec.satelliteNodeId);

    NS_LOG_DEBUG("T310 evaluation for UE " << ueNodeId << ": linkState=" << linkState
                                          << ", rsrp=" << rec.rsrp << " dBm, threshold="
                                          << (m_a4Threshold - 10.0) << " dBm");

    // Check for RLF conditions: link is DOWN OR RSRP falls below A4 threshold minus 10 dB hysteresis
    bool linkFailed = (linkState == LEOSIM_LINK_DOWN) || (rec.rsrp < (m_a4Threshold - 10.0));

    if (linkFailed)
    {
        // Link failure condition detected
        uint32_t& t310Counter = m_t310Counter[ueNodeId];
        t310Counter++;

        NS_LOG_DEBUG("RLF condition detected for UE " << ueNodeId << ", T310 counter incremented to "
                                                      << t310Counter);

        // Check if counter has reached the RLF threshold
        if (t310Counter >= m_n310)
        {
            NS_LOG_WARN("RLF declared for UE " << ueNodeId << " (T310 counter=" << t310Counter
                                               << " >= N310=" << m_n310 << ")");

            // Get ranked candidates and initiate CHO preparation
            std::vector<LeoSimBeamRecord> visibleBeams = ScanVisibleSatellites(ueNodeId);
            std::vector<LeoSimTopsisCandidate> rankedCandidates =
                RankByTopsis(visibleBeams, ueNodeId);

            if (!rankedCandidates.empty())
            {
                // Initiate CHO preparation with the top candidate
                uint32_t topCandidateSatId = rankedCandidates[0].beamRecord.satelliteNodeId;
                InitiateChoPreparation(ueNodeId, topCandidateSatId);

                NS_LOG_INFO("RLF recovery via CHO initiated for UE " << ueNodeId
                                                                     << " to satellite "
                                                                     << topCandidateSatId);
            }
            else
            {
                NS_LOG_WARN("RLF declared for UE " << ueNodeId
                            << " but no handover candidates available");
            }
        }
    }
    else
    {
        // Link is healthy - apply N311 in-sync recovery per 3GPP TS 38.321 §5.3.10
        auto counterIt = m_t310Counter.find(ueNodeId);
        if (counterIt != m_t310Counter.end() && counterIt->second > 0)
        {
            counterIt->second--;
            NS_LOG_DEBUG("Link healthy for UE " << ueNodeId << ", T310 counter decremented to "
                                                 << counterIt->second);
        }
    }
}

void
LeoSimBeamManager::RunTttEvaluation(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);

    // Get current beam record for the UE
    auto beamIt = m_currentBeams.find(ueNodeId);
    if (beamIt == m_currentBeams.end())
    {
        NS_LOG_WARN("No current beam record for UE " << ueNodeId << ", skipping TTT evaluation");
        return;
    }
    // Refresh serving-beam metrics from the channel model before comparing.
    // This keeps A3/A4 decisions aligned with dynamic channel updates.
    if (m_channelModel)
    {
        LeoSimChannelQuality servingQual =
            m_channelModel->GetLinkQuality(ueNodeId, beamIt->second.satelliteNodeId);
        beamIt->second.rsrp = servingQual.signalStrength;
        beamIt->second.snr = servingQual.snr;
        beamIt->second.pathLoss = servingQual.pathLoss;
        beamIt->second.elevationAngle = servingQual.elevationAngle;
    }

    const LeoSimBeamRecord& servingBeam = beamIt->second;

    // Get visible satellites and rank them by TOPSIS
    std::vector<LeoSimBeamRecord> visibleBeams = ScanVisibleSatellites(ueNodeId);
    std::vector<LeoSimTopsisCandidate> rankedCandidates =
        RankByTopsis(visibleBeams, ueNodeId);

    // Return early if no candidates are available
    if (rankedCandidates.empty())
    {
        NS_LOG_DEBUG("No handover candidates for UE " << ueNodeId);

        // Cancel TTT timer if currently running
        auto eventIt = m_tttEventIds.find(ueNodeId);
        if (eventIt != m_tttEventIds.end())
        {
            Simulator::Cancel(eventIt->second);
            m_tttEventIds.erase(eventIt);
            NS_LOG_INFO("TTT timer cancelled for UE " << ueNodeId << " at "
                        << Simulator::Now().GetSeconds() << "s (no candidates)");
        }
        return;
    }

    // Pick the best *neighbor* (excluding the serving satellite). The ranking output can
    // legitimately put the serving satellite first, which would otherwise suppress A3/A4.
    uint32_t servingSatId = servingBeam.satelliteNodeId;
    auto bestNeighborIt = std::find_if(
        rankedCandidates.begin(),
        rankedCandidates.end(),
        [servingSatId](const LeoSimTopsisCandidate& c) {
            return c.beamRecord.satelliteNodeId != servingSatId;
        });

    if (bestNeighborIt == rankedCandidates.end())
    {
        NS_LOG_DEBUG("No neighbor candidates (only serving satellite visible) for UE " << ueNodeId);
        return;
    }

    const LeoSimTopsisCandidate& bestCandidate = *bestNeighborIt;
    double bestRsrp = bestCandidate.beamRecord.rsrp;
    uint32_t bestSatId = bestCandidate.beamRecord.satelliteNodeId;
    // Evaluate A3 condition: best candidate RSRP > serving RSRP + offset AND different satellite
    bool a3Condition = (bestRsrp > servingBeam.rsrp + m_a3Offset) && (bestSatId != servingSatId);

    // Evaluate A4 condition: best candidate RSRP > absolute threshold AND A3 holds
    bool a4Condition = (bestRsrp > m_a4Threshold) && a3Condition;

    if (m_verbose)
    {
        std::cout << "[HO-CHECK][" << Simulator::Now().GetSeconds() << "s] UE " << ueNodeId
                  << " TTT eval: serving=" << servingSatId
                  << " neigh=" << bestSatId
                  << " bestRsrp=" << bestRsrp << "dBm"
                  << " servingRsrp=" << servingBeam.rsrp << "dBm"
                  << " A3=" << (a3Condition ? "1" : "0")
                  << " A4=" << (a4Condition ? "1" : "0")
                  << std::endl;
    }

    NS_LOG_DEBUG("TTT evaluation for UE " << ueNodeId << ": A3=" << a3Condition << " (best=" << bestRsrp
                                          << " > serving=" << servingBeam.rsrp << " + offset="
                                          << m_a3Offset << "), A4=" << a4Condition);

    // If either A3 or A4 condition is true and TTT timer is not running: start TTT
    if ((a3Condition || a4Condition) && m_tttEventIds.find(ueNodeId) == m_tttEventIds.end())
    {
        // Schedule TTT expiry callback using lambda
        EventId eventId = Simulator::Schedule(m_ttt, [this, ueNodeId, bestSatId]() {
            NS_LOG_DEBUG("TTT expired for UE " << ueNodeId << " at "
                        << Simulator::Now().GetSeconds() << "s");

            // Re-evaluate A3 condition at TTT expiry
            auto beamIt = m_currentBeams.find(ueNodeId);
            if (beamIt != m_currentBeams.end())
            {
                std::vector<LeoSimBeamRecord> visibleBeams = ScanVisibleSatellites(ueNodeId);
                std::vector<LeoSimTopsisCandidate> candidates = RankByTopsis(visibleBeams, ueNodeId);

                if (!candidates.empty())
                {
                    const LeoSimBeamRecord& servingBeamCur = beamIt->second;
                    const LeoSimTopsisCandidate& bestCandCur = candidates[0];

                    // Re-evaluate A3: best > serving + offset AND different satellite
                    bool a3StillHolds = (bestCandCur.beamRecord.rsrp >
                                         servingBeamCur.rsrp + m_a3Offset) &&
                                        (bestCandCur.beamRecord.satelliteNodeId !=
                                         servingBeamCur.satelliteNodeId);

                    if (a3StillHolds)
                    {
                        NS_LOG_DEBUG("A3 condition still holds after TTT expiry for UE "
                                    << ueNodeId << ", proceeding with CHO preparation");
                        InitiateChoPreparation(ueNodeId, bestCandCur.beamRecord.satelliteNodeId);
                    }
                    else
                    {
                        NS_LOG_DEBUG("A3 condition no longer holds after TTT expiry for UE "
                                    << ueNodeId << ", aborting CHO");
                    }
                }
            }

            // Erase timer entry
            m_tttEventIds.erase(ueNodeId);
        });

        m_tttEventIds[ueNodeId] = eventId;
        if (m_verbose)
        {
            std::cout << "[HO-CHECK][" << Simulator::Now().GetSeconds() << "s] UE " << ueNodeId
                      << " TTT started: duration=" << m_ttt.GetSeconds()
                      << "s targetSat=" << bestSatId << std::endl;
        }
        NS_LOG_INFO("TTT timer started for UE " << ueNodeId << " at "
                    << Simulator::Now().GetSeconds() << "s (duration: " << m_ttt.GetSeconds()
                    << "s, target satellite: " << bestSatId << ")");
    }
    // If neither condition holds AND TTT timer is currently running: cancel it
    else if (!(a3Condition || a4Condition) && m_tttEventIds.find(ueNodeId) != m_tttEventIds.end())
    {
        Simulator::Cancel(m_tttEventIds[ueNodeId]);
        m_tttEventIds.erase(ueNodeId);
        if (m_verbose)
        {
            std::cout << "[HO-CHECK][" << Simulator::Now().GetSeconds() << "s] UE " << ueNodeId
                      << " TTT cancelled (conditions no longer hold)" << std::endl;
        }
        NS_LOG_INFO("TTT timer cancelled for UE " << ueNodeId << " at "
                    << Simulator::Now().GetSeconds()
                    << "s (measurement conditions no longer hold)");
    }
}

double
LeoSimBeamManager::ComputeTte(uint32_t ueNodeId, uint32_t satNodeId) const
{
    NS_LOG_FUNCTION(this << ueNodeId << satNodeId);

    // Check for necessary dependencies
    if (!m_loader)
    {
        NS_LOG_WARN("Loader not configured, cannot compute TTE");
        return 0.0;
    }

    // Translate ns-3 NodeIds to LeoSim loader IDs.
    uint32_t ueDeviceId = GetLeoSimIdFromNodeId(ueNodeId);
    uint32_t satId = GetLeoSimIdFromNodeId(satNodeId);

    // Get UE ground position
    Vector uePos = m_loader->GetGroundDevicePosition(ueDeviceId);
    NS_LOG_DEBUG("UE " << ueNodeId << " position: " << uePos);

    // Start from current simulation time
    double currentTime = Simulator::Now().GetSeconds();
    double lastValidTime = 0.0;
    bool wasAboveThreshold = true;

    // Check current elevation angle
    Vector currentSatPos = m_loader->GetSatellitePositionAt(satId, Seconds(currentTime));
    
    // Compute current elevation angle
    Vector los = currentSatPos - uePos;
    double losDistance = los.GetLength();
    double ueDistance = uePos.GetLength();

    double currentElevation = 0.0;
    if (losDistance > 0 && ueDistance > 0)
    {
        // Normalize vectors manually
        Vector up = uePos;
        double upMag = sqrt(up.x*up.x + up.y*up.y + up.z*up.z);
        up.x /= upMag;
        up.y /= upMag;
        up.z /= upMag;

        los.x /= losDistance;
        los.y /= losDistance;
        los.z /= losDistance;

        double cosZenith = los.x*up.x + los.y*up.y + los.z*up.z;
        cosZenith = std::min(1.0, std::max(-1.0, cosZenith));
        
        double zenithAngle = std::acos(cosZenith);
        currentElevation = (M_PI / 2.0 - zenithAngle) * 180.0 / M_PI;
    }

    NS_LOG_DEBUG("Current elevation angle for satellite " << satNodeId << " as seen from UE "
                                                           << ueNodeId << ": "
                                                           << currentElevation << "°");

    // If already below threshold, return 0
    if (currentElevation < m_elevationThreshold)
    {
        NS_LOG_DEBUG("Satellite " << satNodeId << " already below elevation threshold "
                                  << m_elevationThreshold << "°");
        return 0.0;
    }

    // Step forward in 1-second increments up to 600 seconds
    const double TIME_STEP = 1.0;
    const double MAX_HORIZON = 600.0;

    for (double dt = TIME_STEP; dt <= MAX_HORIZON; dt += TIME_STEP)
    {
        double futureTime = currentTime + dt;
        Vector futureSatPos = m_loader->GetSatellitePositionAt(satId, Seconds(futureTime));

        // Compute elevation angle at future time
        Vector futureLos = futureSatPos - uePos;
        double futureLosDistance = futureLos.GetLength();

        double futureElevation = 0.0;
        if (futureLosDistance > 0 && ueDistance > 0)
        {
            // Normalize vectors manually
            Vector up = uePos;
            double upMag = sqrt(up.x*up.x + up.y*up.y + up.z*up.z);
            up.x /= upMag;
            up.y /= upMag;
            up.z /= upMag;

            futureLos.x /= futureLosDistance;
            futureLos.y /= futureLosDistance;
            futureLos.z /= futureLosDistance;

            double cosZenith = futureLos.x*up.x + futureLos.y*up.y + futureLos.z*up.z;
            cosZenith = std::min(1.0, std::max(-1.0, cosZenith));
            
            double zenithAngle = std::acos(cosZenith);
            futureElevation = (M_PI / 2.0 - zenithAngle) * 180.0 / M_PI;
        }

        // Check if elevation just dropped below threshold
        if (wasAboveThreshold && futureElevation < m_elevationThreshold)
        {
            NS_LOG_DEBUG("Satellite " << satNodeId << " will drop below elevation threshold "
                                      << m_elevationThreshold << "° at TTE=" << lastValidTime
                                      << "s");
            return lastValidTime;
        }

        // Update tracking
        if (futureElevation >= m_elevationThreshold)
        {
            lastValidTime = dt;
            wasAboveThreshold = true;
        }
        else
        {
            wasAboveThreshold = false;
        }
    }

    // If we reach here, satellite remains above threshold for the entire horizon
    NS_LOG_DEBUG("Satellite " << satNodeId << " remains above threshold throughout horizon");
    return MAX_HORIZON;
}

std::vector<LeoSimBeamRecord>
LeoSimBeamManager::ScanVisibleSatellites(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);
    std::vector<LeoSimBeamRecord> visibleBeams;

    if (!m_loader || !m_channelModel || !m_multiBeamModel || !m_sinrEngine)
    {
        NS_LOG_WARN("Loader, channel model, multi-beam model, or SINR engine not configured");
        return visibleBeams;
    }

    // Iterate through the actual satellite nodes and check link quality with the UE.
    for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
    {
        Ptr<Node> satNode = m_satellites.Get(i);
        if (!satNode)
        {
            continue;
        }

        uint32_t satNodeId = satNode->GetId();

        // Get link quality between UE and this satellite
        LeoSimChannelQuality linkQuality = m_channelModel->GetLinkQuality(ueNodeId, satNodeId);

        // Check link state (must be UP or DEGRADED for visibility)
        LeoSimLinkState state = m_channelModel->GetLinkState(ueNodeId, satNodeId);

        if ((state != LEOSIM_LINK_UP && state != LEOSIM_LINK_DEGRADED) ||
            linkQuality.signalStrength < m_a4Threshold)
        {
            continue;
        }

        // Check for ping-pong pattern between UEs
        // Get previous serving satellite (if any)
        auto beamIt = m_currentBeams.find(ueNodeId);
        if (beamIt != m_currentBeams.end() && IsPingPong(ueNodeId, beamIt->second.satelliteNodeId, satNodeId))
        {
            NS_LOG_DEBUG("Ping-pong detected for UE " << ueNodeId << " between satellites "
                                                      << beamIt->second.satelliteNodeId << " and "
                                                      << satNodeId);
            continue;
        }

        // Compute Time-To-Exit and latency once per satellite (not per-beam)
        double tte = ComputeTte(ueNodeId, satNodeId);
        double latency = ComputeEndToEndLatency(ueNodeId, satNodeId);

        // Get UE position for SINR computation
        Vector uePosVec;
        Ptr<Node> ueNode = NodeList::GetNode(ueNodeId);
        if (ueNode && ueNode->GetObject<LeoSimMobilityModel>())
        {
            uePosVec = ueNode->GetObject<LeoSimMobilityModel>()->GetPosition();
        }
        double ueLat = 0.0, ueLon = 0.0;
        if (m_loader)
        {
            auto latLon = m_loader->GetGroundDeviceLatLon(ueNodeId);
            ueLat = latLon.first;
            ueLon = latLon.second;
        }

        // Get all beams map for SINR computation
        std::map<uint32_t, std::vector<LeoSimSpotBeam>> allBeamsMap;
        for (uint32_t i = 0; i < m_satellites.GetN(); ++i)
        {
            Ptr<Node> sat = m_satellites.Get(i);
            if (sat)
            {
                allBeamsMap[sat->GetId()] = m_multiBeamModel->GetBeamsForSatellite(sat->GetId());
            }
        }

        // Get beam list for this satellite
        const auto& beams = m_multiBeamModel->GetBeamsForSatellite(satNodeId);
        std::vector<std::pair<size_t, double>> beamSinrPairs; // (beam index, SINR)

        for (size_t beamIdx = 0; beamIdx < beams.size(); ++beamIdx)
        {
            const auto& beam = beams[beamIdx];

            // Compute SINR for this beam using the SINR engine
            LeoSimSinrResult sinrResult = m_sinrEngine->ComputeSinr(
                ueNodeId, satNodeId, beam.beamId, allBeamsMap, ueLat, ueLon);
            double sinr = sinrResult.sinr_dB;

            // Store (beam index, SINR) for potential filtering
            beamSinrPairs.push_back({beamIdx, sinr});
        }

        // Filter beams: if hopping is disabled, keep only top-3 by SINR
        std::vector<size_t> selectedBeamIndices;
        if (m_cfg.beamHoppingEnabled)
        {
            // Hopping enabled: include all active beams
            for (size_t beamIdx = 0; beamIdx < beams.size(); ++beamIdx)
            {
                selectedBeamIndices.push_back(beamIdx);
            }
        }
        else
        {
            // Hopping disabled: select top-3 by SINR
            std::sort(beamSinrPairs.begin(), beamSinrPairs.end(),
                      [](const auto& a, const auto& b) { return a.second > b.second; });
            for (size_t j = 0; j < std::min(size_t(3), beamSinrPairs.size()); ++j)
            {
                selectedBeamIndices.push_back(beamSinrPairs[j].first);
            }
        }

        // Generate one record per selected beam
        for (size_t beamIdx : selectedBeamIndices)
        {
            const auto& beam = beams[beamIdx];

            // Compute SINR for this beam
            LeoSimSinrResult sinrResult = m_sinrEngine->ComputeSinr(
                ueNodeId, satNodeId, beam.beamId, allBeamsMap, ueLat, ueLon);
            double sinr = sinrResult.sinr_dB;

            // Count active UEs on this specific beam
            uint32_t beamActiveUeCount = 0;
            for (const auto& [ueId, beamRec] : m_currentBeams)
            {
                if (beamRec.satelliteNodeId == satNodeId && beamRec.beamId == beam.beamId)
                {
                    beamActiveUeCount++;
                }
            }

            // Create beam-specific record
            LeoSimBeamRecord beamRecord;
            beamRecord.ueNodeId = ueNodeId;
            beamRecord.satelliteNodeId = satNodeId;
            beamRecord.beamId = beam.beamId;
            beamRecord.cellId = beam.cellId;
            beamRecord.colorGroup = beam.colorGroup;
            beamRecord.rsrp = linkQuality.signalStrength;
            beamRecord.snr = linkQuality.snr;
            beamRecord.sinr = sinr;  // New: beam-specific SINR
            beamRecord.intraBeamInterference_dBm = sinrResult.intraBeamInterference_dBm;
            beamRecord.interSatInterference_dBm = sinrResult.interSatInterference_dBm;
            beamRecord.beamLoad = beamActiveUeCount;  // New: per-beam load
            beamRecord.beamActive = beam.activeInCurrentSlot;  // New: beam active flag
            beamRecord.pathLoss = linkQuality.pathLoss;
            beamRecord.elevationAngle = linkQuality.elevationAngle;
            beamRecord.remainingServiceTime = tte;
            beamRecord.satelliteLoad = 0;  // Not used in 7-criterion version, kept for compatibility
            beamRecord.endToEndLatency = latency;
            beamRecord.state = LEOSIM_BEAM_CONNECTED;
            beamRecord.associationTime = Simulator::Now();

            visibleBeams.push_back(beamRecord);

            NS_LOG_DEBUG("Visible beam for UE " << ueNodeId << " to satellite " << satNodeId
                                               << " beam " << beam.beamId << ": RSRP="
                                               << beamRecord.rsrp << " dBm, SINR=" << sinr
                                               << " dB, TTE=" << tte << "s, latency=" << latency
                                               << "ms, beamLoad=" << beamActiveUeCount
                                               << ", active=" << (int)beam.activeInCurrentSlot);
        }
    }

    if (m_operatorModel)
    {
        const std::size_t beforeCount = visibleBeams.size();
        visibleBeams.erase(
            std::remove_if(
                visibleBeams.begin(),
                visibleBeams.end(),
                [&](const LeoSimBeamRecord& c) {
                    double alpha = m_operatorModel->GetAlpha(
                        ueNodeId,
                        c.satelliteNodeId,
                        LEOSIM_DIR_DOWNLINK);
                    return alpha == 0.0;
                }),
            visibleBeams.end());

        const std::size_t removed = beforeCount - visibleBeams.size();
        if (m_verbose && removed > 0)
        {
            NS_LOG_INFO("[BeamMgr] Filtered out " << removed << " candidates with alpha=0"
                                                   << " for UE " << ueNodeId);
        }
    }

    NS_LOG_INFO("Found " << visibleBeams.size() << " visible beams for UE " << ueNodeId);
    return visibleBeams;
}

std::vector<LeoSimTopsisCandidate>
LeoSimBeamManager::RankByTopsis(const std::vector<LeoSimBeamRecord>& candidates,
                                 uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId << candidates.size());
    std::vector<LeoSimTopsisCandidate> rankedCandidates;

    if (!m_loader || !m_channelModel)
    {
        NS_LOG_WARN("Loader or channel model not configured");
        return rankedCandidates;
    }

    if (candidates.empty())
    {
        NS_LOG_INFO("No candidates provided for TOPSIS ranking at UE " << ueNodeId);
        return rankedCandidates;
    }

    // Validate TOPSIS weights sum to approximately 1.0
    double weightSum = std::accumulate(m_topsisWeights.begin(), m_topsisWeights.end(), 0.0);
    NS_ASSERT_MSG(std::abs(weightSum - 1.0) < 0.01,
                  "TOPSIS weights must sum to ~1.0, got " << weightSum);
    NS_ASSERT_MSG(m_topsisWeights.size() == 9,
                  "TOPSIS weights array must have exactly 9 elements, got " << m_topsisWeights.size());

    const size_t numCriteria = 9;
    const size_t numCandidates = candidates.size();

    // --- TOPSIS Algorithm (9-Attribute Version) ---
    // Step 1: Build decision matrix (9 criteria x numCandidates)
    // Criteria (all benefit - higher is better):
    //   [0] RSRP (dBm)
    //   [1] SINR (dB)
    //   [2] TTE - Time To Exit (seconds)
    //   [3] 1.0 / beamLoad - inverse of per-beam UE count
    //   [4] 1.0 / endToEndLatency (inverse of ms)
    //   [5] Elevation angle (degrees)
    //   [6] beamActive binary (1.0 if active, 0.0 if dark)
    //   [7] operator compatibility (1.0 same-op, alpha if cross-op)
    //   [8] weather quality score (1.0 clear sky, 0.0 severe fade)
    std::array<std::vector<double>, 9> A;
    std::array<double, 9> sumSquares{};

    for (size_t i = 0; i < numCandidates; i++)
    {
        const auto& candidate = candidates[i];

        // Criterion 0: RSRP (benefit - higher is better)
        A[0].push_back(std::max(candidate.rsrp, -200.0)); // Cap at -200 dBm minimum

        // Criterion 1: SINR in dB (benefit - higher is better)
        // Replace SNR with per-beam SINR from SINR engine
        A[1].push_back(std::max(candidate.sinr, -20.0)); // Cap at -20 dB minimum

        // Criterion 2: TTE in seconds (benefit - longer is better)
        A[2].push_back(candidate.remainingServiceTime);

        // Criterion 3: Inverse of per-beam load (benefit - lower load is better)
        // beamLoad is the UE count on this specific beam
        A[3].push_back((candidate.beamLoad > 0.001) ? (1.0 / candidate.beamLoad) : 100.0);

        // Criterion 4: Inverse of end-to-end latency in ms (benefit - lower is better)
        A[4].push_back((candidate.endToEndLatency > 0.1) ? (1.0 / candidate.endToEndLatency)
                                  : 100.0);

        // Criterion 5: Elevation angle in degrees (benefit - higher coverage is better)
        A[5].push_back(std::max(candidate.elevationAngle, 0.0));

        // Criterion 6: Beam active binary (benefit - active is better)
        A[6].push_back(candidate.beamActive ? 1.0 : 0.0);

        // Criterion 7: operator compatibility (1.0 same-op, alpha if cross-op)
        double opScore = m_operatorModel
                             ? m_operatorModel->GetAlpha(ueNodeId,
                                                         candidate.satelliteNodeId,
                                                         LEOSIM_DIR_DOWNLINK)
                             : 1.0;
        A[7].push_back(opScore);

        // Criterion 8: weather quality score
        // 1.0 = clear sky/no attenuation, 0.0 = severe fade (>=30 dB)
        double wxScore = 1.0;
        if (m_channelModel)
        {
            double atten = m_channelModel->GetLastAttenuation(ueNodeId,
                                                              candidate.satelliteNodeId)
                               .totalAttenuation_dB;
            wxScore = std::max(0.0, 1.0 - atten / 30.0);
        }
        else if (m_weatherModel)
        {
            double atten = m_weatherModel->GetTotalAttenuation(ueNodeId,
                                                               candidate.satelliteNodeId);
            wxScore = std::max(0.0, 1.0 - atten / 30.0);
        }
        A[8].push_back(wxScore);

        // Compute sum of squares for normalization
        for (size_t j = 0; j < numCriteria; j++)
        {
            double value = A[j].back();
            sumSquares[j] += value * value;
        }

        NS_LOG_DEBUG("Candidate sat=" << candidate.satelliteNodeId << " beam="
                                      << candidate.beamId << ": RSRP=" << candidate.rsrp
                                      << " dBm, SINR=" << candidate.sinr << " dB, TTE="
                                      << candidate.remainingServiceTime << "s, beamLoad="
                                      << candidate.beamLoad << ", latency=" << candidate.endToEndLatency
                                      << "ms, elevation=" << candidate.elevationAngle
                                      << ", active=" << (int)candidate.beamActive
                                      << ", weatherScore=" << A[8].back());
    }

    // Step 2: Normalize decision matrix by L2 norm (column-wise)
    std::array<std::vector<double>, 9> N;
    for (size_t i = 0; i < numCandidates; i++)
    {
        for (size_t j = 0; j < numCriteria; j++)
        {
            if (sumSquares[j] > 0)
            {
                N[j].push_back(A[j][i] / std::sqrt(sumSquares[j]));
            }
            else
            {
                N[j].push_back(0.0);
            }
        }
    }

    // Step 3: Apply weights to normalized matrix
    std::array<std::vector<double>, 9> W;
    for (size_t i = 0; i < numCandidates; i++)
    {
        for (size_t j = 0; j < numCriteria; j++)
        {
            W[j].push_back(N[j][i] * m_topsisWeights[j]);
        }
    }

    // Step 4: Determine ideal solution V+ and anti-ideal solution V-
    // All criteria are benefit criteria (higher is better)
    std::array<double, 9> Vpos;
    std::array<double, 9> Vneg;
    Vpos.fill(-1e99);
    Vneg.fill(1e99);

    for (size_t j = 0; j < numCriteria; j++)
    {
        for (size_t i = 0; i < numCandidates; i++)
        {
            Vpos[j] = std::max(Vpos[j], W[j][i]);
            Vneg[j] = std::min(Vneg[j], W[j][i]);
        }
    }

    // Step 5: Calculate separation distances
    std::vector<double> separationFromIdeal(numCandidates, 0.0);
    std::vector<double> separationFromAntiIdeal(numCandidates, 0.0);

    for (size_t i = 0; i < numCandidates; i++)
    {
        for (size_t j = 0; j < numCriteria; j++)
        {
            double diff_ideal = W[j][i] - Vpos[j];
            double diff_antiIdeal = W[j][i] - Vneg[j];

            separationFromIdeal[i] += diff_ideal * diff_ideal;
            separationFromAntiIdeal[i] += diff_antiIdeal * diff_antiIdeal;
        }
        separationFromIdeal[i] = std::sqrt(separationFromIdeal[i]);
        separationFromAntiIdeal[i] = std::sqrt(separationFromAntiIdeal[i]);
    }

    // Step 6: Calculate TOPSIS scores and create candidates
    std::vector<std::pair<size_t, double>> scoredIndices;
    for (size_t i = 0; i < numCandidates; i++)
    {
        double denominator = separationFromIdeal[i] + separationFromAntiIdeal[i];
        double score = 0.0;

        if (denominator > 1e-9)
        {
            score = separationFromAntiIdeal[i] / denominator;
        }

        scoredIndices.push_back({i, score});

        NS_LOG_DEBUG("Candidate " << candidates[i].satelliteNodeId << ": dPos="
                                  << separationFromIdeal[i] << ", dNeg="
                                  << separationFromAntiIdeal[i] << ", score=" << score);
    }

    // Step 7: Sort by TOPSIS score in descending order
    std::sort(scoredIndices.begin(), scoredIndices.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    // Step 8: Build LeoSimTopsisCandidate results
    for (const auto& [idx, score] : scoredIndices)
    {
        LeoSimTopsisCandidate candidate;
        candidate.beamRecord = candidates[idx];
        candidate.topsisScore = score;
        candidate.choConfigReady = false;

        rankedCandidates.push_back(candidate);
        NS_LOG_DEBUG("Ranked satellite " << candidates[idx].satelliteNodeId
                                         << " with TOPSIS score " << score);
    }

    NS_LOG_INFO("TOPSIS ranking complete for UE " << ueNodeId << ": " << rankedCandidates.size()
                                                  << " candidates ranked");
    return rankedCandidates;
}

void
LeoSimBeamManager::InitiateChoPreparation(uint32_t ueNodeId, uint32_t candidateSatId)
{
    NS_LOG_FUNCTION(this << ueNodeId << candidateSatId);
    NS_LOG_INFO("UE " << ueNodeId << " initiating CHO preparation for satellite "
                      << candidateSatId << " at time " << Simulator::Now().GetSeconds() << "s");

    // Wrapper around the full multi-candidate CHO preparation:
    // rank all visible satellites, then ensure the requested candidate is
    // prioritized, and reuse the existing implementation.
    std::vector<LeoSimBeamRecord> visibleBeams = ScanVisibleSatellites(ueNodeId);
    std::vector<LeoSimTopsisCandidate> ranked = RankByTopsis(visibleBeams, ueNodeId);
    if (ranked.empty())
    {
        NS_LOG_WARN("CHO preparation requested for UE " << ueNodeId
                                                        << " but no visible candidates exist");
        return;
    }

    auto candIt = std::find_if(ranked.begin(), ranked.end(),
                               [candidateSatId](const LeoSimTopsisCandidate& c) {
                                   return c.beamRecord.satelliteNodeId == candidateSatId;
                               });
    if (candIt != ranked.end() && candIt != ranked.begin())
    {
        std::iter_swap(ranked.begin(), candIt);
    }

    InitiateChoPreparation(ueNodeId, ranked);
}

void
LeoSimBeamManager::InitiateChoPreparation(uint32_t ueNodeId,
                                           const std::vector<LeoSimTopsisCandidate>& topN)
{
    NS_LOG_FUNCTION(this << ueNodeId << topN.size());

    // Step 1: Get the current beam record and set its state to LEOSIM_BEAM_PREPARING
    auto beamIt = m_currentBeams.find(ueNodeId);
    if (beamIt == m_currentBeams.end())
    {
        NS_LOG_WARN("UE " << ueNodeId << " has no current beam assignment for CHO preparation");
        return;
    }

    uint32_t servingSatId = beamIt->second.satelliteNodeId;
    beamIt->second.state = LEOSIM_BEAM_PREPARING;
    NS_LOG_DEBUG("UE " << ueNodeId << " current beam state changed to LEOSIM_BEAM_PREPARING");

    if (!m_beamStateCallback.IsNull())
    {
        m_beamStateCallback(ueNodeId, beamIt->second, 0.0);
    }

    // Step 2: Clear any existing m_choConfigs[ueNodeId]
    m_choConfigs[ueNodeId].clear();
    NS_LOG_DEBUG("Cleared existing CHO configs for UE " << ueNodeId);

    // Step 3: Iterate over the first min(topN.size(), m_maxCandidates) candidates
    std::vector<LeoSimTopsisCandidate> configured;
    size_t numCandidates = std::min(static_cast<size_t>(m_maxCandidates), topN.size());
    for (size_t i = 0; i < numCandidates; ++i)
    {
        const auto& candidate = topN[i];
        uint32_t candidateSatId = candidate.beamRecord.satelliteNodeId;

        // Skip if candidate is the current serving satellite
        if (candidateSatId == servingSatId)
        {
            NS_LOG_DEBUG("Skipping CHO candidate " << candidateSatId << " (current serving satellite)");
            continue;
        }

        // Step 3a: Build a LeoSimChoConfig with required fields
        LeoSimChoConfig cfg;
        cfg.candidateSatId = candidateSatId;
        cfg.execConditionThresholdA3 = m_a3Offset;
        cfg.execConditionThresholdA4 = m_a4Threshold;
        cfg.execConditionTimeWindow = m_ttt;
        cfg.configValidUntil =
            Simulator::Now() + Seconds(candidate.beamRecord.remainingServiceTime);
        cfg.conditionMet = false;
        cfg.targetAddress = Ipv4Address("0.0.0.0"); // Will be populated during CHO execution

        NS_LOG_DEBUG("Built CHO config for candidate satellite " << candidateSatId
                                                                  << ": validity until "
                                                                  << cfg.configValidUntil.GetSeconds()
                                                                  << "s");

        // Step 3b: Cache routes via routing calculator if available
        if (m_routingCalculator)
        {
            // Note: PreComputeRouteForNode is not exposed in the public API of
            // LeoSimRoutingCalculator. For now, routes will be computed on-demand
            // during CHO execution. In a future enhancement, add method to pre-cache routes.
            NS_LOG_DEBUG("Routing calculator available; routes will be computed on-demand");
        }

        // Step 3c: Push the config into m_choConfigs[ueNodeId]
        m_choConfigs[ueNodeId][candidateSatId] = cfg;
        NS_LOG_DEBUG("Added CHO config for UE " << ueNodeId << " -> satellite " << candidateSatId);

        configured.push_back(candidate);
    }

    if (!m_choConfigCallback.IsNull() && !configured.empty())
    {
        m_choConfigCallback(ueNodeId, servingSatId, configured);
    }

    // Step 4: Log with NS_LOG_INFO
    NS_LOG_INFO("CHO Phase 1 preparation initiated for UE " << ueNodeId << ": "
                                                            << m_choConfigs[ueNodeId].size()
                                                            << " candidates configured, "
                                                            << "serving satellite "
                                                            << servingSatId << " at time "
                                                            << Simulator::Now().GetSeconds() << "s");

    // Step 5: Schedule Simulator::Schedule(m_prepDelay, lambda)
    auto prepCallback = [this, ueNodeId]() {
        // Lambda body: Set state to LEOSIM_BEAM_EVALUATING and call EvaluateChoConditions
        auto beamIt2 = m_currentBeams.find(ueNodeId);
        if (beamIt2 != m_currentBeams.end())
        {
            beamIt2->second.state = LEOSIM_BEAM_EVALUATING;
            NS_LOG_DEBUG("UE " << ueNodeId
                               << " current beam state changed to LEOSIM_BEAM_EVALUATING");

            if (!m_beamStateCallback.IsNull())
            {
                m_beamStateCallback(ueNodeId, beamIt2->second, 0.0);
            }
        }
        EvaluateChoConditions(ueNodeId);
        NS_LOG_DEBUG("CHO evaluation initiated for UE " << ueNodeId << " at "
                                                        << Simulator::Now().GetSeconds() << "s");
    };

    Simulator::Schedule(m_prepDelay, prepCallback);
    NS_LOG_DEBUG("Scheduled CHO evaluation after " << m_prepDelay.GetMilliSeconds()
                                                     << "ms for UE " << ueNodeId);
}

void
LeoSimBeamManager::EvaluateChoConditions(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);

    // Step 1: Get a reference to m_choConfigs[ueNodeId]
    auto configIt = m_choConfigs.find(ueNodeId);
    if (configIt == m_choConfigs.end())
    {
        NS_LOG_DEBUG("No CHO configs found for UE " << ueNodeId);
        return;
    }

    auto& choConfigMap = configIt->second;

    if (choConfigMap.empty())
    {
        NS_LOG_DEBUG("CHO config map is empty for UE " << ueNodeId);
        return;
    }

    // Step 2: Remove expired configs: erase any config where Simulator::Now() > configValidUntil
    auto cfgIt = choConfigMap.begin();
    while (cfgIt != choConfigMap.end())
    {
        if (Simulator::Now() > cfgIt->second.configValidUntil)
        {
            NS_LOG_DEBUG("Removing expired CHO config for UE " << ueNodeId << " -> satellite "
                                                               << cfgIt->first << " (expired at "
                                                               << cfgIt->second.configValidUntil.GetSeconds()
                                                               << "s)");
            cfgIt = choConfigMap.erase(cfgIt);
        }
        else
        {
            ++cfgIt;
        }
    }

    // Step 3: If configs is now empty, set state to LEOSIM_BEAM_SEARCHING and return
    if (choConfigMap.empty())
    {
        auto beamIt = m_currentBeams.find(ueNodeId);
        if (beamIt != m_currentBeams.end())
        {
            beamIt->second.state = LEOSIM_BEAM_SEARCHING;
            NS_LOG_DEBUG("UE " << ueNodeId << " state changed to LEOSIM_BEAM_SEARCHING "
                               << "(all CHO configs expired)");
        }
        return;
    }

    // Get the current serving beam for quality comparison
    auto servingBeamIt = m_currentBeams.find(ueNodeId);
    if (servingBeamIt == m_currentBeams.end())
    {
        NS_LOG_WARN("UE " << ueNodeId << " has no serving beam for CHO condition evaluation");
        return;
    }

    const LeoSimBeamRecord& servingBeam = servingBeamIt->second;
    uint32_t servingSatId = servingBeam.satelliteNodeId;

    // Step 4: For each remaining config
    for (auto& [candidateSatId, cfg] : choConfigMap)
    {
        NS_LOG_DEBUG("Evaluating CHO conditions for UE " << ueNodeId << " -> candidate satellite "
                                                         << candidateSatId);

        if (!m_channelModel)
        {
            NS_LOG_WARN("Channel model not available for CHO condition evaluation");
            continue;
        }

        // Step 4a: Get link quality
        LeoSimChannelQuality candidateQuality = m_channelModel->GetLinkQuality(ueNodeId, candidateSatId);

        // Step 4b: Evaluate A3 condition: qual.rsrp > serving.rsrp + threshold
        bool a3Condition = candidateQuality.signalStrength > (servingBeam.rsrp + cfg.execConditionThresholdA3);
        NS_LOG_DEBUG("  A3: candidate RSRP=" << candidateQuality.signalStrength
                                             << " dBm > serving RSRP=" << servingBeam.rsrp
                                             << " + offset=" << cfg.execConditionThresholdA3
                                             << " => " << (a3Condition ? "TRUE" : "FALSE"));

        // Step 4c: Evaluate A4 condition: qual.rsrp > threshold AND A3 is true
        bool a4Condition = (candidateQuality.signalStrength > cfg.execConditionThresholdA4) && a3Condition;
        NS_LOG_DEBUG("  A4: candidate RSRP=" << candidateQuality.signalStrength
                                             << " dBm > threshold=" << cfg.execConditionThresholdA4
                                             << " AND A3=" << (a3Condition ? "TRUE" : "FALSE")
                                             << " => " << (a4Condition ? "TRUE" : "FALSE"));

        // Step 4d: Evaluate TTE-based condition: ComputeTte < TTE threshold
        double currentTte = ComputeTte(ueNodeId, servingSatId);
        bool tteBased = currentTte < m_tteThreshold.GetSeconds();
        if (m_verbose)
        {
            std::cout << "[HO-CHECK][" << Simulator::Now().GetSeconds() << "s] UE " << ueNodeId
                      << " candidate=" << candidateSatId
                      << " A3=" << (a3Condition ? "1" : "0")
                      << " A4=" << (a4Condition ? "1" : "0")
                      << " TTE=" << currentTte << "s"
                      << " threshold=" << m_tteThreshold.GetSeconds() << "s"
                      << " TTE_TRIGGER=" << (tteBased ? "1" : "0")
                      << std::endl;
        }
        NS_LOG_DEBUG("  TTE-based: current TTE=" << currentTte << "s < threshold="
                                                  << m_tteThreshold.GetSeconds()
                                                  << "s => " << (tteBased ? "TRUE" : "FALSE"));

        bool weatherBased = false;
        if (m_weatherModel)
        {
            double atten = m_weatherModel->GetTotalAttenuation(ueNodeId, servingSatId);
            weatherBased = (atten > m_weatherFadeThresholdDb);
            NS_LOG_DEBUG("  WEATHER-based: attenuation=" << atten << " dB > threshold="
                                                         << m_weatherFadeThresholdDb << " dB => "
                                                         << (weatherBased ? "TRUE" : "FALSE"));
        }

        // Step 4e: If any condition is met, execute handover
        if (a3Condition || a4Condition || tteBased || weatherBased)
        {
            cfg.conditionMet = true;

            // Determine trigger: TIME_BASED if TTE triggered, A4 if A4 met,
            // A3 if A3 met, else WEATHER_FADE.
            LeoSimHandoverTrigger trigger;
            if (tteBased)
            {
                trigger = LEOSIM_HO_TIME_BASED;
                NS_LOG_INFO("CHO condition met for UE " << ueNodeId << " -> satellite " << candidateSatId
                                                        << ": TRIGGER=TIME_BASED (TTE < threshold)");
            }
            else if (a4Condition)
            {
                trigger = LEOSIM_HO_A4;
                NS_LOG_INFO("CHO condition met for UE " << ueNodeId << " -> satellite " << candidateSatId
                                                        << ": TRIGGER=A4 (absolute threshold exceeded)");
            }
            else
            {
                trigger = weatherBased ? LEOSIM_HO_WEATHER_FADE : LEOSIM_HO_A3;
                NS_LOG_INFO("CHO condition met for UE " << ueNodeId << " -> satellite " << candidateSatId
                                                        << ": TRIGGER="
                                                        << (weatherBased ? "WEATHER_FADE" : "A3")
                                                        << (weatherBased
                                                                ? " (attenuation exceeds weather threshold)"
                                                                : " (neighbour exceeds serving + offset)"));
            }

            // Call ExecuteChoHandover and return immediately — first condition wins
            ExecuteChoHandover(ueNodeId, candidateSatId, trigger);
            return;
        }
    }

    // Step 5: If no condition was met, do nothing
    // Re-evaluation happens on the next UpdateCycle() call automatically
    NS_LOG_DEBUG("No CHO conditions met for UE " << ueNodeId
                                                  << "; awaiting next evaluation cycle");
}

void
LeoSimBeamManager::ExecuteChoHandover(uint32_t ueNodeId,
                                      uint32_t targetSatId,
                                      LeoSimHandoverTrigger trigger)
{
    NS_LOG_FUNCTION(this << ueNodeId << targetSatId << trigger);

    // Step 1: Set m_currentBeam[ueNodeId].state = LEOSIM_BEAM_EXECUTING
    auto beamIt = m_currentBeams.find(ueNodeId);
    if (beamIt == m_currentBeams.end())
    {
        NS_LOG_WARN("UE " << ueNodeId << " has no current beam for handover execution");
        return;
    }

    uint32_t sourceSatId = beamIt->second.satelliteNodeId;
    beamIt->second.state = LEOSIM_BEAM_EXECUTING;

    if (!m_beamStateCallback.IsNull())
    {
        m_beamStateCallback(ueNodeId, beamIt->second, 0.0);
    }

    // Step 2: Log NS_LOG_INFO with UE ID, target satellite, and trigger type
    std::string triggerStr;
    switch (trigger)
    {
        case LEOSIM_HO_A3:
            triggerStr = "A3";
            break;
        case LEOSIM_HO_A4:
            triggerStr = "A4";
            break;
        case LEOSIM_HO_TIME_BASED:
            triggerStr = "TIME_BASED";
            break;
        case LEOSIM_HO_LOCATION_BASED:
            triggerStr = "LOCATION_BASED";
            break;
        case LEOSIM_HO_ELEVATION:
            triggerStr = "ELEVATION";
            break;
        case LEOSIM_HO_RLF:
            triggerStr = "RLF";
            break;
        case LEOSIM_HO_LOAD_BALANCE:
            triggerStr = "LOAD_BALANCE";
            break;
        case LEOSIM_HO_WEATHER_FADE:
            triggerStr = "WEATHER_FADE";
            break;
        default:
            triggerStr = "UNKNOWN";
    }

    NS_LOG_INFO("CHO execution initiated for UE " << ueNodeId << ": satellite " << sourceSatId
                                                  << " -> " << targetSatId << ", trigger="
                                                  << triggerStr << " at time "
                                                  << Simulator::Now().GetSeconds() << "s");

    // Step 3: Schedule Simulator::Schedule(m_execDelay, lambda)
    auto execCallback = [this, ueNodeId, sourceSatId, targetSatId, trigger]() {
        // Lambda calls CompleteHandover
        CompleteHandover(ueNodeId, sourceSatId, targetSatId, trigger);
    };

    Simulator::Schedule(m_execDelay, execCallback);
    NS_LOG_DEBUG("CHO execution scheduled after " << m_execDelay.GetMilliSeconds()
                                                   << "ms for UE " << ueNodeId);
}

void
LeoSimBeamManager::CompleteHandover(uint32_t ueNodeId,
                                     uint32_t sourceSatId,
                                     uint32_t targetSatId,
                                     LeoSimHandoverTrigger trigger)
{
    NS_LOG_FUNCTION(this << ueNodeId << sourceSatId << targetSatId << trigger);

    if (!m_channelModel || !m_loader)
    {
        NS_LOG_WARN("Channel model or loader not configured for handover completion");
        return;
    }

    // Step 1: Get new link quality
    LeoSimChannelQuality newLinkQuality =
        m_channelModel->GetLinkQuality(ueNodeId, targetSatId);

    // Step 2: Update m_currentBeam[ueNodeId]
    auto beamIt = m_currentBeams.find(ueNodeId);
    if (beamIt == m_currentBeams.end())
    {
        NS_LOG_WARN("UE " << ueNodeId << " has no beam record for handover completion");
        return;
    }

    LeoSimBeamRecord& currentBeam = beamIt->second;
    currentBeam.satelliteNodeId = targetSatId;
    currentBeam.rsrp = newLinkQuality.signalStrength;
    currentBeam.snr = newLinkQuality.snr;
    currentBeam.elevationAngle = newLinkQuality.elevationAngle;
    currentBeam.remainingServiceTime = ComputeTte(ueNodeId, targetSatId);
    currentBeam.endToEndLatency = ComputeEndToEndLatency(ueNodeId, targetSatId);
    currentBeam.associationTime = Simulator::Now();
    currentBeam.state = LEOSIM_BEAM_CONNECTED;

    if (!m_beamStateCallback.IsNull())
    {
        m_beamStateCallback(ueNodeId, currentBeam, 0.0);
    }

    NS_LOG_DEBUG("Updated beam record for UE " << ueNodeId << " to satellite " << targetSatId
                                               << ": RSRP=" << currentBeam.rsrp
                                               << " dBm, SNR=" << currentBeam.snr << " dB");

    // Step 3: Record m_lastHandoverTime[ueNodeId]
    m_lastHandoverTime[ueNodeId] = Simulator::Now();

    // Step 4: Classify handover type
    uint32_t sourcePlane = m_loader->GetOrbitPlane(GetLeoSimIdFromNodeId(sourceSatId));
    uint32_t targetPlane = m_loader->GetOrbitPlane(GetLeoSimIdFromNodeId(targetSatId));

    LeoSimHandoverType hoType = (sourcePlane == targetPlane) ? LEOSIM_HO_INTER_SATELLITE
                                                             : LEOSIM_HO_INTER_ORBIT;

    NS_LOG_DEBUG("Handover type: " << (hoType == LEOSIM_HO_INTER_SATELLITE ? "INTER_SATELLITE"
                                                                           : "INTER_ORBIT")
                                   << " (source plane " << sourcePlane << ", target plane "
                                   << targetPlane << ")");

    // Step 5: Build a LeoSimHandoverEvent and populate all fields
    LeoSimHandoverEvent hoEvent;
    hoEvent.ueNodeId = ueNodeId;
    hoEvent.sourceSatId = sourceSatId;
    hoEvent.targetSatId = targetSatId;
    hoEvent.sourceBeamId = 0;  // TODO: retrieve from current beam record
    hoEvent.targetBeamId = 0;  // TODO: retrieve from target beam record
    hoEvent.sourceCellId = 0;  // TODO: retrieve from current beam record
    hoEvent.targetCellId = 0;  // TODO: retrieve from target beam record
    hoEvent.mode = LEOSIM_HO_MODE_CHO; // Conditional handover
    hoEvent.type = hoType;
    hoEvent.trigger = trigger;
    hoEvent.initiatedAt = Simulator::Now() - m_prepDelay - m_execDelay;
    hoEvent.completedAt = Simulator::Now();
    hoEvent.handoverLatencyMs = (m_prepDelay + m_execDelay).GetMilliSeconds();
    hoEvent.packetsBuffered = 0;  // TODO: get from buffer manager
    hoEvent.packetsDropped = 0;   // TODO: get from buffer manager
    hoEvent.success = true;
    hoEvent.sinrBefore = 0.0;   // TODO: retrieve from candidate record
    hoEvent.sinrAfter = 0.0;    // TODO: retrieve from ranked candidates

    NS_LOG_DEBUG("Built handover event: latency=" << hoEvent.handoverLatencyMs
                                                   << "ms, success=" << hoEvent.success);

    // Step 6: Push the event to m_handoverHistory
    m_handoverHistory.push_back(hoEvent);
    NS_LOG_DEBUG("Pushed handover event to history (total events: " << m_handoverHistory.size()
                                                                     << ")");

    // Step 7: Call m_routingCalc->InvalidateRoutesForNode(ueNodeId)
    if (m_routingCalculator)
    {
        // TODO: Add InvalidateRoutesForNode method to LeoSimRoutingCalculator
        NS_LOG_DEBUG("TODO: Invalidate cached routes for UE " << ueNodeId
                                                              << " in routing calculator");
    }

    // Step 8: Call m_routingCalc->ForceRouteUpdate(ueNodeId, targetSatId)
    if (m_routingCalculator)
    {
        // TODO: Add ForceRouteUpdate method to LeoSimRoutingCalculator
        NS_LOG_DEBUG("TODO: Force route update from UE " << ueNodeId << " via satellite "
                                                         << targetSatId);
    }

    // Step 9: If m_handoverBufferingEnabled is true, call FlushBuffer(ueNodeId)
    if (m_handoverBufferingEnabled)
    {
        FlushBuffer(ueNodeId);
        NS_LOG_DEBUG("Flushed buffered packets for UE " << ueNodeId);
    }

    // Step 10: Clear m_choConfigs[ueNodeId]
    m_choConfigs[ueNodeId].clear();
    NS_LOG_DEBUG("Cleared CHO configs for UE " << ueNodeId);

    // Step 11: Call PreScheduleEphemerisHandovers(Seconds(600))
    PreScheduleEphemerisHandovers(Seconds(600));
    NS_LOG_DEBUG("Pre-scheduled ephemeris handovers with 600s lookahead for new satellite");

    // Step 12: If m_handoverCallback is not null, fire it with the event
    if (!m_handoverCallback.IsNull())
    {
        m_handoverCallback(hoEvent);
        NS_LOG_DEBUG("Fired handover callback for UE " << ueNodeId);
    }

    // Step 13: Log NS_LOG_INFO
    NS_LOG_INFO("CHO completed for UE " << ueNodeId << ": satellite " << sourceSatId << " -> "
                                        << targetSatId << ", latency=" << hoEvent.handoverLatencyMs
                                        << "ms, trigger=" << trigger << " at time "
                                        << Simulator::Now().GetSeconds() << "s");
}

void
LeoSimBeamManager::PreScheduleEphemerisHandovers(Time lookaheadWindow)
{
    NS_LOG_FUNCTION(this << lookaheadWindow.GetSeconds());

    if (!m_loader || !m_channelModel)
    {
        NS_LOG_WARN("Loader or channel model not configured");
        return;
    }

    NS_LOG_INFO("Pre-scheduling ephemeris-based handovers with lookahead window "
                << lookaheadWindow.GetSeconds() << "s");

    // Iterate over all UEs with current beam assignments
    for (const auto& entry : m_currentBeams)
    {
        uint32_t ueNodeId = entry.first;
        const LeoSimBeamRecord& beamRecord = entry.second;
        uint32_t satNodeId = beamRecord.satelliteNodeId;

        // Compute Time-To-Exit from current serving satellite
        double tte = ComputeTte(ueNodeId, satNodeId);

        // Skip if already out of coverage
        if (tte <= 0)
        {
            NS_LOG_DEBUG("UE " << ueNodeId << " already out of coverage from satellite "
                               << satNodeId << ", skipping ephemeris pre-scheduling");
            continue;
        }

        // Calculate preparation time: TTE - TTE threshold
        // The preparation should start before signal degrades by m_tteThreshold
        Time prepTime = Seconds(tte) - m_tteThreshold;
        if (prepTime < Time(0))
        {
            prepTime = Time(0); // Trigger immediately if already past threshold
        }

        // Only schedule if preparation time is within the lookahead window
        if (prepTime > lookaheadWindow)
        {
            NS_LOG_DEBUG("Prep time " << prepTime.GetSeconds() << "s exceeds lookahead window "
                                      << lookaheadWindow.GetSeconds() << "s for UE " << ueNodeId);
            continue;
        }

        // Create lambda callback for scheduled handover preparation
        // Explicitly capture all required variables by value
        auto prepCallback = [this, ueNodeId, satNodeId, tte]() {
            NS_LOG_INFO("Ephemeris-triggered handover preparation for UE " << ueNodeId
                                                                          << " from satellite "
                                                                          << satNodeId
                                                                          << " (TTE was "
                                                                          << tte
                                                                          << "s) at time "
                                                                          << Simulator::Now()
                                                                              .GetSeconds()
                                                                          << "s");

            // Scan for currently visible satellites
            std::vector<LeoSimBeamRecord> visibleBeamRecs = ScanVisibleSatellites(ueNodeId);
            if (visibleBeamRecs.empty())
            {
                NS_LOG_WARN("No visible satellites found for UE " << ueNodeId
                                                                  << " during ephemeris "
                                                                  << "pre-scheduling preparation");
                return;
            }

            // Rank candidates using TOPSIS
            std::vector<LeoSimTopsisCandidate> candidates =
                RankByTopsis(visibleBeamRecs, ueNodeId);
            if (candidates.empty())
            {
                NS_LOG_WARN("No ranked candidates found for UE " << ueNodeId);
                return;
            }

            // Initiate preparation for top-ranked candidate
            uint32_t targetSat = candidates[0].beamRecord.satelliteNodeId;
            NS_LOG_INFO("Initiating CHO preparation for UE " << ueNodeId << " to satellite "
                                                             << targetSat << " (TOPSIS score: "
                                                             << candidates[0].topsisScore
                                                             << ")");
            InitiateChoPreparation(ueNodeId, targetSat);
        };

        // Schedule the preparation callback
        Simulator::Schedule(prepTime, prepCallback);

        NS_LOG_INFO("Pre-scheduled ephemeris handover for UE " << ueNodeId << " from satellite "
                                                               << satNodeId << " (TTE=" << tte
                                                               << "s) with prep offset "
                                                               << prepTime.GetSeconds() << "s");
    }
}

void
LeoSimBeamManager::EvaluateIntraBeamNeed(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);

    if (!m_loader || !m_sinrEngine || !m_multiBeamModel)
    {
        NS_LOG_DEBUG("Intra-beam evaluation skipped for UE " << ueNodeId
                                                               << " (missing loader/sinr/multi-beam model)");
        return;
    }

    auto currentIt = m_currentBeams.find(ueNodeId);
    if (currentIt == m_currentBeams.end())
    {
        return;
    }

    LeoSimBeamRecord& rec = currentIt->second;
    const uint32_t satId = rec.satelliteNodeId;
    const auto& beams = m_multiBeamModel->GetBeamsForSatellite(satId);
    if (beams.empty())
    {
        return;
    }

    const auto [ueLat, ueLon] = m_loader->GetGroundDeviceLatLon(ueNodeId);
    int32_t naturalBeam = LeoSimBeamLayoutEngine::FindBeamForPosition(beams, ueLat, ueLon);

    const auto& allBeams = m_multiBeamModel->GetAllBeams();

    // Step 5: A3-like intra-beam trigger if UE has moved into another beam footprint.
    if (naturalBeam >= 0 && static_cast<uint32_t>(naturalBeam) != rec.beamId)
    {
        LeoSimSinrResult naturalSinr = m_sinrEngine->ComputeSinr(ueNodeId,
                                                                 satId,
                                                                 static_cast<uint32_t>(naturalBeam),
                                                                 allBeams,
                                                                 ueLat,
                                                                 ueLon);

        if (naturalSinr.sinr_dB > rec.sinr + m_a3Offset)
        {
            ExecuteIntraBeamHandover(ueNodeId,
                                     static_cast<uint32_t>(naturalBeam),
                                     LEOSIM_HO_A3);
            return;
        }
    }

    const auto currentBeamIt = std::find_if(beams.begin(), beams.end(),
                                            [&rec](const LeoSimSpotBeam& b) {
                                                return b.beamId == rec.beamId;
                                            });

    // Step 6: Beam hopping case where current beam is dark.
    if (currentBeamIt != beams.end() && !currentBeamIt->activeInCurrentSlot)
    {
        int32_t bestBeam = FindBestActiveBeam(ueNodeId, satId);
        if (bestBeam >= 0)
        {
            ExecuteIntraBeamHandover(ueNodeId,
                                     static_cast<uint32_t>(bestBeam),
                                     static_cast<LeoSimHandoverTrigger>(LEOSIM_HO_BEAM_HOPPING_DARK));
            return;
        }
    }

    // Step 7: Low-SINR trigger for better same-satellite beam.
    if (rec.sinr < m_sinrThresholdDb)
    {
        int32_t betterBeam = FindBetterBeamOnSameSat(ueNodeId, satId);
        if (betterBeam >= 0)
        {
            ExecuteIntraBeamHandover(ueNodeId,
                                     static_cast<uint32_t>(betterBeam),
                                     LEOSIM_HO_A4);
            return;
        }
    }

    // Weather fade trigger: if serving link attenuation exceeds threshold,
    // prepare CHO toward the top-ranked visible satellite candidate.
    if (m_channelModel || m_weatherModel)
    {
        double atten = 0.0;
        if (m_channelModel)
        {
            atten = m_channelModel->GetLastAttenuation(ueNodeId, satId).totalAttenuation_dB;
        }
        else if (m_weatherModel)
        {
            atten = m_weatherModel->GetTotalAttenuation(ueNodeId, satId);
        }
        if (atten > m_weatherFadeThresholdDb)
        {
            auto candidates = RankByTopsis(ScanVisibleSatellites(ueNodeId), ueNodeId);
            if (!candidates.empty() &&
                candidates[0].beamRecord.satelliteNodeId != satId)
            {
                NS_LOG_INFO("Weather fade trigger for UE " << ueNodeId
                                                           << ": attenuation=" << atten
                                                           << " dB > threshold="
                                                           << m_weatherFadeThresholdDb
                                                           << " dB, preparing CHO to sat "
                                                           << candidates[0].beamRecord.satelliteNodeId);
                InitiateChoPreparation(ueNodeId, candidates);
                return;
            }
        }
    }
}

void
LeoSimBeamManager::ExecuteIntraBeamHandover(uint32_t ueNodeId,
                                            uint32_t targetBeamId,
                                            LeoSimHandoverTrigger trigger)
{
    NS_LOG_FUNCTION(this << ueNodeId << targetBeamId << trigger);

    if (!m_loader || !m_sinrEngine || !m_multiBeamModel)
    {
        return;
    }

    auto it = m_currentBeams.find(ueNodeId);
    if (it == m_currentBeams.end())
    {
        return;
    }

    LeoSimBeamRecord& rec = it->second;
    uint32_t sourceBeamId = rec.beamId;
    const uint32_t satId = rec.satelliteNodeId;

    NS_LOG_INFO("Intra-beam handover start: UE " << ueNodeId << " beam " << sourceBeamId
                                                  << " -> " << targetBeamId << " on satellite "
                                                  << satId << ", trigger=" << trigger);

    rec.state = LEOSIM_BEAM_EXECUTING;
    if (!m_beamStateCallback.IsNull())
    {
        m_beamStateCallback(ueNodeId, rec, 0.0);
    }

    Time delay = MilliSeconds(m_intraBeamHoDelayMs);
    auto complete = [this,
                     ueNodeId,
                     satId,
                     sourceBeamId,
                     targetBeamId,
                     trigger,
                     delay]() {
        auto curIt = m_currentBeams.find(ueNodeId);
        if (curIt == m_currentBeams.end())
        {
            return;
        }

        const auto& satBeams = m_multiBeamModel->GetBeamsForSatellite(satId);
        if (targetBeamId >= satBeams.size())
        {
            NS_LOG_WARN("Intra-beam handover target out of range for UE " << ueNodeId
                                                                            << ": sat=" << satId
                                                                            << " targetBeam="
                                                                            << targetBeamId);
            curIt->second.state = LEOSIM_BEAM_CONNECTED;
            return;
        }

        const LeoSimSpotBeam& targetBeam = satBeams[targetBeamId];

        LeoSimBeamRecord& r = curIt->second;

        r.beamId = targetBeam.beamId;
        r.satelliteNodeId = satId;
        r.cellId = targetBeam.cellId;
        r.colorGroup = targetBeam.colorGroup;
        r.beamLoad = targetBeam.currentLoad;
        r.beamThroughputMbps = targetBeam.currentThroughputMbps;
        r.beamActive = targetBeam.activeInCurrentSlot;

        const auto [ueLat, ueLon] = m_loader->GetGroundDeviceLatLon(ueNodeId);
        const auto& allBeams = m_multiBeamModel->GetAllBeams();
        LeoSimSinrResult sinr =
            m_sinrEngine->ComputeSinr(ueNodeId, satId, targetBeam.beamId, allBeams, ueLat, ueLon);

        r.sinr = sinr.sinr_dB;
        r.snr = sinr.sinr_dB;
        r.rsrp = sinr.signal_dBm;
        r.intraBeamInterference_dBm = sinr.intraBeamInterference_dBm;
        r.interSatInterference_dBm = sinr.interSatInterference_dBm;
        r.associationTime = Simulator::Now();
        r.state = LEOSIM_BEAM_CONNECTED;

        LeoSimHandoverEvent evt;
        evt.ueNodeId = ueNodeId;
        evt.sourceSatId = satId;
        evt.targetSatId = satId;
        evt.mode = m_hoMode;
        evt.type = LEOSIM_HO_INTRA_BEAM;
        evt.trigger = trigger;
        evt.initiatedAt = Simulator::Now() - delay;
        evt.completedAt = Simulator::Now();
        evt.handoverLatencyMs = m_intraBeamHoDelayMs;
        evt.packetsBuffered = 0;
        evt.packetsDropped = 0;
        evt.success = true;
        evt.sourceBeamId = 0;  // TODO: retrieve from current beam
        evt.targetBeamId = 0;  // TODO: retrieve from target beam
        evt.sourceCellId = 0;
        evt.targetCellId = 0;
        evt.sinrBefore = 0.0;
        evt.sinrAfter = 0.0;
        m_handoverHistory.push_back(evt);

        if (!m_handoverCallback.IsNull())
        {
            m_handoverCallback(evt);
        }

        if (!m_beamStateCallback.IsNull())
        {
            m_beamStateCallback(ueNodeId, r, 0.0);
        }

        NS_LOG_INFO("Intra-beam handover complete: UE " << ueNodeId << " beam " << sourceBeamId
                                                         << " -> " << targetBeam.beamId
                                                         << " on satellite " << satId
                                                         << ", SINR=" << r.sinr << " dB");
    };

    Simulator::Schedule(delay, complete);
}

int32_t
LeoSimBeamManager::FindBestActiveBeam(uint32_t ueNodeId, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << ueNodeId << satId);

    if (!m_loader || !m_sinrEngine || !m_multiBeamModel)
    {
        return -1;
    }

    const auto& beams = m_multiBeamModel->GetBeamsForSatellite(satId);
    if (beams.empty())
    {
        return -1;
    }

    const auto [ueLat, ueLon] = m_loader->GetGroundDeviceLatLon(ueNodeId);
    const auto& allBeams = m_multiBeamModel->GetAllBeams();

    int32_t bestBeam = -1;
    double bestSinr = -std::numeric_limits<double>::infinity();
    for (const auto& b : beams)
    {
        if (!b.activeInCurrentSlot)
        {
            continue;
        }

        LeoSimSinrResult s =
            m_sinrEngine->ComputeSinr(ueNodeId, satId, b.beamId, allBeams, ueLat, ueLon);
        if (s.sinr_dB > bestSinr)
        {
            bestSinr = s.sinr_dB;
            bestBeam = static_cast<int32_t>(b.beamId);
        }
    }

    return bestBeam;
}

int32_t
LeoSimBeamManager::FindBetterBeamOnSameSat(uint32_t ueNodeId, uint32_t satId) const
{
    NS_LOG_FUNCTION(this << ueNodeId << satId);

    auto currentIt = m_currentBeams.find(ueNodeId);
    if (currentIt == m_currentBeams.end())
    {
        return -1;
    }
    const LeoSimBeamRecord& rec = currentIt->second;

    int32_t bestBeam = -1;
    double bestSinr = rec.sinr + m_a3Offset;

    if (!m_loader || !m_sinrEngine || !m_multiBeamModel)
    {
        return -1;
    }

    const auto& beams = m_multiBeamModel->GetBeamsForSatellite(satId);
    const auto [ueLat, ueLon] = m_loader->GetGroundDeviceLatLon(ueNodeId);
    const auto& allBeams = m_multiBeamModel->GetAllBeams();

    for (const auto& b : beams)
    {
        if (!b.activeInCurrentSlot || b.beamId == rec.beamId)
        {
            continue;
        }

        LeoSimSinrResult s =
            m_sinrEngine->ComputeSinr(ueNodeId, satId, b.beamId, allBeams, ueLat, ueLon);
        if (s.sinr_dB > bestSinr)
        {
            bestSinr = s.sinr_dB;
            bestBeam = static_cast<int32_t>(b.beamId);
        }
    }

    return bestBeam;
}

Ptr<Node>
LeoSimBeamManager::FindServingBeamForFootprint(uint32_t ueNodeId) const
{
    NS_LOG_FUNCTION(this << ueNodeId);

    // Step 1: Get the UE ground position
    Vector uePos = m_loader->GetGroundDevicePosition(ueNodeId);
    NS_LOG_DEBUG("Looking for serving beam footprint at UE position: (" << uePos.x << ", "
                                                                        << uePos.y << ", "
                                                                        << uePos.z << ")");

    // Step 2: Call ScanVisibleSatellites on a const_cast to get candidates
    // Note: ScanVisibleSatellites is non-const but we're in a const method,
    // so we use const_cast to access it for beam discovery purposes
    std::vector<LeoSimBeamRecord> candidates =
        const_cast<LeoSimBeamManager*>(this)->ScanVisibleSatellites(ueNodeId);

    if (candidates.empty())
    {
        NS_LOG_DEBUG("No visible satellites found for UE " << ueNodeId);
        return nullptr;
    }

    NS_LOG_DEBUG("Found " << candidates.size() << " visible satellites for UE " << ueNodeId);

    // Step 3: Call RankByTopsis on the candidates
    std::vector<LeoSimTopsisCandidate> ranked =
        const_cast<LeoSimBeamManager*>(this)->RankByTopsis(candidates, ueNodeId);

    if (ranked.empty())
    {
        NS_LOG_WARN("TOPSIS ranking returned no candidates for UE " << ueNodeId);
        return nullptr;
    }

    // Step 4: Return ns3::NodeList::GetNode for the top-ranked candidate
    uint32_t topCandidateSatId = ranked[0].beamRecord.satelliteNodeId;
    Ptr<Node> servingNode = NodeList::GetNode(topCandidateSatId);

    NS_LOG_DEBUG("Selected satellite " << topCandidateSatId << " (TOPSIS score="
                                       << ranked[0].topsisScore << ") as serving beam for UE "
                                       << ueNodeId);

    return servingNode;
}

void
LeoSimBeamManager::UpdateCycle()
{
    NS_LOG_FUNCTION(this);
    NS_LOG_DEBUG("Starting beam manager update cycle at time " << Simulator::Now().GetSeconds()
                                                                << "s");

    // Check if simulation has ended
    Time currentTime = Simulator::Now();
    if (currentTime >= m_simStart + m_simDuration)
    {
        NS_LOG_DEBUG("UpdateCycle ending: simulation time " << currentTime.GetSeconds()
                                                            << "s >= simulation end time "
                                                            << (m_simStart + m_simDuration).GetSeconds()
                                                            << "s");
        return;
    }

    // Drive the beam manager state machine for each UE.
    for (uint32_t i = 0; i < m_userEquipment.GetN(); ++i)
    {
        Ptr<Node> ueNode = m_userEquipment.Get(i);
        if (!ueNode)
        {
            continue;
        }

        uint32_t ueNodeId = ueNode->GetId();

        // Ensure each UE has an initial serving beam assignment.
        auto beamIt = m_currentBeams.find(ueNodeId);
        if (beamIt == m_currentBeams.end())
        {
            std::vector<LeoSimBeamRecord> visible = ScanVisibleSatellites(ueNodeId);
            std::vector<LeoSimTopsisCandidate> ranked = RankByTopsis(visible, ueNodeId);
            if (ranked.empty())
            {
                NS_LOG_DEBUG("No visible satellites to attach UE " << ueNodeId
                                                                    << " during UpdateCycle");
                continue;
            }

            LeoSimBeamRecord serving = ranked[0].beamRecord;
            serving.state = LEOSIM_BEAM_CONNECTED;
            serving.associationTime = Simulator::Now();
            m_currentBeams[ueNodeId] = serving;

            if (!m_beamStateCallback.IsNull())
            {
                m_beamStateCallback(ueNodeId, serving, ranked[0].topsisScore);
            }
            continue;
        }

        // Run intra-beam checks first in each UE update iteration.
        EvaluateIntraBeamNeed(ueNodeId);

        LeoSimBeamRecord& currentBeam = beamIt->second;
        uint32_t currentSatId = currentBeam.satelliteNodeId;

        // Refresh current-beam metrics for logging and decision-making.
        if (m_channelModel)
        {
            LeoSimChannelQuality servingQual = m_channelModel->GetLinkQuality(ueNodeId, currentSatId);
            currentBeam.rsrp = servingQual.signalStrength;
            currentBeam.snr = servingQual.snr;
            currentBeam.pathLoss = servingQual.pathLoss;
            currentBeam.elevationAngle = servingQual.elevationAngle;
            currentBeam.remainingServiceTime = ComputeTte(ueNodeId, currentSatId);
            currentBeam.endToEndLatency = ComputeEndToEndLatency(ueNodeId, currentSatId);
        }

        if (!m_beamStateCallback.IsNull())
        {
            m_beamStateCallback(ueNodeId, currentBeam, 0.0);
        }

        // If intra-beam handover has started, skip inter-satellite logic this cycle.
        auto refreshed = m_currentBeams.find(ueNodeId);
        if (refreshed != m_currentBeams.end() &&
            refreshed->second.state != LEOSIM_BEAM_CONNECTED)
        {
            continue;
        }

        // Earth-fixed beam mode: beam steering based on cell footprints.
        if (m_earthFixedBeam)
        {
            NS_LOG_DEBUG("Processing UE " << ueNodeId << " in earth-fixed beam mode");

            Ptr<Node> bestServing = FindServingBeamForFootprint(ueNodeId);
            if (bestServing)
            {
                uint32_t bestServingId = bestServing->GetId();
                if (bestServingId != currentSatId)
                {
                    NS_LOG_INFO("Earth-fixed beam footprint switch for UE " << ueNodeId << ": "
                                                                            << currentSatId << " -> "
                                                                            << bestServingId << " at time "
                                                                            << Simulator::Now().GetSeconds()
                                                                            << "s");

                    std::vector<LeoSimBeamRecord> visibleBeams = ScanVisibleSatellites(ueNodeId);
                    std::vector<LeoSimTopsisCandidate> ranked = RankByTopsis(visibleBeams, ueNodeId);
                    if (!ranked.empty())
                    {
                        InitiateChoPreparation(ueNodeId, ranked);
                    }
                }
            }
        }

        // CHO condition evaluation should be revisited periodically.
        if (currentBeam.state == LEOSIM_BEAM_EVALUATING)
        {
            EvaluateChoConditions(ueNodeId);
        }

        // When connected, run TTT/A3/A4 evaluation to trigger CHO preparation.
        if (currentBeam.state == LEOSIM_BEAM_CONNECTED)
        {
            RunTttEvaluation(ueNodeId);
        }

        // Always run RLF detection.
        RunT310Evaluation(ueNodeId);
    }

    // Reschedule the next update cycle
    m_updateEventId = Simulator::Schedule(m_updateInterval, &LeoSimBeamManager::UpdateCycle, this);
    NS_LOG_DEBUG("Next UpdateCycle scheduled at time " << (currentTime + m_updateInterval).GetSeconds()
                                                        << "s");
}

double
LeoSimBeamManager::ComputeEndToEndLatency(uint32_t ueNodeId, uint32_t satNodeId) const
{
    NS_LOG_FUNCTION(this << ueNodeId << satNodeId);
    
    if (!m_loader)
    {
        NS_LOG_WARN("Loader not configured for latency computation");
        return 100.0; // Default latency in ms
    }

    // Step 1: Translate NodeIds to loader IDs and get positions from loader
    uint32_t ueDeviceId = GetLeoSimIdFromNodeId(ueNodeId);
    uint32_t satId = GetLeoSimIdFromNodeId(satNodeId);

    Vector uePos = m_loader->GetGroundDevicePosition(ueDeviceId);

    // Step 2: Get satellite position at current time
    Vector satPos = m_loader->GetSatellitePositionAt(satId, Simulator::Now());

    // Step 3: Compute UE-to-satellite distance using 3D Euclidean distance
    double dx = satPos.x - uePos.x;
    double dy = satPos.y - uePos.y;
    double dz = satPos.z - uePos.z;
    double ueToSatDistance = std::sqrt(dx * dx + dy * dy + dz * dz);

    // Step 4: Get satellite-to-server distance
    // TODO: If m_routingCalculator provides route distance to nearest server,
    // use that. For now, estimate using satellite altitude as proxy.
    double satToServerDistance = satPos.z; // Rough approximation

    if (m_routingCalculator)
    {
        // Attempt to compute route from satellite to a destination
        // This is a placeholder - in production, identify the target server node
        // For now, accumulate only the UE-to-satellite segment
        NS_LOG_DEBUG("Routing calculator available but server node ID not determined; "
                     "using UE-to-satellite segment only");
    }

    // Step 5: Total distance = UE-to-satellite + satellite-to-server
    double totalDistance = ueToSatDistance + satToServerDistance;

    // Convert total distance in metres to milliseconds:
    // latencyMs = (distanceMeters / speedOfLight) * 1000.0
    const double speedOfLight = 3e8; // m/s
    double latencyMs = (totalDistance / speedOfLight) * 1000.0;

    NS_LOG_DEBUG("UE " << ueNodeId << " to satellite " << satNodeId 
                       << ": UE-sat distance=" << ueToSatDistance << "m, "
                       << "sat-server distance=" << satToServerDistance << "m, "
                       << "total distance=" << totalDistance << "m, "
                       << "latency=" << latencyMs << "ms");

    return latencyMs;
}

bool
LeoSimBeamManager::IsPingPong(uint32_t ueNodeId, uint32_t sourceSatId, uint32_t targetSatId)
{
    NS_LOG_FUNCTION(this << ueNodeId << sourceSatId << targetSatId);

    if (m_handoverHistory.empty())
    {
        // No history yet
        return false;
    }

    // Find all handover events for this UE and extract recent satellite changes
    std::vector<uint32_t> recentHandovers;
    Time nowTime = Simulator::Now();
    Time windowStart = nowTime - Seconds(30.0); // 30-second window

    for (const auto& event : m_handoverHistory)
    {
        if (event.ueNodeId == ueNodeId && event.completedAt >= windowStart &&
            event.completedAt <= nowTime)
        {
            recentHandovers.push_back(event.targetSatId);
        }
    }

    if (recentHandovers.size() < 3)
    {
        return false; // Need at least 3 handovers to detect ping-pong
    }

    // Check if the last three handovers show ping-pong pattern
    // Ping-pong: alternation between two satellites
    size_t n = recentHandovers.size();
    uint32_t sat1 = recentHandovers[n - 1];
    uint32_t sat2 = recentHandovers[n - 2];
    uint32_t sat3 = recentHandovers[n - 3];

    bool isPingPong = (sat1 == targetSatId && sat2 == sourceSatId && sat3 == targetSatId) ||
                      (sat1 == sourceSatId && sat2 == targetSatId && sat3 == sourceSatId);

    if (isPingPong)
    {
        NS_LOG_INFO("Ping-pong detected for UE " << ueNodeId << " between satellites "
                                                 << sourceSatId << " and " << targetSatId);
    }

    return isPingPong;
}

void
LeoSimBeamManager::Start(NodeContainer groundNodes,
                         NodeContainer satellites,
                         Time simStart,
                         Time simDuration)
{
    NS_LOG_FUNCTION(this << groundNodes.GetN() << " ground nodes, " << satellites.GetN()
                         << " satellites");

    m_userEquipment = groundNodes;
    m_satellites = satellites;
    m_simStart = simStart;
    m_simDuration = simDuration;

    NS_LOG_INFO("LeoSimBeamManager started at time " << simStart.GetSeconds()
                                                     << "s with simulation duration "
                                                     << simDuration.GetSeconds() << "s");

    // Initialize raw sockets for each managed ground node to support packet buffering and transmission
    for (uint32_t i = 0; i < groundNodes.GetN(); i++)
    {
        Ptr<Node> ueNode = groundNodes.Get(i);
        uint32_t ueNodeId = ueNode->GetId();

        // Create a raw socket (SOCK_RAW) for packet transmission
        // Using IPPROTO_RAW allows sending raw IP packets
        Ptr<Socket> socket = Socket::CreateSocket(ueNode, TypeId::LookupByName("ns3::UdpSocketFactory"));

        if (socket)
        {
            m_ueSocketMap[ueNodeId] = socket;
            NS_LOG_DEBUG("Created socket for UE " << ueNodeId);
        }
        else
        {
            NS_LOG_WARN("Failed to create socket for UE " << ueNodeId);
        }

        // Extract IPv4 address from UE node for flow monitoring
        Ptr<Ipv4> ipv4 = ueNode->GetObject<Ipv4>();
        if (ipv4)
        {
            // Get the first interface (excluding loopback)
            uint32_t numInterfaces = ipv4->GetNInterfaces();
            for (uint32_t ifIdx = 1; ifIdx < numInterfaces; ifIdx++)
            {
                Ipv4InterfaceAddress ifAddr = ipv4->GetAddress(ifIdx, 0);
                Ipv4Address addr = ifAddr.GetLocal();
                if (addr != Ipv4Address("127.0.0.1"))
                {
                    m_ueAddressMap[ueNodeId] = addr;
                    NS_LOG_DEBUG("Registered UE " << ueNodeId << " with IPv4 address " << addr);
                    break;
                }
            }
        }
    }

    // === Initial Serving Beam Assignment ===
    // Populate m_currentBeams so UpdateCycle() has something to process and so
    // visualization can log the serving satellite from t=0.
    for (uint32_t i = 0; i < groundNodes.GetN(); i++)
    {
        Ptr<Node> ueNode = groundNodes.Get(i);
        if (!ueNode)
        {
            continue;
        }

        uint32_t ueNodeId = ueNode->GetId();
        std::vector<LeoSimBeamRecord> visible = ScanVisibleSatellites(ueNodeId);
        std::vector<LeoSimTopsisCandidate> ranked = RankByTopsis(visible, ueNodeId);

        if (ranked.empty())
        {
            NS_LOG_WARN("No visible satellites to attach UE " << ueNodeId << " at start time ");
            continue;
        }

        LeoSimBeamRecord serving = ranked[0].beamRecord;
        serving.state = LEOSIM_BEAM_CONNECTED;
        serving.associationTime = Simulator::Now();
        m_currentBeams[ueNodeId] = serving;

        if (!m_beamStateCallback.IsNull())
        {
            m_beamStateCallback(ueNodeId, serving, ranked[0].topsisScore);
        }

        NS_LOG_INFO("Initial beam association for UE " << ueNodeId << " -> satellite "
                                                      << serving.satelliteNodeId
                                                      << " (TOPSIS=" << ranked[0].topsisScore
                                                      << ")");
    }

    // Kick off ephemeris-based pre-scheduling once we have an initial serving beam.
    // Without this, TIME_BASED (TTE) handovers will only be scheduled after a
    // previous handover completes.
    PreScheduleEphemerisHandovers(Seconds(600));

    // Schedule the first update cycle after the start time
    m_updateEventId = Simulator::Schedule(simStart + m_updateInterval, &LeoSimBeamManager::UpdateCycle, this);
    NS_LOG_DEBUG("Scheduled first UpdateCycle at time " << (simStart + m_updateInterval).GetSeconds()
                                                         << "s with interval "
                                                         << m_updateInterval.GetSeconds() << "s");

    if (m_cfg.beamHoppingEnabled && m_multiBeamModel)
    {
        auto& allBeams = m_multiBeamModel->GetAllBeamsMutable();
        for (auto& [satId, beams] : allBeams)
        {
            auto mgrIt = m_beamHopManagers.find(satId);
            if (mgrIt == m_beamHopManagers.end() || !mgrIt->second)
            {
                m_beamHopManagers[satId] = CreateObject<LeoSimBeamHoppingManager>(
                    static_cast<uint32_t>(beams.size()),
                    std::max<uint32_t>(1, m_cfg.beamHopCycleSlotsN),
                    std::max<uint32_t>(1, m_cfg.beamHopSlotMs),
                    std::max<uint32_t>(1, m_cfg.frequencyReuseColors));
            }

            std::vector<double> demand;
            demand.reserve(beams.size());
            for (const auto& b : beams)
            {
                demand.push_back(std::max(0.0, b.currentLoad));
            }

            m_beamHopManagers[satId]->GenerateSchedule(demand);
            uint32_t currentSlot =
                (simStart.GetMilliSeconds() / std::max<uint32_t>(1, m_cfg.beamHopSlotMs)) %
                std::max<uint32_t>(1, m_cfg.beamHopCycleSlotsN);
            const auto active = m_beamHopManagers[satId]->GetActiveBeams(currentSlot);
            for (auto& beam : beams)
            {
                beam.activeInCurrentSlot =
                    std::find(active.begin(), active.end(), beam.beamId) != active.end();
            }
        }

        Simulator::Schedule(MilliSeconds(std::max<uint32_t>(1, m_cfg.beamHopSlotMs)),
                            &LeoSimBeamManager::AdvanceBeamHoppingSlot,
                            this);
    }
    
    NS_LOG_INFO("Initialized " << m_ueSocketMap.size() << " UE sockets for handover buffering");
}

void
LeoSimBeamManager::AdvanceBeamHoppingSlot()
{
    NS_LOG_FUNCTION(this);

    if (!m_cfg.beamHoppingEnabled || !m_multiBeamModel)
    {
        return;
    }

    const uint32_t slotMs = std::max<uint32_t>(1, m_cfg.beamHopSlotMs);
    const uint32_t cycleSlots = std::max<uint32_t>(1, m_cfg.beamHopCycleSlotsN);
    uint32_t currentSlot = (Simulator::Now().GetMilliSeconds() / slotMs) % cycleSlots;

    auto& allBeams = m_multiBeamModel->GetAllBeamsMutable();
    for (auto& [satId, beams] : allBeams)
    {
        auto mgrIt = m_beamHopManagers.find(satId);
        if (mgrIt == m_beamHopManagers.end() || !mgrIt->second)
        {
            continue;
        }

        const auto activeList = mgrIt->second->GetActiveBeams(currentSlot);
        for (auto& beam : beams)
        {
            bool wasActive = beam.activeInCurrentSlot;
            bool isActive = std::find(activeList.begin(), activeList.end(), beam.beamId) != activeList.end();
            beam.activeInCurrentSlot = isActive;

            if (wasActive && !isActive)
            {
                HandleBeamDark(satId, beam.beamId);
            }
        }
    }

    Simulator::Schedule(MilliSeconds(slotMs), &LeoSimBeamManager::AdvanceBeamHoppingSlot, this);
}

void
LeoSimBeamManager::HandleBeamDark(uint32_t satId, uint32_t beamId)
{
    NS_LOG_FUNCTION(this << satId << beamId);

    std::vector<uint32_t> impactedUes;
    for (const auto& [ueId, rec] : m_currentBeams)
    {
        if (rec.satelliteNodeId == satId && rec.beamId == beamId)
        {
            impactedUes.push_back(ueId);
        }
    }

    for (uint32_t ueId : impactedUes)
    {
        EvaluateIntraBeamNeed(ueId);
    }
}

LeoSimBeamRecord
LeoSimBeamManager::GetCurrentBeam(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);

    auto it = m_currentBeams.find(ueNodeId);
    if (it != m_currentBeams.end())
    {
        return it->second;
    }

    LeoSimBeamRecord empty;
    empty.ueNodeId = ueNodeId;
    empty.satelliteNodeId = std::numeric_limits<uint32_t>::max();
    empty.rsrp = -200.0;
    empty.snr = 0.0;
    empty.pathLoss = 0.0;
    empty.elevationAngle = 0.0;
    empty.remainingServiceTime = 0.0;
    empty.satelliteLoad = 0.0;
    empty.endToEndLatency = 0.0;
    empty.associationTime = Simulator::Now();
    empty.state = LEOSIM_BEAM_SEARCHING;
    return empty;
}

std::vector<LeoSimTopsisCandidate>
LeoSimBeamManager::GetRankedCandidates(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);
    std::vector<LeoSimBeamRecord> visible = ScanVisibleSatellites(ueNodeId);
    return RankByTopsis(visible, ueNodeId);
}

double
LeoSimBeamManager::GetTimeToExit(uint32_t ueNodeId, uint32_t satNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId << satNodeId);
    return ComputeTte(ueNodeId, satNodeId);
}

std::vector<LeoSimHandoverEvent>
LeoSimBeamManager::GetHandoverHistory() const
{
    return m_handoverHistory;
}

uint32_t
LeoSimBeamManager::GetHandoverCount(uint32_t ueNodeId) const
{
    uint32_t cnt = 0;
    for (const auto& evt : m_handoverHistory)
    {
        if (evt.ueNodeId == ueNodeId)
        {
            cnt++;
        }
    }
    return cnt;
}

double
LeoSimBeamManager::GetAverageHandoverLatencyMs() const
{
    double sum = 0.0;
    uint32_t n = 0;
    for (const auto& evt : m_handoverHistory)
    {
        if (evt.success)
        {
            sum += evt.handoverLatencyMs;
            n++;
        }
    }
    return (n > 0) ? (sum / n) : 0.0;
}

uint32_t
LeoSimBeamManager::GetPingPongCount() const
{
    return m_pingPongCount;
}

void
LeoSimBeamManager::BufferPacket(uint32_t ueNodeId, Ptr<Packet> pkt)
{
    NS_LOG_FUNCTION(this << ueNodeId << pkt);

    if (!m_handoverBufferingEnabled)
    {
        NS_LOG_DEBUG("Handover buffering disabled for UE " << ueNodeId << ", packet not buffered");
        return;
    }

    // Get reference to the buffer queue for this UE
    std::queue<Ptr<Packet>>& buffer = m_handoverBuffers[ueNodeId];

    // Check if buffer has space
    if (buffer.size() < m_maxBufferSize)
    {
        buffer.push(pkt);
        NS_LOG_DEBUG("Buffered packet for UE " << ueNodeId << ", buffer size now: "
                                               << buffer.size() << " packets");

        // Update the packets buffered counter in the most recent handover event for this UE
        if (!m_handoverHistory.empty())
        {
            for (auto it = m_handoverHistory.rbegin(); it != m_handoverHistory.rend(); ++it)
            {
                if (it->ueNodeId == ueNodeId)
                {
                    it->packetsBuffered++;
                    break;
                }
            }
        }
    }
    else
    {
        // Buffer is full - drop the packet and increment drop counter
        NS_LOG_DEBUG("Buffer full for UE " << ueNodeId << " (size=" << buffer.size()
                                           << " >= max=" << m_maxBufferSize
                                           << "), dropping incoming packet");

        // Try to update the drop counter in the current handover event for this UE
        if (!m_handoverHistory.empty())
        {
            // Find the most recent handover event for this UE
            for (auto it = m_handoverHistory.rbegin(); it != m_handoverHistory.rend(); ++it)
            {
                if (it->ueNodeId == ueNodeId)
                {
                    it->packetsDropped++;
                    NS_LOG_DEBUG("Incremented dropped packet counter to " << it->packetsDropped
                                                                          << " for UE " << ueNodeId);
                    break;
                }
            }
        }
    }
}

void
LeoSimBeamManager::FlushBuffer(uint32_t ueNodeId)
{
    NS_LOG_FUNCTION(this << ueNodeId);

    // Return if no buffer exists for this UE
    auto bufferIt = m_handoverBuffers.find(ueNodeId);
    if (bufferIt == m_handoverBuffers.end())
    {
        NS_LOG_DEBUG("No buffer found for UE " << ueNodeId);
        return;
    }

    std::queue<Ptr<Packet>>& buffer = bufferIt->second;
    uint32_t packetsFlushed = 0;

    // Check if socket exists for this UE
    auto socketIt = m_ueSocketMap.find(ueNodeId);
    if (socketIt == m_ueSocketMap.end())
    {
        NS_LOG_WARN("No socket found for UE " << ueNodeId << ", cannot flush buffer");
        // Clear the buffer anyway
        while (!buffer.empty())
        {
            buffer.pop();
        }
        m_handoverBuffers.erase(bufferIt);
        return;
    }

    Ptr<Socket> socket = socketIt->second;

    // Send all buffered packets through the socket
    while (!buffer.empty())
    {
        Ptr<Packet> pkt = buffer.front();
        buffer.pop();

        if (socket)
        {
            socket->Send(pkt);
            packetsFlushed++;
        }
    }

    // Erase the buffer entry for this UE
    m_handoverBuffers.erase(bufferIt);

    NS_LOG_INFO("Flushed " << packetsFlushed << " packets from buffer for UE " << ueNodeId);
}

void
LeoSimBeamManager::SetFlowMonitor(Ptr<FlowMonitor> monitor)
{
    NS_LOG_FUNCTION(this << monitor);
    m_flowMonitor = monitor;
    NS_LOG_DEBUG("Flow monitor set for beam manager");
}

void
LeoSimBeamManager::SetFlowClassifier(Ptr<Ipv4FlowClassifier> classifier)
{
    NS_LOG_FUNCTION(this << classifier);
    m_flowClassifier = classifier;
    NS_LOG_DEBUG("Flow classifier set for beam manager");
}

void
LeoSimBeamManager::SnapshotFlowStats(uint32_t ueNodeId,
                                     LeoSimHandoverEvent& evt,
                                     Ptr<FlowMonitor> monitor,
                                     Ptr<Ipv4FlowClassifier> classifier)
{
    NS_LOG_FUNCTION(this << ueNodeId);

    // Return early if flow monitoring is not enabled
    if (!monitor || !classifier)
    {
        NS_LOG_DEBUG("Flow monitoring disabled, skipping flow stats snapshot for UE " << ueNodeId);
        return;
    }

    // Flush pending packets in the monitor
    monitor->CheckForLostPackets();

    // Get the full stats map
    FlowMonitor::FlowStatsContainer flowStats = monitor->GetFlowStats();

    // Look up the UE's IPv4 address
    auto addrIt = m_ueAddressMap.find(ueNodeId);
    if (addrIt == m_ueAddressMap.end())
    {
        NS_LOG_DEBUG("No IPv4 address registered for UE " << ueNodeId << ", skipping flow stats");
        return;
    }

    Ipv4Address ueAddress = addrIt->second;
    uint32_t totalLostPackets = 0;
    double totalDelayMs = 0.0;
    uint32_t totalFlowsFound = 0;

    // Iterate through all flows and find those involving this UE
    for (auto& flowEntry : flowStats)
    {
        uint32_t flowId = flowEntry.first;
        FlowMonitor::FlowStats& flow = flowEntry.second;

        // Get the 5-tuple for this flow
        Ipv4FlowClassifier::FiveTuple fiveTuple = classifier->FindFlow(flowId);

        // Check if this flow involves the UE (as source or destination)
        if (fiveTuple.sourceAddress == ueAddress || fiveTuple.destinationAddress == ueAddress)
        {
            // Aggregate lost packets from this flow
            totalLostPackets += flow.lostPackets;

            // Calculate mean delay if available
            if (flow.rxPackets > 0)
            {
                Time meanDelay = flow.delaySum / flow.rxPackets;
                totalDelayMs += meanDelay.GetMilliSeconds();
            }

            totalFlowsFound++;

            NS_LOG_DEBUG("Flow " << flowId << " for UE " << ueNodeId << ": "
                                 << "lost=" << flow.lostPackets << ", rxPackets=" << flow.rxPackets
                                 << ", delay=" << (flow.rxPackets > 0 ? (flow.delaySum / flow.rxPackets).GetMilliSeconds() : 0.0)
                                 << "ms");
        }
    }

    // Update the handover event with captured statistics
    evt.packetsDropped += totalLostPackets;

    if (totalFlowsFound > 0)
    {
        NS_LOG_INFO("Captured flow stats for UE " << ueNodeId << " during handover: "
                                                  << "flows=" << totalFlowsFound << ", "
                                                  << "totalLostPackets=" << totalLostPackets << ", "
                                                  << "totalDelayMs=" << totalDelayMs);
    }
    else
    {
        NS_LOG_DEBUG("No flows found for UE " << ueNodeId);
    }
}

void
LeoSimBeamManager::Stop()
{
    NS_LOG_FUNCTION(this);

    // Cancel any pending update cycle event
    if (!m_updateEventId.IsExpired())
    {
        Simulator::Cancel(m_updateEventId);
        NS_LOG_DEBUG("Cancelled pending UpdateCycle event");
    }

    // Cancel all TTT events
    for (auto& entry : m_tttEventIds)
    {
        if (!entry.second.IsExpired())
        {
            Simulator::Cancel(entry.second);
        }
    }
    m_tttEventIds.clear();

    // === Compute KPI Summary ===
    uint32_t totalHo = m_handoverHistory.size();
    uint32_t successfulHo = 0;
    double sumLatency = 0.0;
    double sumScoreImprovement = 0.0;
    uint32_t successfulCnt = 0;

    // Per-type counters
    uint32_t cntIntraBeam = 0;
    uint32_t cntInterSat = 0;
    uint32_t cntInterOrbit = 0;

    // Per-trigger counters
    std::map<LeoSimHandoverTrigger, uint32_t> triggerCounts;
    triggerCounts[LEOSIM_HO_A3] = 0;
    triggerCounts[LEOSIM_HO_A4] = 0;
    triggerCounts[LEOSIM_HO_TIME_BASED] = 0;
    triggerCounts[LEOSIM_HO_LOCATION_BASED] = 0;
    triggerCounts[LEOSIM_HO_ELEVATION] = 0;
    triggerCounts[LEOSIM_HO_RLF] = 0;
    triggerCounts[LEOSIM_HO_LOAD_BALANCE] = 0;

    // Process handover history
    for (const auto& evt : m_handoverHistory)
    {
        if (evt.success)
        {
            successfulHo++;
            sumLatency += evt.handoverLatencyMs;
            sumScoreImprovement += (evt.sinrAfter - evt.sinrBefore);
            successfulCnt++;
        }

        // Count by type
        if (evt.type == LEOSIM_HO_INTRA_BEAM)
            cntIntraBeam++;
        else if (evt.type == LEOSIM_HO_INTER_SATELLITE)
            cntInterSat++;
        else if (evt.type == LEOSIM_HO_INTER_ORBIT)
            cntInterOrbit++;

        // Count by trigger
        triggerCounts[evt.trigger]++;
    }

    uint32_t failedHo = totalHo - successfulHo;
    double failureRate = (totalHo > 0) ? (100.0 * failedHo / totalHo) : 0.0;
    double avgLatency = (successfulCnt > 0) ? (sumLatency / successfulCnt) : 0.0;
    double avgScoreImp = (successfulCnt > 0) ? (sumScoreImprovement / successfulCnt) : 0.0;
    double pingPongRate = (totalHo > 0) ? (100.0 * m_pingPongCount / totalHo) : 0.0;

    // Compute per-UE average dwell time (inter-handover time intervals)
    std::map<uint32_t, std::vector<double>> ueDwellTimes;
    for (const auto& evt : m_handoverHistory)
    {
        ueDwellTimes[evt.ueNodeId].push_back(evt.completedAt.GetSeconds());
    }

    std::map<uint32_t, double> ueAvgDwellTime;
    for (auto& entry : ueDwellTimes)
    {
        uint32_t ueId = entry.first;
        std::vector<double>& times = entry.second;
        if (times.size() > 1)
        {
            std::sort(times.begin(), times.end());
            double sumDwell = 0.0;
            for (size_t i = 1; i < times.size(); ++i)
            {
                sumDwell += (times[i] - times[i - 1]);
            }
            ueAvgDwellTime[ueId] = sumDwell / (times.size() - 1);
        }
        else
        {
            ueAvgDwellTime[ueId] = 0.0;
        }
    }

    // === Output Banner and KPI Summary ===
    NS_LOG_INFO("");
    NS_LOG_INFO("========================================");
    NS_LOG_INFO("  === LeoSim Beam Manager KPI Summary === ");
    NS_LOG_INFO("========================================");
    NS_LOG_INFO("");
    NS_LOG_INFO("Overall Statistics:");
    NS_LOG_INFO("  - Total handovers:                 " << totalHo);
    NS_LOG_INFO("  - Successful handovers:            " << successfulHo);
    NS_LOG_INFO("  - Failed handovers:                " << failedHo);
    NS_LOG_INFO("  - Handover failure rate:           " << std::fixed << std::setprecision(2)
                                                         << failureRate << "%");
    NS_LOG_INFO("  - Average handover latency:        " << std::fixed << std::setprecision(2)
                                                         << avgLatency << " ms");
    NS_LOG_INFO("  - Ping-pong count:                 " << m_pingPongCount);
    NS_LOG_INFO("  - Ping-pong rate:                  " << std::fixed << std::setprecision(2)
                                                         << pingPongRate << "%");
    NS_LOG_INFO("  - Avg TOPSIS score improvement:    " << std::fixed << std::setprecision(4)
                                                         << avgScoreImp);
    NS_LOG_INFO("");
    NS_LOG_INFO("Per-UE Average Dwell Time (seconds):");
    for (const auto& entry : ueAvgDwellTime)
    {
        NS_LOG_INFO("  - UE " << entry.first << ": " << std::fixed << std::setprecision(3)
                             << entry.second << "s");
    }
    NS_LOG_INFO("");
    NS_LOG_INFO("Handover Type Breakdown:");
    NS_LOG_INFO("  - Intra-beam:                      " << cntIntraBeam);
    NS_LOG_INFO("  - Inter-satellite:                 " << cntInterSat);
    NS_LOG_INFO("  - Inter-orbit:                     " << cntInterOrbit);
    NS_LOG_INFO("");
    NS_LOG_INFO("Handover Trigger Breakdown:");
    NS_LOG_INFO("  - A3 event:                        " << triggerCounts[LEOSIM_HO_A3]);
    NS_LOG_INFO("  - A4 event:                        " << triggerCounts[LEOSIM_HO_A4]);
    NS_LOG_INFO("  - Time-based (TTE):                " << triggerCounts[LEOSIM_HO_TIME_BASED]);
    NS_LOG_INFO("  - Location-based:                  " << triggerCounts[LEOSIM_HO_LOCATION_BASED]);
    NS_LOG_INFO("  - Elevation:                       " << triggerCounts[LEOSIM_HO_ELEVATION]);
    NS_LOG_INFO("  - RLF (Radio Link Failure):        " << triggerCounts[LEOSIM_HO_RLF]);
    NS_LOG_INFO("  - Load balancing:                  " << triggerCounts[LEOSIM_HO_LOAD_BALANCE]);
    NS_LOG_INFO("");
    NS_LOG_INFO("========================================");
    NS_LOG_INFO("  Simulation stopped at " << Simulator::Now().GetSeconds() << "s");
    NS_LOG_INFO("========================================");
    NS_LOG_INFO("");
}

} // namespace ns3
