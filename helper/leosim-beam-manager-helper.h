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

#ifndef LEOSIM_BEAM_MANAGER_HELPER_H
#define LEOSIM_BEAM_MANAGER_HELPER_H

#include "../model/leosim-beam-manager.h"
#include "../model/leosim-channel-model.h"
#include "../model/leosim-loader.h"
#include "../model/leosim-routing-calculator.h"

#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"
#include "ns3/flow-monitor.h"
#include "ns3/ipv4-flow-classifier.h"

#include <array>
#include <string>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Helper class to instantiate and configure LeoSimBeamManager
 *
 * This helper simplifies the setup of LEO beam management by providing
 * a fluent interface for configuration. All parameters have sensible
 * defaults matching 3GPP Release 17 CHO specifications.
 *
 * Usage:
 * \code
 * LeoSimBeamManagerHelper beamHelper;
 * beamHelper.SetChannelModel(channelModel);
 * beamHelper.SetRoutingCalculator(routingCalculator);
 * beamHelper.SetVerbose(true);
 * Ptr<LeoSimBeamManager> beamManager = beamHelper.Install(ueNodes, satNodes, simTime);
 * \endcode
 */
class LeoSimBeamManagerHelper
{
  public:
    /**
     * \brief Constructor with default parameter initialization
     */
    LeoSimBeamManagerHelper();

    /**
     * \brief Destructor
     */
    ~LeoSimBeamManagerHelper();

    /**
     * \name Channel and Network Configuration
     * @{
     */

    /**
     * \brief Set the propagation channel model for ground links
     * \param channelModel Pointer to LeoSimChannelModel instance
     */
    void SetChannelModel(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Set the ISL (inter-satellite link) channel model
     * \param islChannelModel Pointer to ISL channel model
     */
    void SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel);

    /**
     * \brief Set the routing calculator
     * \param routingCalculator Pointer to LeoSimRoutingCalculator instance
     */
    void SetRoutingCalculator(Ptr<LeoSimRoutingCalculator> routingCalculator);

    /**
     * \brief Set the satellite/beam loader
     * \param loader Pointer to LeoSimLoader instance
     */
    void SetLoader(Ptr<LeoSimLoader> loader);

    /** @} */

    /**
     * \name Handover Mode Configuration
     * @{
     */

    /**
     * \brief Set the handover mode (BHO or CHO)
     * \param mode Handover mode (LEOSIM_HO_MODE_BHO or LEOSIM_HO_MODE_CHO)
     */
    void SetHandoverMode(LeoSimHandoverMode mode);

    /**
     * \brief Enable or disable earth-fixed beam mode
     * \param enable True for earth-fixed beams, false for body-fixed
     */
    void SetEarthFixedBeamMode(bool enable);

    /** @} */

    /**
     * \name 3GPP Timer Configuration (TS 38.321)
     * @{
     */

    /**
     * \brief Set the TTT (Time-To-Trigger) timer duration
     * \param ttt Timer duration (default: 1 second)
     */
    void SetTtt(Time ttt);

    /**
     * \brief Set the T310 (RLF detection) timer duration
     * \param t310 Timer duration (default: 1 second)
     */
    void SetT310(Time t310);

    /**
     * \brief Set the N310 RLF event counter
     * \param n310 Number of RLF events to trigger (default: 3)
     */
    void SetN310(uint32_t n310);

    /**
     * \brief Set the N311 RLF recovery counter
     * \param n311 Number of in-sync events to recover (default: 3)
     */
    void SetN311(uint32_t n311);

    /** @} */

    /**
     * \name Handover Trigger Thresholds
     * @{
     */

    /**
     * \brief Set the A3 event offset
     * \param offsetDb A3 offset in dB (default: 3.0)
     */
    void SetA3Offset(double offsetDb);

    /**
     * \brief Set the A4 absolute threshold
     * \param thresholdDbm A4 threshold in dBm (default: -110.0)
     */
    void SetA4Threshold(double thresholdDbm);

    /**
     * \brief Set the minimum elevation angle threshold
     * \param elevationDeg Min elevation in degrees (default: 10.0)
     */
    void SetElevationThreshold(double elevationDeg);

    /**
     * \brief Set the Time-To-Exit threshold for handover trigger
     * \param tte TTE threshold (default: 30 seconds)
     */
    void SetTteThreshold(Time tte);

    /** @} */

    /**
     * \name TOPSIS Multi-Criteria Configuration
     * @{
     */

    /**
     * \brief Set TOPSIS weighting coefficients
     * 
     * Coefficients for: RSRP, TTE, Satellite Load (inverted), Latency (inverted), Elevation Angle.
     * Should sum to approximately 1.0. Default: [0.30, 0.30, 0.15, 0.15, 0.10]
     * 
     * \param w1 RSRP weight
     * \param w2 TTE weight
     * \param w3 Load weight (inverted)
     * \param w4 Latency weight (inverted)
     * \param w5 Elevation angle weight
     */
    void SetTopsisWeights(double w1, double w2, double w3, double w4, double w5);

    /**
     * \brief Set the maximum number of CHO candidates
     * \param maxCandidates Max candidates per CHO preparation (default: 3)
     */
    void SetMaxCandidates(uint32_t maxCandidates);

    /** @} */

    /**
     * \name CHO Phase Timing
     * @{
     */

    /**
     * \brief Set CHO Phase 1 (preparation) delay
     * \param delay Preparation delay (default: 100 ms)
     */
    void SetChoPreparationDelay(Time delay);

    /**
     * \brief Set CHO Phase 3 (execution) delay
     * \param delay Execution delay (default: 150 ms)
     */
    void SetChoExecutionDelay(Time delay);

    /**
     * \brief Set the beam manager periodic update interval
     * \param interval Update cycle period (default: 100 ms)
     */
    void SetUpdateInterval(Time interval);

    /** @} */

    /**
     * \name Load Balancing and Buffering
     * @{
     */

    /**
     * \brief Enable or disable load balancing during handovers
     * \param enable True to enable load balancing
     */
    void EnableLoadBalancing(bool enable);

    /**
     * \brief Enable or disable packet buffering during handover
     * \param enable True to enable buffering
     */
    void EnableHandoverBuffering(bool enable);

    /** @} */

    /**
     * \name Flow Monitoring Integration
     * @{
     */

    /**
     * \brief Enable flow monitoring for capturing per-flow statistics
     * 
     * When enabled, SnapshotFlowStats is called at handover completion
     * to capture KPIs like lost packets, latency, and throughput.
     * 
     * \param monitor Pointer to FlowMonitor instance
     * \param classifier Pointer to Ipv4FlowClassifier instance
     */
    void EnableFlowMonitorIntegration(Ptr<FlowMonitor> monitor,
                                      Ptr<Ipv4FlowClassifier> classifier);

    /** @} */

    /**
     * \name Output Tracing and Logging
     * @{
     */

    /**
     * \brief Enable beam event logging to files
     * 
     * Configures output trace files for detailed handover analysis:
     * - beamFile: Current serving beam per UE at each update
     * - handoverFile: Complete handover history with metrics
     * - choFile: CHO state machine events and condition evaluations
     * 
     * \param beamFile Path to beam trace file (empty to disable)
     * \param handoverFile Path to handover trace file
     * \param choFile Path to CHO trace file
     */
    void EnableBeamLogging(std::string beamFile,
                          std::string handoverFile,
                          std::string choFile);

    /**
     * \brief Enable verbose console output for debugging
     * \param verbose True to enable debug logging
     */
    void SetVerbose(bool verbose);

    /** @} */

    /**
     * \name Installation
     * @{
     */

    /**
     * \brief Create and install configured LeoSimBeamManager
     * 
     * This method creates a new LeoSimBeamManager instance, applies all
     * configured parameters, starts the manager with the provided nodes
     * and simulation duration, and returns the configured manager.
     * 
     * \param ueNodes Container of UE nodes
     * \param satNodes Container of satellite nodes
     * \param simTime Total simulation duration
     * \return Pointer to configured and started LeoSimBeamManager
     */
    Ptr<LeoSimBeamManager> Install(NodeContainer ueNodes,
                                    NodeContainer satNodes,
                                    Time simTime);

    /** @} */

  private:
    // Internal beam manager instance
    Ptr<LeoSimBeamManager> m_manager;

    // Channel and networking components
    Ptr<LeoSimChannelModel> m_channelModel;
    Ptr<LeoSimChannelModel> m_islChannelModel;
    Ptr<LeoSimRoutingCalculator> m_routingCalculator;
    Ptr<LeoSimLoader> m_loader;

    // Handover mode configuration
    LeoSimHandoverMode m_hoMode;
    bool m_earthFixedBeam;

    // 3GPP timers
    Time m_ttt;
    Time m_t310;
    uint32_t m_n310;
    uint32_t m_n311;

    // Handover thresholds
    double m_a3Offset;
    double m_a4Threshold;
    double m_elevationThreshold;
    Time m_tteThreshold;

    // TOPSIS weights
    std::array<double, 5> m_topsisWeights;
    uint32_t m_maxCandidates;

    // CHO timing
    Time m_choPreparationDelay;
    Time m_choExecutionDelay;
    Time m_updateInterval;

    // Load balancing and buffering
    bool m_loadBalancingEnabled;
    bool m_handoverBufferingEnabled;

    // Flow monitoring
    Ptr<FlowMonitor> m_flowMonitor;
    Ptr<Ipv4FlowClassifier> m_flowClassifier;

    // Tracing
    std::string m_beamFile;
    std::string m_handoverFile;
    std::string m_choFile;

    // Debug
    bool m_verbose;
};

} // namespace ns3

#endif /* LEOSIM_BEAM_MANAGER_HELPER_H */
