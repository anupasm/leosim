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

#ifndef LEOSIM_BEAM_MANAGER_H
#define LEOSIM_BEAM_MANAGER_H

#include "leosim-channel-model.h"
#include "leosim-beam-hopping-manager.h"
#include "leosim-loader.h"
#include "leosim-multi-beam-model.h"
#include "leosim-operator-model.h"
#include "leosim-routing-calculator.h"
#include "leosim-sinr-engine.h"

#include "ns3/callback.h"
#include "ns3/ipv4-address.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/packet.h"
#include "ns3/socket.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"

#include <array>
#include <cstdint>
#include <map>
#include <queue>
#include <set>
#include <vector>

namespace ns3
{

class LeoSimMultiBeamModel;

/**
 * \ingroup leosim
 * \brief Handover mode for LeoSim beam management
 *
 * Specifies whether the system uses legacy reactive handovers (BHO)
 * or 3GPP Release 17 Conditional Handover (CHO) with predictive capabilities.
 */
enum LeoSimHandoverMode
{
    LEOSIM_HO_MODE_BHO = 0, //!< Legacy reactive handover (Best-effort Handover)
    LEOSIM_HO_MODE_CHO = 1  //!< 3GPP Release 17 Conditional Handover (default)
};

/**
 * \ingroup leosim
 * \brief Type of handover in the Leo constellation
 *
 * Categorizes handovers based on the geographic relationship between
 * the serving and target satellites.
 */
enum LeoSimHandoverType
{
    LEOSIM_HO_INTRA_BEAM = 0,        //!< Same satellite, different spot-beam
    LEOSIM_HO_INTER_SATELLITE = 1,   //!< Different satellite, same orbital plane
    LEOSIM_HO_INTER_ORBIT = 2,       //!< Different satellite, different orbital plane
    LEOSIM_HO_INTRA_BEAM_ADJACENT,   ///< Adjacent hex-ring beam, same satellite
    LEOSIM_HO_INTRA_BEAM_DISTANT,    ///< Non-adjacent beam, same satellite
    LEOSIM_HO_BEAM_HOPPING_DARK,     ///< Forced by beam hopping - serving beam went dark
};

/**
 * \ingroup leosim
 * \brief Trigger event that initiates a handover
 *
 * Specifies the measurement event, failure condition, or optimization trigger
 * that causes the handover to be initiated.
 */
enum LeoSimHandoverTrigger
{
    LEOSIM_HO_A3 = 0,            //!< A3 event: neighbour RSRP exceeds serving + offset
    LEOSIM_HO_A4 = 1,            //!< A4 event: neighbour RSRP exceeds absolute threshold
    LEOSIM_HO_TIME_BASED = 2,    //!< Satellite predicted to leave coverage
    LEOSIM_HO_LOCATION_BASED = 3, //!< UE position-based TTE trigger
    LEOSIM_HO_ELEVATION = 4,     //!< Elevation angle below minimum threshold
    LEOSIM_HO_RLF = 5,           //!< Radio Link Failure (T310 expiry)
    LEOSIM_HO_LOAD_BALANCE = 6,  //!< Traffic load rebalancing
    LEOSIM_HO_WEATHER_FADE = 7   //!< Rain/cloud attenuation triggered handover
};

/**
 * \ingroup leosim
 * \brief State of beam management in Conditional Handover state machine
 *
 * Represents the current state of a beam connection during the CHO
 * process, from initial measurement through completion.
 */
enum LeoSimBeamState
{
    LEOSIM_BEAM_CONNECTED = 0,    //!< Beam is connected and active
    LEOSIM_BEAM_MEASURING = 1,    //!< TTT timer running, measuring neighbours
    LEOSIM_BEAM_PREPARING = 2,    //!< CHO Phase 1, preparing handover
    LEOSIM_BEAM_EVALUATING = 3,   //!< CHO Phase 2, UE-side evaluation
    LEOSIM_BEAM_EXECUTING = 4,    //!< Handover in progress
    LEOSIM_BEAM_SEARCHING = 5     //!< No candidates found, searching
};

/**
 * \ingroup leosim
 * \brief Represents a UE-satellite beam association record
 *
 * Contains all relevant metrics for a beam connection including signal quality,
 * satellite load, latency, and state information.
 */
struct LeoSimBeamRecord
{
    uint32_t ueNodeId;                 //!< UE node identifier
    uint32_t satelliteNodeId;          //!< Satellite node identifier
    double rsrp;                       //!< Reference Signal Received Power (dBm)
    double snr;                        //!< Signal-to-Noise Ratio (dB)
    uint32_t beamId;                   ///< Spot-beam index within satellite
    uint32_t cellId;                   ///< NR Cell ID of this beam
    int colorGroup;                    ///< Frequency reuse colour group
    double sinr;                       ///< SINR in dB (replaces snr as primary metric)
    double intraBeamInterference_dBm;  ///< Co-channel interference from same satellite
    double interSatInterference_dBm;   ///< Co-channel interference from other satellites
    double beamLoad;                   ///< Active UE count in this specific beam
    double beamThroughputMbps;         ///< Current throughput in this beam
    bool beamActive;                   ///< False if beam is dark (beam hopping)
    double pathLoss;                   //!< Path loss (dB)
    double elevationAngle;             //!< Elevation angle (degrees)
    double remainingServiceTime;       //!< Time To Exit (TTE) from ephemeris (seconds)
    double satelliteLoad;              //!< Satellite load (active UE count)
    double endToEndLatency;            //!< End-to-end latency (milliseconds)
    Time associationTime;              //!< Time of beam association
    LeoSimBeamState state;             //!< Current state of beam connection
};

/**
 * \ingroup leosim
 * \brief Wraps a beam record with TOPSIS scoring for multi-criteria decision making
 *
 * Used during the handover candidate evaluation phase to rank potential
 * target beams based on normalized scores across multiple metrics.
 */
struct LeoSimTopsisCandidate
{
    LeoSimBeamRecord beamRecord;       //!< The underlying beam record
    double topsisScore;                //!< TOPSIS normalized score [0,1]
    bool choConfigReady;               //!< CHO configuration ready for execution
};

/**
 * \ingroup leosim
 * \brief Complete handover event record for tracing and analysis
 *
 * Captures all relevant information about a handover event including
 * timing, performance metrics, and success/failure status per 3GPP TS 38.300.
 */
struct LeoSimHandoverEvent
{
    uint32_t ueNodeId;                 //!< UE undergoing handover
    uint32_t sourceSatId;              //!< Source satellite node ID
    uint32_t targetSatId;              //!< Target satellite node ID
    uint32_t sourceBeamId;             //!< Source beam ID (for beam-level HO tracking)
    uint32_t targetBeamId;             //!< Target beam ID (for beam-level HO tracking)
    uint32_t sourceCellId;             //!< Source cell ID
    uint32_t targetCellId;             //!< Target cell ID
    LeoSimHandoverMode mode;           //!< Handover mode (BHO or CHO)
    LeoSimHandoverType type;           //!< Type of handover
    LeoSimHandoverTrigger trigger;     //!< Event that triggered handover
    Time initiatedAt;                  //!< Simulation time handover initiated
    Time completedAt;                  //!< Simulation time handover completed
    double handoverLatencyMs;          //!< Total handover latency (milliseconds)
    uint32_t packetsBuffered;          //!< Packets buffered during handover
    uint32_t packetsDropped;           //!< Packets dropped during handover
    bool success;                      //!< Whether handover completed successfully
    double sinrBefore;                 //!< SINR of source beam before HO (dB)
    double sinrAfter;                  //!< SINR of target beam after HO (dB)
};

/**
 * \ingroup leosim
 * \brief Per-candidate 3GPP Release 17 CHO configuration per TS 38.300 §9.2.3
 *
 * Specifies the execution conditions and validity period for a single
 * conditional handover candidate during the preparation phase.
 */
struct LeoSimChoConfig
{
    uint32_t candidateSatId;           //!< Candidate target satellite ID
    double execConditionThresholdA3;   //!< A3 threshold for execution condition (dB)
    double execConditionThresholdA4;   //!< A4 threshold for execution condition (dBm)
    Time execConditionTimeWindow;      //!< Time window for condition evaluation
    Time configValidUntil;             //!< Validity period of this configuration
    bool conditionMet;                 //!< Whether execution condition is met
    Ipv4Address targetAddress;         //!< Target satellite IPv4 address
};

/**
 * \ingroup leosim
 * \class LeoSimBeamManager
 * \brief Manages beam associations and handovers for LEO satellite constellations
 *
 * Provides comprehensive beam management functionality supporting both legacy
 * reactive handover (BHO) and 3GPP Release 17 Conditional Handover (CHO) modes.
 * Implements TOPSIS-based multi-criteria decision making, predictive Time-To-Exit (TTE)
 * based triggers, and per-candidate CHO configuration per 3GPP TS 38.300 §9.2.3.
 *
 * The beam manager operates on a configurable update cycle, maintaining beam metrics,
 * evaluating handover conditions, and managing the handover state machine. It supports
 * load balancing, radio link failure detection via T310 timer, and comprehensive
 * tracing and analytics.
 */
class LeoSimBeamManager : public Object
{
  public:
    /**
     * \brief Get the type ID of this object
     * \return TypeId for LeoSimBeamManager
     */
    static TypeId GetTypeId();

    /**
     * \name Dependency Injection
     * @{
     */

    /**
     * \brief Set the channel model for propagation calculations
     * \param channelModel Pointer to propagation channel model
     */
    void SetChannelModel(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Set the inter-satellite link channel model
     * \param islChannelModel Pointer to ISL channel model
     */
    void SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel);

    /**
     * \brief Set the routing calculator
     * \param routingCalculator Pointer to routing calculator
     */
    void SetRoutingCalculator(Ptr<LeoSimRoutingCalculator> routingCalculator);

    /**
     * \brief Set the satellite/beam loader
     * \param loader Pointer to loader
     */
    void SetLoader(Ptr<LeoSimLoader> loader);

    /**
     * \brief Set the operator model for operator-aware beam decisions.
     * \param model Pointer to operator model
     */
    void SetOperatorModel(Ptr<LeoSimOperatorModel> model);

    /**
     * \brief Set the weather model for weather-aware HO and TOPSIS scoring
     * \param model Pointer to weather model
     */
    void SetWeatherModel(Ptr<LeoSimWeatherModel> model);

    /**
     * \brief Enable verbose/debug output
     * \param verbose True to enable debug logging
     */
    void SetVerbose(bool verbose);

    /**
     * \brief Set the interval between full beam decision cycles.
     * \param interval Positive update interval
     */
    void SetUpdateInterval(Time interval);
    /** @} */

    /**
     * \name Handover Mode Configuration
     * @{
     */

    /**
     * \brief Set the handover mode (BHO or CHO)
     * \param mode LeoSimHandoverMode value
     */
    void SetHandoverMode(LeoSimHandoverMode mode);

    /**
     * \brief Set beam mode (earth-fixed or body-fixed)
     * \param earthFixed True for earth-fixed beams, false for body-fixed
     */
    void SetBeamMode(bool earthFixed);
    /** @} */

    /**
     * \name 3GPP Timer Configuration
     * @{
     */

    /**
     * \brief Set the Time-To-Trigger (TTT) duration for measurement reporting
     * \param ttt Duration of TTT timer
     */
    void SetTttDuration(Time ttt);

    /**
     * \brief Set the T310 radio link failure timer duration
     * \param t310 Duration of T310 timer
     */
    void SetT310Duration(Time t310);

    /**
     * \brief Set the N310 counter for T310 expiry evaluation
     * \param n310 Number of T310 expirations to trigger RLF
     */
    void SetN310Count(uint32_t n310);

    /**
     * \brief Set the N311 counter for T310 restart evaluation
     * \param n311 Number of successful signals to restart T310
     */
    void SetN311Count(uint32_t n311);
    /** @} */

    /**
     * \name Handover Threshold Configuration
     * @{
     */

    /**
     * \brief Set the A3 offset for neighbor cell handover triggering
     * \param offset A3 offset in dB
     */
    void SetA3Offset(double offset);

    /**
     * \brief Set the A4 absolute threshold for handover
     * \param threshold A4 threshold in dBm
     */
    void SetA4Threshold(double threshold);

    /**
     * \brief Set the minimum elevation angle threshold
     * \param threshold Elevation angle in degrees
     */
    void SetElevationThreshold(double threshold);

    /**
     * \brief Set the minimum Time-To-Exit threshold for handover triggering
     * \param threshold TTE threshold
     */
    void SetTteThreshold(Time threshold);

    /**
     * \brief Set the weather fade threshold for weather-triggered handover
     * \param thresholdDb Attenuation threshold in dB
     */
    void SetWeatherFadeThresholdDb(double thresholdDb);

    /**
     * \brief Set the minimum beam SINR required for serving eligibility.
     * \param thresholdDb SINR threshold in dB
     */
    void SetSinrThresholdDb(double thresholdDb);
    /** @} */

    /**
     * \brief Set TOPSIS weighting factors for multi-criteria decision making
     * \param wRsrp Weight for RSRP (signal quality)
     * \param wTte Weight for Time-To-Exit (satellite visibility)
     * \param wLoad Weight for satellite load balancing
     * \param wLatency Weight for end-to-end latency
     * \param wElevation Weight for elevation angle
     */
    void SetTopsisWeights(double wRsrp,
                          double wSinr,
                          double wTte,
                          double wLoad,
                          double wLatency,
                          double wElevation,
                          double wActive,
                          double wOperatorCompat,
                          double wWeather = 0.14);

    /**
     * \name Single Best Link Mode Configuration
     * @{
     */

    /**
     * \brief Enable single-best-link mode for ground-satellite connections
     *
     * When enabled, only the single best beam (highest SINR) per satellite is
     * established for UE connections. This implements state-of-the-art link
     * selection where secondary/potential links are not considered during route
     * calculation. This optimizes link quality and reduces routing complexity.
     *
     * \param enable True to enable single-best-link mode, false for standard multi-beam (default: false)
     */
    void SetSingleBestLinkMode(bool enable);
    /** @} */

    /**
     * \name Conditional Handover (CHO) Configuration
     * @{
     */

    /**
     * \brief Set maximum number of CHO candidates
     * \param maxCandidates Maximum number of candidate satellites
     */
    void SetMaxCandidates(uint32_t maxCandidates);

    /**
     * \brief Set CHO preparation phase delay (Phase 1)
     * \param delay Delay before condition evaluation starts
     */
    void SetChoPreparationDelay(Time delay);

    /**
     * \brief Set CHO execution phase delay (Phase 2)
     * \param delay Delay before handover execution
     */
    void SetChoExecutionDelay(Time delay);
    /** @} */

    /**
     * \name Load Balancing and Buffering
     * @{
     */

    /**
     * \brief Enable or disable handover-triggered load balancing
     * \param enable True to enable load balancing
     */
    void EnableLoadBalancing(bool enable);

    /**
     * \brief Set the load imbalance threshold for triggering rebalancing
     * \param threshold Load difference threshold (UE count)
     */
    void SetLoadImbalanceThreshold(uint32_t threshold);

    /**
     * \brief Enable or disable packet buffering during handover
     * \param enable True to enable buffering
     */
    void EnableHandoverBuffering(bool enable);

    /**
     * \brief Set the maximum handover buffer size
     * \param maxSize Maximum size in packets
     */
    void SetMaxBufferSize(uint32_t maxSize);
    /** @} */

    /**
     * \name Flow Monitoring and Analytics
     * @{
     */

    /**
     * \brief Set the flow monitor for capturing packet flow statistics
     * \param monitor Pointer to FlowMonitor instance (can be null to disable)
     */
    void SetFlowMonitor(Ptr<FlowMonitor> monitor);

    /**
     * \brief Set the IPv4 flow classifier for flow identification
     * \param classifier Pointer to Ipv4FlowClassifier instance
     */
    void SetFlowClassifier(Ptr<Ipv4FlowClassifier> classifier);
    /** @} */

    /**
     * \name Lifecycle Management
     * @{
     */

    /**
     * \brief Start the beam manager
    * \param groundNodes Container of managed ground nodes (for example UEs and servers)
     * \param satellites Container of satellite nodes
     * \param simStart Simulation start time
     * \param simDuration Simulation duration
     */
    void Start(NodeContainer groundNodes,
               NodeContainer satellites,
               Time simStart,
               Time simDuration);

    /**
     * \brief Stop the beam manager and cleanup resources
     */
    void Stop();
    /** @} */

    /**
     * \name Beam Query and Analysis
     * @{
     */

    /**
     * \brief Get the current serving beam for a UE
     * \param ueNodeId UE node identifier
     * \return Current LeoSimBeamRecord, or empty if not connected
     */
    LeoSimBeamRecord GetCurrentBeam(uint32_t ueNodeId) const;

    /**
     * \brief Check whether a satellite-ground pair is the selected serving access link.
     * \param groundNodeId Ground node identifier.
     * \param satNodeId Satellite node identifier.
     * \return True when the beam manager currently authorizes this serving link.
     */
    bool IsServingAccessLink(uint32_t groundNodeId, uint32_t satNodeId) const;

    /**
     * \brief Get CHO-prepared candidate beams for a ground node.
     * \param groundNodeId Ground node identifier.
     * \return Valid prepared candidate beam records.
     */
    std::vector<LeoSimBeamRecord> GetPreparedCandidateBeams(uint32_t groundNodeId) const;

    /**
     * \brief Get ranked handover candidates for a UE
     * \param ueNodeId UE node identifier
     * \return Vector of LeoSimTopsisCandidate sorted by score (descending)
     */
    std::vector<LeoSimTopsisCandidate> GetRankedCandidates(uint32_t ueNodeId);

    /**
     * \brief Get Time-To-Exit from a satellite for a UE
     * \param ueNodeId UE node identifier
     * \param satNodeId Satellite node identifier
     * \return Time until satellite exits coverage, or negative if already out of coverage
     */
    double GetTimeToExit(uint32_t ueNodeId, uint32_t satNodeId);

    /**
     * \brief Set the multi-beam model for spot-beam topology and phased array steering.
     * \param model Pointer to LeoSimMultiBeamModel instance.
     *
     * Must be called before Start().  The model owns the phased array
     * configuration (LeoSimPhasedArrayConfig) and the per-satellite
     * spot-beam lists.  SteerPhasedArrayBeams() uses this model to
     * steer beams dynamically toward ground nodes.
     */
    void SetMultiBeamModel(Ptr<LeoSimMultiBeamModel> model);

    /**
     * \brief Get the multi-beam model for beam topology queries.
     * \return Pointer to the multi-beam model (may be null if not configured).
     */
    Ptr<LeoSimMultiBeamModel> GetMultiBeamModel() const;

    /**
     * \brief Set the phased array beam steering update interval.
     *
     * Determines how often the phased array is re-steered toward the optimal
     * ground node within the current update cycle.  Defaults to the same
     * period as the beam manager update interval (100 ms).
     *
     * \param interval Steering update period.
     */
    void SetPhasedArraySteeringInterval(Time interval);

    /**
     * \brief Set the beam footprint geometry update interval.
     * \param interval Geometry update period; non-positive disables periodic refresh.
     */
    void SetBeamGeometryUpdateInterval(Time interval);
    /** @} */

    /**
     * \brief Set the callback for handover events
     * \param callback Callback function receiving LeoSimHandoverEvent
     */
    void SetHandoverCallback(Callback<void, LeoSimHandoverEvent> callback);

    /**
     * \brief Set the callback for serving beam state updates
     *
     * This callback is fired when the UE's serving beam record/state changes
     * during the CHO state machine (PREPARING/EVALUATING/EXECUTING/CONNECTED).
     */
    void SetBeamStateCallback(Callback<void, uint32_t, LeoSimBeamRecord, double> callback);

    /**
     * \brief Set callback fired when serving access eligibility changes.
     */
    void SetAccessStateChangeCallback(Callback<void> callback);

    /**
     * \brief Set the callback for CHO candidate configuration (Phase 1)
     *
     * Fired when CHO candidates are configured for a UE. The vector is typically
     * limited to the configured max-candidates.
     */
    void SetChoConfigCallback(
        Callback<void, uint32_t, uint32_t, std::vector<LeoSimTopsisCandidate>> callback);

    /**
     * \name Analytics and Tracing
     * @{
     */

    /**
     * \brief Get the complete handover history
     * \return Vector of all LeoSimHandoverEvent records
     */
    std::vector<LeoSimHandoverEvent> GetHandoverHistory() const;

    /**
     * \brief Get number of handovers for a specific UE
     * \param ueNodeId UE node identifier
     * \return Handover count
     */
    uint32_t GetHandoverCount(uint32_t ueNodeId) const;

    /**
     * \brief Get average handover latency
     * \return Average latency in milliseconds
     */
    double GetAverageHandoverLatencyMs() const;

    /**
     * \brief Get ping-pong handover count
     * \return Number of ping-pong handovers detected
     */
    uint32_t GetPingPongCount() const;
    /** @} */

    /**
     * \brief Rank beam candidates using TOPSIS multi-criteria decision analysis.
     *
    * Hard feasibility filters are applied by ScanVisibleSatellites() before
    * ranking. Criteria order matches SetTopsisWeights (w0..w8):
    * - Radio-access: RSRP, beam SINR, elevation, TTE, beam load
    * - Policy/path: latency, operator compatibility, weather quality
    * - Beam-active is retained for compatibility but normally hard-filtered.
     *
     * \param candidates Vector of LeoSimBeamRecord with full metrics to rank
     * \param ueNodeId UE node identifier for context logging
     * \return Sorted vector of LeoSimTopsisCandidate by TOPSIS score (descending)
     */
    std::vector<LeoSimTopsisCandidate> RankByTopsis(
        const std::vector<LeoSimBeamRecord>& candidates,
        uint32_t ueNodeId);

  private:
    /**
     * \name State Members
     * @{
     */

    // Configuration
    Ptr<LeoSimChannelModel> m_channelModel;            //!< Propagation channel model
    Ptr<LeoSimChannelModel> m_islChannelModel;         //!< Inter-satellite link model
    Ptr<LeoSimRoutingCalculator> m_routingCalculator;  //!< Routing calculator
    Ptr<LeoSimOperatorModel> m_operatorModel;          //!< Operator sharing model
    Ptr<LeoSimWeatherModel> m_weatherModel;            //!< Weather model for HO/TOPSIS
    Ptr<LeoSimLoader> m_loader;                        //!< Satellite/beam loader
    Ptr<LeoSimSinrEngine> m_sinrEngine;                //!< SINR decomposition engine
    Ptr<LeoSimMultiBeamModel> m_multiBeamModel;        //!< Multi-beam topology model
    LeoSimBeamConfig m_cfg;                            //!< Global multi-beam configuration
    double m_intraBeamHoDelayMs = 10.0;                //!< Intra-beam HO execution delay (ms)
    double m_sinrThresholdDb = -10.0;                  //!< SINR outage floor for HO/selection (dB)
    bool m_verbose;                                    //!< Debug output enabled

    // Handover mode and configuration
    LeoSimHandoverMode m_hoMode = LEOSIM_HO_MODE_CHO;    //!< Handover mode (default CHO)
    bool m_earthFixedBeam;                               //!< Earth-fixed vs body-fixed beams

    // 3GPP timers and counters
    Time m_ttt = Seconds(1.0);                           //!< Time-To-Trigger duration
    Time m_t310 = Seconds(1.0);                          //!< RLF timer duration
    uint32_t m_n310 = 3;                                 //!< RLF event counter
    uint32_t m_n311 = 3;                                 //!< RLF recovery counter

    // Handover thresholds
    double m_a3Offset = 3.0;                             //!< A3 offset (dB)
    double m_a4Threshold = -110.0;                       //!< A4 threshold (dBm)
    double m_elevationThreshold = 10.0;                  //!< Min elevation angle (degrees)
    Time m_tteThreshold = Seconds(30.0);                 //!< Min TTE before HO (seconds)
    double m_weatherFadeThresholdDb = 15.0;              //!< Weather fade HO threshold (dB)

    // TOPSIS multi-criteria weighting
    std::array<double, 9> m_topsisWeights = {0.18, 0.22, 0.14, 0.10, 0.08, 0.14, 0.00, 0.10, 0.14};
    //  {wRsrp, wSinr, wTte, wLoad, wLatency, wElevation, wActive, wOperatorCompat, wWeather}

    // CHO configuration
    uint32_t m_maxCandidates = 3;                        //!< Max CHO candidates
    Time m_prepDelay = MilliSeconds(100);                //!< CHO Phase 1 delay
    Time m_execDelay = MilliSeconds(150);                //!< CHO Phase 2 delay

    // Load balancing and buffering
    bool m_loadBalancingEnabled;                         //!< Load balancing active
    uint32_t m_loadImbalanceThreshold;                   //!< Load difference threshold
    bool m_handoverBufferingEnabled;                     //!< Packet buffering enabled
    uint32_t m_maxBufferSize;                            //!< Max buffer size (packets)

    // Ping-pong detection
    Time m_pingPongWindow = Seconds(5.0);                //!< Ping-pong detection window

    // Per-UE state (indexed by UE Node ID)
    std::map<uint32_t, LeoSimBeamRecord> m_currentBeams; //!< Current serving beam per UE
    std::map<uint32_t, LeoSimBeamRecord> m_retiredBeams; //!< Last serving beam before SEARCHING
    std::map<uint32_t, LeoSimBeamState> m_beamStates;    //!< Beam connection state per UE
    std::map<uint32_t, Time> m_tttStartTime;             //!< TTT timer start per UE
    std::map<uint32_t, EventId> m_tttEventIds;           //!< TTT scheduled event IDs per UE
    std::map<uint32_t, EventId> m_ephemerisHandoverEventIds; //!< One predictive HO event per UE
    std::set<uint32_t> m_inFlightInterSatelliteHandovers; //!< Nodes in CHO execution
    bool m_accessRouteRefreshPending = false;             //!< Access graph changed in a HO batch
    std::map<uint32_t, Time> m_t310StartTime;            //!< T310 timer start per UE
    std::map<uint32_t, uint32_t> m_t310Counter;          //!< T310 expiry counter per UE
    std::map<uint32_t, std::vector<LeoSimTopsisCandidate>> m_candidates; //!< Ranked candidates
    std::map<uint32_t, std::vector<LeoSimBeamRecord>> m_visibleScanCache; //!< Per-update visibility results
    bool m_visibleScanCacheEnabled = false;              //!< Restrict cache lifetime to UpdateCycle
    struct GatewayRouteMetrics
    {
        double latencyMs;
        double distance;
        uint32_t gatewayNodeId;
    };
    mutable std::map<std::pair<uint32_t, uint32_t>, GatewayRouteMetrics>
        m_gatewayRouteCache; //!< Per-update route metrics keyed by satellite/excluded gateway
    std::map<uint32_t, std::vector<uint32_t>> m_bufferedPackets; //!< Buffered packets per UE
    std::map<uint32_t, Time> m_lastHandoverTime;         //!< Last HO time per UE
    std::map<uint32_t, Ptr<LeoSimBeamHoppingManager>> m_beamHopManagers; //!< Per-satellite hopping managers

    // Per-candidate CHO configurations (indexed by [UE ID][sat ID])
    std::map<uint32_t, std::map<uint32_t, LeoSimChoConfig>> m_choConfigs;
    std::map<uint32_t, std::map<uint32_t, LeoSimBeamRecord>> m_preparedCandidateBeams; //!< Prepared CHO beam records

    // Packet buffering during handover
    std::map<uint32_t, std::queue<Ptr<Packet>>> m_handoverBuffers; //!< Buffered packets per UE
    std::map<uint32_t, Ptr<Socket>> m_ueSocketMap;                 //!< Sockets for UE nodes

    // UE address mapping and flow monitoring
    std::map<uint32_t, Ipv4Address> m_ueAddressMap;                //!< IPv4 address per UE node ID
    Ptr<FlowMonitor> m_flowMonitor;                                //!< Flow monitor instance
    Ptr<Ipv4FlowClassifier> m_flowClassifier;                      //!< IPv4 flow classifier

    // Analytics
    std::vector<LeoSimHandoverEvent> m_handoverHistory;  //!< Complete HO history
    std::map<uint32_t, uint32_t> m_ueHandoverCount;      //!< HO count per UE
    uint32_t m_pingPongCount = 0;                        //!< Total ping-pong count

    // Phased array steering state
    Time m_phasedArraySteeringInterval{MilliSeconds(100)}; //!< Phased array re-steering period
    EventId m_phasedArraySteeringEventId;                  //!< Scheduled steering event
    Time m_beamGeometryUpdateInterval{MilliSeconds(100)};   //!< Beam footprint refresh period

    // Lifecycle
    NodeContainer m_groundNodes;                         //!< Managed ground nodes (UEs/servers)
    NodeContainer m_satellites;                          //!< Satellite nodes
    Time m_simStart;                                     //!< Simulation start time
    Time m_simDuration;                                  //!< Simulation duration
    Time m_updateInterval = MilliSeconds(100);           //!< Beam manager update interval
    EventId m_updateEventId;                             //!< Scheduled update cycle event
    EventId m_beamGeometryEventId;                       //!< Scheduled beam geometry update event

    // Callbacks
    Callback<void, LeoSimHandoverEvent> m_handoverCallback; //!< HO event callback
    Callback<void, uint32_t, LeoSimBeamRecord, double> m_beamStateCallback; //!< Serving beam state callback
    Callback<void> m_accessStateChangeCallback; //!< Route refresh trigger for access changes
    Callback<void, uint32_t, uint32_t, std::vector<LeoSimTopsisCandidate>> m_choConfigCallback; //!< CHO config callback

    /** @} */

    /**
     * \name Private Methods
     * @{
     */

    /**
     * \brief Main update cycle called periodically
     */
    void UpdateCycle();

    void NotifyAccessStateChanged();
    void FinishInterSatelliteHandover(uint32_t ueNodeId);

    /**
     * \brief Refresh model-owned beam footprints from current satellite positions.
     */
    void UpdateBeamGeometry();

    /**
     * \brief Update beam metrics (RSRP, SNR, path loss, latency) for all UE-satellite pairs
     */
    void UpdateBeamMetrics();

    /**
     * \brief Evaluate TTT timers and triggering conditions for all UEs
     */
    void RunTttEvaluation(uint32_t ueNodeId);

    /**
     * \brief Evaluate T310 RLF timers and conditions for all UEs
     */
    void RunT310Evaluation(uint32_t ueNodeId);

    /**
     * \brief Scan and identify visible satellites for a UE with full metrics
     * 
     * Queries all active links for the UE and populates LeoSimBeamRecord entries
     * with complete beam metrics including RSRP, latency, TTE, satellite load,
     * and ping-pong detection.
     * 
     * \param ueNodeId UE node identifier
     * \return Vector of LeoSimBeamRecord with full metrics for each visible satellite
     */
    std::vector<LeoSimBeamRecord> ScanVisibleSatellites(uint32_t ueNodeId);

    /**
     * \brief Rank beam candidates using TOPSIS multi-criteria decision method
     * 
     * Implements the TOPSIS (Technique for Order of Preference by Similarity to
     * Ideal Solution) algorithm with five weighted criteria:
     * - RSRP (signal strength): higher is better
     * - Time-To-Exit (TTE): longer visibility is better
     * - Satellite load: lower is better (inverted)
     * - End-to-end latency: lower is better (inverted)
     * - Elevation angle: higher coverage is better
     * 
     * \param candidates Vector of LeoSimBeamRecord with full metrics to rank
     * \param ueNodeId UE node identifier for context logging
     * \return Sorted vector of LeoSimTopsisCandidate by TOPSIS score (descending)
     */

    /**
     * \brief Compute Time-To-Exit (TTE) for a UE-satellite pair
     * \param ueNodeId UE node identifier
     * \param satNodeId Satellite node identifier
     * \return TTE in seconds (negative if out of coverage)
     */
    double ComputeTte(uint32_t ueNodeId, uint32_t satNodeId) const;

    /**
     * \brief Compute end-to-end latency for a UE-satellite path
     * \param ueNodeId UE node identifier
     * \param satNodeId Satellite node identifier
     * \return Latency in milliseconds
     */
    double ComputeEndToEndLatency(uint32_t ueNodeId, uint32_t satNodeId) const;

    /**
     * \brief Initiate CHO preparation phase (Phase 1) for candidate
     * \param ueNodeId UE node identifier
     * \param candidateSatId Target satellite node ID
     */
    void InitiateChoPreparation(uint32_t ueNodeId, uint32_t candidateSatId);

    /**
     * \brief Initiate CHO preparation phase (Phase 1) with ranked candidates
     * \param ueNodeId UE node identifier
     * \param topN Vector of top TOPSIS-ranked candidates for CHO configuration
     * 
     * This overload accepts a vector of ranked candidates and prepares multiple
     * configurations, iterating up to m_maxCandidates. It sets the current beam
     * state to LEOSIM_BEAM_PREPARING, caches routes via routing calculator if
     * available, and schedules the transition to LEOSIM_BEAM_EVALUATING state
     * after m_prepDelay.
     */
    void InitiateChoPreparation(uint32_t ueNodeId,
                                const std::vector<LeoSimTopsisCandidate>& topN);

    /**
     * \brief Evaluate CHO execution conditions for all configured candidates
     * \param ueNodeId UE node identifier
     */
    void EvaluateChoConditions(uint32_t ueNodeId);

    /**
     * \brief Execute a CHO for specified UE-satellite pair
     * \param ueNodeId UE node identifier
     * \param targetSatId Target satellite node ID
     * \param trigger Handover trigger event
     */
    void ExecuteChoHandover(uint32_t ueNodeId,
                            uint32_t targetSatId,
                            LeoSimHandoverTrigger trigger);

    /**
     * \brief Complete handover processing and update state
     * \param ueNodeId UE node identifier
     * \param sourceSatId Source satellite node ID
     * \param targetSatId Target satellite node ID
     * \param trigger Handover trigger type
     */
    void CompleteHandover(uint32_t ueNodeId,
                          uint32_t sourceSatId,
                          uint32_t targetSatId,
                          LeoSimHandoverTrigger trigger);

    /**
     * \brief Pre-schedule ephemeris-based handovers for upcoming coverage changes
     * 
     * Uses orbital geometry to predict satellite coverage transitions and schedules
     * handover preparation in advance of signal degradation. Handovers are only
     * scheduled if preparation time falls within the lookahead window.
     * 
     * \param lookaheadWindow Maximum time window for pre-scheduling handovers
     */
    void PreScheduleEphemerisHandovers(Time lookaheadWindow);

    /**
     * \brief Check if a UE is within the footprint of a satellite beam
     * \param ueNodeId UE node identifier
     * \param satNodeId Satellite node identifier
     * \return True if UE is in beam coverage
     */
    bool IsUeInBeamFootprint(uint32_t ueNodeId, uint32_t satNodeId);

    /**
     * \brief Evaluate whether intra-beam handover is needed on current serving satellite.
     * \param ueNodeId UE node identifier.
     */
    void EvaluateIntraBeamNeed(uint32_t ueNodeId);

    /**
     * \brief Execute an intra-beam handover to a target beam on same satellite.
     * \param ueNodeId UE node identifier.
     * \param targetBeamId Target spot-beam identifier on serving satellite.
     * \param trigger Trigger causing this handover.
     */
    void ExecuteIntraBeamHandover(uint32_t ueNodeId,
                                  uint32_t targetBeamId,
                                  LeoSimHandoverTrigger trigger);

    /**
     * \brief Find the best currently active beam on a satellite for a UE.
     * \param ueNodeId UE node identifier.
     * \param satId Satellite node identifier.
     * \return Target beam ID, or -1 if no active suitable beam exists.
     */
    int32_t FindBestActiveBeam(uint32_t ueNodeId, uint32_t satId) const;

    /**
     * \brief Find a better beam candidate on the same satellite than current serving beam.
     * \param ueNodeId UE node identifier.
     * \param satId Satellite node identifier.
     * \return Better beam ID, or -1 if no better beam is found.
     */
    int32_t FindBetterBeamOnSameSat(uint32_t ueNodeId, uint32_t satId) const;

    /**
     * \brief Advance beam hopping slot and refresh active/dark beams.
     */
    void AdvanceBeamHoppingSlot();

    /**
     * \brief Handle a beam transitioning to dark state.
     * \param satId Satellite node identifier.
     * \param beamId Beam identifier on satellite.
     */
    void HandleBeamDark(uint32_t satId, uint32_t beamId);

    /**
     * \brief Steer phased array antennas on all satellites toward optimal ground nodes.
     *
     * Called each update cycle (after channel metrics are refreshed) or on its own
     * independent steering schedule.  For each satellite the method:
     * 1. Selects the served UE that maximises the aggregate TOPSIS benefit
     *    (i.e., the UE with the lowest current SINR that needs the most help).
     * 2. Calls LeoSimMultiBeamModel::SteerBeam() to electronically point the
     *    phased array toward that UE.
     * 3. For every UE currently served by the satellite, computes the
     *    direction-dependent array gain via LeoSimMultiBeamModel::CalculateGain()
     *    and adds the beamforming gain contribution to the UE's stored RSRP.
     */
    void SteerPhasedArrayBeams();

    /**
     * \brief Find serving beam for footprint-based beam steering
     * \param ueNodeId UE node identifier
     * \return Pointer to best-ranked serving satellite node, or nullptr if empty
     * 
     * When earth-fixed beam mode is active, beams are anchored to geographic cells
     * rather than tracking individual satellites. This method scans visible satellites,
     * ranks them by TOPSIS, and returns the top-ranked candidate node.
     */
    Ptr<Node> FindServingBeamForFootprint(uint32_t ueNodeId) const;

    /**
     * \brief Buffer a data packet during handover
     * 
     * Queues a packet for a UE during handover preparation/execution. If the buffer
     * is full (size >= m_maxBufferSize), the packet is dropped and a drop counter
     * is incremented in the current handover event.
     * 
     * \param ueNodeId UE node identifier
     * \param pkt Packet to buffer
     */
    void BufferPacket(uint32_t ueNodeId, Ptr<Packet> pkt);

    /**
     * \brief Flush buffered packets after successful handover
     * 
     * Sends all buffered packets for a UE through its socket and clears the buffer.
     * Uses m_ueSocketMap[ueNodeId]->Send() to dispatch each packet.
     * 
     * \param ueNodeId UE node identifier
     */
    void FlushBuffer(uint32_t ueNodeId);

    /**
     * \brief Capture flow-level KPIs at handover completion
     * 
     * Snapshots flow statistics from the flow monitor at handover completion time.
     * Queries all flows involving the UE and aggregates key metrics:
     * - Lost packets (flow-level packet drops)
     * - Mean delay (latency)
     * - Throughput
     * 
     * The captured metrics are stored in the handover event for analytics and tracing.
     * 
     * \param ueNodeId UE node identifier
     * \param evt Reference to handover event being completed
     * \param monitor Pointer to FlowMonitor (can be null if monitoring disabled)
     * \param classifier Pointer to Ipv4FlowClassifier (can be null if monitoring disabled)
     */
    void SnapshotFlowStats(uint32_t ueNodeId,
                          LeoSimHandoverEvent& evt,
                          Ptr<FlowMonitor> monitor,
                          Ptr<Ipv4FlowClassifier> classifier);

    /**
     * \brief Record a serving-satellite change in handover history/callbacks.
     */
    void RecordInterSatelliteHandover(uint32_t ueNodeId,
                                      const LeoSimBeamRecord& source,
                                      const LeoSimBeamRecord& target,
                                      LeoSimHandoverTrigger trigger,
                                      Time initiatedAt,
                                      Time completedAt);

    /**
     * \brief Detect ping-pong handover pattern
     * \param ueNodeId UE node identifier
     * \param sourceSatId Source satellite from previous HO
     * \param targetSatId Target satellite of current HO
     * \return True if ping-pong detected
     */
    bool IsPingPong(uint32_t ueNodeId, uint32_t sourceSatId, uint32_t targetSatId);

    /** @} */
};

} // namespace ns3

#endif /* LEOSIM_BEAM_MANAGER_H */
