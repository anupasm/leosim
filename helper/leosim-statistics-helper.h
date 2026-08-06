/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef LEOSIM_STATISTICS_HELPER_H
#define LEOSIM_STATISTICS_HELPER_H

#include "ns3/leosim-channel-model.h"
#include "ns3/event-id.h"
#include "ns3/flow-monitor.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/ptr.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

class LeoSimBeamManager;
class LeoSimChannel;
class LeoSimChannelModel;
class LeoSimIslLoadModel;
class NetDevice;
class Node;
class Packet;
class PacketSink;

/** Numerically stable descriptive statistics used by LeoSim KPI collectors. */
struct LeoSimRunningStatistics
{
    uint64_t count{0};
    double min{0.0};
    double max{0.0};
    double mean{0.0};
    double m2{0.0};

    void Add(double value);
    double GetVariance() const;
    double GetStandardDeviation() const;
};

/** Per IPv4 flow transport results derived from ns-3 FlowMonitor. */
struct LeoSimFlowStatistics
{
    uint32_t flowId{0};
    std::string sourceAddress;
    std::string destinationAddress;
    uint16_t sourcePort{0};
    uint16_t destinationPort{0};
    uint8_t protocol{0};
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t lostPackets{0};
    uint64_t txBytes{0};
    uint64_t rxBytes{0};
    double throughputMbps{0.0};
    double meanDelayMs{0.0};
    double meanJitterMs{0.0};
    double packetDeliveryRatio{0.0};
    double meanHopCount{0.0};
};

/** Compact routing aggregates retained without verbose per-route CSV logging. */
struct LeoSimRouteStatistics
{
    uint64_t samples{0};
    uint64_t validSamples{0};
    uint64_t changes{0};
    uint64_t uniquePaths{0};
    double meanHops{0.0};
    double meanDistanceKm{0.0};
    double meanMinSnrDb{0.0};
    double minimumSnrDb{0.0};
    double meanPathLossDb{0.0};
    double meanMinSignalDbm{0.0};
};

/** Complete network and satellite-link KPI snapshot. Ratios are in [0, 1]. */
struct LeoSimStatisticsSnapshot
{
    Time timestamp;
    uint64_t txPackets{0};
    uint64_t rxPackets{0};
    uint64_t lostPackets{0};
    uint64_t txBytes{0};
    uint64_t rxBytes{0};
    double throughputMbps{0.0};
    double offeredLoadMbps{0.0};
    uint64_t applicationRxBytes{0};
    double applicationGoodputMbps{0.0};
    double applicationMeasurementSeconds{0.0};
    uint32_t applicationSinkCount{0};
    uint32_t applicationSinksWithRx{0};
    double meanSinkGoodputMbps{0.0};
    double minimumSinkGoodputMbps{0.0};
    double maximumSinkGoodputMbps{0.0};
    double sinkGoodputJainFairness{0.0};
    double packetDeliveryRatio{0.0};
    double meanDelayMs{0.0};
    double meanJitterMs{0.0};
    double meanHopCount{0.0};
    LeoSimRunningStatistics snrDb;
    LeoSimRunningStatistics dopplerHz;
    LeoSimRunningStatistics pathLossDb;
    uint64_t linkUpEvents{0};
    uint64_t linkDownEvents{0};
    uint64_t linkDegradedEvents{0};
    uint64_t handovers{0};
    uint64_t successfulHandovers{0};
    uint64_t pingPongs{0};
    double handoverSuccessRatio{1.0};
    double meanHandoverLatencyMs{0.0};
    double p95HandoverLatencyMs{0.0};
};

/**
 * Satellite-aware statistics facade over ns-3 FlowMonitor and LeoSim traces.
 *
 * Attach the monitor/classifier and the satellite model objects before Run().
 * Call StartPeriodicSampling() for a time-series CSV and WriteSummary() after
 * Simulator::Run() for machine-readable aggregate and per-flow results.
 */
class LeoSimStatisticsHelper : public Object
{
  public:
    static TypeId GetTypeId();
    LeoSimStatisticsHelper();
    ~LeoSimStatisticsHelper() override;

    void SetFlowMonitor(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier);
    bool AttachChannel(Ptr<LeoSimChannel> channel);
    bool AttachChannelModel(Ptr<LeoSimChannelModel> channelModel);
    void SetBeamManager(Ptr<LeoSimBeamManager> beamManager);
    void SetRouteStatistics(const LeoSimRouteStatistics& statistics);
    void SetIslLoadModel(Ptr<LeoSimIslLoadModel> loadModel);
    /** Register an application PacketSink for exact payload-byte goodput accounting. */
    void AddApplicationSink(Ptr<PacketSink> sink);
    void ClearApplicationSinks();
    /** Set the interval over which application goodput is normalized. */
    void SetApplicationMeasurementWindow(Time start, Time stop);

    void StartPeriodicSampling(Time interval, const std::string& csvFilename);
    void StopPeriodicSampling();
    /** Append one unscheduled sample at the current simulation time. */
    void WriteFinalSample();
    LeoSimStatisticsSnapshot GetSnapshot(bool checkLostPackets = true) const;
    std::vector<LeoSimFlowStatistics> GetFlowStatistics() const;
    void WriteSummary(const std::string& filename, bool includeFlows = true) const;
    void ResetSatelliteStatistics();

    // Public ingestion points also support models with equivalent external traces.
    void RecordSnr(double snrDb);
    void RecordDoppler(double dopplerHz);
    void RecordPathLoss(double pathLossDb);
    void RecordLinkState(LeoSimLinkState state);

  private:
    void OnSnr(Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, double value);
    void OnDoppler(Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, double value);
    void OnLinkState(Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, bool up);
    void OnModelLinkState(Ptr<Node>, Ptr<Node>, LeoSimLinkState state);
    void OnPathLoss(Ptr<Node>, Ptr<Node>, double value);
    void OnModelSnr(Ptr<Node>, Ptr<Node>, double value);
    void OnModelDoppler(Ptr<Node>, Ptr<Node>, double value);
    void Sample();
    void AppendCsvSnapshot();
    void WriteCsvHeader();

    Ptr<FlowMonitor> m_monitor;
    Ptr<Ipv4FlowClassifier> m_classifier;
    Ptr<LeoSimBeamManager> m_beamManager;
    Ptr<LeoSimIslLoadModel> m_islLoadModel;
    std::vector<Ptr<PacketSink>> m_applicationSinks;
    Time m_applicationWindowStart;
    Time m_applicationWindowStop;
    bool m_hasApplicationMeasurementWindow{false};
    LeoSimRouteStatistics m_routeStatistics;
    LeoSimRunningStatistics m_snr;
    LeoSimRunningStatistics m_doppler;
    LeoSimRunningStatistics m_pathLoss;
    uint64_t m_linkUp{0};
    uint64_t m_linkDown{0};
    uint64_t m_linkDegraded{0};
    Time m_sampleInterval;
    std::string m_csvFilename;
    EventId m_sampleEvent;
    bool m_sampling{false};
    double m_lastCsvSampleSeconds{-1.0};
    uint64_t m_lastCsvApplicationRxBytes{0};
};

} // namespace ns3

#endif
