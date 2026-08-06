/* SPDX-License-Identifier: GPL-2.0-only */
#include "leosim-statistics-helper.h"
#include "ns3/leosim-task-profiler.h"

#include "ns3/leosim-beam-manager.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-channel.h"
#include "ns3/leosim-isl-load-model.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/packet-sink.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimStatisticsHelper");
NS_OBJECT_ENSURE_REGISTERED(LeoSimStatisticsHelper);

void
LeoSimStatisticsHelper::SetIslLoadModel(Ptr<LeoSimIslLoadModel> loadModel)
{
    m_islLoadModel = loadModel;
}

TypeId
LeoSimStatisticsHelper::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimStatisticsHelper")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimStatisticsHelper>();
    return tid;
}

LeoSimStatisticsHelper::LeoSimStatisticsHelper() = default;
LeoSimStatisticsHelper::~LeoSimStatisticsHelper()
{
    StopPeriodicSampling();
}

void
LeoSimRunningStatistics::Add(double value)
{
    if (!std::isfinite(value))
    {
        return;
    }
    ++count;
    if (count == 1)
    {
        min = max = mean = value;
        return;
    }
    min = std::min(min, value);
    max = std::max(max, value);
    const double delta = value - mean;
    mean += delta / static_cast<double>(count);
    m2 += delta * (value - mean);
}

double LeoSimRunningStatistics::GetVariance() const { return count > 1 ? m2 / (count - 1) : 0.0; }
double LeoSimRunningStatistics::GetStandardDeviation() const { return std::sqrt(GetVariance()); }

void
LeoSimStatisticsHelper::SetFlowMonitor(Ptr<FlowMonitor> monitor,
                                       Ptr<Ipv4FlowClassifier> classifier)
{
    m_monitor = monitor;
    m_classifier = classifier;
}

void
LeoSimStatisticsHelper::AddApplicationSink(Ptr<PacketSink> sink)
{
    if (!sink)
    {
        throw std::invalid_argument("application PacketSink must not be null");
    }
    m_applicationSinks.push_back(sink);
}

void
LeoSimStatisticsHelper::ClearApplicationSinks()
{
    m_applicationSinks.clear();
}

void
LeoSimStatisticsHelper::SetApplicationMeasurementWindow(Time start, Time stop)
{
    if (start.IsNegative() || stop <= start)
    {
        throw std::invalid_argument("application measurement stop must be after non-negative start");
    }
    m_applicationWindowStart = start;
    m_applicationWindowStop = stop;
    m_hasApplicationMeasurementWindow = true;
}

bool
LeoSimStatisticsHelper::AttachChannel(Ptr<LeoSimChannel> channel)
{
    if (!channel) return false;
    bool ok = channel->TraceConnectWithoutContext("SnrDb", MakeCallback(&LeoSimStatisticsHelper::OnSnr, this));
    ok = channel->TraceConnectWithoutContext("DopplerHz", MakeCallback(&LeoSimStatisticsHelper::OnDoppler, this)) && ok;
    ok = channel->TraceConnectWithoutContext("LinkState", MakeCallback(&LeoSimStatisticsHelper::OnLinkState, this)) && ok;
    return ok;
}

bool
LeoSimStatisticsHelper::AttachChannelModel(Ptr<LeoSimChannelModel> model)
{
    if (!model) return false;
    bool ok = model->TraceConnectWithoutContext("LinkStateChange", MakeCallback(&LeoSimStatisticsHelper::OnModelLinkState, this));
    ok = model->TraceConnectWithoutContext("PathLoss", MakeCallback(&LeoSimStatisticsHelper::OnPathLoss, this)) && ok;
    ok = model->TraceConnectWithoutContext("SnrDb", MakeCallback(&LeoSimStatisticsHelper::OnModelSnr, this)) && ok;
    ok = model->TraceConnectWithoutContext("DopplerHz", MakeCallback(&LeoSimStatisticsHelper::OnModelDoppler, this)) && ok;
    return ok;
}

void LeoSimStatisticsHelper::SetBeamManager(Ptr<LeoSimBeamManager> manager) { m_beamManager = manager; }
void LeoSimStatisticsHelper::RecordSnr(double value) { m_snr.Add(value); }
void LeoSimStatisticsHelper::RecordDoppler(double value) { m_doppler.Add(value); }
void LeoSimStatisticsHelper::RecordPathLoss(double value) { m_pathLoss.Add(value); }

void
LeoSimStatisticsHelper::RecordLinkState(LeoSimLinkState state)
{
    if (state == LEOSIM_LINK_UP) ++m_linkUp;
    else if (state == LEOSIM_LINK_DOWN) ++m_linkDown;
    else ++m_linkDegraded;
}

void LeoSimStatisticsHelper::OnSnr(Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, double v) { RecordSnr(v); }
void LeoSimStatisticsHelper::OnDoppler(Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, double v) { RecordDoppler(v); }
void LeoSimStatisticsHelper::OnLinkState(Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, bool up) { RecordLinkState(up ? LEOSIM_LINK_UP : LEOSIM_LINK_DOWN); }
void LeoSimStatisticsHelper::OnModelLinkState(Ptr<Node>, Ptr<Node>, LeoSimLinkState s) { RecordLinkState(s); }
void LeoSimStatisticsHelper::OnPathLoss(Ptr<Node>, Ptr<Node>, double v) { RecordPathLoss(v); }
void LeoSimStatisticsHelper::OnModelSnr(Ptr<Node>, Ptr<Node>, double v) { RecordSnr(v); }
void LeoSimStatisticsHelper::OnModelDoppler(Ptr<Node>, Ptr<Node>, double v) { RecordDoppler(v); }

std::vector<LeoSimFlowStatistics>
LeoSimStatisticsHelper::GetFlowStatistics() const
{
    std::vector<LeoSimFlowStatistics> result;
    if (!m_monitor || !m_classifier) return result;
    for (const auto& entry : m_monitor->GetFlowStats())
    {
        const auto tuple = m_classifier->FindFlow(entry.first);
        const auto& s = entry.second;
        LeoSimFlowStatistics out;
        out.flowId = entry.first;
        std::ostringstream src, dst;
        src << tuple.sourceAddress; dst << tuple.destinationAddress;
        out.sourceAddress = src.str(); out.destinationAddress = dst.str();
        out.sourcePort = tuple.sourcePort; out.destinationPort = tuple.destinationPort;
        out.protocol = tuple.protocol;
        out.txPackets = s.txPackets; out.rxPackets = s.rxPackets;
        out.lostPackets = s.lostPackets;
        out.txBytes = s.txBytes; out.rxBytes = s.rxBytes;
        out.packetDeliveryRatio = s.txPackets ? static_cast<double>(s.rxPackets) / s.txPackets : 0.0;
        out.meanDelayMs =
            s.rxPackets ? s.delaySum.GetSeconds() * 1000.0 / static_cast<double>(s.rxPackets)
                        : 0.0;
        out.meanJitterMs = s.rxPackets > 1
                               ? s.jitterSum.GetSeconds() * 1000.0 /
                                     static_cast<double>(s.rxPackets - 1)
                               : 0.0;
        out.meanHopCount = s.rxPackets ? 1.0 + static_cast<double>(s.timesForwarded) / s.rxPackets : 0.0;
        const double active = (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
        out.throughputMbps = active > 0 ? (8.0 * s.rxBytes / active / 1e6) : 0.0;
        result.push_back(out);
    }
    return result;
}

LeoSimStatisticsSnapshot
LeoSimStatisticsHelper::GetSnapshot(bool checkLostPackets) const
{
    LeoSimStatisticsSnapshot out;
    out.timestamp = Simulator::Now();
    out.snrDb = m_snr; out.dopplerHz = m_doppler; out.pathLossDb = m_pathLoss;
    out.linkUpEvents = m_linkUp; out.linkDownEvents = m_linkDown; out.linkDegradedEvents = m_linkDegraded;
    if (m_monitor && checkLostPackets) m_monitor->CheckForLostPackets();
    double weightedDelay = 0, weightedJitter = 0, weightedHops = 0;
    uint64_t jitterSamples = 0;
    for (const auto& f : GetFlowStatistics())
    {
        out.txPackets += f.txPackets; out.rxPackets += f.rxPackets;
        out.txBytes += f.txBytes; out.rxBytes += f.rxBytes; out.lostPackets += f.lostPackets;
        weightedDelay += f.meanDelayMs * f.rxPackets;
        const uint64_t flowJitterSamples = f.rxPackets > 1 ? f.rxPackets - 1 : 0;
        weightedJitter += f.meanJitterMs * flowJitterSamples;
        jitterSamples += flowJitterSamples;
        weightedHops += f.meanHopCount * f.rxPackets;
    }
    out.packetDeliveryRatio = out.txPackets ? static_cast<double>(out.rxPackets) / out.txPackets : 0.0;
    out.meanDelayMs = out.rxPackets ? weightedDelay / out.rxPackets : 0.0;
    out.meanJitterMs = jitterSamples ? weightedJitter / jitterSamples : 0.0;
    out.meanHopCount = out.rxPackets ? weightedHops / out.rxPackets : 0.0;
    const double elapsed = Simulator::Now().GetSeconds();
    if (elapsed > 0)
    {
        out.throughputMbps = 8.0 * out.rxBytes / elapsed / 1e6;
        out.offeredLoadMbps = 8.0 * out.txBytes / elapsed / 1e6;
    }
    const Time now = Simulator::Now();
    if (m_hasApplicationMeasurementWindow)
    {
        const Time measurementEnd = std::min(now, m_applicationWindowStop);
        if (measurementEnd > m_applicationWindowStart)
        {
            out.applicationMeasurementSeconds =
                (measurementEnd - m_applicationWindowStart).GetSeconds();
        }
    }
    else
    {
        out.applicationMeasurementSeconds = elapsed;
    }
    out.applicationSinkCount = static_cast<uint32_t>(m_applicationSinks.size());
    long double squaredApplicationBytes = 0.0;
    uint64_t minimumApplicationBytes = 0;
    uint64_t maximumApplicationBytes = 0;
    bool firstApplicationSink = true;
    for (const auto& sink : m_applicationSinks)
    {
        const uint64_t bytes = sink ? sink->GetTotalRx() : 0;
        out.applicationRxBytes += bytes;
        squaredApplicationBytes += static_cast<long double>(bytes) * bytes;
        if (bytes > 0)
        {
            ++out.applicationSinksWithRx;
        }
        if (firstApplicationSink)
        {
            minimumApplicationBytes = maximumApplicationBytes = bytes;
            firstApplicationSink = false;
        }
        else
        {
            minimumApplicationBytes = std::min(minimumApplicationBytes, bytes);
            maximumApplicationBytes = std::max(maximumApplicationBytes, bytes);
        }
    }
    if (out.applicationMeasurementSeconds > 0.0)
    {
        const double bytesToMbps = 8.0 / out.applicationMeasurementSeconds / 1e6;
        out.applicationGoodputMbps = out.applicationRxBytes * bytesToMbps;
        if (out.applicationSinkCount > 0)
        {
            out.meanSinkGoodputMbps = out.applicationGoodputMbps / out.applicationSinkCount;
            out.minimumSinkGoodputMbps = minimumApplicationBytes * bytesToMbps;
            out.maximumSinkGoodputMbps = maximumApplicationBytes * bytesToMbps;
        }
    }
    if (out.applicationSinkCount > 0 && squaredApplicationBytes > 0.0)
    {
        const long double total = out.applicationRxBytes;
        out.sinkGoodputJainFairness = static_cast<double>(
            total * total / (static_cast<long double>(out.applicationSinkCount) *
                             squaredApplicationBytes));
    }
    if (m_beamManager)
    {
        auto events = m_beamManager->GetHandoverHistory();
        std::vector<double> latency;
        double sum = 0;
        out.handovers = events.size(); out.pingPongs = m_beamManager->GetPingPongCount();
        for (const auto& e : events)
        {
            if (e.success)
            {
                ++out.successfulHandovers;
                sum += e.handoverLatencyMs;
                latency.push_back(e.handoverLatencyMs);
            }
        }
        out.handoverSuccessRatio = events.empty() ? 1.0 : static_cast<double>(out.successfulHandovers) / events.size();
        out.meanHandoverLatencyMs = latency.empty() ? 0.0 : sum / latency.size();
        if (!latency.empty()) { std::sort(latency.begin(), latency.end()); const size_t i = static_cast<size_t>(std::ceil(0.95 * latency.size())) - 1; out.p95HandoverLatencyMs = latency[i]; }
    }
    return out;
}

void
LeoSimStatisticsHelper::StartPeriodicSampling(Time interval, const std::string& filename)
{
    if (!interval.IsStrictlyPositive()) throw std::invalid_argument("statistics sampling interval must be positive");
    StopPeriodicSampling();
    m_sampleInterval = interval; m_csvFilename = filename; m_sampling = true;
    m_lastCsvSampleSeconds = -1.0;
    m_lastCsvApplicationRxBytes = 0;
    WriteCsvHeader();
    m_sampleEvent = Simulator::Schedule(interval, &LeoSimStatisticsHelper::Sample, this);
}

void
LeoSimStatisticsHelper::StopPeriodicSampling()
{
    m_sampling = false;
    if (m_sampleEvent.IsPending()) Simulator::Cancel(m_sampleEvent);
}

void
LeoSimStatisticsHelper::SetRouteStatistics(const LeoSimRouteStatistics& statistics)
{
    m_routeStatistics = statistics;
}

void
LeoSimStatisticsHelper::WriteCsvHeader()
{
    std::ofstream out(m_csvFilename, std::ios::trunc);
    if (!out) throw std::runtime_error("cannot open statistics CSV: " + m_csvFilename);
    out << "time_s,tx_packets,rx_packets,lost_packets,pdr,throughput_mbps,offered_mbps,app_rx_bytes,goodput_mbps,interval_goodput_mbps,app_measurement_s,app_sinks,app_sinks_with_rx,sink_goodput_jain_fairness,delay_ms,jitter_ms,hop_count,snr_mean_db,snr_min_db,snr_max_db,doppler_mean_hz,path_loss_mean_db,link_up,link_down,link_degraded,handovers,ho_success_ratio,ho_latency_mean_ms,ho_latency_p95_ms,ping_pongs\n";
}

void
LeoSimStatisticsHelper::Sample()
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.statistics_sampling");
    if (!m_sampling) return;
    AppendCsvSnapshot();
    m_sampleEvent = Simulator::Schedule(m_sampleInterval, &LeoSimStatisticsHelper::Sample, this);
}

void
LeoSimStatisticsHelper::WriteFinalSample()
{
    if (!m_csvFilename.empty())
    {
        AppendCsvSnapshot();
    }
}

void
LeoSimStatisticsHelper::AppendCsvSnapshot()
{
    const double nowSeconds = Simulator::Now().GetSeconds();
    if (nowSeconds == m_lastCsvSampleSeconds)
    {
        return;
    }
    const auto s = GetSnapshot(false);
    double intervalGoodputMbps = 0.0;
    if (m_lastCsvSampleSeconds >= 0.0 && nowSeconds > m_lastCsvSampleSeconds &&
        s.applicationRxBytes >= m_lastCsvApplicationRxBytes)
    {
        intervalGoodputMbps =
            8.0 * (s.applicationRxBytes - m_lastCsvApplicationRxBytes) /
            (nowSeconds - m_lastCsvSampleSeconds) / 1e6;
    }
    else if (s.applicationMeasurementSeconds > 0.0)
    {
        intervalGoodputMbps = s.applicationGoodputMbps;
    }
    std::ofstream out(m_csvFilename, std::ios::app);
    if (!out) throw std::runtime_error("cannot append statistics CSV: " + m_csvFilename);
    out << std::setprecision(10) << s.timestamp.GetSeconds() << ',' << s.txPackets << ',' << s.rxPackets << ',' << s.lostPackets << ','
        << s.packetDeliveryRatio << ',' << s.throughputMbps << ',' << s.offeredLoadMbps << ','
        << s.applicationRxBytes << ',' << s.applicationGoodputMbps << ','
        << intervalGoodputMbps << ','
        << s.applicationMeasurementSeconds << ',' << s.applicationSinkCount << ','
        << s.applicationSinksWithRx << ',' << s.sinkGoodputJainFairness << ','
        << s.meanDelayMs << ',' << s.meanJitterMs << ',' << s.meanHopCount << ','
        << s.snrDb.mean << ',' << s.snrDb.min << ',' << s.snrDb.max << ',' << s.dopplerHz.mean << ',' << s.pathLossDb.mean << ','
        << s.linkUpEvents << ',' << s.linkDownEvents << ',' << s.linkDegradedEvents << ',' << s.handovers << ',' << s.handoverSuccessRatio << ','
        << s.meanHandoverLatencyMs << ',' << s.p95HandoverLatencyMs << ',' << s.pingPongs << '\n';
    m_lastCsvSampleSeconds = nowSeconds;
    m_lastCsvApplicationRxBytes = s.applicationRxBytes;
}

void
LeoSimStatisticsHelper::WriteSummary(const std::string& filename, bool includeFlows) const
{
    const auto s = GetSnapshot(true);
    std::ofstream out(filename, std::ios::trunc);
    if (!out) throw std::runtime_error("cannot open statistics summary: " + filename);
    out << std::setprecision(10) << "{\n  \"schema\": \"leosim.statistics.v1\",\n  \"time_s\": " << s.timestamp.GetSeconds()
        << ",\n  \"network\": {\"tx_packets\": " << s.txPackets << ", \"rx_packets\": " << s.rxPackets << ", \"lost_packets\": " << s.lostPackets
        << ", \"tx_bytes\": " << s.txBytes << ", \"rx_bytes\": " << s.rxBytes << ", \"pdr\": " << s.packetDeliveryRatio
        << ", \"throughput_mbps\": " << s.throughputMbps << ", \"offered_mbps\": " << s.offeredLoadMbps << ", \"delay_ms\": " << s.meanDelayMs
        << ", \"jitter_ms\": " << s.meanJitterMs << ", \"mean_hop_count\": " << s.meanHopCount << "},\n"
        << "  \"application\": {\"rx_bytes\": " << s.applicationRxBytes
        << ", \"goodput_mbps\": " << s.applicationGoodputMbps
        << ", \"measurement_s\": " << s.applicationMeasurementSeconds
        << ", \"sink_count\": " << s.applicationSinkCount
        << ", \"sinks_with_rx\": " << s.applicationSinksWithRx
        << ", \"sink_goodput_mbps\": {\"mean\": " << s.meanSinkGoodputMbps
        << ", \"min\": " << s.minimumSinkGoodputMbps
        << ", \"max\": " << s.maximumSinkGoodputMbps
        << ", \"jain_fairness\": " << s.sinkGoodputJainFairness << "}},\n"
        << "  \"satellite_link\": {\"snr_db\": {\"samples\": " << s.snrDb.count << ", \"mean\": " << s.snrDb.mean << ", \"min\": " << s.snrDb.min << ", \"max\": " << s.snrDb.max << ", \"stddev\": " << s.snrDb.GetStandardDeviation()
        << "}, \"doppler_hz\": {\"samples\": " << s.dopplerHz.count << ", \"mean\": " << s.dopplerHz.mean << "}, \"path_loss_db\": {\"samples\": " << s.pathLossDb.count << ", \"mean\": " << s.pathLossDb.mean
        << "}, \"link_up_events\": " << s.linkUpEvents << ", \"link_down_events\": " << s.linkDownEvents << ", \"link_degraded_events\": " << s.linkDegradedEvents << "},\n"
        << "  \"handover\": {\"total\": " << s.handovers << ", \"successful\": " << s.successfulHandovers << ", \"success_ratio\": " << s.handoverSuccessRatio << ", \"latency_mean_ms\": " << s.meanHandoverLatencyMs << ", \"latency_p95_ms\": " << s.p95HandoverLatencyMs << ", \"ping_pongs\": " << s.pingPongs << "},\n"
        << "  \"routing\": {\"samples\": " << m_routeStatistics.samples
        << ", \"valid_samples\": " << m_routeStatistics.validSamples
        << ", \"valid_ratio\": "
        << (m_routeStatistics.samples
                ? static_cast<double>(m_routeStatistics.validSamples) /
                      m_routeStatistics.samples
                : 0.0)
        << ", \"changes\": " << m_routeStatistics.changes
        << ", \"unique_paths\": " << m_routeStatistics.uniquePaths
        << ", \"mean_hops\": " << m_routeStatistics.meanHops
        << ", \"mean_distance_km\": " << m_routeStatistics.meanDistanceKm
        << ", \"mean_min_snr_db\": " << m_routeStatistics.meanMinSnrDb
        << ", \"minimum_snr_db\": " << m_routeStatistics.minimumSnrDb
        << ", \"mean_path_loss_db\": " << m_routeStatistics.meanPathLossDb
        << ", \"mean_min_signal_dbm\": " << m_routeStatistics.meanMinSignalDbm << "}";
    if (m_islLoadModel)
    {
        const auto links = m_islLoadModel->GetAllIslLoads();
        const auto satellites = m_islLoadModel->GetAllSatelliteCapacities();
        uint64_t requested = 0;
        uint64_t admitted = 0;
        uint64_t dropped = 0;
        uint64_t residual = 0;
        uint64_t effective = 0;
        uint64_t saturatedLinks = 0;
        uint64_t activeLinks = 0;
        uint64_t saturatedSatellites = 0;
        double requestedLoadSum = 0.0;
        double activeRequestedLoadSum = 0.0;
        double effectiveUtilizationSum = 0.0;
        double routingCostSum = 0.0;
        double maximumRoutingCost = 0.0;
        for (const auto& link : links)
        {
            requestedLoadSum += link.requestedSyntheticLoad;
            if (!link.active)
            {
                continue;
            }
            ++activeLinks;
            activeRequestedLoadSum += link.requestedSyntheticLoad;
            requested += link.requestedBackgroundBps;
            admitted += link.admittedBackgroundBps;
            dropped += link.droppedBackgroundBps;
            residual += link.remainingCapacityBps;
            effective += link.effectiveCapacityBps;
            effectiveUtilizationSum += link.effectiveUtilization;
            const double cost =
                m_islLoadModel->GetRoutingCost(link.sourceSatelliteId,
                                               link.destinationSatelliteId);
            routingCostSum += cost;
            maximumRoutingCost = std::max(maximumRoutingCost, cost);
            saturatedLinks += link.effectiveCapacityBps == 0 ? 1 : 0;
        }
        double satelliteUtilizationSum = 0.0;
        double maximumSatelliteUtilization = 0.0;
        for (const auto& satellite : satellites)
        {
            satelliteUtilizationSum += satellite.utilization;
            maximumSatelliteUtilization =
                std::max(maximumSatelliteUtilization, satellite.utilization);
            saturatedSatellites += satellite.remainingCapacityBps == 0 ? 1 : 0;
        }
        const double linkCount = static_cast<double>(links.size());
        const double activeLinkCount = static_cast<double>(activeLinks);
        const double satelliteCount = static_cast<double>(satellites.size());
        out << ",\n  \"synthetic_isl_load\": {\"directed_links\": " << links.size()
            << ", \"active_directed_links\": " << activeLinks
            << ", \"satellites\": " << satellites.size()
            << ", \"requested_background_bps\": " << requested
            << ", \"admitted_background_bps\": " << admitted
            << ", \"dropped_background_bps\": " << dropped
            << ", \"link_residual_bps\": " << residual
            << ", \"effective_capacity_bps\": " << effective
            << ", \"mean_candidate_requested_load\": "
            << (links.empty() ? 0.0 : requestedLoadSum / linkCount)
            << ", \"mean_active_requested_load\": "
            << (activeLinks == 0 ? 0.0 : activeRequestedLoadSum / activeLinkCount)
            << ", \"mean_effective_utilization\": "
            << (activeLinks == 0 ? 0.0 : effectiveUtilizationSum / activeLinkCount)
            << ", \"mean_satellite_utilization\": "
            << (satellites.empty() ? 0.0 : satelliteUtilizationSum / satelliteCount)
            << ", \"max_satellite_utilization\": " << maximumSatelliteUtilization
            << ", \"mean_routing_cost\": "
            << (activeLinks == 0 ? 0.0 : routingCostSum / activeLinkCount)
            << ", \"max_routing_cost\": " << maximumRoutingCost
            << ", \"saturated_links\": " << saturatedLinks
            << ", \"saturated_satellites\": " << saturatedSatellites << "}";
    }
    if (includeFlows)
    {
        out << ",\n  \"flows\": [";
        const auto flows = GetFlowStatistics();
        for (size_t i = 0; i < flows.size(); ++i)
        {
            const auto& f = flows[i]; if (i) out << ',';
            out << "\n    {\"id\": " << f.flowId << ", \"source\": \"" << f.sourceAddress << "\", \"destination\": \"" << f.destinationAddress
                << "\", \"source_port\": " << f.sourcePort << ", \"destination_port\": " << f.destinationPort << ", \"protocol\": " << unsigned(f.protocol)
                << ", \"tx_packets\": " << f.txPackets << ", \"rx_packets\": " << f.rxPackets << ", \"lost_packets\": " << f.lostPackets
                << ", \"throughput_mbps\": " << f.throughputMbps << ", \"pdr\": " << f.packetDeliveryRatio << ", \"delay_ms\": " << f.meanDelayMs << ", \"jitter_ms\": " << f.meanJitterMs << ", \"mean_hop_count\": " << f.meanHopCount << '}';
        }
        out << (flows.empty() ? "]" : "\n  ]");
    }
    out << "\n}\n";
}

void
LeoSimStatisticsHelper::ResetSatelliteStatistics()
{
    m_snr = {}; m_doppler = {}; m_pathLoss = {};
    m_linkUp = m_linkDown = m_linkDegraded = 0;
}

} // namespace ns3
