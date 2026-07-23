/* SPDX-License-Identifier: GPL-2.0-only */
#include "leosim-statistics-helper.h"

#include "ns3/leosim-beam-manager.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-channel.h"
#include "ns3/log.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/packet.h"
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
    if (m_beamManager)
    {
        auto events = m_beamManager->GetHandoverHistory();
        std::vector<double> latency;
        double sum = 0;
        out.handovers = events.size(); out.pingPongs = m_beamManager->GetPingPongCount();
        for (const auto& e : events) { out.successfulHandovers += e.success; sum += e.handoverLatencyMs; latency.push_back(e.handoverLatencyMs); }
        out.handoverSuccessRatio = events.empty() ? 1.0 : static_cast<double>(out.successfulHandovers) / events.size();
        out.meanHandoverLatencyMs = events.empty() ? 0.0 : sum / events.size();
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
LeoSimStatisticsHelper::WriteCsvHeader()
{
    std::ofstream out(m_csvFilename, std::ios::trunc);
    if (!out) throw std::runtime_error("cannot open statistics CSV: " + m_csvFilename);
    out << "time_s,tx_packets,rx_packets,lost_packets,pdr,throughput_mbps,offered_mbps,delay_ms,jitter_ms,hop_count,snr_mean_db,snr_min_db,snr_max_db,doppler_mean_hz,path_loss_mean_db,link_up,link_down,link_degraded,handovers,ho_success_ratio,ho_latency_mean_ms,ho_latency_p95_ms,ping_pongs\n";
}

void
LeoSimStatisticsHelper::Sample()
{
    if (!m_sampling) return;
    const auto s = GetSnapshot(false);
    std::ofstream out(m_csvFilename, std::ios::app);
    if (!out) throw std::runtime_error("cannot append statistics CSV: " + m_csvFilename);
    out << std::setprecision(10) << s.timestamp.GetSeconds() << ',' << s.txPackets << ',' << s.rxPackets << ',' << s.lostPackets << ','
        << s.packetDeliveryRatio << ',' << s.throughputMbps << ',' << s.offeredLoadMbps << ',' << s.meanDelayMs << ',' << s.meanJitterMs << ',' << s.meanHopCount << ','
        << s.snrDb.mean << ',' << s.snrDb.min << ',' << s.snrDb.max << ',' << s.dopplerHz.mean << ',' << s.pathLossDb.mean << ','
        << s.linkUpEvents << ',' << s.linkDownEvents << ',' << s.linkDegradedEvents << ',' << s.handovers << ',' << s.handoverSuccessRatio << ','
        << s.meanHandoverLatencyMs << ',' << s.p95HandoverLatencyMs << ',' << s.pingPongs << '\n';
    m_sampleEvent = Simulator::Schedule(m_sampleInterval, &LeoSimStatisticsHelper::Sample, this);
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
        << "  \"satellite_link\": {\"snr_db\": {\"samples\": " << s.snrDb.count << ", \"mean\": " << s.snrDb.mean << ", \"min\": " << s.snrDb.min << ", \"max\": " << s.snrDb.max << ", \"stddev\": " << s.snrDb.GetStandardDeviation()
        << "}, \"doppler_hz\": {\"samples\": " << s.dopplerHz.count << ", \"mean\": " << s.dopplerHz.mean << "}, \"path_loss_db\": {\"samples\": " << s.pathLossDb.count << ", \"mean\": " << s.pathLossDb.mean
        << "}, \"link_up_events\": " << s.linkUpEvents << ", \"link_down_events\": " << s.linkDownEvents << ", \"link_degraded_events\": " << s.linkDegradedEvents << "},\n"
        << "  \"handover\": {\"total\": " << s.handovers << ", \"successful\": " << s.successfulHandovers << ", \"success_ratio\": " << s.handoverSuccessRatio << ", \"latency_mean_ms\": " << s.meanHandoverLatencyMs << ", \"latency_p95_ms\": " << s.p95HandoverLatencyMs << ", \"ping_pongs\": " << s.pingPongs << "}";
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
