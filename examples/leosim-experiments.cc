/*
 * Copyright (c) 2026 Anupa De Silva
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 */

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/internet-apps-module.h"
#include "ns3/leosim-beam-manager-helper.h"
#include "ns3/leosim-beam-capacity-manager.h"
#include "ns3/leosim-beam-layout-engine.h"
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-device-installer.h"
#include "ns3/leosim-external-routing-helper.h"
#include "ns3/leosim-isl-load-model.h"
#include "ns3/leosim-loader-helper.h"
#include "ns3/leosim-loader.h"
#include "ns3/leosim-mobility-helper.h"
#include "ns3/leosim-multi-beam-model.h"
#include "ns3/leosim-operator-helper.h"
#include "ns3/leosim-routing-calculator-helper.h"
#include "ns3/leosim-routing-calculator.h"
#include "ns3/leosim-statistics-helper.h"
#include "ns3/leosim-task-profiler.h"
#include "ns3/leosim-tcp-traffic-application.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/rng-seed-manager.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

using namespace ns3;

namespace
{

std::vector<uint32_t>
LoadSatelliteSelection(const std::string& filename)
{
    std::ifstream input(filename);
    if (!input.is_open())
    {
        throw std::runtime_error("cannot open satellite selection file: " + filename);
    }
    std::vector<uint32_t> ids;
    std::set<uint32_t> seen;
    std::string line;
    while (std::getline(input, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        const std::size_t comma = line.find(',');
        const std::string token = line.substr(0, comma);
        try
        {
            std::size_t parsed = 0;
            const unsigned long value = std::stoul(token, &parsed);
            if (parsed == token.size() && value <= std::numeric_limits<uint32_t>::max() &&
                seen.insert(static_cast<uint32_t>(value)).second)
            {
                ids.push_back(static_cast<uint32_t>(value));
            }
        }
        catch (const std::exception&)
        {
            // Permit a CSV header such as "satellite_id,...".
        }
    }
    if (ids.empty())
    {
        throw std::runtime_error("satellite selection file contains no satellite IDs: " + filename);
    }
    return ids;
}

std::string
BuildRunSuffix(const std::string& handoverMode, uint32_t rngSeed, uint64_t rngRun)
{
    const auto now = std::chrono::system_clock::now();
    const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                            now.time_since_epoch())
                            .count();
    const char* schedulerJobId = std::getenv("SLURM_JOB_ID");

    std::ostringstream suffix;
    suffix << '-' << handoverMode << "-seed" << rngSeed << "-run" << rngRun;
    if (schedulerJobId && schedulerJobId[0] != '\0')
    {
        suffix << "-job" << schedulerJobId;
    }
    suffix << "-t" << micros << "-pid" << getpid();
    return suffix.str();
}

class OutputPrefixLock
{
  public:
    explicit OutputPrefixLock(const std::string& outputPrefix)
        : m_path(outputPrefix + ".lock"),
          m_fd(open(m_path.c_str(), O_CREAT | O_RDWR, 0666))
    {
        if (m_fd < 0)
        {
            throw std::runtime_error("Cannot create output lock " + m_path + ": " +
                                     std::strerror(errno));
        }
        if (flock(m_fd, LOCK_EX | LOCK_NB) != 0)
        {
            const std::string reason = std::strerror(errno);
            close(m_fd);
            m_fd = -1;
            throw std::runtime_error("Output prefix is already in use: " + outputPrefix +
                                     " (lock: " + m_path + "): " + reason);
        }
    }

    ~OutputPrefixLock()
    {
        if (m_fd >= 0)
        {
            flock(m_fd, LOCK_UN);
            close(m_fd);
        }
    }

    OutputPrefixLock(const OutputPrefixLock&) = delete;
    OutputPrefixLock& operator=(const OutputPrefixLock&) = delete;

  private:
    std::string m_path;
    int m_fd;
};

} // namespace

namespace
{

class DebugTimer
{
  public:
    DebugTimer()
        : m_start(std::chrono::steady_clock::now()),
          m_last(m_start)
    {
    }

    void Log(const std::string& operation)
    {
        const auto now = std::chrono::steady_clock::now();
        const auto operationStart = m_operationActive ? m_operationStart : m_last;
        const double stepSeconds =
            std::chrono::duration<double>(now - operationStart).count();
        const double totalSeconds = std::chrono::duration<double>(now - m_start).count();
        std::cout << "[timing] " << operation << " completed; step=" << std::fixed
                  << std::setprecision(3) << stepSeconds << "s, total=" << totalSeconds
                  << "s, sim=" << Simulator::Now().GetSeconds() << "s" << std::endl;
        m_measurements.push_back({operation, stepSeconds});
        m_last = now;
        m_operationActive = false;
    }

    void Begin(const std::string& operation)
    {
        m_operationStart = std::chrono::steady_clock::now();
        m_operationActive = true;
        const double totalSeconds = std::chrono::duration<double>(m_operationStart - m_start).count();
        std::cout << "[timing] BEGIN " << operation << "; total=" << std::fixed
                  << std::setprecision(3) << totalSeconds << "s, sim="
                  << Simulator::Now().GetSeconds() << "s" << std::endl;
    }

    void PrintSummary() const
    {
        if (m_measurements.empty())
        {
            return;
        }

        std::vector<Measurement> ranked = m_measurements;
        std::sort(ranked.begin(),
                  ranked.end(),
                  [](const Measurement& lhs, const Measurement& rhs) {
                      return lhs.seconds > rhs.seconds;
                  });

        std::cout << "\n[timing] Process ranking (wall-clock, slowest first)" << std::endl;
        for (std::size_t rank = 0; rank < ranked.size(); ++rank)
        {
            std::cout << "[timing]   " << rank + 1 << ". " << ranked[rank].operation << ": "
                      << std::fixed << std::setprecision(3) << ranked[rank].seconds << "s"
                      << std::endl;
        }
        std::cout << "[timing] SLOWEST PROCESS: " << ranked.front().operation << " consumed "
                  << std::fixed << std::setprecision(3) << ranked.front().seconds << "s"
                  << std::endl;
    }

  private:
    struct Measurement
    {
        std::string operation;
        double seconds;
    };

    std::chrono::steady_clock::time_point m_start;
    std::chrono::steady_clock::time_point m_last;
    std::chrono::steady_clock::time_point m_operationStart;
    bool m_operationActive{false};
    std::vector<Measurement> m_measurements;
};

struct SimulationProgress
{
    std::chrono::steady_clock::time_point wallStart;
    bool taskProfilerEnabled{false};
};

void
LogSimulationProgress(Time interval, Time stopTime, SimulationProgress* progress)
{
    const double wallSeconds = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - progress->wallStart)
                                   .count();
    std::cout << "[timing] simulation progress; sim=" << std::fixed << std::setprecision(3)
              << Simulator::Now().GetSeconds() << "/" << stopTime.GetSeconds()
              << "s, run-wall=" << wallSeconds << "s" << std::endl;
    if (progress->taskProfilerEnabled)
    {
        LeoSimTaskProfiler::PrintIntervalSummary(wallSeconds);
    }
    std::cout << std::flush;
    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval, &LogSimulationProgress, interval, stopTime, progress);
    }
}

LeoSimRoutingCalculator::RoutingMetric
ParseRoutingMetric(const std::string& value)
{
    if (value == "hop" || value == "hop-count")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT;
    if (value == "distance")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_DISTANCE;
    if (value == "path-loss")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_PATH_LOSS;
    if (value == "snr")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_SNR;
    if (value == "signal-strength")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_SIGNAL_STRENGTH;
    if (value == "lifetime" || value == "remaining-lifetime")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_REMAINING_LIFETIME;
    if (value == "load")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_LOAD;
    if (value == "combined" || value == "aldsr")
        return LeoSimRoutingCalculator::LEOSIM_METRIC_COMBINED;

    NS_FATAL_ERROR("Unknown routingMetric '" << value
                                              << "'; use hop, distance, path-loss, snr, "
                                                 "signal-strength, lifetime, load, or combined");
    return LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT;
}

Ipv4Address
GetFirstNonLoopbackAddress(Ptr<Node> node)
{
    Ptr<Ipv4> ipv4 = node->GetObject<Ipv4>();
    if (!ipv4)
    {
        return Ipv4Address::GetAny();
    }

    for (uint32_t iface = 1; iface < ipv4->GetNInterfaces(); ++iface)
    {
        for (uint32_t addr = 0; addr < ipv4->GetNAddresses(iface); ++addr)
        {
            Ipv4Address candidate = ipv4->GetAddress(iface, addr).GetLocal();
            if (candidate != Ipv4Address::GetAny())
            {
                return candidate;
            }
        }
    }
    return Ipv4Address::GetAny();
}

void
AssignPerLinkSubnets(NetDeviceContainer devices, Ipv4AddressHelper& ipv4)
{
    for (uint32_t i = 0; i < devices.GetN(); i += 2)
    {
        NetDeviceContainer linkDevices;
        linkDevices.Add(devices.Get(i));
        if (i + 1 < devices.GetN())
        {
            linkDevices.Add(devices.Get(i + 1));
        }
        ipv4.Assign(linkDevices);
        ipv4.NewNetwork();
    }
}

ApplicationContainer
InstallSingleTcpFlow(NodeContainer ueNodes,
                     NodeContainer serverNodes,
                     Ptr<LeoSimBeamManager> beamManager,
                     uint32_t ueId,
                     uint32_t serverId,
                     uint16_t port,
                     double startTime,
                     double stopTime,
                     const std::string& tcpRate,
                     uint32_t packetSize)
{
    if (ueId >= ueNodes.GetN())
    {
        NS_FATAL_ERROR("ueId " << ueId << " is out of range; loaded " << ueNodes.GetN()
                               << " UEs");
    }
    if (serverId >= serverNodes.GetN())
    {
        NS_FATAL_ERROR("serverId " << serverId << " is out of range; loaded "
                                   << serverNodes.GetN() << " servers");
    }
    if (stopTime <= startTime)
    {
        NS_FATAL_ERROR("appStop must be greater than appStart");
    }

    Ptr<Node> ueNode = ueNodes.Get(ueId);
    Ptr<Node> serverNode = serverNodes.Get(serverId);
    Ipv4Address serverAddress = GetFirstNonLoopbackAddress(serverNode);
    if (serverAddress == Ipv4Address::GetAny())
    {
        NS_FATAL_ERROR("server " << serverId << " has no non-loopback IPv4 address");
    }

    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer apps = sinkHelper.Install(serverNode);
    apps.Start(Seconds(startTime > 0.1 ? startTime - 0.1 : 0.0));
    apps.Stop(Seconds(stopTime));

    Ptr<LeoSimTcpTrafficApplication> source = CreateObject<LeoSimTcpTrafficApplication>();
    source->Configure(InetSocketAddress(serverAddress, port),
                      DataRate(tcpRate),
                      packetSize,
                      Seconds(1.0));
    source->SetHandoverManager(beamManager);
    ueNode->AddApplication(source);
    ApplicationContainer sourceApps(source);
    sourceApps.Start(Seconds(startTime));
    sourceApps.Stop(Seconds(stopTime));
    apps.Add(sourceApps);

    std::cout << "Traffic UE-" << ueId << " -> Server-" << serverId << " (" << serverAddress
              << ':' << port << "), rate=" << tcpRate << ", packetSize=" << packetSize
              << ", start=" << startTime << "s, stop=" << stopTime << "s" << std::endl;
    return apps;
}

ApplicationContainer
InstallAllTcpFlows(NodeContainer ueNodes,
                   NodeContainer serverNodes,
                   Ptr<LeoSimBeamManager> beamManager,
                   uint16_t basePort,
                   double startTime,
                   double stopTime,
                   const std::string& tcpRate,
                   uint32_t packetSize,
                   const std::vector<double>& ueActivationTimes)
{
    ApplicationContainer apps;
    uint32_t flowIndex = 0;
    uint32_t skippedFlows = 0;
    for (uint32_t ue = 0; ue < ueNodes.GetN(); ++ue)
    {
        const double ueStart = ue < ueActivationTimes.size()
                                   ? std::max(startTime, ueActivationTimes[ue])
                                   : startTime;
        for (uint32_t server = 0; server < serverNodes.GetN(); ++server)
        {
            if (ueStart >= stopTime)
            {
                ++skippedFlows;
                continue;
            }
            const uint32_t port = static_cast<uint32_t>(basePort) + flowIndex;
            if (port > 65535)
            {
                NS_FATAL_ERROR("Too many UE-to-server flows for base port " << basePort);
            }
            apps.Add(InstallSingleTcpFlow(ueNodes,
                                          serverNodes,
                                          beamManager,
                                          ue,
                                          server,
                                          static_cast<uint16_t>(port),
                                          ueStart,
                                          stopTime,
                                          tcpRate,
                                          packetSize));
            ++flowIndex;
        }
    }

    std::cout << "Installed full traffic matrix: " << ueNodes.GetN() << " UEs x "
              << serverNodes.GetN() << " servers = " << flowIndex << " TCP flows";
    if (skippedFlows > 0)
    {
        std::cout << " (" << skippedFlows << " skipped because activation is after appStop)";
    }
    std::cout << std::endl;
    return apps;
}

ApplicationContainer
InstallSingleUdpFlow(NodeContainer sourceNodes,
                     NodeContainer destinationNodes,
                     uint32_t sourceIndex,
                     uint32_t destinationIndex,
                     uint16_t port,
                     double startTime,
                     double stopTime,
                     const std::string& udpRate,
                     uint32_t packetSize)
{
    if (sourceIndex >= sourceNodes.GetN())
    {
        NS_FATAL_ERROR("sourceIndex " << sourceIndex << " is out of range; loaded "
                                      << sourceNodes.GetN() << " source nodes");
    }
    if (destinationIndex >= destinationNodes.GetN())
    {
        NS_FATAL_ERROR("destinationIndex " << destinationIndex << " is out of range; loaded "
                                           << destinationNodes.GetN() << " destination nodes");
    }
    if (stopTime <= startTime)
    {
        NS_FATAL_ERROR("appStop must be greater than appStart");
    }

    Ptr<Node> sourceNode = sourceNodes.Get(sourceIndex);
    Ptr<Node> destinationNode = destinationNodes.Get(destinationIndex);
    Ipv4Address destinationAddress = GetFirstNonLoopbackAddress(destinationNode);
    if (destinationAddress == Ipv4Address::GetAny())
    {
        NS_FATAL_ERROR("destination node " << destinationIndex
                                           << " has no non-loopback IPv4 address");
    }

    // UDP sink on the receiving node.
    PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer apps = sinkHelper.Install(destinationNode);
    apps.Start(Seconds(startTime > 0.1 ? startTime - 0.1 : 0.0));
    apps.Stop(Seconds(stopTime));

    // Continuous CBR UDP source (OnOff with constant on-time). Datagrams sent
    // while a handover has no route are dropped silently by the socket, so the
    // flow naturally exposes handover-induced loss without TCP-style retries.
    OnOffHelper source("ns3::UdpSocketFactory", InetSocketAddress(destinationAddress, port));
    source.SetConstantRate(DataRate(udpRate), packetSize);
    ApplicationContainer sourceApps = source.Install(sourceNode);
    sourceApps.Start(Seconds(startTime));
    sourceApps.Stop(Seconds(stopTime));
    apps.Add(sourceApps);

    std::cout << "UDP flow node " << sourceNode->GetId() << " -> node "
              << destinationNode->GetId() << " (" << destinationAddress << ':' << port
              << "), rate=" << udpRate << ", packetSize=" << packetSize << ", start="
              << startTime << "s, stop=" << stopTime << "s" << std::endl;
    return apps;
}

ApplicationContainer
InstallBidirectionalUdpFlows(NodeContainer ueNodes,
                             NodeContainer serverNodes,
                             uint32_t ueId,
                             uint32_t serverId,
                             uint16_t basePort,
                             double startTime,
                             double stopTime,
                             const std::string& udpRate,
                             uint32_t packetSize)
{
    ApplicationContainer apps;
    // Uplink: UE -> server.
    apps.Add(InstallSingleUdpFlow(ueNodes,
                                  serverNodes,
                                  ueId,
                                  serverId,
                                  basePort,
                                  startTime,
                                  stopTime,
                                  udpRate,
                                  packetSize));
    // Downlink: server -> UE.
    apps.Add(InstallSingleUdpFlow(serverNodes,
                                  ueNodes,
                                  serverId,
                                  ueId,
                                  static_cast<uint16_t>(basePort + 1),
                                  startTime,
                                  stopTime,
                                  udpRate,
                                  packetSize));
    std::cout << "Installed bidirectional UDP flows UE-" << ueId << " <-> Server-" << serverId
              << " (ports " << basePort << "/" << (basePort + 1) << ", " << udpRate << ", "
              << packetSize << " B, " << startTime << "s.." << stopTime << "s)" << std::endl;
    return apps;
}

ApplicationContainer
InstallBoundedUePings(NodeContainer ueNodes,
                      const std::vector<std::pair<uint32_t, uint32_t>>& pairs,
                      const std::vector<double>& ueActivationTimes,
                      double startTime,
                      double stopTime,
                      double interval,
                      uint32_t packetSize,
                      double staggerWindow)
{
    ApplicationContainer apps;
    uint32_t installed = 0;
    for (uint32_t index = 0; index < pairs.size(); ++index)
    {
        const auto [sourceIndex, destinationIndex] = pairs[index];
        const double activationTime = std::max(ueActivationTimes.at(sourceIndex),
                                               ueActivationTimes.at(destinationIndex));
        const double stagger = pairs.size() > 1
                                   ? staggerWindow * static_cast<double>(index) /
                                         static_cast<double>(pairs.size())
                                   : 0.0;
        const double pairStart = std::max(startTime, activationTime) + stagger;
        if (pairStart >= stopTime)
        {
            continue;
        }
        const Ipv4Address destination = GetFirstNonLoopbackAddress(ueNodes.Get(destinationIndex));
        if (destination == Ipv4Address::GetAny())
        {
            NS_FATAL_ERROR("UE " << destinationIndex << " has no non-loopback IPv4 address");
        }
        PingHelper ping(destination);
        ping.SetAttribute("Interval", TimeValue(Seconds(interval)));
        ping.SetAttribute("Size", UintegerValue(packetSize));
        ping.SetAttribute("Count", UintegerValue(0));
        // QUIET emits one aggregate delivery/RTT report at shutdown, avoiding
        // per-packet console and CSV overhead while keeping ping results observable.
        ping.SetAttribute("VerboseMode", EnumValue(Ping::QUIET));
        ApplicationContainer pairApps = ping.Install(ueNodes.Get(sourceIndex));
        pairApps.Start(Seconds(pairStart));
        pairApps.Stop(Seconds(stopTime));
        apps.Add(pairApps);
        ++installed;
    }
    std::cout << "Installed " << installed << " bounded UE-to-UE ping pairs (requested="
              << pairs.size() << ", interval=" << interval << "s, size=" << packetSize
              << " bytes, staggerWindow=" << staggerWindow << "s)" << std::endl;
    return apps;
}

/**
 * \brief Per-UE ICMP pinger that targets each UE's currently-connected satellite.
 *
 * Every UE with index >= satellitePingStartIndex keeps a continuous ICMP ping
 * running to the satellite it is currently associated with (its serving beam).
 * The destination follows handovers: whenever the beam manager reports a new
 * serving satellite for a UE (via the per-UE beam-state callback), the live
 * Ping destination is retargeted without resetting its send clock. Each ping
 * runs in QUIET mode. Long simulations are split into
 * wrap-safe segments, so one aggregate delivery/RTT report is printed per
 * segment rather than allowing ns-3's 16-bit Ping counter to wrap. Pings are
 * single-hop over the UE<->satellite access link, so they measure the direct
 * access-link latency and do not require constellation route trees.
 */
class SatellitePinger
{
  public:
    SatellitePinger(Ptr<LeoSimBeamManager> beamManager,
                    LeoSimDeviceInstaller* accessInstaller,
                    const NodeContainer& ueNodes,
                    const NodeContainer& satelliteNodes,
                    double interval,
                    uint32_t packetSize,
                    double stopTime,
                    double rateRamp,
                    double maxRate,
                    double rampStep)
        : m_beamManager(beamManager),
          m_accessInstaller(accessInstaller),
          m_interval(interval),
          m_packetSize(packetSize),
          m_stopTime(stopTime),
          m_rateRamp(rateRamp),
          m_maxRate(maxRate),
          m_rampStep(rampStep),
          m_rampStart(Simulator::Now())
    {
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            Ptr<Node> ueNode = ueNodes.Get(i);
            m_ueNodeById[ueNode->GetId()] = ueNode;
            m_ueIndexByNodeId[ueNode->GetId()] = i;
        }
        for (uint32_t i = 0; i < satelliteNodes.GetN(); ++i)
        {
            Ptr<Node> satellite = satelliteNodes.Get(i);
            m_satNodeById[satellite->GetId()] = satellite;
        }
    }

    /** Write compact per-UE totals and one-second aggregate time-series data. */
    void WriteCsvOutputs(const std::string& perUeFilename,
                         const std::string& timeSeriesFilename) const
    {
        std::ofstream perUe(perUeFilename, std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!perUe.is_open(), "Cannot open satellite-ping CSV: " << perUeFilename);
        perUe << "ue_index,ue_node_id,expected_start_s,measurement_duration_s,tx_packets,"
                 "rx_packets,lost_packets,drop_events,tx_payload_bytes,rx_payload_bytes,"
                 "offered_payload_mbps,received_payload_mbps,delivery_ratio,rtt_samples,"
                 "rtt_mean_ms,rtt_min_ms,rtt_max_ms,target_changes,last_satellite_id,"
                 "first_tx_s,last_tx_s\n";
        perUe << std::setprecision(10);

        std::vector<uint32_t> ueNodeIds;
        ueNodeIds.reserve(m_pingMetrics.size());
        for (const auto& [ueNodeId, metrics] : m_pingMetrics)
        {
            (void)metrics;
            ueNodeIds.push_back(ueNodeId);
        }
        std::sort(ueNodeIds.begin(), ueNodeIds.end(), [this](uint32_t lhs, uint32_t rhs) {
            return m_pingMetrics.at(lhs).ueIndex < m_pingMetrics.at(rhs).ueIndex;
        });
        for (uint32_t ueNodeId : ueNodeIds)
        {
            const PingMetrics& metrics = m_pingMetrics.at(ueNodeId);
            const uint64_t lost = metrics.txPackets >= metrics.rxPackets
                                      ? metrics.txPackets - metrics.rxPackets
                                      : 0;
            const double duration =
                std::max(0.0, m_stopTime - metrics.expectedStart.GetSeconds());
            const double offeredMbps =
                duration > 0.0 ? metrics.txPayloadBytes * 8.0 / duration / 1000000.0 : 0.0;
            const double receivedMbps =
                duration > 0.0 ? metrics.rxPayloadBytes * 8.0 / duration / 1000000.0 : 0.0;
            const double deliveryRatio = metrics.txPackets > 0
                                             ? static_cast<double>(metrics.rxPackets) /
                                                   static_cast<double>(metrics.txPackets)
                                             : 0.0;
            const double rttMean = metrics.rttSamples > 0
                                       ? metrics.rttSumMs /
                                             static_cast<double>(metrics.rttSamples)
                                       : 0.0;
            perUe << metrics.ueIndex << ',' << ueNodeId << ','
                  << metrics.expectedStart.GetSeconds() << ',' << duration << ','
                  << metrics.txPackets << ',' << metrics.rxPackets << ',' << lost << ','
                  << metrics.dropEvents << ',' << metrics.txPayloadBytes << ','
                  << metrics.rxPayloadBytes << ',' << offeredMbps << ',' << receivedMbps << ','
                  << deliveryRatio << ',' << metrics.rttSamples << ',' << rttMean << ','
                  << (metrics.rttSamples > 0 ? metrics.rttMinMs : 0.0) << ','
                  << (metrics.rttSamples > 0 ? metrics.rttMaxMs : 0.0) << ','
                  << metrics.targetChanges << ',' << metrics.lastSatelliteNodeId << ','
                  << (metrics.hasTx ? metrics.firstTx.GetSeconds() : -1.0) << ','
                  << (metrics.hasTx ? metrics.lastTx.GetSeconds() : -1.0) << '\n';
        }
        perUe.flush();

        std::ofstream timeSeries(timeSeriesFilename, std::ios::out | std::ios::trunc);
        NS_ABORT_MSG_IF(!timeSeries.is_open(),
                        "Cannot open satellite-ping time-series CSV: " << timeSeriesFilename);
        timeSeries << "time_start_s,time_end_s,active_ues,tx_packets,rx_packets,drop_events,"
                      "tx_payload_bytes,rx_payload_bytes,offered_payload_mbps,"
                      "received_payload_mbps,delivery_ratio,rtt_samples,rtt_mean_ms,"
                      "rtt_min_ms,rtt_max_ms\n";
        timeSeries << std::setprecision(10);
        for (const auto& [second, metrics] : m_timeSeriesMetrics)
        {
            const double deliveryRatio = metrics.txPackets > 0
                                             ? static_cast<double>(metrics.rxPackets) /
                                                   static_cast<double>(metrics.txPackets)
                                             : 0.0;
            const double rttMean = metrics.rttSamples > 0
                                       ? metrics.rttSumMs /
                                             static_cast<double>(metrics.rttSamples)
                                       : 0.0;
            timeSeries << second << ',' << second + 1 << ',' << metrics.activeUeNodeIds.size()
                       << ',' << metrics.txPackets << ',' << metrics.rxPackets << ','
                       << metrics.dropEvents << ',' << metrics.txPayloadBytes << ','
                       << metrics.rxPayloadBytes << ','
                       << metrics.txPayloadBytes * 8.0 / 1000000.0 << ','
                       << metrics.rxPayloadBytes * 8.0 / 1000000.0 << ',' << deliveryRatio << ','
                       << metrics.rttSamples << ',' << rttMean << ','
                       << (metrics.rttSamples > 0 ? metrics.rttMinMs : 0.0) << ','
                       << (metrics.rttSamples > 0 ? metrics.rttMaxMs : 0.0) << '\n';
        }
        timeSeries.flush();
    }

    /**
     * Schedule a per-UE initial ping at each UE's (staggered) activation time.
     * UEs already connected at that time get a ping immediately; UEs introduced
     * later are handled by OnBeamState() when they first attach to a satellite.
     */
    void ScheduleInitialPings(NodeContainer ueNodes,
                              uint32_t startIndex,
                              double startTime,
                              double stopTime,
                              double staggerWindow,
                              const std::vector<double>& ueActivationTimes)
    {
        const uint32_t count = ueNodes.GetN();
        m_eligibleUeNodeIds.clear();
        m_eligibleStartTimes.clear();
        m_pingMetrics.clear();
        m_timeSeriesMetrics.clear();
        m_rampStart = Seconds(startTime);
        for (uint32_t i = startIndex; i < count; ++i)
        {
            m_eligibleUeNodeIds.insert(ueNodes.Get(i)->GetId());
        }
        for (uint32_t i = startIndex; i < count; ++i)
        {
            const uint32_t ueNodeId = ueNodes.Get(i)->GetId();
            const double activation = std::max(startTime, ueActivationTimes.at(i));
            const double stagger = count > startIndex
                                       ? staggerWindow * static_cast<double>(i - startIndex) /
                                             static_cast<double>(count - startIndex)
                                       : 0.0;
            const double pingStart = activation + stagger;
            m_eligibleStartTimes[ueNodeId] = Seconds(pingStart);
            PingMetrics& metrics = m_pingMetrics[ueNodeId];
            metrics.ueIndex = m_ueIndexByNodeId.at(ueNodeId);
            metrics.expectedStart = Seconds(pingStart);
            if (pingStart >= stopTime)
            {
                continue;
            }
            Simulator::Schedule(Seconds(pingStart), [this, ueNodeId]() {
                const LeoSimBeamRecord beam = m_beamManager->GetCurrentBeam(ueNodeId);
                if (beam.satelliteNodeId != std::numeric_limits<uint32_t>::max())
                {
                    OnBeamState(ueNodeId, beam, 0.0);
                }
            });
        }

        // Optional: ramp the per-UE ping rate upward over the simulation so the
        // aggregate serving-beam access load grows with time.
        if (m_rateRamp > 0.0)
        {
            m_rampEventId = Simulator::Schedule(Seconds(startTime + m_rampStep),
                                                &SatellitePinger::RampLoad,
                                                this);
        }
    }

    /** Current per-UE ping rate (pings/sec), grown by the load ramp. */
    double CurrentRate() const
    {
        const double rate0 = 1.0 / m_interval;
        const double elapsed = std::max(0.0,
                                        Simulator::Now().GetSeconds() -
                                            m_rampStart.GetSeconds());
        return std::min(m_maxRate, std::max(rate0, rate0 + m_rateRamp * elapsed));
    }

    /** Current ping interval (seconds) corresponding to CurrentRate(). */
    double CurrentInterval() const
    {
        return 1.0 / CurrentRate();
    }

    /**
     * Periodic re-application of the ramped ping rate to every active ping.
     * Ping::Send() re-reads its Interval attribute before each transmission, so
     * updating it here raises the rate from the next request onward.
     */
    void RampLoad()
    {
        const double interval = CurrentInterval();
        for (const auto& [ueNodeId, app] : m_pingApplications)
        {
            (void)ueNodeId;
            if (app)
            {
                app->SetAttribute("Interval", TimeValue(Seconds(interval)));
            }
        }
        m_rampEventId =
            Simulator::Schedule(Seconds(m_rampStep), &SatellitePinger::RampLoad, this);
    }

    /** Called by the beam manager whenever a UE's serving beam changes. */
    void OnBeamState(uint32_t ueNodeId, LeoSimBeamRecord beam, double /*score*/)
    {
        // The beam-state callback is global, so it also reports UEs below
        // satellitePingStartIndex. Keep the configured selection authoritative
        // for initial attachment and every later handover.
        if (m_eligibleUeNodeIds.find(ueNodeId) == m_eligibleUeNodeIds.end())
        {
            return;
        }
        const auto eligibleStart = m_eligibleStartTimes.find(ueNodeId);
        if (eligibleStart == m_eligibleStartTimes.end() ||
            Simulator::Now() < eligibleStart->second)
        {
            return;
        }
        // GetCurrentBeam() uses UINT32_MAX (not 0) as the "no serving satellite"
        // sentinel; satellite node id 0 is a real satellite in the container.
        if (beam.satelliteNodeId == std::numeric_limits<uint32_t>::max())
        {
            return;
        }
        const auto target = m_targetSatellite.find(ueNodeId);
        if (target != m_targetSatellite.end() && target->second == beam.satelliteNodeId)
        {
            return;
        }
        const Ipv4Address targetAddress =
            ResolveSatelliteLinkAddress(ueNodeId, beam.satelliteNodeId);
        if (targetAddress == Ipv4Address::GetAny())
        {
            return;
        }
        auto metrics = m_pingMetrics.find(ueNodeId);
        if (metrics != m_pingMetrics.end())
        {
            if (target != m_targetSatellite.end())
            {
                ++metrics->second.targetChanges;
            }
            metrics->second.lastSatelliteNodeId = beam.satelliteNodeId;
        }
        const auto existingPing = m_pingApplications.find(ueNodeId);
        if (existingPing != m_pingApplications.end() && existingPing->second)
        {
            // Ping::Send reads Destination for every request. Retargeting the
            // existing application preserves its send phase across policies.
            existingPing->second->SetAttribute("Destination", AddressValue(targetAddress));
            m_targetSatellite[ueNodeId] = beam.satelliteNodeId;
            return;
        }
        InstallPing(ueNodeId, beam.satelliteNodeId);
    }

  private:
    struct PingMetrics
    {
        uint32_t ueIndex{0};
        Time expectedStart{Seconds(0)};
        uint64_t txPackets{0};
        uint64_t rxPackets{0};
        uint64_t dropEvents{0};
        uint64_t txPayloadBytes{0};
        uint64_t rxPayloadBytes{0};
        uint64_t rttSamples{0};
        double rttSumMs{0.0};
        double rttMinMs{std::numeric_limits<double>::infinity()};
        double rttMaxMs{0.0};
        uint32_t targetChanges{0};
        uint32_t lastSatelliteNodeId{std::numeric_limits<uint32_t>::max()};
        bool hasTx{false};
        Time firstTx{Seconds(0)};
        Time lastTx{Seconds(0)};
    };

    struct TimeSeriesMetrics
    {
        uint64_t txPackets{0};
        uint64_t rxPackets{0};
        uint64_t dropEvents{0};
        uint64_t txPayloadBytes{0};
        uint64_t rxPayloadBytes{0};
        uint64_t rttSamples{0};
        double rttSumMs{0.0};
        double rttMinMs{std::numeric_limits<double>::infinity()};
        double rttMaxMs{0.0};
        std::set<uint32_t> activeUeNodeIds;
    };

    // ns-3 Ping uses a 16-bit sequence number for both its wire header and its
    // internal request lookup. Keep every LeoSim-created Ping comfortably below
    // the 65,536-request wrap boundary, then replace it with a fresh segment.
    // This leaves the ns-3 core untouched while keeping each printed RTT/loss
    // summary valid.
    static constexpr uint32_t PING_SEGMENT_REQUEST_LIMIT = 60000;

    uint64_t CurrentTimeBin() const
    {
        return static_cast<uint64_t>(std::max(0.0, Simulator::Now().GetSeconds()));
    }

    void OnPingTx(uint32_t ueNodeId, uint16_t /*sequence*/, Ptr<Packet> /*packet*/)
    {
        auto metrics = m_pingMetrics.find(ueNodeId);
        if (metrics == m_pingMetrics.end())
        {
            return;
        }
        ++metrics->second.txPackets;
        metrics->second.txPayloadBytes += m_packetSize;
        metrics->second.lastTx = Simulator::Now();
        if (!metrics->second.hasTx)
        {
            metrics->second.hasTx = true;
            metrics->second.firstTx = Simulator::Now();
        }
        TimeSeriesMetrics& timeSeries = m_timeSeriesMetrics[CurrentTimeBin()];
        ++timeSeries.txPackets;
        timeSeries.txPayloadBytes += m_packetSize;
        timeSeries.activeUeNodeIds.insert(ueNodeId);
    }

    void OnPingRtt(uint32_t ueNodeId, uint16_t /*sequence*/, Time rtt)
    {
        auto metrics = m_pingMetrics.find(ueNodeId);
        if (metrics == m_pingMetrics.end())
        {
            return;
        }
        const double rttMs = rtt.GetSeconds() * 1000.0;
        ++metrics->second.rxPackets;
        metrics->second.rxPayloadBytes += m_packetSize;
        ++metrics->second.rttSamples;
        metrics->second.rttSumMs += rttMs;
        metrics->second.rttMinMs = std::min(metrics->second.rttMinMs, rttMs);
        metrics->second.rttMaxMs = std::max(metrics->second.rttMaxMs, rttMs);

        TimeSeriesMetrics& timeSeries = m_timeSeriesMetrics[CurrentTimeBin()];
        ++timeSeries.rxPackets;
        timeSeries.rxPayloadBytes += m_packetSize;
        ++timeSeries.rttSamples;
        timeSeries.rttSumMs += rttMs;
        timeSeries.rttMinMs = std::min(timeSeries.rttMinMs, rttMs);
        timeSeries.rttMaxMs = std::max(timeSeries.rttMaxMs, rttMs);
    }

    void OnPingDrop(uint32_t ueNodeId,
                    uint16_t /*sequence*/,
                    Ping::DropReason /*reason*/)
    {
        auto metrics = m_pingMetrics.find(ueNodeId);
        if (metrics == m_pingMetrics.end())
        {
            return;
        }
        ++metrics->second.dropEvents;
        ++m_timeSeriesMetrics[CurrentTimeBin()].dropEvents;
    }

    Ipv4Address ResolveSatelliteLinkAddress(uint32_t ueNodeId, uint32_t satelliteNodeId) const
    {
        const auto ueIt = m_ueNodeById.find(ueNodeId);
        const auto satIt = m_satNodeById.find(satelliteNodeId);
        if (ueIt == m_ueNodeById.end() || satIt == m_satNodeById.end())
        {
            return Ipv4Address::GetAny();
        }
        Ptr<Ipv4> satIpv4 = satIt->second->GetObject<Ipv4>();
        if (!satIpv4)
        {
            return Ipv4Address::GetAny();
        }
        const NetDeviceContainer linkDevices =
            m_accessInstaller->GetDevicesForLink(ueIt->second, satIt->second);
        for (uint32_t d = 0; d < linkDevices.GetN(); ++d)
        {
            Ptr<NetDevice> device = linkDevices.Get(d);
            if (device->GetNode() != satIt->second)
            {
                continue;
            }
            const int32_t interface = satIpv4->GetInterfaceForDevice(device);
            if (interface >= 0 && satIpv4->GetNAddresses(interface) > 0)
            {
                return satIpv4->GetAddress(interface, 0).GetLocal();
            }
        }
        return Ipv4Address::GetAny();
    }

    void InstallPing(uint32_t ueNodeId, uint32_t satelliteNodeId)
    {
        // Never create a ping after the application window ends. A ping created
        // so close to Simulator::Stop might never have its StartApplication event
        // run; its report (Ping::PrintReport) then divides by m_seq == 0 and the
        // process dies with SIGFPE at Simulator::Destroy.
        if (Simulator::Now().GetSeconds() >= m_stopTime)
        {
            return;
        }
        const auto ueIt = m_ueNodeById.find(ueNodeId);
        if (ueIt == m_ueNodeById.end())
        {
            return;
        }
        const Ipv4Address target = ResolveSatelliteLinkAddress(ueNodeId, satelliteNodeId);
        if (target == Ipv4Address::GetAny())
        {
            return;
        }

        // Retire the previous wrap-safe segment, if any. Handovers retarget the
        // live Ping in OnBeamState() and never enter this replacement path. A Ping that was
        // installed at this exact sim time has not started yet (its Initialize and
        // Start events are still queued): disposing it there would leave those
        // events pending (a stale ping to the old satellite) and call
        // Ping::PrintReport with zero packets sent. Stopping it instead lets it
        // send at most one stray ping and then stop cleanly.
        const auto previous = m_pingApplications.find(ueNodeId);
        if (previous != m_pingApplications.end() && previous->second)
        {
            const auto startIt = m_pingStartTime.find(ueNodeId);
            if (startIt == m_pingStartTime.end() || Simulator::Now() > startIt->second)
            {
                previous->second->Dispose();
            }
            else
            {
                previous->second->SetStopTime(Simulator::Now());
            }
        }

        // A scheduled segment rollover supersedes the previous rollover event.
        // Cancel it before installing the replacement ping.
        const auto rollover = m_pingRolloverEvents.find(ueNodeId);
        if (rollover != m_pingRolloverEvents.end() && rollover->second.IsPending())
        {
            rollover->second.Cancel();
        }

        PingHelper ping(target);
        ping.SetAttribute("Interval", TimeValue(Seconds(CurrentInterval())));
        ping.SetAttribute("Size", UintegerValue(m_packetSize));
        ping.SetAttribute("Count", UintegerValue(PING_SEGMENT_REQUEST_LIMIT));
        // QUIET emits one aggregate delivery/RTT report at shutdown, avoiding
        // per-packet console overhead while keeping RTT to the serving satellite
        // observable for each wrap-safe association segment.
        ping.SetAttribute("VerboseMode", EnumValue(Ping::QUIET));
        ApplicationContainer apps = ping.Install(ueIt->second);
        Ptr<Ping> pingApplication = DynamicCast<Ping>(apps.Get(0));
        NS_ABORT_MSG_IF(!pingApplication, "Satellite PingHelper installed a non-Ping application");
        pingApplication->TraceConnectWithoutContext(
            "Tx",
            MakeCallback(&SatellitePinger::OnPingTx, this, ueNodeId));
        pingApplication->TraceConnectWithoutContext(
            "Rtt",
            MakeCallback(&SatellitePinger::OnPingRtt, this, ueNodeId));
        pingApplication->TraceConnectWithoutContext(
            "Drop",
            MakeCallback(&SatellitePinger::OnPingDrop, this, ueNodeId));
        // Applications added after Simulator::Run begins schedule start/stop
        // values as relative delays. Start now and stop at the remaining part
        // of the common absolute application window.
        apps.Start(Seconds(0));
        apps.Stop(Seconds(m_stopTime - Simulator::Now().GetSeconds()));
        m_pingApplications[ueNodeId] = pingApplication;
        m_targetSatellite[ueNodeId] = satelliteNodeId;
        m_pingStartTime[ueNodeId] = Simulator::Now();

        // CurrentRate() is always bounded by m_maxRate, so this replacement is
        // scheduled no later than the time needed to send the segment limit.
        // Count is also capped as a second line of defence. A report emitted at
        // rollover describes only this wrap-safe segment.
        const double rolloverDelay =
            static_cast<double>(PING_SEGMENT_REQUEST_LIMIT) / m_maxRate;
        if (Simulator::Now().GetSeconds() + rolloverDelay < m_stopTime)
        {
            m_pingRolloverEvents[ueNodeId] =
                Simulator::Schedule(Seconds(rolloverDelay),
                                    &SatellitePinger::RolloverPing,
                                    this,
                                    ueNodeId);
        }
    }

    void RolloverPing(uint32_t ueNodeId)
    {
        if (Simulator::Now().GetSeconds() >= m_stopTime)
        {
            return;
        }
        const auto target = m_targetSatellite.find(ueNodeId);
        if (target == m_targetSatellite.end())
        {
            return;
        }
        InstallPing(ueNodeId, target->second);
    }

    Ptr<LeoSimBeamManager> m_beamManager;
    LeoSimDeviceInstaller* m_accessInstaller;
    std::map<uint32_t, Ptr<Node>> m_ueNodeById;
    std::map<uint32_t, uint32_t> m_ueIndexByNodeId;
    std::map<uint32_t, Ptr<Node>> m_satNodeById;
    std::set<uint32_t> m_eligibleUeNodeIds;
    std::map<uint32_t, Time> m_eligibleStartTimes;
    std::map<uint32_t, uint32_t> m_targetSatellite;
    std::map<uint32_t, Ptr<Application>> m_pingApplications;
    std::map<uint32_t, Time> m_pingStartTime;
    std::map<uint32_t, EventId> m_pingRolloverEvents;
    std::map<uint32_t, PingMetrics> m_pingMetrics;
    std::map<uint64_t, TimeSeriesMetrics> m_timeSeriesMetrics;
    double m_interval;
    uint32_t m_packetSize;
    double m_stopTime;
    double m_rateRamp;
    double m_maxRate;
    double m_rampStep;
    Time m_rampStart;
    EventId m_rampEventId;
};

void
PrintFlowMonitorSummary(Ptr<FlowMonitor> monitor, Ptr<Ipv4FlowClassifier> classifier)
{
    monitor->CheckForLostPackets();
    const auto stats = monitor->GetFlowStats();

    std::cout << "\nFlow summary" << std::endl;
    for (const auto& entry : stats)
    {
        Ipv4FlowClassifier::FiveTuple tuple = classifier->FindFlow(entry.first);
        const FlowMonitor::FlowStats& flow = entry.second;
        const double duration =
            (flow.timeLastRxPacket - flow.timeFirstTxPacket).GetSeconds();
        const double throughputMbps =
            duration > 0.0 ? (flow.rxBytes * 8.0 / duration / 1000000.0) : 0.0;
        const double meanDelayMs =
            flow.rxPackets > 0
                ? (flow.delaySum.GetSeconds() * 1000.0 / static_cast<double>(flow.rxPackets))
                : 0.0;

        std::cout << "  flow=" << entry.first << " " << tuple.sourceAddress << ':'
                  << tuple.sourcePort << " -> " << tuple.destinationAddress << ':'
                  << tuple.destinationPort << " txPackets=" << flow.txPackets
                  << " rxPackets=" << flow.rxPackets << " lostPackets=" << flow.lostPackets
                  << " throughputMbps=" << std::fixed << std::setprecision(6)
                  << throughputMbps << " meanDelayMs=" << meanDelayMs << std::endl;
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string leosimDataDir = "contrib/leosim/data";
    std::string satelliteFile = leosimDataDir + "/prepro/satellite_mobility.tcl";
    std::string groundDeviceFile;
    std::string outputPrefix = "leosim-experiments";
    bool uniqueOutputPrefix = true;
    double simTime = 200.0;
    bool useTrace = true;
    bool verbose = false;

    uint32_t numSatellites = 500;
    std::string satelliteSelectionFile;
    uint32_t numServers = 0;
    uint32_t numUes = 0;
    uint32_t groundDevicesPerOperator = 0;
    double minElevation = 10.0;
    double accessMaxDistance = 2500000.0;
    std::string accessDataRate = "100Mbps";
    bool enableSharedBeamCapacity = true;
    std::string beamCapacity;
    std::string beamDownlinkCapacity = "1Gbps";
    std::string beamUplinkCapacity = "250Mbps";
    std::string satelliteDownlinkCapacity = "20Gbps";
    std::string satelliteUplinkCapacity = "5Gbps";
    std::string uePackageDownlinkRate = "100Mbps";
    std::string uePackageUplinkRate = "20Mbps";
    std::string ueServiceProfileFile;
    std::string beamScheduler = "alpha-fair";
    double alphaFairness = 1.0;
    double queuePressureWeight = 1.0;
    bool beamDemandAware = true;
    double beamActiveUserTimeoutMs = 200.0;
    double beamCapacityUpdateIntervalMs = 100.0;
    double beamCapacityLogIntervalMs = 0.0;
    bool enableBeamCapacityLogging = false;
    std::string accessDelay = "1ms";
    std::string delayMode = "geometry";
    double delayUpdateInterval = 1.0;
    double propagationSpeed = 299792458.0;
    uint32_t maxAccessSatellites = 8;
    double accessCandidateSampleInterval = 10.0;

    bool enableIsl = true;
    bool islFullMesh = false;
    uint32_t maxIslNeighbors = 4;
    double islMaxDistance = 5000000.0;
    double islFrequency = 26.0e9;
    double islTransmitPower = 30.0;
    double islAntennaGain = 35.0;
    std::string islDataRate = "10Gbps";
    std::string islDelay = "100us";
    bool enableSyntheticIslLoad = true;
    bool enableIslLoadLogging = true;
    uint64_t syntheticLoadSeed = 12345;
    double syntheticLoadMin = 0.1;
    double syntheticLoadMax = 0.9;
    std::string syntheticLoadDistribution = "uniform";
    std::string satelliteIslCapacity = "30Gbps";

    bool enableDynamicRouting = true;
    double routingUpdateInterval = 30.0;
    std::string routingMetricName = "hop";
    double combinedLoadWeight = 0.50;
    double combinedDistanceWeight = 0.30;
    double combinedLifetimeWeight = 0.15;
    double combinedSnrWeight = 0.0;
    double combinedHopWeight = 0.05;
    double combinedMaxDistance = 5000000.0;
    double combinedTargetLifetime = 120.0;
    double combinedMinLifetime = 60.0;
    double combinedMinSnr = 10.0;
    double combinedGoodSnr = 20.0;
    double combinedCriticalUtilization = 0.90;
    bool useRouteTreeCache = true;
    std::string routeTreeEngine = "contrib/leosim/utils/rengine/leosim-rengine";
    std::string routeTreeWorkDir = "/tmp/leosim-experiments-route-trees";
    uint32_t routeTreeWorkers = 8;
    uint64_t routeTreeMaxEntries = 10000000ULL;
    uint32_t beamNumRings = 2;
    double beamRadiusKm = 250.0;
    uint32_t beamReuseColors = 3;
    uint32_t serverId = 0;
    uint32_t ueId = 0;
    uint16_t port = 9000;
    double appStart = 1.0;
    double appStop = 599.0;
    std::string tcpRate = "1Mbps";
    uint32_t tcpPacketSize = 1024;
    std::string udpRate = "1Mbps";
    uint32_t udpPacketSize = 1400;
    bool enableDataTraffic = true;
    bool allToAllTraffic = true;
    bool enableGroundStations = true;
    bool enableUePing = false;
    uint32_t uePingStartIndex = 1;
    uint32_t uePingPairs = 100;
    double uePingInterval = 1.0;
    uint32_t uePingPacketSize = 56;
    double uePingStaggerWindow = 1.0;
    bool enableSatellitePing = false;
    uint32_t satellitePingStartIndex = 1;
    double satellitePingInterval = 1.0;
    uint32_t satellitePingPacketSize = 56;
    double satellitePingStaggerWindow = 1.0;
    double satellitePingLoadRamp = 0.0;
    double satellitePingMaxRate = 50.0;
    double satellitePingRampInterval = 1.0;
    uint32_t satellitePingMetricsVersion = 1;
    bool enableDynamicGroundNodes = false;
    uint32_t initialUes = 0;
    double ueIntroductionStart = 30.0;
    double ueIntroductionInterval = 10.0;
    uint32_t uesPerIntroduction = 100;
    bool enableGroundNodeLifecycleLogging = true;
    bool writeFlowMonitor = true;
    bool enableVisualization = false;
    double visualizationInterval = 1.0;
    bool enableStatistics = true;
    bool enableTaskProfiler = true;
    double statisticsInterval = 1.0;
    bool enableRouteLogging = false;
    bool enableHandoverLogging = true;
    double progressLogInterval = 5.0;
    std::string hoModeName = "CHO";
    uint32_t maxCandidates = 3;
    bool enableHoBuffering = true;
    uint32_t hoBufferSize = 1024;
    bool enableLoadBalancing = true;
    double ttt = 1.0;
    double t310 = 1.0;
    uint32_t n310 = 3;
    uint32_t n311 = 3;
    double a3Offset = 3.0;
    double a4Threshold = -110.0;
    double tteTrigger = 30.0;
    double choPrepMs = 100.0;
    double choExecMs = 150.0;
    double beamUpdateIntervalMs = 1000.0;
    uint32_t rngSeed = 1;
    uint64_t rngRun = 1;

    CommandLine cmd;
    cmd.AddValue("satellites", "Path to satellite mobility trace or position CSV", satelliteFile);
    cmd.AddValue("rngSeed", "ns-3 random-number seed", rngSeed);
    cmd.AddValue("rngRun", "ns-3 independent run number", rngRun);
    cmd.AddValue("dataDir", "Path to LeoSim data directory with gss/ and ues/ folders", leosimDataDir);
    cmd.AddValue("groundDevices", "Optional legacy ground device CSV; overrides dataDir when set", groundDeviceFile);
    cmd.AddValue("useTrace", "Load satellites from ns-2 trace format", useTrace);
    cmd.AddValue("simTime", "Simulation duration in seconds", simTime);
    cmd.AddValue("verbose", "Enable verbose helper output", verbose);
    cmd.AddValue("numSatellites", "Number of satellites to use; 0 means all loaded satellites", numSatellites);
    cmd.AddValue("satelliteSelectionFile",
                 "CSV whose first column lists original satellite IDs to use exclusively",
                 satelliteSelectionFile);
    cmd.AddValue("numServers", "Number of servers/GSS to use; 0 means all loaded servers", numServers);
    cmd.AddValue("numUes", "Number of UEs to use; 0 means all loaded UEs", numUes);
    cmd.AddValue("groundDevicesPerOperator",
                 "Select at most this many servers and UEs from each operator; 0 disables balanced selection",
                 groundDevicesPerOperator);
    cmd.AddValue("minElevation", "Minimum satellite access-link elevation angle in degrees", minElevation);
    cmd.AddValue("accessMaxDistance", "Maximum satellite-ground link distance in meters", accessMaxDistance);
    cmd.AddValue("accessDataRate", "Satellite-ground point-to-point data rate", accessDataRate);
    cmd.AddValue("enableSharedBeamCapacity",
                 "Share a finite beam capacity among associated ground nodes",
                 enableSharedBeamCapacity);
    cmd.AddValue("beamCapacity",
                 "Deprecated symmetric beam capacity override; empty uses separate UL/DL values",
                 beamCapacity);
    cmd.AddValue("beamDownlinkCapacity", "Total downlink capacity per beam", beamDownlinkCapacity);
    cmd.AddValue("beamUplinkCapacity", "Total uplink capacity per beam", beamUplinkCapacity);
    cmd.AddValue("satelliteDownlinkCapacity",
                 "Aggregate downlink capacity shared by all beams on a satellite",
                 satelliteDownlinkCapacity);
    cmd.AddValue("satelliteUplinkCapacity",
                 "Aggregate uplink capacity shared by all beams on a satellite",
                 satelliteUplinkCapacity);
    cmd.AddValue("uePackageDownlinkRate", "Per-UE subscription downlink peak", uePackageDownlinkRate);
    cmd.AddValue("uePackageUplinkRate", "Per-UE subscription uplink peak", uePackageUplinkRate);
    cmd.AddValue("ueServiceProfileFile",
                 "Optional UE profile CSV: node_id,dl_peak,dl_min,ul_peak,ul_min,weight",
                 ueServiceProfileFile);
    cmd.AddValue("beamScheduler", "Beam scheduler: alpha-fair, pf, or equal", beamScheduler);
    cmd.AddValue("alphaFairness", "Alpha parameter for alpha-fair allocation", alphaFairness);
    cmd.AddValue("queuePressureWeight",
                 "Weight applied to normalized transmit-queue pressure",
                 queuePressureWeight);
    cmd.AddValue("beamDemandAware",
                 "Allocate shared capacity only to recently active users",
                 beamDemandAware);
    cmd.AddValue("beamActiveUserTimeoutMs",
                 "Time after last offered packet that a direction remains active",
                 beamActiveUserTimeoutMs);
    cmd.AddValue("beamCapacityUpdateIntervalMs",
                 "Beam capacity allocation refresh interval in milliseconds",
                 beamCapacityUpdateIntervalMs);
    cmd.AddValue("beamCapacityLogIntervalMs",
                 "Minimum interval between capacity CSV snapshots in milliseconds; "
                 "0 logs every update",
                 beamCapacityLogIntervalMs);
    cmd.AddValue("enableBeamCapacityLogging",
                 "Write per-user beam capacity allocations to CSV",
                 enableBeamCapacityLogging);
    cmd.AddValue("accessDelay", "Satellite-ground propagation delay, e.g. 1ms", accessDelay);
    cmd.AddValue("delayMode",
                 "Packet propagation-delay model: constant or geometry",
                 delayMode);
    cmd.AddValue("delayUpdateInterval",
                 "Geometry-delay refresh interval in seconds",
                 delayUpdateInterval);
    cmd.AddValue("propagationSpeed",
                 "Signal propagation speed in metres per second for geometry delay",
                 propagationSpeed);
    cmd.AddValue("maxAccessSatellites",
                 "Maximum candidate satellite access links created per ground node",
                 maxAccessSatellites);
    cmd.AddValue("accessCandidateSampleInterval",
                 "Seconds between trajectory samples used to provision access candidates",
                 accessCandidateSampleInterval);
    cmd.AddValue("enableIsl", "Enable inter-satellite links", enableIsl);
    cmd.AddValue("islFullMesh", "Build all-pairs ISL mesh instead of bounded nearest-neighbor mesh", islFullMesh);
    cmd.AddValue("maxIslNeighbors", "Maximum nearest-neighbor ISL degree per satellite", maxIslNeighbors);
    cmd.AddValue("islMaxDistance", "Maximum ISL distance in meters", islMaxDistance);
    cmd.AddValue("islFrequency", "ISL carrier frequency in Hz", islFrequency);
    cmd.AddValue("islTransmitPower", "ISL transmit power in dBm", islTransmitPower);
    cmd.AddValue("islAntennaGain", "ISL antenna gain in dB", islAntennaGain);
    cmd.AddValue("islDataRate", "ISL point-to-point data rate", islDataRate);
    cmd.AddValue("islDelay", "ISL propagation delay, e.g. 100us", islDelay);
    cmd.AddValue("enableSyntheticIslLoad",
                 "Generate deterministic synthetic utilization records for directed ISLs",
                 enableSyntheticIslLoad);
    cmd.AddValue("enableIslLoadLogging",
                 "Write synthetic ISL utilization records to <outputPrefix>-isl-load.csv",
                 enableIslLoadLogging);
    cmd.AddValue("syntheticLoadSeed",
                 "Seed used to deterministically generate directed-ISL load",
                 syntheticLoadSeed);
    cmd.AddValue("syntheticLoadMin",
                 "Minimum generated synthetic ISL utilization in [0,1]",
                 syntheticLoadMin);
    cmd.AddValue("syntheticLoadMax",
                 "Maximum generated synthetic ISL utilization in [0,1]",
                 syntheticLoadMax);
    cmd.AddValue("syntheticLoadDistribution",
                 "Synthetic ISL load distribution (currently: uniform)",
                 syntheticLoadDistribution);
    cmd.AddValue("satelliteIslCapacity",
                 "Maximum aggregate outgoing ISL capacity recorded per satellite",
                 satelliteIslCapacity);
    cmd.AddValue("enableDynamicRouting", "Recompute routes periodically during the run", enableDynamicRouting);
    cmd.AddValue("routingUpdateInterval", "Dynamic routing update interval in seconds", routingUpdateInterval);
    cmd.AddValue("routingMetric",
                 "Dijkstra metric: hop, distance, path-loss, snr, signal-strength, lifetime, load, or combined",
                 routingMetricName);
    cmd.AddValue("combinedLoadWeight", "ALDSR load weight", combinedLoadWeight);
    cmd.AddValue("combinedDistanceWeight", "ALDSR distance weight", combinedDistanceWeight);
    cmd.AddValue("combinedLifetimeWeight", "ALDSR remaining-lifetime weight", combinedLifetimeWeight);
    cmd.AddValue("combinedSnrWeight", "ALDSR SNR-margin weight", combinedSnrWeight);
    cmd.AddValue("combinedHopWeight", "ALDSR per-hop weight", combinedHopWeight);
    cmd.AddValue("combinedMaxDistance", "ALDSR distance normalization bound in metres", combinedMaxDistance);
    cmd.AddValue("combinedTargetLifetime", "ALDSR lifetime normalization target in seconds", combinedTargetLifetime);
    cmd.AddValue("combinedMinLifetime", "ALDSR minimum predicted ISL lifetime in seconds", combinedMinLifetime);
    cmd.AddValue("combinedMinSnr", "ALDSR minimum usable SNR in dB", combinedMinSnr);
    cmd.AddValue("combinedGoodSnr", "ALDSR zero-penalty SNR in dB", combinedGoodSnr);
    cmd.AddValue("combinedCriticalUtilization", "ALDSR utilization saturation point in (0,1]", combinedCriticalUtilization);
    cmd.AddValue("useRouteTreeCache",
                 "Use one cached reverse shortest-path tree per traffic destination "
                 "for all supported routing metrics",
                 useRouteTreeCache);
    cmd.AddValue("routeTreeEngine",
                 "Path to the LeoSim destination-tree routing engine",
                 routeTreeEngine);
    cmd.AddValue("routeTreeWorkDir",
                 "Directory for cached routing-tree snapshots",
                 routeTreeWorkDir);
    cmd.AddValue("routeTreeWorkers",
                 "Worker threads used to calculate destination trees",
                 routeTreeWorkers);
    cmd.AddValue("routeTreeMaxEntries",
                 "Maximum next-hop entries materialized in one routing snapshot",
                 routeTreeMaxEntries);
    cmd.AddValue("beamNumRings", "Number of spot-beam rings per satellite", beamNumRings);
    cmd.AddValue("beamRadiusKm", "Spot-beam footprint radius in kilometres", beamRadiusKm);
    cmd.AddValue("beamReuseColors", "Number of spot-beam frequency reuse colours", beamReuseColors);
    cmd.AddValue("serverId", "Server/GSS index used by this single scenario", serverId);
    cmd.AddValue("ueId", "UE index used by this single scenario", ueId);
    cmd.AddValue("port", "TCP destination port", port);
    cmd.AddValue("appStart", "Traffic application start time in seconds", appStart);
    cmd.AddValue("appStop", "Traffic application stop time in seconds", appStop);
    cmd.AddValue("tcpRate", "TCP OnOff offered rate, e.g. 1Mbps", tcpRate);
    cmd.AddValue("tcpPacketSize", "TCP application packet size in bytes", tcpPacketSize);
    cmd.AddValue("enableDataTraffic",
                 "Install TCP/UDP data applications (ping applications are controlled separately)",
                 enableDataTraffic);
    cmd.AddValue("allToAllTraffic",
                 "Install one TCP flow from every UE to every server/GSS",
                 allToAllTraffic);
    cmd.AddValue("enableGroundStations",
                 "Load and simulate ground stations (servers); disable for a UE-only "
                 "ping-to-serving-satellite scenario with no data traffic",
                 enableGroundStations);
    cmd.AddValue("enableUePing",
                 "Install scalable ICMP ping traffic between bounded disjoint UE pairs",
                 enableUePing);
    cmd.AddValue("uePingStartIndex",
                 "First UE index used by bounded pairing (1 skips the special UE 0 endpoint)",
                 uePingStartIndex);
    cmd.AddValue("uePingPairs",
                 "Maximum disjoint UE pairs to ping; bounded by floor(numUes/2)",
                 uePingPairs);
    cmd.AddValue("uePingInterval", "Seconds between ping requests in each UE pair", uePingInterval);
    cmd.AddValue("uePingPacketSize", "ICMP ping payload bytes", uePingPacketSize);
    cmd.AddValue("uePingStaggerWindow",
                 "Seconds over which UE-pair ping starts are evenly staggered",
                 uePingStaggerWindow);
    cmd.AddValue("enableSatellitePing",
                 "Install per-UE ICMP pings to each UE's currently-connected serving satellite",
                 enableSatellitePing);
    cmd.AddValue("satellitePingStartIndex",
                 "First UE index that pings its serving satellite (1 skips the UE-0 traffic endpoint)",
                 satellitePingStartIndex);
    cmd.AddValue("satellitePingInterval",
                 "Seconds between ICMP ping requests a UE sends to its serving satellite",
                 satellitePingInterval);
    cmd.AddValue("satellitePingPacketSize", "ICMP ping payload bytes for satellite pings",
                 satellitePingPacketSize);
    cmd.AddValue("satellitePingStaggerWindow",
                 "Seconds over which per-UE satellite-ping starts are evenly staggered",
                 satellitePingStaggerWindow);
    cmd.AddValue("satellitePingLoadRamp",
                 "Per-UE ping-rate growth in pings/sec per second of simulation (0 = constant)",
                 satellitePingLoadRamp);
    cmd.AddValue("satellitePingMaxRate",
                 "Cap on per-UE ping rate (pings/sec) applied while the load ramp is active",
                 satellitePingMaxRate);
    cmd.AddValue("satellitePingRampInterval",
                 "Seconds between ping-load ramp re-applications",
                 satellitePingRampInterval);
    cmd.AddValue("satellitePingMetricsVersion",
                 "Required structured satellite-ping metrics schema version (currently 1)",
                 satellitePingMetricsVersion);
    cmd.AddValue("udpRate", "Per-direction UDP CBR offered rate, e.g. 1Mbps", udpRate);
    cmd.AddValue("udpPacketSize", "UDP application packet size in bytes", udpPacketSize);
    cmd.AddValue("enableDynamicGroundNodes",
                 "Introduce UEs progressively instead of activating all at time zero",
                 enableDynamicGroundNodes);
    cmd.AddValue("initialUes",
                 "UEs active at time zero when dynamic introduction is enabled",
                 initialUes);
    cmd.AddValue("ueIntroductionStart",
                 "Simulation time in seconds for the first UE activation batch",
                 ueIntroductionStart);
    cmd.AddValue("ueIntroductionInterval",
                 "Seconds between UE activation batches",
                 ueIntroductionInterval);
    cmd.AddValue("uesPerIntroduction",
                 "Number of UEs activated in each periodic batch",
                 uesPerIntroduction);
    cmd.AddValue("enableGroundNodeLifecycleLogging",
                 "Write periodic ground-node activation summaries to CSV",
                 enableGroundNodeLifecycleLogging);
    cmd.AddValue("outputPrefix", "Base prefix used for simulation output files", outputPrefix);
    cmd.AddValue("uniqueOutputPrefix",
                 "Append handover mode, RNG seed/run, job ID, timestamp, and PID to the output prefix",
                 uniqueOutputPrefix);
    cmd.AddValue("writeFlowMonitor", "Write FlowMonitor XML output", writeFlowMonitor);
    cmd.AddValue("enableVisualization",
                 "Write CSV files for the LeoSim 3D visualizer",
                 enableVisualization);
    cmd.AddValue("visualizationInterval",
                 "Visualization logging interval in seconds",
                 visualizationInterval);
    cmd.AddValue("enableStatistics", "Write periodic CSV and summary JSON statistics", enableStatistics);
    cmd.AddValue("enableTaskProfiler",
                 "Enable wall-clock task profiling and timing summaries",
                 enableTaskProfiler);
    cmd.AddValue("statisticsInterval", "Statistics sampling interval in seconds", statisticsInterval);
    cmd.AddValue("enableRouteLogging",
                 "Write selected paths and path-specific routing metrics",
                 enableRouteLogging);
    cmd.AddValue("enableHandoverLogging",
                 "Write handover events to <outputPrefix>-handovers.csv",
                 enableHandoverLogging);
    cmd.AddValue("progressLogInterval",
                 "Simulation-time interval for wall-clock progress logs; 0 disables",
                 progressLogInterval);
    cmd.AddValue("hoMode", "Handover policy: BHO or CHO", hoModeName);
    cmd.AddValue("maxCandidates", "Maximum prepared CHO candidates", maxCandidates);
    cmd.AddValue("enableHoBuffering", "Buffer packets during handover execution", enableHoBuffering);
    cmd.AddValue("hoBufferSize",
                 "Maximum packets buffered per ground node during handover",
                 hoBufferSize);
    cmd.AddValue("enableLoadBalancing", "Include satellite load in handover selection", enableLoadBalancing);
    cmd.AddValue("ttt", "Handover time-to-trigger in seconds", ttt);
    cmd.AddValue("t310", "Radio-link failure timer in seconds", t310);
    cmd.AddValue("n310", "Consecutive out-of-sync indications before T310", n310);
    cmd.AddValue("n311", "Consecutive in-sync indications for recovery", n311);
    cmd.AddValue("a3Offset", "A3 neighbour-better offset in dB", a3Offset);
    cmd.AddValue("a4Threshold", "A4 absolute RSRP threshold in dBm", a4Threshold);
    cmd.AddValue("tteTrigger", "Predictive time-to-exit trigger in seconds", tteTrigger);
    cmd.AddValue("choPrep", "CHO preparation delay in milliseconds", choPrepMs);
    cmd.AddValue("choExec", "CHO execution delay in milliseconds", choExecMs);
    cmd.AddValue("beamUpdateIntervalMs", "Beam manager update interval in milliseconds", beamUpdateIntervalMs);
    cmd.Parse(argc, argv);

    if (accessCandidateSampleInterval <= 0.0)
    {
        NS_FATAL_ERROR("accessCandidateSampleInterval must be positive");
    }

    RngSeedManager::SetSeed(rngSeed);
    RngSeedManager::SetRun(rngRun);

    DebugTimer timer;
    timer.Log("command-line parsing");

    if (enableVisualization && visualizationInterval <= 0.0)
    {
        NS_FATAL_ERROR("visualizationInterval must be greater than zero");
    }
    if (enableStatistics && statisticsInterval <= 0.0)
    {
        NS_FATAL_ERROR("statisticsInterval must be greater than zero");
    }
    if (hoModeName != "BHO" && hoModeName != "CHO")
    {
        NS_FATAL_ERROR("hoMode must be BHO or CHO");
    }
    if (maxCandidates == 0 || hoBufferSize == 0 || ttt < 0.0 || t310 < 0.0 || tteTrigger < 0.0 ||
        choPrepMs < 0.0 || choExecMs < 0.0 || beamUpdateIntervalMs <= 0.0)
    {
        NS_FATAL_ERROR("Invalid handover timer or candidate configuration");
    }
    if (beamScheduler != "alpha-fair" && beamScheduler != "pf" && beamScheduler != "equal")
    {
        NS_FATAL_ERROR("beamScheduler must be alpha-fair, pf, or equal");
    }
    if (beamCapacityUpdateIntervalMs <= 0.0 || beamCapacityLogIntervalMs < 0.0 ||
        beamActiveUserTimeoutMs < 0.0 ||
        alphaFairness < 0.0 || queuePressureWeight < 0.0)
    {
        NS_FATAL_ERROR("Invalid beam capacity scheduler timing configuration");
    }
    if (enableDynamicGroundNodes &&
        (ueIntroductionStart < 0.0 || ueIntroductionInterval <= 0.0 ||
         uesPerIntroduction == 0))
    {
        NS_FATAL_ERROR("Dynamic UE introduction requires a non-negative start time, positive "
                       "interval, and non-zero batch size");
    }
    constexpr uint32_t MAX_IPV4_ICMP_PAYLOAD = 65507;
    if (enableUePing &&
        (uePingPairs == 0 || uePingInterval <= 0.0 || uePingPacketSize < 16 ||
         uePingPacketSize > MAX_IPV4_ICMP_PAYLOAD ||
         uePingStaggerWindow < 0.0))
    {
        NS_FATAL_ERROR("UE ping requires positive pair count/interval, a 16..65507-byte IPv4 "
                       "payload, and non-negative stagger window");
    }
    if (enableSatellitePing &&
        (satellitePingInterval <= 0.0 || satellitePingPacketSize < 16 ||
         satellitePingPacketSize > MAX_IPV4_ICMP_PAYLOAD ||
         satellitePingStaggerWindow < 0.0 || satellitePingLoadRamp < 0.0 ||
         satellitePingMaxRate <= 0.0 || satellitePingRampInterval <= 0.0))
    {
        NS_FATAL_ERROR("Satellite ping requires positive interval, a 16..65507-byte IPv4 "
                       "payload, non-negative stagger window/load ramp, and positive max "
                       "rate and ramp interval");
    }
    if (satellitePingMetricsVersion != 1)
    {
        NS_FATAL_ERROR("satellitePingMetricsVersion must be 1");
    }
    if (uniqueOutputPrefix)
    {
        outputPrefix += BuildRunSuffix(hoModeName, rngSeed, rngRun);
    }
    OutputPrefixLock outputPrefixLock(outputPrefix);
    std::cout << "Output prefix: " << outputPrefix << std::endl;
    const auto routingMetric = ParseRoutingMetric(routingMetricName);
    if ((routingMetric == LeoSimRoutingCalculator::LEOSIM_METRIC_LOAD ||
         routingMetric == LeoSimRoutingCalculator::LEOSIM_METRIC_COMBINED) &&
        !enableSyntheticIslLoad)
    {
        enableSyntheticIslLoad = true;
        std::cout << "load-aware routing: automatically enabling deterministic synthetic ISL load"
                  << std::endl;
    }

    Time::SetResolution(Time::NS);

    timer.Begin("input data loading");
    LeoSimLoaderHelper loaderHelper;
    loaderHelper.SetVerbose(verbose);
    if (useTrace)
    {
        loaderHelper.LoadSatellitesFromTrace(satelliteFile);
    }
    else
    {
        loaderHelper.LoadSatellitesFromCsv(satelliteFile);
    }
    if (!groundDeviceFile.empty())
    {
        loaderHelper.LoadGroundDevicesFromCsv(groundDeviceFile);
    }
    else
    {
        loaderHelper.LoadGroundDevicesFromDataDirectory(leosimDataDir);
    }
    timer.Log("input data loading");

    Ptr<LeoSimLoader> loader = loaderHelper.GetLoader();
    auto serverDeviceIds = loader->GetGroundDeviceIdsByType("SERVER");
    auto ueDeviceIds = loader->GetGroundDeviceIdsByType("UE");
    if (!enableGroundStations)
    {
        // Ground stations are disabled: drop every loaded server so the
        // scenario is UE-only (servers would drive no traffic and are not
        // simulated at all).
        serverDeviceIds.clear();
    }
    if (groundDevicesPerOperator > 0)
    {
        auto selectPerOperator = [&](const std::vector<uint32_t>& deviceIds,
                                     const std::string& deviceType) {
            std::map<LeoSimOperatorId, uint32_t> available;
            std::map<LeoSimOperatorId, uint32_t> selected;
            std::vector<uint32_t> balancedIds;
            for (uint32_t id : deviceIds)
            {
                const LeoSimOperatorId opId = loader->GetGroundDeviceOperator(id);
                ++available[opId];
                if (selected[opId] < groundDevicesPerOperator)
                {
                    balancedIds.push_back(id);
                    ++selected[opId];
                }
            }
            for (const auto& [opId, count] : available)
            {
                if (count < groundDevicesPerOperator)
                {
                    std::ostringstream message;
                    message << "requested " << groundDevicesPerOperator << " " << deviceType
                            << " devices for operator " << opId << ", but only " << count
                            << " were loaded";
                    throw std::runtime_error(message.str());
                }
            }
            return balancedIds;
        };
        try
        {
            serverDeviceIds = selectPerOperator(serverDeviceIds, "SERVER");
            ueDeviceIds = selectPerOperator(ueDeviceIds, "UE");
        }
        catch (const std::runtime_error& error)
        {
            std::cerr << "Error: " << error.what() << std::endl;
            return 1;
        }
    }
    if (loader->GetNumSatellites() == 0 || ueDeviceIds.empty() ||
        (enableGroundStations && serverDeviceIds.empty()))
    {
        std::cerr << "Error: need at least 1 satellite and 1 UE"
                  << (enableGroundStations ? ", and 1 server" : "") << "." << std::endl;
        return 1;
    }

    std::vector<uint32_t> selectedSatelliteIds;
    if (!satelliteSelectionFile.empty())
    {
        try
        {
            selectedSatelliteIds = LoadSatelliteSelection(satelliteSelectionFile);
        }
        catch (const std::runtime_error& error)
        {
            std::cerr << "Error: " << error.what() << std::endl;
            return 1;
        }
        for (uint32_t satelliteId : selectedSatelliteIds)
        {
            if (satelliteId >= loader->GetNumSatellites())
            {
                std::cerr << "Error: satellite selection ID " << satelliteId
                          << " is outside loaded range 0.."
                          << loader->GetNumSatellites() - 1 << std::endl;
                return 1;
            }
        }
        numSatellites = selectedSatelliteIds.size();
    }
    else if (numSatellites == 0)
    {
        numSatellites = loader->GetNumSatellites();
    }
    else if (numSatellites > loader->GetNumSatellites())
    {
        std::cerr << "Error: requested " << numSatellites << " satellites, but only "
                  << loader->GetNumSatellites() << " were loaded." << std::endl;
        return 1;
    }
    if (selectedSatelliteIds.empty())
    {
        selectedSatelliteIds.reserve(numSatellites);
        for (uint32_t satelliteId = 0; satelliteId < numSatellites; ++satelliteId)
        {
            selectedSatelliteIds.push_back(satelliteId);
        }
    }
    if (numServers == 0)
    {
        numServers = serverDeviceIds.size();
    }
    else if (numServers > serverDeviceIds.size())
    {
        std::cerr << "Error: requested " << numServers << " servers, but only "
                  << serverDeviceIds.size() << " were loaded." << std::endl;
        return 1;
    }
    if (numUes == 0)
    {
        numUes = ueDeviceIds.size();
    }
    else if (numUes > ueDeviceIds.size())
    {
        std::cerr << "Error: requested " << numUes << " UEs, but only "
                  << ueDeviceIds.size() << " were loaded." << std::endl;
        return 1;
    }
    // With no ground stations (numServers == 0) no server/UE data endpoint is
    // selected, so the endpoint bounds check is skipped entirely.
    if (numServers > 0 && (serverId >= numServers || ueId >= numUes))
    {
        std::cerr << "Error: selected endpoint is outside loaded subset: serverId=" << serverId
                  << "/" << numServers << ", ueId=" << ueId << "/" << numUes << std::endl;
        return 1;
    }
    if (enableDynamicGroundNodes && initialUes > numUes)
    {
        std::cerr << "Error: initialUes=" << initialUes << " exceeds loaded UEs=" << numUes
                  << std::endl;
        return 1;
    }

    std::cout << "Scenario: satellites=" << numSatellites << ", servers=" << numServers
              << ", UEs=" << numUes << ", accessMaxDistance=" << accessMaxDistance
              << "m, delayMode=" << delayMode << ", accessDelay=" << accessDelay
              << ", delayUpdateInterval=" << delayUpdateInterval
              << "s, propagationSpeed=" << propagationSpeed
              << "m/s, islMaxDistance=" << islMaxDistance
              << "m, islDelay=" << islDelay << std::endl;
    if (!satelliteSelectionFile.empty())
    {
        std::cout << "Selected original satellite IDs:";
        for (uint32_t satelliteId : selectedSatelliteIds)
        {
            std::cout << ' ' << satelliteId;
        }
        std::cout << std::endl;
    }

    NodeContainer satelliteNodes;
    NodeContainer serverNodes;
    NodeContainer ueNodes;
    satelliteNodes.Create(numSatellites);
    serverNodes.Create(numServers);
    ueNodes.Create(numUes);
    timer.Log("node creation");

    std::vector<double> ueActivationTimes(numUes, 0.0);
    std::map<uint32_t, Time> groundNodeActivationTimes;
    if (enableDynamicGroundNodes)
    {
        uint32_t outsideSimulation = 0;
        for (uint32_t i = 0; i < numUes; ++i)
        {
            double activationTime = 0.0;
            if (i >= initialUes)
            {
                const uint32_t batch = (i - initialUes) / uesPerIntroduction;
                activationTime = ueIntroductionStart +
                                 static_cast<double>(batch) * ueIntroductionInterval;
            }
            ueActivationTimes[i] = activationTime;
            groundNodeActivationTimes[ueNodes.Get(i)->GetId()] = Seconds(activationTime);
            outsideSimulation += activationTime > simTime ? 1 : 0;
        }

        std::cout << "Dynamic UE introduction: initial=" << initialUes
                  << ", firstBatch=" << ueIntroductionStart << "s, interval="
                  << ueIntroductionInterval << "s, batchSize=" << uesPerIntroduction
                  << std::endl;
        if (outsideSimulation > 0)
        {
            std::cout << "Warning: " << outsideSimulation
                      << " UEs have introduction times after simTime and will remain dormant"
                      << std::endl;
        }
    }

    timer.Begin("mobility installation");
    LeoSimMobilityHelper mobilityHelper;
    mobilityHelper.SetLoader(loader);
    mobilityHelper.SetVerbose(verbose);
    mobilityHelper.SetVelocityCalculation(true);
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        const uint32_t satelliteId = selectedSatelliteIds[i];
        mobilityHelper.InstallSatellite(satelliteNodes.Get(i),
                                        satelliteId,
                                        loader->GetSatelliteName(satelliteId));
    }
    for (uint32_t i = 0; i < numServers; ++i)
    {
        uint32_t id = serverDeviceIds[i];
        mobilityHelper.InstallGateway(serverNodes.Get(i),
                                      id,
                                      loader->GetGroundDeviceName(id),
                                      loader->GetGroundDevicePosition(id));
    }
    for (uint32_t i = 0; i < numUes; ++i)
    {
        uint32_t id = ueDeviceIds[i];
        mobilityHelper.InstallUE(ueNodes.Get(i),
                                 id,
                                 loader->GetGroundDeviceName(id),
                                 loader->GetGroundDevicePosition(id));
    }
    mobilityHelper.StartAll();
    timer.Log("mobility installation and startup");

    timer.Begin("operator model construction");
    loader->LoadSatelliteOperatorsFromDataDirectory(leosimDataDir);
    LeoSimOperatorHelper opHelper;
    opHelper.SetLoader(loader);
    opHelper.SetVerbose(verbose);
    opHelper.RegisterSatellites(satelliteNodes);
    opHelper.RegisterGroundDevices(ueNodes, serverNodes);
    Ptr<LeoSimOperatorModel> operatorModel = opHelper.Build();
    timer.Log("operator model construction");

    LeoSimVisualizationHelper visualizationHelper;
    visualizationHelper.SetGroundNodeContainers(serverNodes, ueNodes);
    const std::string positionFile = outputPrefix + "-positions.csv";
    const std::string linkFile = outputPrefix + "-links.csv";
    const std::string packetFile = outputPrefix + "-packets.csv";
    const std::string handoverFile = outputPrefix + "-handovers.csv";
    const std::string choCandidateFile = outputPrefix + "-cho-candidates.csv";
    if (enableVisualization)
    {
        visualizationHelper.SetOutputFile(positionFile);
        visualizationHelper.EnablePositionGeolocationLogging(true);
        visualizationHelper.SetUnifiedLinkStateFile(linkFile);
        visualizationHelper.SetPacketFile(packetFile);
        visualizationHelper.EnablePacketLogging(true);
        visualizationHelper.EnablePacketGeolocationLogging(true);
        visualizationHelper.SetLoaderHelper(loaderHelper);
        if (enableHandoverLogging)
        {
            visualizationHelper.SetBeamFile("");
            visualizationHelper.SetChoFile(choCandidateFile);
            visualizationHelper.SetHandoverFile(handoverFile);
            visualizationHelper.EnableBeamLogging(true);
        }
        visualizationHelper.Initialize();
        timer.Log("visualization initialization");
    }
    else if (enableHandoverLogging)
    {
        // Initialize only the event stream; do not create the other visualization CSVs.
        visualizationHelper.SetBeamFile("");
        visualizationHelper.SetChoFile(choCandidateFile);
        visualizationHelper.SetHandoverFile(handoverFile);
        visualizationHelper.EnableBeamLogging(true);
        visualizationHelper.InitializeBeamLogging();
        timer.Log("handover logging initialization");
    }

    NodeContainer allGroundNodes;
    allGroundNodes.Add(ueNodes);
    allGroundNodes.Add(serverNodes);

    LeoSimChannelHelper accessChannelHelper;
    timer.Begin("access channel creation");
    accessChannelHelper.SetMinElevationAngle(minElevation);
    accessChannelHelper.SetMaxLinkDistance(accessMaxDistance);
    accessChannelHelper.SetMaxGroundLinksPerNode(maxAccessSatellites);
    accessChannelHelper.SetGroundAccessPlanningWindow(Seconds(simTime),
                                                      Seconds(accessCandidateSampleInterval));
    accessChannelHelper.SetUpdateInterval(Seconds(1.0));
    accessChannelHelper.SetVerbose(verbose);
    Ptr<LeoSimChannelModel> accessChannel =
        accessChannelHelper.CreateChannels(satelliteNodes, allGroundNodes);
    accessChannel->SetOperatorModel(operatorModel);
    timer.Log("access channel creation");

    Ptr<LeoSimChannelModel> islChannel;
    if (enableIsl)
    {
        timer.Begin("ISL channel creation");
        LeoSimChannelHelper islChannelHelper;
        islChannelHelper.SetIslFrequency(islFrequency);
        islChannelHelper.SetIslMaxDistance(islMaxDistance);
        islChannelHelper.SetIslTransmitPower(islTransmitPower);
        islChannelHelper.SetIslAntennaGain(islAntennaGain);
        islChannelHelper.SetUpdateInterval(Seconds(1.0));
        islChannelHelper.SetDynamicIslSelectionInterval(Seconds(routingUpdateInterval));
        islChannelHelper.SetVerbose(verbose);
        islChannel = islFullMesh
                         ? islChannelHelper.CreateIslMesh(satelliteNodes)
                         : islChannelHelper.CreateIslNearestNeighborMesh(satelliteNodes,
                                                                         maxIslNeighbors);
        islChannel->SetOperatorModel(operatorModel);
        timer.Log("ISL channel creation");
    }

    if (enableVisualization)
    {
        visualizationHelper.SetChannelModel(accessChannel);
        visualizationHelper.SetIslChannelModel(islChannel);
        visualizationHelper.SchedulePositionLogging(satelliteNodes,
                                                    serverNodes,
                                                    ueNodes,
                                                    visualizationInterval,
                                                    simTime);
    }
    if (enableHandoverLogging)
    {
        std::cout << "Handover data: " << handoverFile << ", " << choCandidateFile << std::endl;
    }

    LeoSimDeviceInstaller accessInstaller;
    timer.Begin("access device installation");
    accessInstaller.SetChannelModel(accessChannel);
    accessInstaller.SetOperatorModel(operatorModel);
    accessInstaller.SetDeviceDataRate(accessDataRate);
    accessInstaller.SetDeviceDelay(accessDelay);
    accessInstaller.SetDelayMode(delayMode);
    accessInstaller.SetDelayUpdateInterval(Seconds(delayUpdateInterval));
    accessInstaller.SetPropagationSpeed(propagationSpeed);
    accessInstaller.SetDeviceMtu(1500);
    accessInstaller.SetVerbose(verbose);
    NetDeviceContainer accessDevices = accessInstaller.Install(satelliteNodes, allGroundNodes);
    accessInstaller.ApplySharingRates(accessDevices);
    accessInstaller.EnableLinkStateCallbacks(accessChannel);
    timer.Log("access device installation");

    NetDeviceContainer islDevices;
    Ptr<LeoSimIslLoadModel> syntheticIslLoadModel;
    LeoSimDeviceInstaller islInstaller;
    if (enableIsl && islChannel)
    {
        timer.Begin("ISL device installation");
        islInstaller.SetChannelModel(islChannel);
        islInstaller.SetOperatorModel(operatorModel);
        islInstaller.SetDeviceDataRate(islDataRate);
        islInstaller.SetDeviceDelay(islDelay);
        islInstaller.SetDelayMode(delayMode);
        islInstaller.SetDelayUpdateInterval(Seconds(delayUpdateInterval));
        islInstaller.SetPropagationSpeed(propagationSpeed);
        islInstaller.SetDeviceMtu(1500);
        islInstaller.SetVerbose(verbose);
        islDevices = islInstaller.Install(satelliteNodes, NodeContainer());
        islInstaller.ApplySharingRates(islDevices);
        islInstaller.EnableLinkStateCallbacks(islChannel);

        if (enableSyntheticIslLoad)
        {
            syntheticIslLoadModel = CreateObject<LeoSimIslLoadModel>();
            syntheticIslLoadModel->SetSeed(syntheticLoadSeed);
            syntheticIslLoadModel->SetLoadRange(syntheticLoadMin, syntheticLoadMax);
            syntheticIslLoadModel->SetDistribution(syntheticLoadDistribution);

            const uint64_t satelliteCapacityBps = DataRate(satelliteIslCapacity).GetBitRate();
            const uint64_t linkCapacityBps = DataRate(islDataRate).GetBitRate();
            std::map<uint32_t, uint32_t> nodeIdToSatelliteId;
            for (uint32_t i = 0; i < satelliteNodes.GetN(); ++i)
            {
                const uint32_t nodeId = satelliteNodes.Get(i)->GetId();
                nodeIdToSatelliteId[nodeId] = nodeId;
                syntheticIslLoadModel->RegisterSatellite(nodeId, satelliteCapacityBps);
            }

            for (uint32_t i = 0; i + 1 < islDevices.GetN(); i += 2)
            {
                Ptr<Node> nodeA = islDevices.Get(i)->GetNode();
                Ptr<Node> nodeB = islDevices.Get(i + 1)->GetNode();
                if (!nodeA || !nodeB)
                {
                    continue;
                }
                const auto satA = nodeIdToSatelliteId.find(nodeA->GetId());
                const auto satB = nodeIdToSatelliteId.find(nodeB->GetId());
                if (satA == nodeIdToSatelliteId.end() || satB == nodeIdToSatelliteId.end())
                {
                    continue;
                }
                syntheticIslLoadModel->RegisterDirectedIsl(satA->second,
                                                            satB->second,
                                                            linkCapacityBps);
                syntheticIslLoadModel->RegisterDirectedIsl(satB->second,
                                                            satA->second,
                                                            linkCapacityBps);
            }
            syntheticIslLoadModel->AttachChannelModel(islChannel, islDevices);
            syntheticIslLoadModel->Generate(0);
            syntheticIslLoadModel->ApplyToIslDevices(islDevices);
            std::cout << "Generated deterministic synthetic load for "
                      << syntheticIslLoadModel->GetAllIslLoads().size()
                      << " directed ISLs (seed=" << syntheticLoadSeed << ")" << std::endl;
            if (enableIslLoadLogging)
            {
                const std::string syntheticLoadCsvFile = outputPrefix + "-isl-load.csv";
                syntheticIslLoadModel->EnableCsvOutput(syntheticLoadCsvFile);
                std::cout << "Synthetic ISL load data: " << syntheticLoadCsvFile << std::endl;
            }
        }
        timer.Log("ISL device installation");
    }

    timer.Begin("internet stack installation");
    InternetStackHelper stack;
    stack.Install(satelliteNodes);
    stack.Install(serverNodes);
    stack.Install(ueNodes);
    timer.Log("internet stack installation");

    timer.Begin("IPv4 address assignment");
    Ipv4AddressHelper accessIpv4;
    accessIpv4.SetBase(Ipv4Address("10.0.0.0"), Ipv4Mask("255.255.255.252"));
    AssignPerLinkSubnets(accessDevices, accessIpv4);

    if (enableIsl && islDevices.GetN() > 0)
    {
        Ipv4AddressHelper islIpv4;
        islIpv4.SetBase(Ipv4Address("10.128.0.0"), Ipv4Mask("255.255.255.252"));
        AssignPerLinkSubnets(islDevices, islIpv4);
    }
    timer.Log("IPv4 address assignment");

    // Destination-tree routing installs its initial static host routes
    // synchronously during routing setup below. Populating ns-3 global routes
    // here performs an expensive all-pairs calculation that is immediately
    // superseded by those routes, which is especially costly for 11k nodes.
    // Retain global routing only as the fallback for the legacy non-tree path.
    const bool useTrees = useRouteTreeCache;
    if (!useTrees)
    {
        timer.Begin("global routing table population");
        Ipv4GlobalRoutingHelper::PopulateRoutingTables();
        timer.Log("global routing table population");
    }
    else
    {
        std::cout << "[routing] Skipping redundant ns-3 global route population; "
                     "destination trees will install initial routes"
                  << std::endl;
    }

    NodeContainer allNodes;
    allNodes.Add(satelliteNodes);
    allNodes.Add(serverNodes);
    allNodes.Add(ueNodes);

    std::vector<std::pair<uint32_t, uint32_t>> uePingPairIndices;
    if (enableUePing)
    {
        if (uePingStartIndex >= ueNodes.GetN())
        {
            NS_FATAL_ERROR("uePingStartIndex " << uePingStartIndex
                                                << " is outside the loaded UE subset");
        }
        const uint32_t pairCount =
            std::min(uePingPairs, (ueNodes.GetN() - uePingStartIndex) / 2);
        uePingPairIndices.reserve(pairCount);
        for (uint32_t pair = 0; pair < pairCount; ++pair)
        {
            const uint32_t source = uePingStartIndex + pair * 2;
            uePingPairIndices.emplace_back(source, source + 1);
        }
        if (pairCount == 0)
        {
            NS_FATAL_ERROR("UE ping requires at least two loaded UEs");
        }
    }

    NodeContainer routingDestinations;
    NodeContainer routingSources;
    NodeContainer statisticsRouteSources;
    NodeContainer statisticsRouteDestinations;
    // Satellites need forwarding entries for endpoint destinations, but
    // unrelated ground nodes do not need routes in single-flow mode.
    routingSources.Add(satelliteNodes);
    if (allToAllTraffic)
    {
        routingDestinations.Add(serverNodes);
        routingDestinations.Add(ueNodes);
        routingSources.Add(serverNodes);
        routingSources.Add(ueNodes);
        statisticsRouteSources.Add(ueNodes);
        statisticsRouteDestinations.Add(serverNodes);
    }
    else
    {
        // TCP data and acknowledgements require both selected endpoints, but
        // unrelated ground nodes do not need trees or host routes. With ground
        // stations disabled there is no server endpoint; the selected UE is the
        // only application endpoint (satellite pings traverse the serving access
        // link directly and need no ISL destination trees).
        if (numServers > 0)
        {
            routingDestinations.Add(serverNodes.Get(serverId));
            routingSources.Add(serverNodes.Get(serverId));
            statisticsRouteDestinations.Add(serverNodes.Get(serverId));
        }
        routingDestinations.Add(ueNodes.Get(ueId));
        routingSources.Add(ueNodes.Get(ueId));
        statisticsRouteSources.Add(ueNodes.Get(ueId));
    }
    if (enableUePing && !allToAllTraffic)
    {
        std::set<uint32_t> addedNodeIds;
        if (numServers > 0)
        {
            addedNodeIds.insert(serverNodes.Get(serverId)->GetId());
        }
        addedNodeIds.insert(ueNodes.Get(ueId)->GetId());
        for (const auto& [sourceIndex, destinationIndex] : uePingPairIndices)
        {
            for (uint32_t ueIndex : {sourceIndex, destinationIndex})
            {
                Ptr<Node> endpoint = ueNodes.Get(ueIndex);
                if (addedNodeIds.insert(endpoint->GetId()).second)
                {
                    routingDestinations.Add(endpoint);
                    routingSources.Add(endpoint);
                }
            }
        }
    }

    LeoSimRoutingCalculatorHelper routingHelper;
    const std::string routeLogFile = outputPrefix + "-routes.csv";
    if (enableRouteLogging && !useTrees)
    {
        routingHelper.EnableRouteLogging(routeLogFile);
    }
    Ptr<LeoSimRoutingCalculator> routingCalculator =
        routingHelper.CreateUnifiedRoutingCalculator(accessChannel, islChannel, verbose);
    routingCalculator->SetOperatorModel(operatorModel);
    routingCalculator->SetIslLoadModel(syntheticIslLoadModel);
    routingCalculator->SetCombinedMetricWeights(combinedLoadWeight,
                                                combinedDistanceWeight,
                                                combinedLifetimeWeight,
                                                combinedSnrWeight,
                                                combinedHopWeight);
    routingCalculator->SetCombinedMetricBounds(combinedMaxDistance,
                                               combinedTargetLifetime,
                                               combinedMinLifetime,
                                               combinedMinSnr,
                                               combinedGoodSnr,
                                               combinedCriticalUtilization);
    if (!allToAllTraffic)
    {
        std::set<uint32_t> routableIds;
        if (numServers > 0)
        {
            routableIds.insert(serverNodes.Get(serverId)->GetId());
        }
        routableIds.insert(ueNodes.Get(ueId)->GetId());
        for (const auto& [sourceIndex, destinationIndex] : uePingPairIndices)
        {
            routableIds.insert(ueNodes.Get(sourceIndex)->GetId());
            routableIds.insert(ueNodes.Get(destinationIndex)->GetId());
        }
        routingCalculator->SetRoutableGroundNodes(routableIds);
    }
    timer.Log("routing calculator construction");

    // === Beam Management & Handover (3GPP NTN CHO) ===
    LeoSimBeamManagerHelper beamHelper;
    beamHelper.SetVerbose(verbose);
    beamHelper.SetChannelModel(accessChannel);
    if (enableIsl && islChannel)
    {
        beamHelper.SetIslChannelModel(islChannel);
    }
    beamHelper.SetRoutingCalculator(routingCalculator);
    beamHelper.SetLoader(loader);
    beamHelper.SetHandoverMode(hoModeName == "BHO" ? LEOSIM_HO_MODE_BHO : LEOSIM_HO_MODE_CHO);
    beamHelper.SetTtt(Seconds(ttt));
    beamHelper.SetT310(Seconds(t310));
    beamHelper.SetN310(n310);
    beamHelper.SetN311(n311);
    beamHelper.SetA3Offset(a3Offset);
    beamHelper.SetA4Threshold(a4Threshold);
    beamHelper.SetTteThreshold(Seconds(tteTrigger));
    beamHelper.SetSinrThreshold(-10.0);
    beamHelper.SetChoPreparationDelay(MilliSeconds(choPrepMs));
    beamHelper.SetChoExecutionDelay(MilliSeconds(choExecMs));
    beamHelper.SetTopsisWeights(0.30, 0.25, 0.20, 0.15, 0.10, 0.05, 0.05);
    beamHelper.SetMaxCandidates(maxCandidates);
    beamHelper.SetUpdateInterval(MilliSeconds(beamUpdateIntervalMs));
    if (enableDynamicGroundNodes)
    {
        beamHelper.SetGroundNodeActivationTimes(groundNodeActivationTimes);
        if (enableGroundNodeLifecycleLogging)
        {
            const std::string lifecycleFile = outputPrefix + "-ground-node-lifecycle.csv";
            beamHelper.EnableGroundNodeLifecycleLogging(lifecycleFile);
            std::cout << "Ground-node lifecycle data: " << lifecycleFile << std::endl;
        }
    }
    beamHelper.EnableLoadBalancing(enableLoadBalancing);
    beamHelper.EnableHandoverBuffering(enableHoBuffering);
    beamHelper.SetMaxHandoverBufferSize(hoBufferSize);

    // The beam manager's visibility scan requires a populated multi-beam model.
    // Without it no serving access link can be selected, so the routing calculator
    // removes every ground edge and all metric runs silently fall back to global routing.
    Ptr<LeoSimMultiBeamModel> multiBeamModel = CreateObject<LeoSimMultiBeamModel>();
    for (uint32_t i = 0; i < satelliteNodes.GetN(); ++i)
    {
        Ptr<Node> satellite = satelliteNodes.Get(i);
        Ptr<MobilityModel> mobility = satellite->GetObject<MobilityModel>();
        if (!mobility)
        {
            continue;
        }
        const uint32_t satelliteId = satellite->GetId();
        multiBeamModel->SetBeamsForSatellite(
            satelliteId,
            LeoSimBeamLayoutEngine::GenerateHexLayout(satelliteId,
                                                       mobility->GetPosition(),
                                                       std::max<uint32_t>(1, beamNumRings),
                                                       beamRadiusKm,
                                                       std::max<uint32_t>(1, beamReuseColors),
                                                       satelliteId * 1000));
    }
    multiBeamModel->UpdateGeometry(satelliteNodes, Simulator::Now());
    beamHelper.SetMultiBeamModel(multiBeamModel);

    timer.Begin("beam manager installation");
    Ptr<LeoSimBeamManager> beamManager =
        beamHelper.Install(allGroundNodes, satelliteNodes, Seconds(simTime));
    beamManager->SetOperatorModel(operatorModel);
    // Handover success should validate the paths used by installed data flows.
    // With ping-only traffic to the serving satellite there is no remote ground
    // peer, so leaving this map empty deliberately selects the beam manager's
    // access-link validation path.
    if (enableDataTraffic && allToAllTraffic)
    {
        for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
        {
            beamManager->SetHandoverValidationPeers(ueNodes.Get(i)->GetId(), serverNodes);
        }
        for (uint32_t i = 0; i < serverNodes.GetN(); ++i)
        {
            beamManager->SetHandoverValidationPeers(serverNodes.Get(i)->GetId(), ueNodes);
        }
    }
    else if (enableDataTraffic && numServers > 0)
    {
        NodeContainer selectedServer;
        selectedServer.Add(serverNodes.Get(serverId));
        NodeContainer selectedUe;
        selectedUe.Add(ueNodes.Get(ueId));
        beamManager->SetHandoverValidationPeers(ueNodes.Get(ueId)->GetId(), selectedServer);
        beamManager->SetHandoverValidationPeers(serverNodes.Get(serverId)->GetId(), selectedUe);
    }
    routingCalculator->SetBeamManager(beamManager);
    if (enableVisualization || enableHandoverLogging)
    {
        visualizationHelper.SetBeamManager(beamManager);
    }
    timer.Log("beam manager installation");

    // Optional: every UE in the configured index range continuously pings its
    // currently connected satellite, retargeted on handover.
    // Declared here (outside the gate) so the beam-state callback and the
    // scheduled initial pings stay valid for the whole Simulator::Run below.
    const std::string satellitePingCsvFile = outputPrefix + "-satellite-ping.csv";
    const std::string satellitePingTimeSeriesCsvFile =
        outputPrefix + "-satellite-ping-timeseries.csv";
    SatellitePinger satellitePinger(beamManager,
                                    &accessInstaller,
                                    ueNodes,
                                    satelliteNodes,
                                    satellitePingInterval,
                                    satellitePingPacketSize,
                                    appStop,
                                    satellitePingLoadRamp,
                                    satellitePingMaxRate,
                                    satellitePingRampInterval);
    if (enableSatellitePing)
    {
        if (satellitePingStartIndex >= numUes)
        {
            NS_FATAL_ERROR("satellitePingStartIndex " << satellitePingStartIndex
                                                      << " is outside the loaded UE subset ("
                                                      << numUes << ")");
        }
        beamManager->SetBeamStateCallback(
            MakeCallback(&SatellitePinger::OnBeamState, &satellitePinger));
        satellitePinger.ScheduleInitialPings(ueNodes,
                                             satellitePingStartIndex,
                                             appStart,
                                             appStop,
                                             satellitePingStaggerWindow,
                                             ueActivationTimes);
    }
    timer.Log("satellite ping setup");

    Ptr<LeoSimBeamCapacityManager> beamCapacityManager;
    if (enableSharedBeamCapacity)
    {
        timer.Begin("beam capacity manager setup");
        beamCapacityManager = CreateObject<LeoSimBeamCapacityManager>();
        beamCapacityManager->SetBeamManager(beamManager);
        beamCapacityManager->SetUplinkBeamCapacity(DataRate(beamUplinkCapacity));
        beamCapacityManager->SetDownlinkBeamCapacity(DataRate(beamDownlinkCapacity));
        beamCapacityManager->SetSatelliteUplinkCapacity(DataRate(satelliteUplinkCapacity));
        beamCapacityManager->SetSatelliteDownlinkCapacity(DataRate(satelliteDownlinkCapacity));
        if (!beamCapacity.empty())
        {
            beamCapacityManager->SetBeamCapacity(DataRate(beamCapacity));
        }
        beamCapacityManager->SetDefaultUplinkPackageRate(DataRate(uePackageUplinkRate));
        beamCapacityManager->SetDefaultDownlinkPackageRate(DataRate(uePackageDownlinkRate));
        beamCapacityManager->SetScheduler(
            beamScheduler == "equal"
                ? LEOSIM_BEAM_SCHEDULER_EQUAL
                : (beamScheduler == "alpha-fair" ? LEOSIM_BEAM_SCHEDULER_ALPHA_FAIR
                                                  : LEOSIM_BEAM_SCHEDULER_PROPORTIONAL_FAIR));
        beamCapacityManager->SetAlphaFairness(alphaFairness);
        beamCapacityManager->SetQueueDelayWeight(queuePressureWeight);
        beamCapacityManager->SetDemandAware(beamDemandAware);
        beamCapacityManager->SetActiveUserTimeout(MilliSeconds(beamActiveUserTimeoutMs));
        beamCapacityManager->SetUpdateInterval(MilliSeconds(beamCapacityUpdateIntervalMs));
        if (!ueServiceProfileFile.empty() &&
            !beamCapacityManager->LoadUeServiceProfiles(ueServiceProfileFile))
        {
            NS_FATAL_ERROR("Unable to parse UE service profile file: " << ueServiceProfileFile);
        }

        std::set<uint32_t> groundNodeIds;
        for (uint32_t i = 0; i < allGroundNodes.GetN(); ++i)
        {
            groundNodeIds.insert(allGroundNodes.Get(i)->GetId());
        }
        for (uint32_t i = 0; i + 1 < accessDevices.GetN(); i += 2)
        {
            Ptr<NetDevice> firstDevice = accessDevices.Get(i);
            Ptr<NetDevice> secondDevice = accessDevices.Get(i + 1);
            Ptr<Node> firstNode = firstDevice->GetNode();
            Ptr<Node> secondNode = secondDevice->GetNode();
            if (!firstNode || !secondNode)
            {
                continue;
            }
            if (groundNodeIds.count(firstNode->GetId()) != 0)
            {
                beamCapacityManager->RegisterAccessLink(firstNode,
                                                        secondNode,
                                                        firstDevice,
                                                        secondDevice);
            }
            else if (groundNodeIds.count(secondNode->GetId()) != 0)
            {
                beamCapacityManager->RegisterAccessLink(secondNode,
                                                        firstNode,
                                                        secondDevice,
                                                        firstDevice);
            }
        }
        if (enableBeamCapacityLogging)
        {
            const std::string beamCapacityFile = outputPrefix + "-beam-capacity.csv";
            beamCapacityManager->SetCsvOutputInterval(MilliSeconds(beamCapacityLogIntervalMs));
            beamCapacityManager->EnableCsvOutput(beamCapacityFile);
            std::cout << "Beam capacity data: " << beamCapacityFile << std::endl;
        }
        beamCapacityManager->Start();
        timer.Log("beam capacity manager setup");
    }

    timer.Begin("routing setup");
    // With few application endpoints and thousands of satellite transit nodes,
    // a reverse tree per destination avoids running Dijkstra independently for
    // every (satellite, destination) pair. The external engine returns the
    // cached next hop for every graph node, so forwarding tables remain complete.
    LeoSimExternalRoutingHelper routeTreeHelper;

    if (useTrees)
    {
        routeTreeHelper.SetEnginePath(routeTreeEngine);
        routeTreeHelper.SetWorkingDirectory(routeTreeWorkDir);
        routeTreeHelper.SetWorkerCount(std::max<uint32_t>(1, routeTreeWorkers));
        routeTreeHelper.SetMode(
            LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_DESTINATION_TREE);
        LeoSimExternalRoutingHelper::ExternalRoutingMetric treeMetric =
            LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_HOP_COUNT;
        switch (routingMetric)
        {
        case LeoSimRoutingCalculator::LEOSIM_METRIC_DISTANCE:
            treeMetric = LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_DISTANCE;
            break;
        case LeoSimRoutingCalculator::LEOSIM_METRIC_PATH_LOSS:
            treeMetric = LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_PATH_LOSS;
            break;
        case LeoSimRoutingCalculator::LEOSIM_METRIC_SNR:
            treeMetric = LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_SNR;
            break;
        case LeoSimRoutingCalculator::LEOSIM_METRIC_SIGNAL_STRENGTH:
            treeMetric =
                LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_SIGNAL_STRENGTH;
            break;
        case LeoSimRoutingCalculator::LEOSIM_METRIC_REMAINING_LIFETIME:
            treeMetric =
                LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_REMAINING_LIFETIME;
            break;
        case LeoSimRoutingCalculator::LEOSIM_METRIC_LOAD:
            treeMetric = LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_LOAD;
            break;
        case LeoSimRoutingCalculator::LEOSIM_METRIC_COMBINED:
            treeMetric = LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_COMBINED;
            break;
        case LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT:
            break;
        }
        routeTreeHelper.SetMetric(treeMetric);
        routeTreeHelper.SetMaxRouteRequests(routeTreeMaxEntries);
        // routingSources already contains every satellite required for
        // forwarding plus only the application endpoints. Requesting the
        // entire graph here would also calculate and install routes for UEs
        // that never generate application traffic.
        routeTreeHelper.SetDestinationTreeAllNodes(false);
        routeTreeHelper.SetStatisticsEndpoints(statisticsRouteSources,
                                               statisticsRouteDestinations);
        if (enableRouteLogging)
        {
            routeTreeHelper.EnableRouteLogging(routeLogFile);
        }

        if (enableDynamicRouting)
        {
            routeTreeHelper.EnableDynamicRouting(routingCalculator,
                                                 routingSources,
                                                 routingDestinations,
                                                 Seconds(routingUpdateInterval),
                                                 simTime,
                                                 verbose);
        }
        else
        {
            routeTreeHelper.SetStaticRoutes(routingCalculator,
                                            routingSources,
                                            routingDestinations,
                                            verbose);
        }
        // Beam management releases access notifications after its active
        // handover batch completes. Zero delay coalesces same-time callbacks
        // without relying on an empirical quiet period.
        routeTreeHelper.EnableReactiveRouteRefresh(routingCalculator,
                                                   routingSources,
                                                   routingDestinations,
                                                   MilliSeconds(0),
                                                   verbose);
        if (enableIsl && islChannel)
        {
            islChannel->SetDynamicIslTopologyChangeCallback(
                MakeCallback(&LeoSimExternalRoutingHelper::RequestRouteRefresh,
                             &routeTreeHelper));
        }
        beamManager->SetAccessStateChangeCallback(
            MakeCallback(&LeoSimExternalRoutingHelper::RequestRouteRefresh,
                         &routeTreeHelper));
        std::cout << "[routing] Destination-tree cache enabled: "
                  << routingDestinations.GetN() << " trees for "
                  << routingSources.GetN() << " route sources ("
                  << allNodes.GetN() << " graph nodes)" << std::endl;
    }
    else if (enableDynamicRouting)
    {
        routingHelper.EnableDynamicRouting(routingCalculator,
                                           routingSources,
                                           routingDestinations,
                                           Seconds(routingUpdateInterval),
                                           simTime,
                                           verbose,
                                           routingMetric);
    }
    else
    {
        routingHelper.SetStaticRoutes(routingCalculator,
                                      routingSources,
                                      routingDestinations,
                                      verbose,
                                      routingMetric);
    }

    if (!useTrees)
    {
        // Reactive routing: update routes when links change or handover occurs
        // Match the destination-tree event coalescing above for in-process routing.
        routingHelper.EnableReactiveLinkTriggeredRouting(
            routingCalculator,
            routingSources,
            routingDestinations,
            accessChannel,
            enableIsl ? islChannel : nullptr,
            MilliSeconds(0),
            verbose,
            routingMetric);
        beamManager->SetAccessStateChangeCallback(
            MakeCallback(&LeoSimRoutingCalculatorHelper::RequestRouteRefresh,
                         &routingHelper));
    }
    timer.Log("routing setup");

    timer.Begin("traffic application installation");
    ApplicationContainer trafficApps;
    if (!enableDataTraffic)
    {
        std::cout << "TCP/UDP data traffic disabled; only explicitly enabled ping applications "
                     "will run"
                  << std::endl;
    }
    else if (allToAllTraffic)
    {
        trafficApps = InstallAllTcpFlows(ueNodes,
                                         serverNodes,
                                         beamManager,
                                         port,
                                         appStart,
                                         appStop,
                                         tcpRate,
                                         tcpPacketSize,
                                         ueActivationTimes);
    }
    else
    {
        if (numServers == 0)
        {
            NS_FATAL_ERROR("Single-flow UDP data traffic requires a ground station (server); "
                           "enable ground stations or disable data traffic");
        }
        const double selectedUeStart =
            std::max(appStart, ueActivationTimes.at(ueId));
        if (selectedUeStart >= appStop)
        {
            NS_FATAL_ERROR("Selected UE activates at "
                           << selectedUeStart << "s, which is not before appStop=" << appStop
                           << "s");
        }
        trafficApps = InstallBidirectionalUdpFlows(ueNodes,
                                                   serverNodes,
                                                   ueId,
                                                   serverId,
                                                   port,
                                                   selectedUeStart,
                                                   appStop,
                                                   udpRate,
                                                   udpPacketSize);
    }
    ApplicationContainer uePingApps;
    if (enableUePing)
    {
        uePingApps = InstallBoundedUePings(ueNodes,
                                           uePingPairIndices,
                                           ueActivationTimes,
                                           appStart,
                                           appStop,
                                           uePingInterval,
                                           uePingPacketSize,
                                           uePingStaggerWindow);
    }
    timer.Log("traffic application installation");

    timer.Begin("FlowMonitor installation");
    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    timer.Log("FlowMonitor installation");

    Ptr<LeoSimStatisticsHelper> statistics;
    const std::string statisticsCsvFile = outputPrefix + "-statistics.csv";
    const std::string statisticsJsonFile = outputPrefix + "-statistics.json";
    if (enableStatistics)
    {
        statistics = CreateObject<LeoSimStatisticsHelper>();
        statistics->SetFlowMonitor(flowMonitor, classifier);
        statistics->SetBeamManager(beamManager);
        statistics->SetIslLoadModel(syntheticIslLoadModel);
        statistics->SetApplicationMeasurementWindow(Seconds(appStart), Seconds(appStop));
        for (uint32_t i = 0; i < trafficApps.GetN(); ++i)
        {
            Ptr<PacketSink> sink = DynamicCast<PacketSink>(trafficApps.Get(i));
            if (sink)
            {
                statistics->AddApplicationSink(sink);
            }
        }
        if (!statistics->AttachChannelModel(accessChannel))
        {
            std::cerr << "Warning: failed to attach statistics to access channel traces"
                      << std::endl;
        }
        if (enableIsl && islChannel && !statistics->AttachChannelModel(islChannel))
        {
            std::cerr << "Warning: failed to attach statistics to ISL channel traces" << std::endl;
        }
        statistics->StartPeriodicSampling(Seconds(statisticsInterval), statisticsCsvFile);
        timer.Log("statistics setup");
    }

    if (enableVisualization)
    {
        visualizationHelper.InstallPacketLogging(satelliteNodes, serverNodes, ueNodes);
    }

    SimulationProgress simulationProgress{std::chrono::steady_clock::now(),
                                          enableTaskProfiler};
    if (progressLogInterval > 0.0 && progressLogInterval <= simTime)
    {
        Simulator::Schedule(Seconds(progressLogInterval),
                            &LogSimulationProgress,
                            Seconds(progressLogInterval),
                            Seconds(simTime),
                            &simulationProgress);
    }
    timer.Begin("Simulator::Run");
    if (enableTaskProfiler)
    {
        LeoSimTaskProfiler::Reset();
        LeoSimTaskProfiler::EnableAggregation();
    }
    const auto simulatorWallStart = std::chrono::steady_clock::now();
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    const double simulatorWallSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - simulatorWallStart)
            .count();
    if (enableTaskProfiler)
    {
        LeoSimTaskProfiler::EnableAggregation(false);
    }
    timer.Log("Simulator::Run");
    if (enableTaskProfiler)
    {
        LeoSimTaskProfiler::PrintSummary(simulatorWallSeconds);
    }

    timer.Begin("result post-processing");
    PrintFlowMonitorSummary(flowMonitor, classifier);
    if (writeFlowMonitor)
    {
        flowMonitor->SerializeToXmlFile(outputPrefix + "-flowmon.xml", true, true);
    }
    if (enableStatistics)
    {
        statistics->WriteFinalSample();
        statistics->StopPeriodicSampling();
        if (useTrees)
        {
            statistics->SetRouteStatistics(routeTreeHelper.GetRouteStatistics());
        }
        statistics->WriteSummary(statisticsJsonFile);
        std::cout << "Statistics data: " << statisticsCsvFile << ", " << statisticsJsonFile
                  << std::endl;
    }
    if (enableRouteLogging)
    {
        std::cout << "Route data: " << routeLogFile << std::endl;
    }
    if (enableHandoverLogging)
    {
        std::cout << "Handover data: " << handoverFile << std::endl;
    }
    if (enableSatellitePing)
    {
        satellitePinger.WriteCsvOutputs(satellitePingCsvFile,
                                        satellitePingTimeSeriesCsvFile);
        std::cout << "Satellite ping data: " << satellitePingCsvFile << ", "
                  << satellitePingTimeSeriesCsvFile << std::endl;
    }

    if (enableVisualization || enableHandoverLogging)
    {
        visualizationHelper.Finalize();
    }
    if (enableVisualization)
    {
        std::cout << "Visualization data: " << positionFile << ", " << linkFile << ", "
                  << packetFile << std::endl;
        std::cout << "Render with: python3 contrib/leosim/utils/visualize_3d.py"
                  << " --position_file " << positionFile << " --links " << linkFile
                  << " --packets " << packetFile
                  << " --output " << outputPrefix << "-visualization.html" << std::endl;
    }
    if (beamCapacityManager)
    {
        beamCapacityManager->Stop();
    }
    timer.Log("result post-processing");

    Simulator::Destroy();
    timer.Log("Simulator::Destroy");
    timer.PrintSummary();
    return 0;
}
