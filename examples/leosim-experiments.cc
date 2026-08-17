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
#include <iomanip>
#include <iostream>
#include <map>
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
                   uint32_t packetSize)
{
    ApplicationContainer apps;
    uint32_t flowIndex = 0;
    for (uint32_t ue = 0; ue < ueNodes.GetN(); ++ue)
    {
        for (uint32_t server = 0; server < serverNodes.GetN(); ++server)
        {
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
                                          startTime,
                                          stopTime,
                                          tcpRate,
                                          packetSize));
            ++flowIndex;
        }
    }

    std::cout << "Installed full traffic matrix: " << ueNodes.GetN() << " UEs x "
              << serverNodes.GetN() << " servers = " << flowIndex << " TCP flows" << std::endl;
    return apps;
}

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

    uint32_t numSatellites = 0;
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
    bool enableBeamCapacityLogging = true;
    std::string accessDelay = "1ms";
    std::string delayMode = "geometry";
    double delayUpdateInterval = 1.0;
    double propagationSpeed = 299792458.0;
    uint32_t maxAccessSatellites = 8;

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
    bool allToAllTraffic = true;
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
    cmd.AddValue("appStart", "TCP application start time in seconds", appStart);
    cmd.AddValue("appStop", "TCP application stop time in seconds", appStop);
    cmd.AddValue("tcpRate", "TCP OnOff offered rate, e.g. 1Mbps", tcpRate);
    cmd.AddValue("tcpPacketSize", "TCP application packet size in bytes", tcpPacketSize);
    cmd.AddValue("allToAllTraffic",
                 "Install one TCP flow from every UE to every server/GSS",
                 allToAllTraffic);
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
    if (beamCapacityUpdateIntervalMs <= 0.0 || beamActiveUserTimeoutMs < 0.0 ||
        alphaFairness < 0.0 || queuePressureWeight < 0.0)
    {
        NS_FATAL_ERROR("Invalid beam capacity scheduler timing configuration");
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
    if (loader->GetNumSatellites() == 0 || serverDeviceIds.empty() || ueDeviceIds.empty())
    {
        std::cerr << "Error: need at least 1 satellite, 1 server, and 1 UE." << std::endl;
        return 1;
    }

    if (numSatellites == 0)
    {
        numSatellites = loader->GetNumSatellites();
    }
    else if (numSatellites > loader->GetNumSatellites())
    {
        std::cerr << "Error: requested " << numSatellites << " satellites, but only "
                  << loader->GetNumSatellites() << " were loaded." << std::endl;
        return 1;
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
    if (serverId >= numServers || ueId >= numUes)
    {
        std::cerr << "Error: selected endpoint is outside loaded subset: serverId=" << serverId
                  << "/" << numServers << ", ueId=" << ueId << "/" << numUes << std::endl;
        return 1;
    }

    std::cout << "Scenario: satellites=" << numSatellites << ", servers=" << numServers
              << ", UEs=" << numUes << ", accessMaxDistance=" << accessMaxDistance
              << "m, delayMode=" << delayMode << ", accessDelay=" << accessDelay
              << ", delayUpdateInterval=" << delayUpdateInterval
              << "s, propagationSpeed=" << propagationSpeed
              << "m/s, islMaxDistance=" << islMaxDistance
              << "m, islDelay=" << islDelay << std::endl;

    NodeContainer satelliteNodes;
    NodeContainer serverNodes;
    NodeContainer ueNodes;
    satelliteNodes.Create(numSatellites);
    serverNodes.Create(numServers);
    ueNodes.Create(numUes);
    timer.Log("node creation");

    timer.Begin("mobility installation");
    LeoSimMobilityHelper mobilityHelper;
    mobilityHelper.SetLoader(loader);
    mobilityHelper.SetVerbose(verbose);
    mobilityHelper.SetVelocityCalculation(true);
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        mobilityHelper.InstallSatellite(satelliteNodes.Get(i), i, loader->GetSatelliteName(i));
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

    NodeContainer routingDestinations;
    NodeContainer routingSources;
    NodeContainer statisticsRouteSources;
    NodeContainer statisticsRouteDestinations;
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
        // unrelated ground nodes do not need trees or host routes.
        routingDestinations.Add(serverNodes.Get(serverId));
        routingDestinations.Add(ueNodes.Get(ueId));
        routingSources.Add(serverNodes.Get(serverId));
        routingSources.Add(ueNodes.Get(ueId));
        statisticsRouteSources.Add(ueNodes.Get(ueId));
        statisticsRouteDestinations.Add(serverNodes.Get(serverId));
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
    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        beamManager->SetHandoverValidationPeers(ueNodes.Get(i)->GetId(), serverNodes);
    }
    for (uint32_t i = 0; i < serverNodes.GetN(); ++i)
    {
        beamManager->SetHandoverValidationPeers(serverNodes.Get(i)->GetId(), ueNodes);
    }
    routingCalculator->SetBeamManager(beamManager);
    if (enableVisualization || enableHandoverLogging)
    {
        visualizationHelper.SetBeamManager(beamManager);
    }
    timer.Log("beam manager installation");

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
        routeTreeHelper.SetDestinationTreeAllNodes(true);
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
                  << allNodes.GetN() << " graph nodes" << std::endl;
    }
    else if (enableDynamicRouting)
    {
        routingHelper.EnableDynamicRouting(routingCalculator,
                                           allNodes,
                                           routingDestinations,
                                           Seconds(routingUpdateInterval),
                                           simTime,
                                           verbose,
                                           routingMetric);
    }
    else
    {
        routingHelper.SetStaticRoutes(routingCalculator,
                                      allNodes,
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
            allNodes,
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
    if (allToAllTraffic)
    {
        trafficApps = InstallAllTcpFlows(ueNodes,
                                         serverNodes,
                                         beamManager,
                                         port,
                                         appStart,
                                         appStop,
                                         tcpRate,
                                         tcpPacketSize);
    }
    else
    {
        trafficApps = InstallSingleTcpFlow(ueNodes,
                                           serverNodes,
                                           beamManager,
                                           ueId,
                                           serverId,
                                           port,
                                           appStart,
                                           appStop,
                                           tcpRate,
                                           tcpPacketSize);
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
