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
#include "ns3/leosim-beam-layout-engine.h"
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-device-installer.h"
#include "ns3/leosim-external-routing-helper.h"
#include "ns3/leosim-loader-helper.h"
#include "ns3/leosim-loader.h"
#include "ns3/leosim-mobility-helper.h"
#include "ns3/leosim-multi-beam-model.h"
#include "ns3/leosim-operator-helper.h"
#include "ns3/leosim-routing-calculator-helper.h"
#include "ns3/leosim-routing-calculator.h"
#include "ns3/leosim-statistics-helper.h"
#include "ns3/leosim-task-profiler.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

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
        LeoSimTaskProfiler::PrintSummary(wallSeconds);
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

    NS_FATAL_ERROR("Unknown routingMetric '" << value
                                              << "'; use hop, distance, path-loss, snr, or signal-strength");
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

    OnOffHelper sourceHelper("ns3::TcpSocketFactory", InetSocketAddress(serverAddress, port));
    sourceHelper.SetAttribute("DataRate", DataRateValue(DataRate(tcpRate)));
    sourceHelper.SetAttribute("PacketSize", UintegerValue(packetSize));
    sourceHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    sourceHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

    ApplicationContainer sourceApps = sourceHelper.Install(ueNode);
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
    std::string outputPrefix = "leosim-param-scenario";
    double simTime = 60.0;
    bool useTrace = true;
    bool verbose = false;

    uint32_t numSatellites = 0;
    uint32_t numServers = 0;
    uint32_t numUes = 0;
    double minElevation = 10.0;
    double accessMaxDistance = 2500000.0;
    std::string accessDataRate = "100Mbps";
    std::string accessDelay = "1ms";
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

    bool enableDynamicRouting = true;
    double routingUpdateInterval = 10.0;
    std::string routingMetricName = "hop";
    bool useRouteTreeCache = true;
    std::string routeTreeEngine = "contrib/leosim/utils/rengine/leosim-rengine";
    std::string routeTreeWorkDir = "/tmp/leosim-param-route-trees";
    uint32_t routeTreeWorkers = 8;
    uint64_t routeTreeMaxEntries = 10000000ULL;
    uint32_t beamNumRings = 2;
    double beamRadiusKm = 250.0;
    uint32_t beamReuseColors = 3;
    uint32_t serverId = 0;
    uint32_t ueId = 0;
    uint16_t port = 9000;
    double appStart = 1.0;
    double appStop = 59.0;
    std::string tcpRate = "1Mbps";
    uint32_t tcpPacketSize = 1024;
    bool allToAllTraffic = true;
    bool writeFlowMonitor = true;
    bool enableVisualization = false;
    double visualizationInterval = 1.0;
    bool enableStatistics = true;
    bool enableTaskProfiler = true;
    double statisticsInterval = 1.0;
    bool enableRouteLogging = true;
    double progressLogInterval = 5.0;

    CommandLine cmd;
    cmd.AddValue("satellites", "Path to satellite mobility trace or position CSV", satelliteFile);
    cmd.AddValue("dataDir", "Path to LeoSim data directory with gss/ and ues/ folders", leosimDataDir);
    cmd.AddValue("groundDevices", "Optional legacy ground device CSV; overrides dataDir when set", groundDeviceFile);
    cmd.AddValue("useTrace", "Load satellites from ns-2 trace format", useTrace);
    cmd.AddValue("simTime", "Simulation duration in seconds", simTime);
    cmd.AddValue("verbose", "Enable verbose helper output", verbose);
    cmd.AddValue("numSatellites", "Number of satellites to use; 0 means all loaded satellites", numSatellites);
    cmd.AddValue("numServers", "Number of servers/GSS to use; 0 means all loaded servers", numServers);
    cmd.AddValue("numUes", "Number of UEs to use; 0 means all loaded UEs", numUes);
    cmd.AddValue("minElevation", "Minimum satellite access-link elevation angle in degrees", minElevation);
    cmd.AddValue("accessMaxDistance", "Maximum satellite-ground link distance in meters", accessMaxDistance);
    cmd.AddValue("accessDataRate", "Satellite-ground point-to-point data rate", accessDataRate);
    cmd.AddValue("accessDelay", "Satellite-ground propagation delay, e.g. 1ms", accessDelay);
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
    cmd.AddValue("enableDynamicRouting", "Recompute routes periodically during the run", enableDynamicRouting);
    cmd.AddValue("routingUpdateInterval", "Dynamic routing update interval in seconds", routingUpdateInterval);
    cmd.AddValue("routingMetric",
                 "Dijkstra metric: hop, distance, path-loss, snr, or signal-strength",
                 routingMetricName);
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
    cmd.AddValue("outputPrefix", "Prefix used for FlowMonitor XML output", outputPrefix);
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
    cmd.AddValue("progressLogInterval",
                 "Simulation-time interval for wall-clock progress logs; 0 disables",
                 progressLogInterval);
    cmd.Parse(argc, argv);

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
    const auto routingMetric = ParseRoutingMetric(routingMetricName);

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
    if (loader->GetNumSatellites() == 0 || serverDeviceIds.empty() || ueDeviceIds.empty())
    {
        std::cerr << "Error: need at least 1 satellite, 1 server, and 1 UE." << std::endl;
        return 1;
    }

    if (numSatellites == 0 || numSatellites > loader->GetNumSatellites())
    {
        numSatellites = loader->GetNumSatellites();
    }
    if (numServers == 0 || numServers > serverDeviceIds.size())
    {
        numServers = serverDeviceIds.size();
    }
    if (numUes == 0 || numUes > ueDeviceIds.size())
    {
        numUes = ueDeviceIds.size();
    }
    if (serverId >= numServers || ueId >= numUes)
    {
        std::cerr << "Error: selected endpoint is outside loaded subset: serverId=" << serverId
                  << "/" << numServers << ", ueId=" << ueId << "/" << numUes << std::endl;
        return 1;
    }

    std::cout << "Scenario: satellites=" << numSatellites << ", servers=" << numServers
              << ", UEs=" << numUes << ", accessMaxDistance=" << accessMaxDistance
              << "m, accessDelay=" << accessDelay << ", islMaxDistance=" << islMaxDistance
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
    const std::string positionFile = outputPrefix + "-positions.csv";
    const std::string linkFile = outputPrefix + "-links.csv";
    const std::string packetFile = outputPrefix + "-packets.csv";
    if (enableVisualization)
    {
        visualizationHelper.SetOutputFile(positionFile);
        visualizationHelper.EnablePositionGeolocationLogging(true);
        visualizationHelper.SetUnifiedLinkStateFile(linkFile);
        visualizationHelper.SetPacketFile(packetFile);
        visualizationHelper.EnablePacketLogging(true);
        visualizationHelper.EnablePacketGeolocationLogging(true);
        visualizationHelper.SetLoaderHelper(loaderHelper);
        visualizationHelper.Initialize();
        timer.Log("visualization initialization");
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

    LeoSimDeviceInstaller accessInstaller;
    timer.Begin("access device installation");
    accessInstaller.SetChannelModel(accessChannel);
    accessInstaller.SetOperatorModel(operatorModel);
    accessInstaller.SetDeviceDataRate(accessDataRate);
    accessInstaller.SetDeviceDelay(accessDelay);
    accessInstaller.SetDeviceMtu(1500);
    accessInstaller.SetVerbose(verbose);
    NetDeviceContainer accessDevices = accessInstaller.Install(satelliteNodes, allGroundNodes);
    accessInstaller.ApplySharingRates(accessDevices);
    accessInstaller.EnableLinkStateCallbacks(accessChannel);
    timer.Log("access device installation");

    NetDeviceContainer islDevices;
    LeoSimDeviceInstaller islInstaller;
    if (enableIsl && islChannel)
    {
        timer.Begin("ISL device installation");
        islInstaller.SetChannelModel(islChannel);
        islInstaller.SetOperatorModel(operatorModel);
        islInstaller.SetDeviceDataRate(islDataRate);
        islInstaller.SetDeviceDelay(islDelay);
        islInstaller.SetDeviceMtu(1500);
        islInstaller.SetVerbose(verbose);
        islDevices = islInstaller.Install(satelliteNodes, NodeContainer());
        islInstaller.ApplySharingRates(islDevices);
        islInstaller.EnableLinkStateCallbacks(islChannel);
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
    if (enableRouteLogging)
    {
        routingHelper.EnableRouteLogging(routeLogFile);
    }
    Ptr<LeoSimRoutingCalculator> routingCalculator =
        routingHelper.CreateUnifiedRoutingCalculator(accessChannel, islChannel, verbose);
    routingCalculator->SetOperatorModel(operatorModel);
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
    beamHelper.SetHandoverMode(LEOSIM_HO_MODE_CHO);
    beamHelper.SetTtt(Seconds(1.0));
    beamHelper.SetT310(Seconds(1.0));
    beamHelper.SetN310(3);
    beamHelper.SetN311(3);
    beamHelper.SetA3Offset(3.0);
    beamHelper.SetA4Threshold(-110.0);
    beamHelper.SetTteThreshold(Seconds(30.0));
    beamHelper.SetSinrThreshold(-10.0);
    beamHelper.SetChoPreparationDelay(Seconds(0.1));
    beamHelper.SetChoExecutionDelay(Seconds(0.15));
    beamHelper.SetTopsisWeights(0.30, 0.25, 0.20, 0.15, 0.10, 0.05, 0.05);
    beamHelper.SetMaxCandidates(3);
    beamHelper.SetUpdateInterval(MilliSeconds(1000.0));
    beamHelper.EnableLoadBalancing(true);
    beamHelper.EnableHandoverBuffering(true);

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
    routingCalculator->SetBeamManager(beamManager);
    timer.Log("beam manager installation");

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
        case LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT:
            break;
        }
        routeTreeHelper.SetMetric(treeMetric);
        routeTreeHelper.SetMaxRouteRequests(routeTreeMaxEntries);
        routeTreeHelper.SetDestinationTreeAllNodes(true);
        routeTreeHelper.SetStatisticsEndpoints(statisticsRouteSources,
                                               statisticsRouteDestinations);

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
        routeTreeHelper.EnableReactiveRouteRefresh(routingCalculator,
                                                   routingSources,
                                                   routingDestinations,
                                                   MilliSeconds(200),
                                                   verbose);
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
        routingHelper.EnableReactiveLinkTriggeredRouting(
            routingCalculator,
            allNodes,
            routingDestinations,
            accessChannel,
            enableIsl ? islChannel : nullptr,
            MilliSeconds(200),
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
                                           ueId,
                                           serverId,
                                           port,
                                           appStart,
                                           appStop,
                                           tcpRate,
                                           tcpPacketSize);
    }
    (void)trafficApps;
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

    if (enableVisualization)
    {
        visualizationHelper.Finalize();
        std::cout << "Visualization data: " << positionFile << ", " << linkFile << ", "
                  << packetFile << std::endl;
        std::cout << "Render with: python3 contrib/leosim/utils/visualize_3d.py"
                  << " --position_file " << positionFile << " --links " << linkFile
                  << " --packets " << packetFile
                  << " --output " << outputPrefix << "-visualization.html" << std::endl;
    }
    timer.Log("result post-processing");

    Simulator::Destroy();
    timer.Log("Simulator::Destroy");
    timer.PrintSummary();
    return 0;
}
