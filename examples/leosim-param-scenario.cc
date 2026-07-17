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
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-device-installer.h"
#include "ns3/leosim-loader-helper.h"
#include "ns3/leosim-loader.h"
#include "ns3/leosim-mobility-helper.h"
#include "ns3/leosim-operator-helper.h"
#include "ns3/leosim-routing-calculator-helper.h"
#include "ns3/leosim-routing-calculator.h"
#include "ns3/leosim-statistics-helper.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"

#include <iomanip>
#include <iostream>
#include <string>

using namespace ns3;

namespace
{

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
    double statisticsInterval = 1.0;
    bool enableRouteLogging = true;

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
    cmd.AddValue("statisticsInterval", "Statistics sampling interval in seconds", statisticsInterval);
    cmd.AddValue("enableRouteLogging",
                 "Write selected paths and path-specific routing metrics",
                 enableRouteLogging);
    cmd.Parse(argc, argv);

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

    loader->LoadSatelliteOperatorsFromDataDirectory(leosimDataDir);
    LeoSimOperatorHelper opHelper;
    opHelper.SetLoader(loader);
    opHelper.SetVerbose(verbose);
    opHelper.RegisterSatellites(satelliteNodes);
    opHelper.RegisterGroundDevices(ueNodes, serverNodes);
    Ptr<LeoSimOperatorModel> operatorModel = opHelper.Build();

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
    }

    NodeContainer allGroundNodes;
    allGroundNodes.Add(ueNodes);
    allGroundNodes.Add(serverNodes);

    LeoSimChannelHelper accessChannelHelper;
    accessChannelHelper.SetMinElevationAngle(minElevation);
    accessChannelHelper.SetMaxLinkDistance(accessMaxDistance);
    accessChannelHelper.SetUpdateInterval(Seconds(1.0));
    accessChannelHelper.SetVerbose(verbose);
    Ptr<LeoSimChannelModel> accessChannel =
        accessChannelHelper.CreateChannels(satelliteNodes, allGroundNodes);
    accessChannel->SetOperatorModel(operatorModel);

    Ptr<LeoSimChannelModel> islChannel;
    if (enableIsl)
    {
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
    accessInstaller.SetChannelModel(accessChannel);
    accessInstaller.SetOperatorModel(operatorModel);
    accessInstaller.SetDeviceDataRate(accessDataRate);
    accessInstaller.SetDeviceDelay(accessDelay);
    accessInstaller.SetDeviceMtu(1500);
    accessInstaller.SetVerbose(verbose);
    NetDeviceContainer accessDevices = accessInstaller.Install(satelliteNodes, allGroundNodes);
    accessInstaller.ApplySharingRates(accessDevices);
    accessInstaller.EnableLinkStateCallbacks(accessChannel);

    NetDeviceContainer islDevices;
    LeoSimDeviceInstaller islInstaller;
    if (enableIsl && islChannel)
    {
        islInstaller.SetChannelModel(islChannel);
        islInstaller.SetOperatorModel(operatorModel);
        islInstaller.SetDeviceDataRate(islDataRate);
        islInstaller.SetDeviceDelay(islDelay);
        islInstaller.SetDeviceMtu(1500);
        islInstaller.SetVerbose(verbose);
        islDevices = islInstaller.Install(satelliteNodes, NodeContainer());
        islInstaller.ApplySharingRates(islDevices);
        islInstaller.EnableLinkStateCallbacks(islChannel);
    }

    InternetStackHelper stack;
    stack.Install(satelliteNodes);
    stack.Install(serverNodes);
    stack.Install(ueNodes);

    Ipv4AddressHelper accessIpv4;
    accessIpv4.SetBase(Ipv4Address("10.0.0.0"), Ipv4Mask("255.255.255.252"));
    AssignPerLinkSubnets(accessDevices, accessIpv4);

    if (enableIsl && islDevices.GetN() > 0)
    {
        Ipv4AddressHelper islIpv4;
        islIpv4.SetBase(Ipv4Address("10.128.0.0"), Ipv4Mask("255.255.255.252"));
        AssignPerLinkSubnets(islDevices, islIpv4);
    }

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    NodeContainer allNodes;
    allNodes.Add(satelliteNodes);
    allNodes.Add(serverNodes);
    allNodes.Add(ueNodes);

    NodeContainer routingDestinations;
    routingDestinations.Add(serverNodes);
    routingDestinations.Add(ueNodes);

    LeoSimRoutingCalculatorHelper routingHelper;
    const std::string routeLogFile = outputPrefix + "-routes.csv";
    if (enableRouteLogging)
    {
        routingHelper.EnableRouteLogging(routeLogFile);
    }
    Ptr<LeoSimRoutingCalculator> routingCalculator =
        routingHelper.CreateUnifiedRoutingCalculator(accessChannel, islChannel, verbose);
    routingCalculator->SetOperatorModel(operatorModel);
    if (enableDynamicRouting)
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

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowMonitor = flowmonHelper.InstallAll();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());

    Ptr<LeoSimStatisticsHelper> statistics;
    const std::string statisticsCsvFile = outputPrefix + "-statistics.csv";
    const std::string statisticsJsonFile = outputPrefix + "-statistics.json";
    if (enableStatistics)
    {
        statistics = CreateObject<LeoSimStatisticsHelper>();
        statistics->SetFlowMonitor(flowMonitor, classifier);
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
    }

    if (enableVisualization)
    {
        visualizationHelper.InstallPacketLogging(satelliteNodes, serverNodes, ueNodes);
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    PrintFlowMonitorSummary(flowMonitor, classifier);
    if (writeFlowMonitor)
    {
        flowMonitor->SerializeToXmlFile(outputPrefix + "-flowmon.xml", true, true);
    }
    if (enableStatistics)
    {
        statistics->StopPeriodicSampling();
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

    Simulator::Destroy();
    return 0;
}
