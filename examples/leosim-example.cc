/*
 * Copyright (c) 2024 Anupa De Silva
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

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/leosim-beam-manager-helper.h"
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-channel.h"
#include "ns3/leosim-device-installer.h"
#include "ns3/leosim-isl-routing-model.h"
#include "ns3/leosim-loader-helper.h"
#include "ns3/leosim-loader.h"
#include "ns3/leosim-mobility-helper.h"
#include "ns3/leosim-operator-helper.h"
#include "ns3/leosim-routing-calculator-helper.h"
#include "ns3/leosim-routing-calculator.h"
#include "ns3/leosim-traffic-generator-helper.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/output-stream-wrapper.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeoSimTcpExample");

int
main(int argc, char* argv[])
{
    std::string satelliteFile = "contrib/leosim/utils/satellite_mobility.tcl";
    std::string groundDeviceFile = "contrib/leosim/utils/ground_devices.csv";
    double simTime = 60.0;
    double logInterval = 1.0;
    double minElevation = 10.0;
    double frequency = 12.0e9;
    std::string positionFile = "leosim_positions.csv";
    std::string linkFile = "leosim_links.csv";
    std::string packetFile = "leosim_packets.csv";
    std::string beamFile = "leosim_beams.csv";
    std::string handoverFile = "leosim_handovers.csv";
    std::string choFile = "leosim_cho.csv";
    bool logPackets = true;
    bool logBeams = true;
    bool verbose = true;
    bool useTrace = true;
    bool enablePeriodicRouting = true;
    double routingUpdateInterval = 10.0;
    double pathPrintInterval = 10.0;
    uint32_t numSatellites = 300;
    uint32_t numServers = 1;
    uint32_t numUes = 1;
    bool enableIsl = true;
    double islFrequency = 26.0e9;
    double islMaxDistance = 5000000.0;
    double islTransmitPower = 30.0;
    double islAntennaGain = 35.0;

    // === Application Traffic Configuration ===
    std::string tcpRate = "1Mbps";
    uint32_t tcpPacketSize = 512;
    double appStart = 1.0;

    // === Beam Manager & Handover Configuration (3GPP NTN CHO) ===
    std::string hoMode = "CHO";
    bool earthFixedBeam = false;
    double tttSeconds = 1.0;
    double t310Seconds = 1.0;
    uint32_t n310 = 3;
    uint32_t n311 = 3;
    double a3OffsetDb = 3.0;
    double a4ThresholdDbm = -110.0;
    double tteTriggerSeconds = 30.0;
    double wRsrp = 0.30;
    double wSinr = 0.25;
    double wTte = 0.20;
    double wLoad = 0.15;
    double wLatency = 0.10;
    double wElevation = 0.05;
    double wActive = 0.05;
    uint32_t maxCandidates = 3;
    double choPrep = 100.0;
    double choExec = 150.0;
    bool enableLoadBalancing = true;
    bool enableHoBuffering = true;

    // === Operator Sharing ===
    std::string satOperatorsFile = "";
    std::string sharingMatrixFile = "";
    double defaultAlpha = 1.0;
    bool operatorIsolation = false;

    CommandLine cmd;
    cmd.AddValue("satellites", "Path to satellite position file", satelliteFile);
    cmd.AddValue("groundDevices", "Path to ground device CSV file", groundDeviceFile);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("logInterval", "Interval for position/link logging (seconds)", logInterval);
    cmd.AddValue("minElevation", "Minimum elevation angle (degrees)", minElevation);
    cmd.AddValue("frequency", "Carrier frequency (Hz)", frequency);
    cmd.AddValue("positions", "Output file for position data", positionFile);
    cmd.AddValue("links", "Output file for link data", linkFile);
    cmd.AddValue("packets", "Output file for packet data", packetFile);
    cmd.AddValue("beams", "Output file for beam state data", beamFile);
    cmd.AddValue("handovers", "Output file for handover events", handoverFile);
    cmd.AddValue("cho", "Output file for CHO candidate configuration", choFile);
    cmd.AddValue("logPackets", "Enable packet logging", logPackets);
    cmd.AddValue("logBeams", "Enable beam + handover logging", logBeams);
    cmd.AddValue("verbose", "Enable verbose logging", verbose);
    cmd.AddValue("useTrace", "Use ns-3 trace file format for satellites", useTrace);
    cmd.AddValue("enablePeriodicRouting", "Enable periodic routing updates", enablePeriodicRouting);
    cmd.AddValue("routingUpdateInterval",
                 "Routing update interval (seconds)",
                 routingUpdateInterval);
    cmd.AddValue("pathPrintInterval",
                 "Interval for printing UE->Server node-id paths (seconds)",
                 pathPrintInterval);
    cmd.AddValue("numSatellites", "Number of satellites to use from the file", numSatellites);
    cmd.AddValue("numServers", "Number of server ground stations to use", numServers);
    cmd.AddValue("numUes", "Number of UEs to use", numUes);
    cmd.AddValue("enableIsl", "Enable Inter-Satellite Links (ISL)", enableIsl);
    cmd.AddValue("islFrequency", "ISL carrier frequency (Hz)", islFrequency);
    cmd.AddValue("islMaxDistance", "Maximum ISL distance (meters)", islMaxDistance);
    cmd.AddValue("islTransmitPower", "ISL transmit power (dBm)", islTransmitPower);
    cmd.AddValue("islAntennaGain", "ISL antenna gain (dB)", islAntennaGain);

    cmd.AddValue("tcpRate", "Per-UE offered TCP rate (e.g., 100Kbps, 1Mbps)", tcpRate);
    cmd.AddValue("tcpPacketSize", "TCP application packet size in bytes", tcpPacketSize);
    cmd.AddValue("appStart", "Application start time in seconds", appStart);

    // === Beam Manager & Handover Parameters (3GPP NTN CHO) ===
    cmd.AddValue("hoMode",
                 "Handover mode: CHO (Conditional, 3GPP Rel-17) or BHO (reactive)",
                 hoMode);
    cmd.AddValue("earthFixedBeam",
                 "Use earth-fixed beam footprint instead of satellite-fixed",
                 earthFixedBeam);
    cmd.AddValue("ttt", "Time-to-Trigger duration in seconds (default 1.0)", tttSeconds);
    cmd.AddValue("t310", "T310 RLF detection timer in seconds (default 1.0)", t310Seconds);
    cmd.AddValue("n310", "N310: consecutive out-of-sync detections before RLF", n310);
    cmd.AddValue("n311", "N311: consecutive in-sync recoveries to cancel T310", n311);
    cmd.AddValue("a3Offset", "A3 event RSRP offset in dB (default 3.0)", a3OffsetDb);
    cmd.AddValue("a4Threshold",
                 "A4 absolute RSRP threshold in dBm (default -110.0)",
                 a4ThresholdDbm);
    cmd.AddValue("tteTrigger",
                 "Ephemeris handover lead time before TTE expires, seconds",
                 tteTriggerSeconds);
    cmd.AddValue("maxCandidates", "Maximum CHO candidate satellites pre-positioned", maxCandidates);
    cmd.AddValue("choPrep", "CHO preparation phase delay in milliseconds (default 100)", choPrep);
    cmd.AddValue("choExec", "CHO execution phase delay in milliseconds (default 150)", choExec);
    cmd.AddValue("wRsrp", "TOPSIS weight for RSRP", wRsrp);
    cmd.AddValue("wSinr", "TOPSIS weight for SINR", wSinr);
    cmd.AddValue("wTte", "TOPSIS weight for propagation delay", wTte);
    cmd.AddValue("wLoad", "TOPSIS weight for link load", wLoad);
    cmd.AddValue("wLatency", "TOPSIS weight for latency", wLatency);
    cmd.AddValue("wElevation", "TOPSIS weight for elevation", wElevation);
    cmd.AddValue("wActive", "TOPSIS weight for active-beam preference", wActive);
    cmd.AddValue("enableLoadBalancing", "Enable load-balancing handovers", enableLoadBalancing);
    cmd.AddValue("enableHoBuffering", "Enable packet buffering during handover", enableHoBuffering);

    // === Operator Sharing ===
    cmd.AddValue("satOperators",
                 "CSV file: SatelliteIndex,Operator assignments",
                 satOperatorsFile);
    cmd.AddValue("sharingMatrix",
                 "CSV file: OperatorA,OperatorB,AlphaDL,AlphaUL,AlphaISL",
                 sharingMatrixFile);
    cmd.AddValue("defaultAlpha",
                 "Default cross-operator alpha when no matrix given [0,1]",
                 defaultAlpha);
    cmd.AddValue("operatorIsolation",
                 "If true, routes never cross operator boundaries",
                 operatorIsolation);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);

    if (verbose)
    {
        // LogComponentEnable("LeoSimTcpExample", LOG_LEVEL_INFO);

        LogComponentEnable("LeoSimBeamManager", LOG_LEVEL_DEBUG);
        LogComponentEnable("LeoSimBeamManagerHelper", LOG_LEVEL_INFO);

        // LogComponentEnable("LeoSimChannelModel", LOG_LEVEL_INFO);
        // LogComponentEnable("LeoSimChannelHelper", LOG_LEVEL_INFO);
        // LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
        // LogComponentEnable("Ipv4GlobalRouting", LOG_LEVEL_INFO);
        // LogComponentEnable("LeoSimRoutingCalculatorHelper", LOG_LEVEL_INFO);
    }

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
    loaderHelper.LoadGroundDevicesFromCsv(groundDeviceFile);

    Ptr<LeoSimLoader> loader = loaderHelper.GetLoader();
    auto serverDeviceIds = loader->GetGroundDeviceIdsByType("SERVER");
    auto ueDeviceIds = loader->GetGroundDeviceIdsByType("UE");

    if (loader->GetNumSatellites() == 0 || serverDeviceIds.empty() || ueDeviceIds.empty())
    {
        std::cerr << "Error: Need at least 1 satellite, 1 server, and 1 UE for this example."
                  << std::endl;
        return 1;
    }

    // Validate requested numbers against available devices
    if (numSatellites > loader->GetNumSatellites())
    {
        std::cerr << "Warning: Requested " << numSatellites << " satellites, but only "
                  << loader->GetNumSatellites() << " available. Using all available." << std::endl;
        numSatellites = loader->GetNumSatellites();
    }

    if (numServers > serverDeviceIds.size())
    {
        std::cerr << "Warning: Requested " << numServers << " servers, but only "
                  << serverDeviceIds.size() << " available. Using all available." << std::endl;
        numServers = serverDeviceIds.size();
    }

    if (numUes > ueDeviceIds.size())
    {
        std::cerr << "Warning: Requested " << numUes << " UEs, but only " << ueDeviceIds.size()
                  << " available. Using all available." << std::endl;
        numUes = ueDeviceIds.size();
    }

    std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Setting up multi-node topology" << std::endl;
    std::cout << "  Satellites: " << numSatellites << std::endl;
    std::cout << "  Servers: " << numServers << std::endl;
    std::cout << "  UEs: " << numUes << std::endl;

    // Create nodes for each role
    NodeContainer ueNodes;
    ueNodes.Create(numUes);

    NodeContainer satelliteNodes;
    satelliteNodes.Create(numSatellites);

    NodeContainer serverNodes;
    serverNodes.Create(numServers);

    LeoSimMobilityHelper mobilityHelper;
    mobilityHelper.SetLoader(loader);
    mobilityHelper.SetVerbose(verbose);
    mobilityHelper.SetVelocityCalculation(true);

    // Install mobility for all satellites
    for (uint32_t i = 0; i < numSatellites; i++)
    {
        mobilityHelper.InstallSatellite(satelliteNodes.Get(i), i, loader->GetSatelliteName(i));
        if (verbose)
        {
            std::cout << "  [t=" << Simulator::Now().GetSeconds() << "s] Satellite " << i << ": " << loader->GetSatelliteName(i) << std::endl;
        }
    }

    // Install mobility for all UEs
    for (uint32_t i = 0; i < numUes; i++)
    {
        uint32_t ueId = ueDeviceIds[i];
        mobilityHelper.InstallGateway(ueNodes.Get(i),
                                      ueId,
                                      loader->GetGroundDeviceName(ueId),
                                      loader->GetGroundDevicePosition(ueId));
        if (verbose)
        {
            std::cout << "  [t=" << Simulator::Now().GetSeconds() << "s] UE " << i << ": " << loader->GetGroundDeviceName(ueId) << std::endl;
        }
    }

    // Install mobility for all servers
    for (uint32_t i = 0; i < numServers; i++)
    {
        uint32_t serverId = serverDeviceIds[i];
        mobilityHelper.InstallGateway(serverNodes.Get(i),
                                      serverId,
                                      loader->GetGroundDeviceName(serverId),
                                      loader->GetGroundDevicePosition(serverId));
        if (verbose)
        {
            std::cout << "  [t=" << Simulator::Now().GetSeconds() << "s] Server " << i << ": " << loader->GetGroundDeviceName(serverId)
                      << std::endl;
        }
    }

    mobilityHelper.StartAll();

    // === Operator Model Setup (Phase 8 of LeoSim) ===
    if (!satOperatorsFile.empty())
    {
        loader->LoadSatelliteOperatorsFromCsv(satOperatorsFile);
    }

    LeoSimOperatorHelper opHelper;
    opHelper.SetLoader(loader);
    opHelper.SetVerbose(verbose);
    opHelper.RegisterSatellites(satelliteNodes);
    opHelper.RegisterGroundDevices(ueNodes, serverNodes);

    if (!sharingMatrixFile.empty())
    {
        opHelper.LoadSharingMatrix(sharingMatrixFile);
    }
    else if (defaultAlpha < 1.0)
    {
        opHelper.SetUniformCrossOperatorAlpha(defaultAlpha, defaultAlpha, defaultAlpha);
    }

    Ptr<LeoSimOperatorModel> operatorModel = opHelper.Build();

    // Create visualization helper
    LeoSimVisualizationHelper vizHelper;
    vizHelper.SetOutputFile(positionFile);
    vizHelper.SetLinkFile(linkFile);
    vizHelper.SetPacketFile(packetFile);
    vizHelper.SetBeamFile(beamFile);
    vizHelper.SetHandoverFile(handoverFile);
    vizHelper.SetChoFile(choFile);
    vizHelper.EnablePacketLogging(logPackets);
    vizHelper.EnableBeamLogging(logBeams);
    vizHelper.SetLoaderHelper(loaderHelper);
    vizHelper.Initialize();

    // Create dynamic channel model for logging/visualization
    LeoSimChannelHelper channelHelper;
    channelHelper.SetMinElevationAngle(minElevation);
    channelHelper.SetFrequency(frequency);
    channelHelper.SetTransmitPower(40.0);
    channelHelper.SetMaxLinkDistance(2500000.0);
    channelHelper.SetUpdateInterval(Seconds(1.0));
    channelHelper.SetVerbose(verbose);

    // Combine UE and server nodes for channel model
    NodeContainer allGroundNodes;
    allGroundNodes.Add(ueNodes);
    allGroundNodes.Add(serverNodes);

    Ptr<LeoSimChannelModel> channelModel =
        channelHelper.CreateChannels(satelliteNodes, allGroundNodes);
    channelModel->SetOperatorModel(operatorModel);

    channelModel->StartUpdates();

    // Create ISL mesh if enabled
    Ptr<LeoSimChannelModel> islChannelModel;
    std::map<Ptr<Node>, std::vector<Ipv4Address>> islAddresses;
    if (enableIsl)
    {
        std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Creating ISL mesh between satellites..." << std::endl;
        LeoSimChannelHelper islHelper;
        islHelper.SetIslFrequency(islFrequency);
        islHelper.SetIslMaxDistance(islMaxDistance);
        islHelper.SetIslTransmitPower(islTransmitPower);
        islHelper.SetIslAntennaGain(islAntennaGain);
        islHelper.SetUpdateInterval(Seconds(1.0));
        islHelper.SetVerbose(verbose);

        islChannelModel = islHelper.CreateIslMesh(satelliteNodes);
        islChannelModel->SetOperatorModel(operatorModel);
        islChannelModel->StartUpdates();

        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] ISL configuration:" << std::endl;
        std::cout << "  Frequency: " << islFrequency / 1e9 << " GHz" << std::endl;
        std::cout << "  Max Distance: " << islMaxDistance / 1000.0 << " km" << std::endl;
        std::cout << "  Tx Power: " << islTransmitPower << " dBm" << std::endl;
        std::cout << "  Antenna Gain: " << islAntennaGain << " dB" << std::endl;

        // Set ISL channel model for ISL link visualization
        vizHelper.SetIslChannelModel(islChannelModel);
        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] ISL visualization enabled" << std::endl;
    }

    // Set channel model in visualization helper for ground link tracking
    vizHelper.SetChannelModel(channelModel);

    // Install network devices on all nodes based on channel model links
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Installing network devices from channel model links..." << std::endl;

    // Install devices for ground links (satellite-to-UE, satellite-to-Server)
    LeoSimDeviceInstaller deviceInstaller;
    deviceInstaller.SetChannelModel(channelModel);
    deviceInstaller.SetOperatorModel(operatorModel);
    deviceInstaller.SetDeviceDataRate("100Mbps");
    deviceInstaller.SetDeviceDelay("1ms");
    deviceInstaller.SetDeviceMtu(1500);
    deviceInstaller.SetVerbose(verbose);

    NetDeviceContainer groundDevices = deviceInstaller.Install(satelliteNodes, allGroundNodes);
    deviceInstaller.ApplySharingRates(groundDevices);
    std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Installed " << groundDevices.GetN() << " ground link devices" << std::endl;

    // Install devices for ISL links if enabled
    NetDeviceContainer islDevices;
    if (enableIsl && islChannelModel)
    {
        LeoSimDeviceInstaller islDeviceInstaller;
        islDeviceInstaller.SetChannelModel(islChannelModel);
        islDeviceInstaller.SetOperatorModel(operatorModel);
        islDeviceInstaller.SetDeviceDataRate("10Gbps"); // ISL uses higher data rate
        islDeviceInstaller.SetDeviceDelay("100us");     // ISL lower latency
        islDeviceInstaller.SetDeviceMtu(1500);
        islDeviceInstaller.SetVerbose(verbose);

        islDevices = islDeviceInstaller.Install(satelliteNodes, NodeContainer());
        islDeviceInstaller.ApplySharingRates(islDevices);
        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Installed " << islDevices.GetN() << " ISL devices" << std::endl;
    }

    //  Install Internet stack on all nodes
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Installing Internet stack on all nodes..." << std::endl;
    InternetStackHelper stack;
    stack.Install(satelliteNodes);
    stack.Install(ueNodes);
    stack.Install(serverNodes);
    std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Internet stack installed" << std::endl;

    // Assign IP addresses - unique per-link subnets to avoid address collisions
    // Ground links on 10.0-99.0.0/24, ISL links on 10.100-199.0.0/24
    // This ensures each interface has a unique address and no collisions between nodes
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Assigning IP addresses (unique per-link subnets)..." << std::endl;

    uint32_t groundSubnetIndex = 0;
    uint32_t islSubnetIndex = 100;

    // Assign ground devices - create proper P2P links with IP address pairs
    std::cout << "  [t=" << Simulator::Now().GetSeconds() << "s] Assigning ground link subnets..." << std::endl;

    // For each pair of devices that form a link, assign IP address
    uint32_t assignedLinks = 0;
    for (uint32_t i = 0; i < groundDevices.GetN(); i += 2)
    {
        NetDeviceContainer linkDevices;
        linkDevices.Add(groundDevices.Get(i));

        if (i + 1 < groundDevices.GetN())
        {
            linkDevices.Add(groundDevices.Get(i + 1));
        }
        else
        {
            // If we have an odd device, find its pair by checking which nodes they connect
            // For now, just assign the single device its own subnet
            std::cout << "    Warning: Odd number of devices, device " << i << " unpaired"
                      << std::endl;
        }

        Ipv4AddressHelper groundIpv4;
        char baseAddrStr[32];
        snprintf(baseAddrStr, sizeof(baseAddrStr), "10.%d.0.0", groundSubnetIndex);
        groundIpv4.SetBase(Ipv4Address(baseAddrStr), Ipv4Mask("255.255.255.0"));
        groundIpv4.Assign(linkDevices);

        groundSubnetIndex++;
        assignedLinks++;
    }

    // Assign ISL devices - create proper P2P links with IP address pairs
    if (enableIsl && islDevices.GetN() > 0)
    {
        std::cout << "  [t=" << Simulator::Now().GetSeconds() << "s] Assigning ISL link subnets..." << std::endl;
        for (uint32_t i = 0; i < islDevices.GetN(); i += 2)
        {
            NetDeviceContainer linkDevices;
            linkDevices.Add(islDevices.Get(i));

            if (i + 1 < islDevices.GetN())
            {
                linkDevices.Add(islDevices.Get(i + 1));
            }
            else
            {
                std::cout << "    Warning: Odd number of ISL devices, device " << i << " unpaired"
                          << std::endl;
            }

            Ipv4AddressHelper islIpv4;
            char baseAddrStr[32];
            snprintf(baseAddrStr, sizeof(baseAddrStr), "10.%d.0.0", islSubnetIndex);
            islIpv4.SetBase(Ipv4Address(baseAddrStr), Ipv4Mask("255.255.255.0"));
            islIpv4.Assign(linkDevices);

            if (verbose)
            {
                std::cout << "    ISL " << (islSubnetIndex - 100) << ": " << baseAddrStr << "/24"
                          << std::endl;
            }

            islSubnetIndex++;
        }
    }

    std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Address assignment complete: " << assignedLinks << " ground links, "
              << (islSubnetIndex - 100) << " ISL links" << std::endl;

    // Combine all nodes for routing
    NodeContainer allNodes;
    allNodes.Add(satelliteNodes);
    allNodes.Add(ueNodes);
    allNodes.Add(serverNodes);

    vizHelper.SetOperatorFile("leosim_operators.csv");
    vizHelper.SetSharingFile("leosim_sharing.csv");
    vizHelper.InitOperatorLogging(operatorModel, allNodes);

    // Install computed routes into static routing tables
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Setting up dynamic routing with periodic updates..." << std::endl;
    LeoSimRoutingCalculatorHelper routingHelper;

    // Create unified routing calculator that handles both ground and ISL links
    Ptr<LeoSimRoutingCalculator> unifiedCalc =
        routingHelper.CreateUnifiedRoutingCalculator(channelModel, islChannelModel, verbose);
    unifiedCalc->SetOperatorModel(operatorModel);
    if (operatorIsolation)
    {
        unifiedCalc->SetPathType(LeoSimRoutingCalculator::LEOSIM_PATH_SAME_OPERATOR_ONLY);
    }

    // Combine all nodes for routing (already created above)

    // Enable dynamic routing with periodic updates
    // This continuously recalculates and updates routes based on changing topology
    routingHelper.EnableDynamicRouting(unifiedCalc,
                                       allNodes,
                                       allNodes,
                                       Seconds(routingUpdateInterval),
                                       simTime,
                                       verbose);

    if (verbose)
    {
        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Dynamic routing enabled with update interval: " << routingUpdateInterval
                  << " seconds" << std::endl;
    }

    // === Beam Management & Handover (3GPP NTN CHO) ===
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Setting up beam manager with conditional handover support..." << std::endl;

    LeoSimBeamManagerHelper beamHelper;
    beamHelper.SetVerbose(verbose);
    beamHelper.SetChannelModel(channelModel);
    if (enableIsl && islChannelModel)
    {
        beamHelper.SetIslChannelModel(islChannelModel);
    }
    beamHelper.SetRoutingCalculator(unifiedCalc);
    beamHelper.SetLoader(loader);

    // Set handover mode and earth-fixed beam configuration
    if (hoMode == "BHO")
    {
        beamHelper.SetHandoverMode(LEOSIM_HO_MODE_BHO);
    }
    else
    {
        beamHelper.SetHandoverMode(LEOSIM_HO_MODE_CHO);
    }

    if (earthFixedBeam)
    {
        beamHelper.SetEarthFixedBeamMode(true);
    }

    // Set 3GPP timers and counters
    beamHelper.SetTtt(Seconds(tttSeconds));
    beamHelper.SetT310(Seconds(t310Seconds));
    beamHelper.SetN310(n310);
    beamHelper.SetN311(n311);

    // Set handover decision thresholds
    beamHelper.SetA3Offset(a3OffsetDb);
    beamHelper.SetA4Threshold(a4ThresholdDbm);
    beamHelper.SetTteThreshold(Seconds(tteTriggerSeconds));

    // Set CHO timing (convert from milliseconds to seconds)
    beamHelper.SetChoPreparationDelay(Seconds(choPrep / 1000.0));
    beamHelper.SetChoExecutionDelay(Seconds(choExec / 1000.0));

    // Set TOPSIS prioritization weights (must sum to 1.0)
    beamHelper.SetTopsisWeights(wRsrp, wSinr, wTte, wLoad, wLatency, wElevation, wActive);
    beamHelper.SetMaxCandidates(maxCandidates);

    // Set features
    beamHelper.EnableLoadBalancing(enableLoadBalancing);
    beamHelper.EnableHandoverBuffering(enableHoBuffering);

    // Install beam manager on all ground nodes (UEs + servers)
    Ptr<LeoSimBeamManager> beamManager =
        beamHelper.Install(allGroundNodes, satelliteNodes, Seconds(simTime));
    beamManager->SetOperatorModel(operatorModel);

    if (logBeams)
    {
        vizHelper.SetBeamManager(beamManager);
    }

    if (verbose)
    {
        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Beam manager installed:" << std::endl;
        std::cout << "  Mode: " << hoMode << std::endl;
        std::cout << "  Earth-fixed beam: " << (earthFixedBeam ? "enabled" : "disabled")
                  << std::endl;
        std::cout << "  TTT: " << tttSeconds << "s, T310: " << t310Seconds << "s" << std::endl;
        std::cout << "  N310: " << n310 << ", N311: " << n311 << std::endl;
        std::cout << "  A3 offset: " << a3OffsetDb << " dB, A4 threshold: " << a4ThresholdDbm
                  << " dBm" << std::endl;
        std::cout << "  TTE trigger: " << tteTriggerSeconds << "s" << std::endl;
        std::cout << "  CHO prep: " << choPrep << "ms, exec: " << choExec << "ms" << std::endl;
        std::cout << "  Load balancing: " << (enableLoadBalancing ? "enabled" : "disabled")
                  << std::endl;
        std::cout << "  Handover buffering: " << (enableHoBuffering ? "enabled" : "disabled")
                  << std::endl;
    }

    // Schedule position and link logging
    vizHelper.SchedulePositionLogging(satelliteNodes, serverNodes, ueNodes, logInterval, simTime);
    for (double t = 0; t <= simTime; t += logInterval)
    {
        Simulator::Schedule(Seconds(t),
                            &LeoSimVisualizationHelper::LogSharingState,
                            &vizHelper,
                            operatorModel,
                            channelModel,
                            islChannelModel,
                            t);
    }

    // Install TCP traffic from UEs to server
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Installing TCP traffic from UEs to server..." << std::endl;

    // Get server IP address
    Ptr<Ipv4> serverIpv4 = serverNodes.Get(0)->GetObject<Ipv4>();
    Ipv4Address serverAddress = Ipv4Address::GetZero();

    // Find the server's interface address (skip loopback at index 0)
    if (serverIpv4->GetNAddresses(1) > 0)
    {
        serverAddress = serverIpv4->GetAddress(1, 0).GetLocal();
    }

    if (serverAddress != Ipv4Address::GetZero())
    {
        // Install PacketSink on server to receive TCP traffic
        PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), 9));
        ApplicationContainer sinkApps = sinkHelper.Install(serverNodes.Get(0));
        sinkApps.Start(Seconds(0.0));
        sinkApps.Stop(Seconds(simTime));

        // Install paced TCP traffic on each UE.
        // BulkSend is "as fast as possible" and can overwhelm the topology; OnOff lets us set a
        // rate.
        OnOffHelper onOffHelper("ns3::TcpSocketFactory", InetSocketAddress(serverAddress, 9));
        onOffHelper.SetAttribute("DataRate", DataRateValue(DataRate(tcpRate)));
        onOffHelper.SetAttribute("PacketSize", UintegerValue(tcpPacketSize));
        onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));

        ApplicationContainer tcpApps;
        for (uint32_t i = 0; i < ueNodes.GetN(); i++)
        {
            ApplicationContainer ueApp = onOffHelper.Install(ueNodes.Get(i));
            ueApp.Start(Seconds(appStart));
            ueApp.Stop(Seconds(simTime - 1.0));
            tcpApps.Add(ueApp);
        }

        if (verbose)
        {
            std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Installed " << tcpApps.GetN() << " TCP client applications" << std::endl;
            std::cout << "  Target server address: " << serverAddress << ":9" << std::endl;
            std::cout << "  Start time: " << appStart << "s, Stop time: " << (simTime - 1.0) << "s"
                      << std::endl;
            std::cout << "  TCP source: OnOffApplication" << std::endl;
            std::cout << "  Per-UE rate: " << tcpRate << ", packetSize: " << tcpPacketSize
                      << " bytes" << std::endl;
        }
    }
    else
    {
        std::cerr << "Warning: Could not determine server IP address for TCP traffic" << std::endl;
    }

    // Install packet logging hooks for visualization (after network setup complete)
    if (logPackets)
    {
        vizHelper.InstallPacketLogging(satelliteNodes, serverNodes, ueNodes);
        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Packet logging activated" << std::endl;
    }

    // === Add Flow Monitor for Packet Loss Analysis ===
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowmonHelper;
    flowMonitor = flowmonHelper.InstallAll();

    // Run the simulation for the specified duration
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

    // Print flow monitor statistics
    flowMonitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier());
    FlowMonitor::FlowStatsContainer stats = flowMonitor->GetFlowStats();

    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] === PACKET LOSS ANALYSIS ===" << std::endl;
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end();
         ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Flow " << i->first << " (" << t.sourceAddress << " -> "
                  << t.destinationAddress << ")" << std::endl;
        std::cout << "  Tx Packets: " << i->second.txPackets << std::endl;
        std::cout << "  Rx Packets: " << i->second.rxPackets << std::endl;
        std::cout << "  Lost Packets: " << (i->second.txPackets - i->second.rxPackets) << std::endl;
        std::cout << "  Loss Rate: "
                  << (100.0 * (i->second.txPackets - i->second.rxPackets) / i->second.txPackets)
                  << "%" << std::endl;
        std::cout << "  Delay (ms): "
                  << (i->second.delaySum.GetMilliSeconds() / i->second.rxPackets) << std::endl;
    }
    std::cout << "=======================================" << std::endl;

    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] === LeoSim Visualization Outputs ===" << std::endl;
    std::cout << "positions:  " << positionFile << std::endl;
    std::cout << "links:      " << linkFile << std::endl;
    if (logPackets)
    {
        std::cout << "packets:    " << packetFile << std::endl;
    }
    else
    {
        std::cout << "packets:    (disabled)" << std::endl;
    }
    if (logBeams)
    {
        std::cout << "beams:      " << beamFile << std::endl;
        std::cout << "handovers:  " << handoverFile << std::endl;
        std::cout << "cho:        " << choFile << std::endl;
    }
    else
    {
        std::cout << "beams/handovers/cho: (disabled)" << std::endl;
    }
    std::cout << "\nTo visualize (from ns3/ directory):" << std::endl;
    std::cout << "  python contrib/leosim/utils/visualize_3d.py \\\n+  --position_file "
              << positionFile << " \\\n+  --links " << linkFile;
    if (logPackets)
    {
        std::cout << " \\\n+  --packets " << packetFile;
    }
    if (logBeams)
    {
        std::cout << " \\\n+  --beams " << beamFile << " \\\n+  --handovers " << handoverFile;
    }
    std::cout << " \\\n+  --output visualization.html" << std::endl;
    std::cout << "===================================" << std::endl;

    // Channel updates and visualization finalize
    channelModel->StopUpdates();
    if (enableIsl && islChannelModel)
    {
        islChannelModel->StopUpdates();
    }
    vizHelper.Finalize();

    Simulator::Destroy();

    return 0;
}
