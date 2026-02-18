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
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/output-stream-wrapper.h"
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-channel.h"
#include "ns3/leosim-isl-routing-model.h"
#include "ns3/leosim-loader-helper.h"
#include "ns3/leosim-loader.h"
#include "ns3/leosim-mobility-helper.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-routing-table-entry.h"
#include "ns3/ipv4-global-routing.h"
#include "ns3/ipv4-list-routing.h"
#include "ns3/ipv4-static-routing.h"
#include "ns3/arp-cache.h"
#include "ns3/ipv4-l3-protocol.h"

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include <iomanip>

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
    bool logPackets = true;
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
    cmd.AddValue("logPackets", "Enable packet logging", logPackets);
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
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);

    if (verbose)
    {
        LogComponentEnable("LeoSimTcpExample", LOG_LEVEL_INFO);
        LogComponentEnable("LeoSimChannelModel", LOG_LEVEL_INFO);
        LogComponentEnable("LeoSimChannelHelper", LOG_LEVEL_INFO);
        LogComponentEnable("LeoSimVisualizationHelper", LOG_LEVEL_DEBUG);
        LogComponentEnable("OnOffApplication", LOG_LEVEL_INFO);
        LogComponentEnable("PacketSink", LOG_LEVEL_INFO);
        LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
        LogComponentEnable("Ipv4GlobalRouting", LOG_LEVEL_INFO);
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

    std::cout << "Setting up multi-node topology" << std::endl;
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
            std::cout << "  Satellite " << i << ": " << loader->GetSatelliteName(i) << std::endl;
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
            std::cout << "  UE " << i << ": " << loader->GetGroundDeviceName(ueId) << std::endl;
        }
    }

    // Install mobility for all Servers
    for (uint32_t i = 0; i < numServers; i++)
    {
        uint32_t serverId = serverDeviceIds[i];
        mobilityHelper.InstallGateway(serverNodes.Get(i),
                                      serverId,
                                      loader->GetGroundDeviceName(serverId),
                                      loader->GetGroundDevicePosition(serverId));
        if (verbose)
        {
            std::cout << "  Server " << i << ": " << loader->GetGroundDeviceName(serverId)
                      << std::endl;
        }
    }

    mobilityHelper.StartAll();


    // Create visualization helper
    LeoSimVisualizationHelper vizHelper;
    vizHelper.SetOutputFile(positionFile);
    vizHelper.SetLinkFile(linkFile);
    vizHelper.SetPacketFile(packetFile);
    vizHelper.EnablePacketLogging(logPackets);
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
    
    channelModel->StartUpdates();

    // Create ISL mesh if enabled
    Ptr<LeoSimChannelModel> islChannelModel;
    std::map<Ptr<Node>, std::vector<Ipv4Address>> islAddresses;
    if (enableIsl)
    {
        std::cout << "\nCreating ISL mesh between satellites..." << std::endl;
        LeoSimChannelHelper islHelper;
        islHelper.SetIslFrequency(islFrequency);
        islHelper.SetIslMaxDistance(islMaxDistance);
        islHelper.SetIslTransmitPower(islTransmitPower);
        islHelper.SetIslAntennaGain(islAntennaGain);
        islHelper.SetUpdateInterval(Seconds(1.0));
        islHelper.SetVerbose(verbose);

        islChannelModel = islHelper.CreateIslMesh(satelliteNodes);
        islChannelModel->StartUpdates();

        std::cout << "ISL configuration:" << std::endl;
        std::cout << "  Frequency: " << islFrequency / 1e9 << " GHz" << std::endl;
        std::cout << "  Max Distance: " << islMaxDistance / 1000.0 << " km" << std::endl;
        std::cout << "  Tx Power: " << islTransmitPower << " dBm" << std::endl;
        std::cout << "  Antenna Gain: " << islAntennaGain << " dB" << std::endl;

        // Set ISL channel model for ISL link visualization
        vizHelper.SetIslChannelModel(islChannelModel);
        std::cout << "ISL visualization enabled" << std::endl;
    }

    // Set channel model in visualization helper for ground link tracking
    vizHelper.SetChannelModel(channelModel);

    // Schedule position and link logging
    vizHelper.SchedulePositionLogging(satelliteNodes, serverNodes, ueNodes, logInterval, simTime);

    // ========================================================================
    // Install packet logging hooks for visualization (after network setup complete)
    if (logPackets)
    {
        vizHelper.InstallPacketLogging();
        std::cout << "Packet logging activated" << std::endl;
    }

    // Run the simulation for the specified duration
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();

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
