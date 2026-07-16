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
#include "ns3/leosim-beam-layout-engine.h"
#include "ns3/leosim-beam-manager-helper.h"
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-channel.h"
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
#include "ns3/leosim-traffic-generator-helper.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/leosim-weather-helper.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/output-stream-wrapper.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <algorithm>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeoSimTcpExample");

namespace
{

class LeoSimTaskTimingLogger
{
  public:
    explicit LeoSimTaskTimingLogger(std::ostream* output)
        : m_output(output),
          m_active(false),
          m_startSimTimeSeconds(0.0)
    {
    }

    ~LeoSimTaskTimingLogger()
    {
        Finish();
    }

    void Start(const std::string& taskName)
    {
        Finish();
        if (m_output == nullptr)
        {
            return;
        }

        m_active = true;
        m_taskName = taskName;
        m_startWallTime = std::chrono::steady_clock::now();
        m_startSimTimeSeconds = Simulator::Now().GetSeconds();
    }

    void Finish()
    {
        if (!m_active || m_output == nullptr)
        {
            return;
        }

        const auto endWallTime = std::chrono::steady_clock::now();
        const double elapsedMs =
            std::chrono::duration<double, std::milli>(endWallTime - m_startWallTime).count();

        (*m_output) << m_taskName << "," << std::fixed << std::setprecision(6)
                    << m_startSimTimeSeconds << "," << Simulator::Now().GetSeconds() << ","
                    << elapsedMs << std::endl;
        m_active = false;
    }

  private:
    std::ostream* m_output;
    bool m_active;
    std::string m_taskName;
    std::chrono::steady_clock::time_point m_startWallTime;
    double m_startSimTimeSeconds;
};

struct LeoSimRuntimeKpiStats
{
    uint64_t totalSamples = 0;
    uint64_t connectedSamples = 0;
    double signalStrengthSumDbm = 0.0;
    double signalStrengthMinDbm = std::numeric_limits<double>::max();
    double signalStrengthMaxDbm = -std::numeric_limits<double>::max();
};

struct LeoSimProgressState
{
    std::chrono::steady_clock::time_point wallStart;
};

void
LogSimulationProgress(Time interval, Time stopTime, LeoSimProgressState* state)
{
    if (state == nullptr)
    {
        return;
    }

    const double wallSeconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - state->wallStart).count();
    std::cout << "[progress] sim=" << std::fixed << std::setprecision(1)
              << Simulator::Now().GetSeconds() << "/" << stopTime.GetSeconds()
              << "s wall=" << std::setprecision(1) << wallSeconds << "s" << std::endl;

    if (Simulator::Now() + interval <= stopTime)
    {
        Simulator::Schedule(interval, &LogSimulationProgress, interval, stopTime, state);
    }
}

void
SampleServingLinkMetrics(Ptr<LeoSimBeamManager> beamManager,
                         Ptr<LeoSimChannelModel> channelModel,
                         Ptr<LeoSimStatisticsHelper> statistics,
                         NodeContainer ueNodes,
                         Time sampleInterval,
                         Time stopTime,
                         LeoSimRuntimeKpiStats* kpiStats)
{
    LeoSimTaskProfiler::ScopedEvent profile("run_simulation.kpi_sampling");

    if (!beamManager || !channelModel || kpiStats == nullptr)
    {
        return;
    }

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        uint32_t ueNodeId = ueNodes.Get(i)->GetId();
        LeoSimBeamRecord servingBeam = beamManager->GetCurrentBeam(ueNodeId);

        kpiStats->totalSamples++;
        if (servingBeam.satelliteNodeId == std::numeric_limits<uint32_t>::max())
        {
            continue;
        }

        LeoSimChannelQuality quality =
            channelModel->GetLinkQuality(ueNodeId, servingBeam.satelliteNodeId);
        if (statistics)
        {
            statistics->RecordSnr(quality.snr);
        }
        if (quality.linkState == LEOSIM_LINK_DOWN)
        {
            continue;
        }

        kpiStats->connectedSamples++;
        kpiStats->signalStrengthSumDbm += quality.signalStrength;
        kpiStats->signalStrengthMinDbm =
            std::min(kpiStats->signalStrengthMinDbm, quality.signalStrength);
        kpiStats->signalStrengthMaxDbm =
            std::max(kpiStats->signalStrengthMaxDbm, quality.signalStrength);
    }

    if (Simulator::Now() + sampleInterval <= stopTime)
    {
        Simulator::Schedule(sampleInterval,
                            &SampleServingLinkMetrics,
                            beamManager,
                            channelModel,
                            statistics,
                            ueNodes,
                            sampleInterval,
                            stopTime,
                            kpiStats);
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string leosimDataDir = "contrib/leosim/data";
    std::string satelliteFile = leosimDataDir + "/prepro/satellite_mobility.tcl";
    std::string groundDeviceFile = "";
    double simTime = 60.0;
    double logInterval = 1.0;
    double minElevation = 10.0;
    double frequency = 12.0e9;
    std::string positionFile = "leosim_positions.csv";
    std::string packetFile = "leosim_packets.csv";
    std::string linkStateFile = "leosim_link_state.csv";
    std::string beamAssociationFile = "leosim_beam_associations.csv";
    std::string handoverFile = "leosim_handovers.csv";
    std::string choFile = "leosim_cho.csv";
    std::string coverageFile = "leosim_coverage.csv";
    std::string taskTimingFile = "leosim_task_times.csv";
    std::string statisticsFile = "leosim_statistics.json";
    std::string statisticsTimeSeriesFile = "leosim_statistics.csv";
    double statisticsIntervalSeconds = 1.0;
    bool logPackets = true;
    bool logBeams = true;
    uint32_t packetTraceNodeLimit = 150000;
    std::string flowMonitorScope = "endpoints";
    bool verbose = true;
    bool useTrace = true;
    bool enablePeriodicRouting = true;
    bool useExternalRouting = true;
    std::string externalRoutingEngine = "contrib/leosim/utils/rengine/leosim-rengine";
    std::string externalRoutingWorkDir = "/tmp/leosim-routing";
    uint32_t externalRoutingWorkers = 10;
    std::string externalRoutingMode = "destination-tree";
    std::string externalRoutingMetric = "distance";
    std::string routingEndpoints = "ground";
    uint64_t externalRoutingMaxRequests = 10000000ULL;
    bool destinationTreeAllNodes = true;
    double routingReactiveDebounceMs = 200.0;
    double routingUpdateInterval = 10.0;
    double pathPrintInterval = 10.0;
    double progressLogInterval = 1.0;
    uint32_t numSatellites = 30000;
    uint32_t numServers = 1000;
    uint32_t numUes = 1000000;
    bool enableIsl = true;
    bool islFullMesh = false;
    uint32_t maxIslNeighbors = 4;
    uint32_t islSatellitesPerPlane = 20;
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
    double beamSinrThresholdDb = -10.0;
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

    // === Phased Array Beam Steering Configuration ===
    bool enablePhasedArray = true;
    uint32_t phasedArrayElementsAz = 8;
    uint32_t phasedArrayElementsEl = 8;
    double phasedArrayElementSpacingLambda = 0.5;
    double phasedArrayOperatingFrequencyGhz = 20.0;
    double phasedArrayMaxSteeringAngleDeg = 60.0;
    double phasedArrayElementGainDbi = 5.0;
    double phasedArraySteeringIntervalMs = 100.0;
    double beamUpdateIntervalMs = 1000.0;
    uint32_t beamNumRings = 2;
    double beamRadiusKm = 100.0;
    uint32_t beamReuseColors = 3;
    double beamGeometryUpdateIntervalSeconds = 1.0;
    double kpiSampleIntervalSeconds = 0.5;

    // === Operator Sharing ===
    std::string satOperatorsFile = "";
    std::string sharingMatrixFile = "";
    double defaultAlpha = 1.0;
    bool operatorIsolation = false;

    // === Weather Model ===
    std::string weatherTrace = "";
    std::string weatherMarkov = "";
    std::string initialWeather = "CLEAR";
    double rainFadeThreshold = 10.0;
    double weatherHoThreshold = 15.0;
    bool enableWeather = false;

    CommandLine cmd;
    cmd.AddValue("satellites", "Path to satellite position file", satelliteFile);
    cmd.AddValue("dataDir", "Path to LeoSim data directory with gss/ and ues/ folders", leosimDataDir);
    cmd.AddValue("groundDevices", "Optional legacy ground device CSV file; overrides dataDir when set", groundDeviceFile);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("logInterval", "Interval for position/link logging (seconds)", logInterval);
    cmd.AddValue("minElevation", "Minimum elevation angle (degrees)", minElevation);
    cmd.AddValue("frequency", "Carrier frequency (Hz)", frequency);
    cmd.AddValue("positions", "Output file for position data", positionFile);
    cmd.AddValue("packets", "Output file for packet data", packetFile);
    cmd.AddValue("linkState",
                 "Unified output file for link, channel-quality, and beam-authority state",
                 linkStateFile);
    cmd.AddValue("beamAssociations",
                 "Output file for compact ground-to-serving-beam associations",
                 beamAssociationFile);
    cmd.AddValue("handovers", "Output file for handover events", handoverFile);
    cmd.AddValue("cho", "Output file for CHO candidate configuration", choFile);
    cmd.AddValue("coverage", "Output file for satellite ground coverage data", coverageFile);
    cmd.AddValue("taskTimingFile",
                 "CSV output for LeoSim complex task wall-clock timing; empty disables it",
                 taskTimingFile);
    cmd.AddValue("statisticsFile",
                 "JSON output for aggregate, satellite-link, handover, and per-flow KPIs; empty disables it",
                 statisticsFile);
    cmd.AddValue("statisticsTimeSeriesFile",
                 "CSV output for periodic cumulative KPI samples; empty disables it",
                 statisticsTimeSeriesFile);
    cmd.AddValue("statisticsInterval",
                 "Statistics CSV sampling interval in seconds",
                 statisticsIntervalSeconds);
    cmd.AddValue("logPackets", "Enable packet logging", logPackets);
    cmd.AddValue("logBeams", "Enable beam + handover logging", logBeams);
    cmd.AddValue("packetTraceNodeLimit",
                 "Maximum nodes for packet tracing; 0 disables the limit",
                 packetTraceNodeLimit);
    cmd.AddValue("flowMonitorScope",
                 "FlowMonitor installation scope: endpoints, all, or off",
                 flowMonitorScope);
    cmd.AddValue("verbose", "Enable verbose logging", verbose);
    cmd.AddValue("useTrace", "Use ns-3 trace file format for satellites", useTrace);
    cmd.AddValue("enablePeriodicRouting", "Enable periodic routing updates", enablePeriodicRouting);
    cmd.AddValue("useExternalRouting",
                 "Use process-parallel LeoSimExternalRoutingHelper instead of in-process Dijkstra",
                 useExternalRouting);
    cmd.AddValue("externalRoutingEngine",
                 "Path to contrib/leosim/utils/rengine/leosim-rengine executable",
                 externalRoutingEngine);
    cmd.AddValue("externalRoutingWorkDir",
                 "Directory for external routing graph/request/result snapshots",
                 externalRoutingWorkDir);
    cmd.AddValue("externalRoutingWorkers",
                 "Number of external routing worker processes",
                 externalRoutingWorkers);
    cmd.AddValue("externalRoutingMode",
                 "External routing mode: destination-tree or pair",
                 externalRoutingMode);
    cmd.AddValue("externalRoutingMetric",
                 "External routing metric: distance or hop",
                 externalRoutingMetric);
    cmd.AddValue("routingEndpoints",
                 "Route destinations: ground (UEs and gateways), gateways, ues, or all",
                 routingEndpoints);
    cmd.AddValue("externalRoutingMaxRequests",
                 "Maximum route records materialized per external-routing snapshot",
                 externalRoutingMaxRequests);
    cmd.AddValue("destinationTreeAllNodes",
                 "Generate destination-tree next hops for every graph/transit node",
                 destinationTreeAllNodes);
    cmd.AddValue("routingReactiveDebounceMs",
                 "Debounce interval for handover-triggered routing refreshes (milliseconds)",
                 routingReactiveDebounceMs);
    cmd.AddValue("routingUpdateInterval",
                 "Routing update interval (seconds)",
                 routingUpdateInterval);
    cmd.AddValue("pathPrintInterval",
                 "Interval for printing UE->Server node-id paths (seconds)",
                 pathPrintInterval);
    cmd.AddValue("progressLogInterval",
                 "Simulation progress log interval in seconds; 0 disables it",
                 progressLogInterval);
    cmd.AddValue("numSatellites", "Number of satellites to use from the file", numSatellites);
    cmd.AddValue("numServers", "Number of server ground stations to use; 0 means all loaded servers", numServers);
    cmd.AddValue("numUes", "Number of UEs to use", numUes);
    cmd.AddValue("enableIsl", "Enable Inter-Satellite Links (ISL)", enableIsl);
    cmd.AddValue("islFullMesh", "Create all-pairs ISL mesh; not recommended for large constellations", islFullMesh);
    cmd.AddValue("maxIslNeighbors", "Maximum spatial ISL degree per satellite", maxIslNeighbors);
    cmd.AddValue("islSatellitesPerPlane",
                 "Satellites per orbital plane for bounded ISL grid topology",
                 islSatellitesPerPlane);
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
    cmd.AddValue("beamSinrThreshold",
                 "Minimum beam SINR outage floor in dB required for serving-beam eligibility",
                 beamSinrThresholdDb);
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

    // === Phased Array Parameters ===
    cmd.AddValue("enablePhasedArray",
                 "Enable phased-array multi-beam steering on satellites",
                 enablePhasedArray);
    cmd.AddValue("paElementsAz",
                 "Phased-array azimuth element count",
                 phasedArrayElementsAz);
    cmd.AddValue("paElementsEl",
                 "Phased-array elevation element count",
                 phasedArrayElementsEl);
    cmd.AddValue("paSpacingLambda",
                 "Phased-array element spacing (d/lambda)",
                 phasedArrayElementSpacingLambda);
    cmd.AddValue("paFrequencyGhz",
                 "Phased-array operating frequency (GHz)",
                 phasedArrayOperatingFrequencyGhz);
    cmd.AddValue("paMaxSteeringDeg",
                 "Maximum electronic steering angle from nadir (degrees)",
                 phasedArrayMaxSteeringAngleDeg);
    cmd.AddValue("paElementGainDbi",
                 "Single element gain (dBi)",
                 phasedArrayElementGainDbi);
    cmd.AddValue("paSteeringIntervalMs",
                 "Phased-array steering update interval (milliseconds)",
                 phasedArraySteeringIntervalMs);
    cmd.AddValue("beamUpdateIntervalMs",
                 "Beam-manager decision cycle interval (milliseconds)",
                 beamUpdateIntervalMs);
    cmd.AddValue("beamNumRings",
                 "Hex-beam layout ring count for real geometric coverage",
                 beamNumRings);
    cmd.AddValue("beamRadiusKm",
                 "Beam footprint radius in kilometers for geometric coverage",
                 beamRadiusKm);
    cmd.AddValue("beamReuseColors",
                 "Frequency reuse color groups for geometric beam layout",
                 beamReuseColors);
    cmd.AddValue("beamGeometryUpdateInterval",
                 "Beam center geometry refresh interval in seconds",
                 beamGeometryUpdateIntervalSeconds);
    cmd.AddValue("kpiSampleInterval",
                 "Runtime KPI sampling interval for signal/stability (seconds)",
                 kpiSampleIntervalSeconds);

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

    cmd.AddValue("weatherTrace", "Per-node weather time series CSV", weatherTrace);
    cmd.AddValue("weatherMarkov", "Markov transition matrix CSV", weatherMarkov);
    cmd.AddValue("initialWeather",
                 "Initial weather state for all nodes: CLEAR|CLOUDY|LIGHT_RAIN|HEAVY_RAIN",
                 initialWeather);
    cmd.AddValue("rainFadeThreshold",
                 "Rain fade degradation threshold (dB)",
                 rainFadeThreshold);
    cmd.AddValue("weatherHoThresh",
                 "Weather fade handover trigger threshold (dB)",
                 weatherHoThreshold);
    cmd.AddValue("enableWeather",
                 "Enable atmospheric weather attenuation model",
                 enableWeather);
    cmd.Parse(argc, argv);

    Time::SetResolution(Time::NS);

    std::ofstream taskTimingStream;
    if (!taskTimingFile.empty())
    {
        taskTimingStream.open(taskTimingFile);
        if (taskTimingStream.is_open())
        {
            taskTimingStream
                << "task,sim_time_start_s,sim_time_end_s,wall_time_elapsed_ms" << std::endl;
        }
        else
        {
            std::cerr << "Warning: Could not open task timing file: " << taskTimingFile
                      << std::endl;
        }
    }
    LeoSimTaskTimingLogger taskTimer(taskTimingStream.is_open() ? &taskTimingStream : nullptr);
    LeoSimTaskProfiler::SetOutputStream(taskTimingStream.is_open() ? &taskTimingStream : nullptr);

    if (verbose)
    {
        // LogComponentEnable("LeoSimTcpExample", LOG_LEVEL_INFO);

        LogComponentEnable("LeoSimBeamManager", LOG_LEVEL_INFO);
        LogComponentEnable("LeoSimBeamManagerHelper", LOG_LEVEL_INFO);

        // LogComponentEnable("LeoSimChannelModel", LOG_LEVEL_INFO);
        // LogComponentEnable("LeoSimChannelHelper", LOG_LEVEL_INFO);
        // LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
        // LogComponentEnable("Ipv4GlobalRouting", LOG_LEVEL_INFO);
        // LogComponentEnable("LeoSimRoutingCalculatorHelper", LOG_LEVEL_INFO);
    }

    taskTimer.Start("load_input_data");

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
    loader->SetSatellitesPerPlane(islSatellitesPerPlane);
    auto serverDeviceIds = loader->GetGroundDeviceIdsByType("SERVER");
    auto ueDeviceIds = loader->GetGroundDeviceIdsByType("UE");
    taskTimer.Finish();

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

    if (numServers == 0)
    {
        numServers = serverDeviceIds.size();
    }
    else if (numServers > serverDeviceIds.size())
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

    taskTimer.Start("create_nodes_and_install_mobility");

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
    }

    // Install mobility for all UEs
    for (uint32_t i = 0; i < numUes; i++)
    {
        uint32_t ueId = ueDeviceIds[i];
        mobilityHelper.InstallUE(ueNodes.Get(i),
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
    taskTimer.Finish();

    // === Operator Model Setup (Phase 8 of LeoSim) ===
    taskTimer.Start("setup_operator_model");
    if (!satOperatorsFile.empty())
    {
        loader->LoadSatelliteOperatorsFromCsv(satOperatorsFile);
    }
    else
    {
        uint32_t loadedSatelliteOperators =
            loader->LoadSatelliteOperatorsFromDataDirectory(leosimDataDir);
        if (loadedSatelliteOperators != loader->GetNumSatellites())
        {
            std::cerr << "Warning: Loaded " << loader->GetNumSatellites()
                      << " satellites from the mobility trace, but found "
                      << loadedSatelliteOperators << " satellite operator assignments in "
                      << leosimDataDir
                      << "/tles. Regenerate the mobility trace from the same TLE directory."
                      << std::endl;
        }
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
    taskTimer.Finish();

    // Create visualization helper
    taskTimer.Start("initialize_visualization_logging");
    LeoSimVisualizationHelper vizHelper;
    vizHelper.SetOutputFile(positionFile);
    vizHelper.SetPacketFile(packetFile);
    vizHelper.SetUnifiedLinkStateFile(linkStateFile);
    vizHelper.SetBeamAssociationFile(beamAssociationFile);
    vizHelper.SetHandoverFile(handoverFile);
    vizHelper.SetChoFile(choFile);
    vizHelper.SetCoverageFile(coverageFile);
    vizHelper.EnablePacketLogging(logPackets);
    vizHelper.EnableBeamLogging(logBeams);
    vizHelper.SetLoaderHelper(loaderHelper);
    vizHelper.Initialize();
    taskTimer.Finish();

    // Create dynamic channel model for logging/visualization
    taskTimer.Start("create_ground_channel_model");
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

    // Optional weather model wiring
    Ptr<LeoSimWeatherModel> weatherModel;
    if (enableWeather)
    {
        LeoSimWeatherHelper weatherHelper;
        weatherHelper.SetVerbose(verbose);
        weatherHelper.SetFrequency(frequency);
        weatherHelper.SetGroundStationHeight(0.0);
        weatherHelper.SetRainFadeThreshold(rainFadeThreshold);
        weatherHelper.SetSnrFloor(-5.0);
        weatherHelper.SetWeatherFadeHoThreshold(weatherHoThreshold);

        if (!weatherTrace.empty())
        {
            weatherHelper.LoadWeatherTrace(weatherTrace);
        }
        if (!weatherMarkov.empty())
        {
            weatherHelper.SetMarkovMatrix(weatherMarkov);
        }

        if (initialWeather == "CLEAR")
        {
            weatherHelper.SetUniformWeather(LEOSIM_WX_CLEAR, 0.0, 0.0);
        }
        else if (initialWeather == "CLOUDY")
        {
            weatherHelper.SetUniformWeather(LEOSIM_WX_CLOUDY, 0.0, 0.3);
        }
        else if (initialWeather == "LIGHT_RAIN")
        {
            weatherHelper.SetUniformWeather(LEOSIM_WX_LIGHT_RAIN, 2.0, 0.3);
        }
        else if (initialWeather == "HEAVY_RAIN")
        {
            weatherHelper.SetUniformWeather(LEOSIM_WX_HEAVY_RAIN, 25.0, 1.0);
        }
        else
        {
            NS_LOG_WARN("Unknown initialWeather='" << initialWeather
                                                    << "', defaulting to CLEAR");
            weatherHelper.SetUniformWeather(LEOSIM_WX_CLEAR, 0.0, 0.0);
        }

        weatherModel = weatherHelper.Install(allGroundNodes, Seconds(simTime));
        channelModel->SetWeatherModel(weatherModel);
        channelModel->SetRainFadeThresholdDb(rainFadeThreshold);
        channelModel->SetSnrFloorDb(-5.0);
    }
    taskTimer.Finish();

    // Create ISL mesh if enabled
    taskTimer.Start("create_isl_channel_model");
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

        if (islFullMesh)
        {
            islChannelModel = islHelper.CreateIslMesh(satelliteNodes);
        }
        else
        {
            islChannelModel =
                islHelper.CreateIslNearestNeighborMesh(satelliteNodes, maxIslNeighbors);
        }
        islChannelModel->SetOperatorModel(operatorModel);

        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] ISL configuration:" << std::endl;
        std::cout << "  Frequency: " << islFrequency / 1e9 << " GHz" << std::endl;
        std::cout << "  Max Distance: " << islMaxDistance / 1000.0 << " km" << std::endl;
        std::cout << "  Topology: "
                  << (islFullMesh ? "full mesh"
                                  : "bounded spatial nearest-neighbor graph")
                  << std::endl;
        if (!islFullMesh)
        {
            std::cout << "  ISL Degree Bound: " << maxIslNeighbors
                      << " links per satellite" << std::endl;
        }
        std::cout << "  Tx Power: " << islTransmitPower << " dBm" << std::endl;
        std::cout << "  Antenna Gain: " << islAntennaGain << " dB" << std::endl;

        // Set ISL channel model for ISL link visualization
        vizHelper.SetIslChannelModel(islChannelModel);
        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] ISL visualization enabled" << std::endl;
    }
    taskTimer.Finish();

    // Set channel model in visualization helper for ground link tracking
    vizHelper.SetChannelModel(channelModel);

    // Install network devices on all nodes based on channel model links
    taskTimer.Start("install_network_devices");
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
    taskTimer.Finish();

    //  Install Internet stack on all nodes
    taskTimer.Start("install_internet_stack_and_assign_addresses");
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Installing Internet stack on all nodes..." << std::endl;
    InternetStackHelper stack;
    stack.Install(satelliteNodes);
    stack.Install(ueNodes);
    stack.Install(serverNodes);
    std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Internet stack installed" << std::endl;

    // Assign IP addresses - one /30 subnet per point-to-point link.
    // Ground links and ISLs use disjoint address blocks to avoid collisions even
    // when the number of ground links grows with all loaded servers/UEs.
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Assigning IP addresses (unique per-link subnets)..." << std::endl;

    Ipv4AddressHelper groundIpv4;
    groundIpv4.SetBase(Ipv4Address("10.0.0.0"), Ipv4Mask("255.255.255.252"));

    Ipv4AddressHelper islIpv4;
    islIpv4.SetBase(Ipv4Address("10.128.0.0"), Ipv4Mask("255.255.255.252"));

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

        groundIpv4.Assign(linkDevices);
        groundIpv4.NewNetwork();

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

            islIpv4.Assign(linkDevices);

            islIpv4.NewNetwork();
        }
    }

    std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] Address assignment complete: " << assignedLinks << " ground links, "
              << (enableIsl ? (islDevices.GetN() / 2) : 0) << " ISL links" << std::endl;
    taskTimer.Finish();

    // Combine all nodes for routing
    taskTimer.Start("prepare_routing_calculator");
    NodeContainer allNodes;
    allNodes.Add(satelliteNodes);
    allNodes.Add(ueNodes);
    allNodes.Add(serverNodes);

    NodeContainer routingDestinationNodes;
    if (routingEndpoints == "ground")
    {
        routingDestinationNodes.Add(ueNodes);
        routingDestinationNodes.Add(serverNodes);
    }
    else if (routingEndpoints == "gateways")
    {
        routingDestinationNodes.Add(serverNodes);
    }
    else if (routingEndpoints == "ues")
    {
        routingDestinationNodes.Add(ueNodes);
    }
    else if (routingEndpoints == "all")
    {
        routingDestinationNodes.Add(allNodes);
    }
    else
    {
        std::cerr << "Error: routingEndpoints must be ground, gateways, ues, or all"
                  << std::endl;
        return 1;
    }
    // Application endpoints are explicit sources. In destination-tree mode
    // the helper expands these to every registered graph node when
    // destinationTreeAllNodes is enabled, ensuring transit forwarding state.
    NodeContainer externalRoutingSourceNodes;
    externalRoutingSourceNodes.Add(ueNodes);
    externalRoutingSourceNodes.Add(serverNodes);

    vizHelper.SetOperatorFile("leosim_operators.csv");
    vizHelper.SetSharingFile("leosim_sharing.csv");
    vizHelper.InitOperatorLogging(operatorModel, allNodes);

    // Prepare the routing calculator before beam-manager installation, but defer route
    // installation until the beam manager is attached as the access-link authority.
    std::cout << "\n[t=" << Simulator::Now().GetSeconds()
              << "s] Preparing unified routing calculator..." << std::endl;
    LeoSimRoutingCalculatorHelper routingHelper;
    LeoSimExternalRoutingHelper externalRoutingHelper;

    // Create unified routing calculator that handles both ground and ISL links
    Ptr<LeoSimRoutingCalculator> unifiedCalc =
        routingHelper.CreateUnifiedRoutingCalculator(channelModel, islChannelModel, verbose);
    unifiedCalc->SetOperatorModel(operatorModel);
    if (operatorIsolation)
    {
        unifiedCalc->SetPathType(LeoSimRoutingCalculator::LEOSIM_PATH_SAME_OPERATOR_ONLY);
    }
    taskTimer.Finish();

    // === Beam Management & Handover (3GPP NTN CHO) ===
    taskTimer.Start("setup_beam_manager_and_handover");
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
    beamHelper.SetSinrThreshold(beamSinrThresholdDb);

    // Set CHO timing (convert from milliseconds to seconds)
    beamHelper.SetChoPreparationDelay(Seconds(choPrep / 1000.0));
    beamHelper.SetChoExecutionDelay(Seconds(choExec / 1000.0));

    // Set TOPSIS prioritization weights (must sum to 1.0)
    beamHelper.SetTopsisWeights(wRsrp, wSinr, wTte, wLoad, wLatency, wElevation, wActive);
    beamHelper.SetMaxCandidates(maxCandidates);
    beamHelper.SetUpdateInterval(MilliSeconds(beamUpdateIntervalMs));

    // Set features
    beamHelper.EnableLoadBalancing(enableLoadBalancing);
    beamHelper.EnableHandoverBuffering(enableHoBuffering);

    Ptr<LeoSimMultiBeamModel> multiBeamModel;
    if (enablePhasedArray)
    {
        multiBeamModel = CreateObject<LeoSimMultiBeamModel>();
        LeoSimPhasedArrayConfig paCfg;
        paCfg.numElementsAz = phasedArrayElementsAz;
        paCfg.numElementsEl = phasedArrayElementsEl;
        paCfg.elementSpacingLambda = phasedArrayElementSpacingLambda;
        paCfg.operatingFrequencyGHz = phasedArrayOperatingFrequencyGhz;
        paCfg.maxSteeringAngleDeg = phasedArrayMaxSteeringAngleDeg;
        paCfg.elementGainDbi = phasedArrayElementGainDbi;
        multiBeamModel->SetPhasedArrayConfig(paCfg);

        // Populate real per-satellite beam geometry from current satellite positions.
        uint32_t totalInitializedBeams = 0;
        for (uint32_t i = 0; i < satelliteNodes.GetN(); ++i)
        {
            Ptr<Node> sat = satelliteNodes.Get(i);
            if (!sat)
            {
                continue;
            }

            Ptr<MobilityModel> mobility = sat->GetObject<MobilityModel>();
            if (!mobility)
            {
                continue;
            }

            uint32_t satId = sat->GetId();
            uint32_t cellIdBase = satId * 1000;
            auto beams = LeoSimBeamLayoutEngine::GenerateHexLayout(satId,
                                                                    mobility->GetPosition(),
                                                                    std::max<uint32_t>(1, beamNumRings),
                                                                    beamRadiusKm,
                                                                    std::max<uint32_t>(1, beamReuseColors),
                                                                    cellIdBase);
            totalInitializedBeams += beams.size();
            multiBeamModel->SetBeamsForSatellite(satId, beams);
        }

        multiBeamModel->UpdateGeometry(satelliteNodes, Simulator::Now());

        beamHelper.SetMultiBeamModel(multiBeamModel);
        beamHelper.SetPhasedArraySteeringInterval(MilliSeconds(phasedArraySteeringIntervalMs));
        beamHelper.SetBeamGeometryUpdateInterval(Seconds(beamGeometryUpdateIntervalSeconds));
        vizHelper.SetMultiBeamModel(multiBeamModel);

        if (verbose)
        {
            std::cout << "  Initialized geometric beams: " << totalInitializedBeams
                      << " (rings=" << beamNumRings << ", radius=" << beamRadiusKm
                      << " km, reuseColors=" << beamReuseColors << ")" << std::endl;
        }
    }

    // Install beam manager on all ground nodes (UEs + servers)
    Ptr<LeoSimBeamManager> beamManager =
        beamHelper.Install(allGroundNodes, satelliteNodes, Seconds(simTime));
    beamManager->SetOperatorModel(operatorModel);
    unifiedCalc->SetBeamManager(beamManager);
    if (enableWeather && weatherModel)
    {
        beamManager->SetWeatherModel(weatherModel);
        beamManager->SetWeatherFadeThresholdDb(weatherHoThreshold);
    }

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
        std::cout << "  Phased-array steering: " << (enablePhasedArray ? "enabled" : "disabled")
                  << std::endl;
        if (enablePhasedArray)
        {
            std::cout << "  Array elements (Az x El): " << phasedArrayElementsAz << " x "
                      << phasedArrayElementsEl << std::endl;
            std::cout << "  Element spacing (d/lambda): " << phasedArrayElementSpacingLambda
                      << std::endl;
            std::cout << "  Max steering angle: " << phasedArrayMaxSteeringAngleDeg << " deg"
                      << std::endl;
            std::cout << "  Steering interval: " << phasedArraySteeringIntervalMs << " ms"
                      << std::endl;
        }
    }
    taskTimer.Finish();

    // Install computed routes only after the beam manager is active, so satellite-ground
    // access links must pass serving/prepared-beam authority from the first packet onward.
    taskTimer.Start("setup_dynamic_routing");
    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Setting up "
              << (useExternalRouting ? "external process-parallel " : "")
              << (enablePeriodicRouting ? "dynamic routing with periodic updates"
                                         : "static routing snapshot")
              << "..." << std::endl;
    if (useExternalRouting)
    {
        externalRoutingHelper.SetEnginePath(externalRoutingEngine);
        externalRoutingHelper.SetWorkingDirectory(externalRoutingWorkDir);
        externalRoutingHelper.SetWorkerCount(externalRoutingWorkers);
        externalRoutingHelper.SetMode(
            externalRoutingMode == "pair"
                ? LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_PAIR
                : LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_DESTINATION_TREE);
        externalRoutingHelper.SetMetric(
            externalRoutingMetric == "hop"
                ? LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_HOP_COUNT
                : LeoSimExternalRoutingHelper::LEOSIM_EXTERNAL_WEIGHT_DISTANCE);
        externalRoutingHelper.SetMaxRouteRequests(externalRoutingMaxRequests);
        externalRoutingHelper.SetDestinationTreeAllNodes(destinationTreeAllNodes);

        if (enablePeriodicRouting)
        {
            externalRoutingHelper.EnableDynamicRouting(unifiedCalc,
                                                       externalRoutingSourceNodes,
                                                       routingDestinationNodes,
                                                       Seconds(routingUpdateInterval),
                                                       simTime,
                                                       verbose);
            if (verbose)
            {
                std::cout << "[t=" << Simulator::Now().GetSeconds()
                          << "s] External dynamic routing enabled: engine="
                          << externalRoutingEngine
                          << ", workers=" << externalRoutingWorkers
                          << ", interval=" << routingUpdateInterval << " seconds"
                          << std::endl;
            }
        }
        else
        {
            externalRoutingHelper.SetStaticRoutes(unifiedCalc,
                                                  externalRoutingSourceNodes,
                                                  routingDestinationNodes,
                                                  verbose);
            if (verbose)
            {
                std::cout << "[t=" << Simulator::Now().GetSeconds()
                          << "s] External static routing snapshot installed" << std::endl;
            }
        }

        externalRoutingHelper.EnableReactiveRouteRefresh(unifiedCalc,
                                                         externalRoutingSourceNodes,
                                                         routingDestinationNodes,
                                                         MilliSeconds(routingReactiveDebounceMs),
                                                         verbose);
        beamManager->SetAccessStateChangeCallback(
            MakeCallback(&LeoSimExternalRoutingHelper::RequestRouteRefresh,
                         &externalRoutingHelper));
    }
    else if (enablePeriodicRouting)
    {
        // Continuously recalculate routes based on changing topology.
        routingHelper.EnableDynamicRouting(unifiedCalc,
                                           allNodes,
                                           routingDestinationNodes,
                                           Seconds(routingUpdateInterval),
                                           simTime,
                                           verbose);

        if (verbose)
        {
            std::cout << "[t=" << Simulator::Now().GetSeconds()
                      << "s] Dynamic routing enabled with update interval: "
                      << routingUpdateInterval << " seconds" << std::endl;
        }
    }
    else
    {
        routingHelper.SetStaticRoutes(unifiedCalc,
                                      allNodes,
                                      routingDestinationNodes,
                                      verbose);
        if (verbose)
        {
            std::cout << "[t=" << Simulator::Now().GetSeconds()
                      << "s] Static routing snapshot installed" << std::endl;
        }
    }

    if (!useExternalRouting)
    {
        routingHelper.EnableReactiveLinkTriggeredRouting(unifiedCalc,
                                                         allNodes,
                                                         routingDestinationNodes,
                                                         channelModel,
                                                         enableIsl ? islChannelModel : nullptr,
                                                         MilliSeconds(routingReactiveDebounceMs),
                                                         verbose);
        beamManager->SetAccessStateChangeCallback(
            MakeCallback(&LeoSimRoutingCalculatorHelper::RequestRouteRefresh, &routingHelper));
    }
    taskTimer.Finish();

    taskTimer.Start("schedule_kpi_and_visualization_logging");
    Ptr<LeoSimStatisticsHelper> statistics = CreateObject<LeoSimStatisticsHelper>();
    statistics->SetBeamManager(beamManager);
    if (!statistics->AttachChannelModel(channelModel))
    {
        std::cerr << "Warning: failed to attach statistics to access channel traces" << std::endl;
    }
    if (enableIsl && islChannelModel && !statistics->AttachChannelModel(islChannelModel))
    {
        std::cerr << "Warning: failed to attach statistics to ISL channel traces" << std::endl;
    }
    LeoSimRuntimeKpiStats runtimeKpiStats;
    Simulator::Schedule(Seconds(0.0),
                        &SampleServingLinkMetrics,
                        beamManager,
                        channelModel,
                        statistics,
                        ueNodes,
                        Seconds(kpiSampleIntervalSeconds),
                        Seconds(simTime),
                        &runtimeKpiStats);

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
    taskTimer.Finish();

    // Install TCP traffic from UEs to server
    taskTimer.Start("install_traffic_and_packet_logging");
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
        const uint64_t traceNodeCount = static_cast<uint64_t>(satelliteNodes.GetN()) +
                                        serverNodes.GetN() + ueNodes.GetN();
        if (packetTraceNodeLimit > 0 && traceNodeCount > packetTraceNodeLimit)
        {
            std::cout << "[t=" << Simulator::Now().GetSeconds()
                      << "s] Packet logging skipped: " << traceNodeCount
                      << " nodes exceeds --packetTraceNodeLimit=" << packetTraceNodeLimit
                      << ". Use --packetTraceNodeLimit=0 to force tracing." << std::endl;
        }
        else
        {
            vizHelper.InstallPacketLogging(satelliteNodes, serverNodes, ueNodes);
            std::cout << "[t=" << Simulator::Now().GetSeconds()
                      << "s] Packet logging activated" << std::endl;
        }
    }
    taskTimer.Finish();

    // === Add Flow Monitor for Packet Loss Analysis ===
    taskTimer.Start("install_flow_monitor");
    Ptr<FlowMonitor> flowMonitor;
    FlowMonitorHelper flowmonHelper;
    if (flowMonitorScope == "endpoints")
    {
        flowMonitor = flowmonHelper.Install(externalRoutingSourceNodes);
        std::cout << "[t=" << Simulator::Now().GetSeconds()
                  << "s] FlowMonitor installed on " << externalRoutingSourceNodes.GetN()
                  << " traffic endpoints" << std::endl;
    }
    else if (flowMonitorScope == "all")
    {
        flowMonitor = flowmonHelper.InstallAll();
        std::cout << "[t=" << Simulator::Now().GetSeconds()
                  << "s] FlowMonitor installed on all " << allNodes.GetN() << " nodes"
                  << std::endl;
    }
    else if (flowMonitorScope == "off")
    {
        std::cout << "[t=" << Simulator::Now().GetSeconds() << "s] FlowMonitor disabled"
                  << std::endl;
    }
    else
    {
        std::cerr << "Error: flowMonitorScope must be endpoints, all, or off" << std::endl;
        return 1;
    }
    if (flowMonitor)
    {
        statistics->SetFlowMonitor(
            flowMonitor,
            DynamicCast<Ipv4FlowClassifier>(flowmonHelper.GetClassifier()));
    }
    if (!statisticsTimeSeriesFile.empty())
    {
        if (statisticsIntervalSeconds <= 0.0)
        {
            std::cerr << "Error: statisticsInterval must be greater than zero" << std::endl;
            return 1;
        }
        statistics->StartPeriodicSampling(Seconds(statisticsIntervalSeconds),
                                          statisticsTimeSeriesFile);
    }
    taskTimer.Finish();

    // Run the simulation for the specified duration
    taskTimer.Start("run_simulation");
    LeoSimProgressState progressState{std::chrono::steady_clock::now()};
    if (progressLogInterval > 0.0 && progressLogInterval <= simTime)
    {
        Simulator::Schedule(Seconds(progressLogInterval),
                            &LogSimulationProgress,
                            Seconds(progressLogInterval),
                            Seconds(simTime),
                            &progressState);
    }
    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    taskTimer.Finish();

    // Consolidate FlowMonitor and satellite-specific telemetry.
    taskTimer.Start("postprocess_results_and_write_summary");
    statistics->StopPeriodicSampling();
    const LeoSimStatisticsSnapshot networkStats = statistics->GetSnapshot(true);
    const std::vector<LeoSimFlowStatistics> flowStats = statistics->GetFlowStatistics();

    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] === PACKET LOSS ANALYSIS ===" << std::endl;
    if (!flowMonitor)
    {
        std::cout << "  Disabled (--flowMonitorScope=off)" << std::endl;
    }
    for (const auto& flow : flowStats)
    {
        std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] Flow " << flow.flowId
                  << " (" << flow.sourceAddress << ':' << flow.sourcePort << " -> "
                  << flow.destinationAddress << ':' << flow.destinationPort << ")" << std::endl;
        std::cout << "  Tx/Rx/Lost Packets: " << flow.txPackets << '/' << flow.rxPackets << '/'
                  << flow.lostPackets << std::endl;
        std::cout << "  Delivery Ratio: " << (100.0 * flow.packetDeliveryRatio) << "%" << std::endl;
        std::cout << "  Throughput: " << flow.throughputMbps << " Mbps" << std::endl;
        std::cout << "  Mean Delay/Jitter: " << flow.meanDelayMs << '/' << flow.meanJitterMs
                  << " ms" << std::endl;
        std::cout << "  Mean Hop Count: " << flow.meanHopCount << std::endl;
    }
    std::cout << "=======================================" << std::endl;

    const double avgSignalStrengthDbm =
        runtimeKpiStats.connectedSamples > 0
            ? (runtimeKpiStats.signalStrengthSumDbm /
               static_cast<double>(runtimeKpiStats.connectedSamples))
            : 0.0;
    const double connectedRatioPct =
        runtimeKpiStats.totalSamples > 0
            ? (100.0 * static_cast<double>(runtimeKpiStats.connectedSamples) /
               static_cast<double>(runtimeKpiStats.totalSamples))
            : 0.0;
    const double handoverSuccessPct = 100.0 * networkStats.handoverSuccessRatio;
    const double packetDeliveryPct = 100.0 * networkStats.packetDeliveryRatio;

    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] === PHASED-ARRAY KPI SUMMARY ==="
              << std::endl;
    std::cout << "Signal strength (serving-link RSS):" << std::endl;
    std::cout << "  Samples: " << runtimeKpiStats.connectedSamples << "/"
              << runtimeKpiStats.totalSamples << std::endl;
    if (runtimeKpiStats.connectedSamples > 0)
    {
        std::cout << "  Avg RSS: " << avgSignalStrengthDbm << " dBm" << std::endl;
        std::cout << "  Min RSS: " << runtimeKpiStats.signalStrengthMinDbm << " dBm"
                  << std::endl;
        std::cout << "  Max RSS: " << runtimeKpiStats.signalStrengthMaxDbm << " dBm"
                  << std::endl;
    }
    else
    {
        std::cout << "  No connected serving-link samples captured." << std::endl;
    }

    std::cout << "Beam switching delay (handover latency):" << std::endl;
    std::cout << "  Handover events: " << networkStats.handovers << std::endl;
    std::cout << "  Avg delay: " << networkStats.meanHandoverLatencyMs << " ms" << std::endl;
    std::cout << "  P95 delay: " << networkStats.p95HandoverLatencyMs << " ms" << std::endl;

    std::cout << "Connection stability:" << std::endl;
    std::cout << "  Connected sample ratio: " << connectedRatioPct << "%" << std::endl;
    std::cout << "  Handover success rate: " << handoverSuccessPct << "%" << std::endl;
    std::cout << "  Ping-pong count: " << networkStats.pingPongs << std::endl;
    std::cout << "  Packet delivery ratio: " << packetDeliveryPct << "%" << std::endl;
    std::cout << "  Aggregate throughput: " << networkStats.throughputMbps << " Mbps" << std::endl;
    std::cout << "  Mean delay/jitter: " << networkStats.meanDelayMs << '/'
              << networkStats.meanJitterMs << " ms" << std::endl;
    std::cout << "Satellite channel:" << std::endl;
    std::cout << "  SNR samples/mean/min/max: " << networkStats.snrDb.count << '/'
              << networkStats.snrDb.mean << '/' << networkStats.snrDb.min << '/'
              << networkStats.snrDb.max << " dB" << std::endl;
    std::cout << "  Mean path loss: " << networkStats.pathLossDb.mean << " dB" << std::endl;
    std::cout << "  Link up/down/degraded events: " << networkStats.linkUpEvents << '/'
              << networkStats.linkDownEvents << '/' << networkStats.linkDegradedEvents << std::endl;
    std::cout << "==========================================" << std::endl;

    if (!statisticsFile.empty())
    {
        statistics->WriteSummary(statisticsFile);
    }

    std::cout << "\n[t=" << Simulator::Now().GetSeconds() << "s] === LeoSim Visualization Outputs ===" << std::endl;
    std::cout << "positions:  " << positionFile << std::endl;
    std::cout << "linkState:  " << linkStateFile << std::endl;
    std::cout << "coverage:   " << coverageFile << std::endl;
    std::cout << "statistics: "
              << (statisticsFile.empty() ? "(disabled)" : statisticsFile) << std::endl;
    std::cout << "statSeries: "
              << (statisticsTimeSeriesFile.empty() ? "(disabled)" : statisticsTimeSeriesFile)
              << std::endl;
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
        std::cout << "assoc:      " << beamAssociationFile << std::endl;
        std::cout << "handovers:  " << handoverFile << std::endl;
        std::cout << "cho:        " << choFile << std::endl;
    }
    else
    {
        std::cout << "beams/handovers/cho: (disabled)" << std::endl;
    }
    std::cout << "\nTo visualize (from ns3/ directory):" << std::endl;
    std::cout << "  python contrib/leosim/utils/visualize_3d.py \\\n  --position_file "
              << positionFile << " \\\n  --links " << linkStateFile;
    if (logPackets)
    {
        std::cout << " \\\n  --packets " << packetFile;
    }
    if (logBeams)
    {
        std::cout << " \\\n  --beams " << beamAssociationFile << " \\\n  --handovers " << handoverFile;
    }
    std::cout << " \\\n  --output visualization.html" << std::endl;
    if (!taskTimingFile.empty() && taskTimingStream.is_open())
    {
        std::cout << "taskTimes:  " << taskTimingFile << std::endl;
    }
    std::cout << "===================================" << std::endl;
    taskTimer.Finish();

    // Channel updates and visualization finalize
    taskTimer.Start("finalize_and_cleanup");
    channelModel->StopUpdates();
    if (enableIsl && islChannelModel)
    {
        islChannelModel->StopUpdates();
    }
    vizHelper.Finalize();
    taskTimer.Finish();

    Simulator::Destroy();
    LeoSimTaskProfiler::SetOutputStream(nullptr);

    return 0;
}
