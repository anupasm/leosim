/*
 * LeoSim routing research example.
 *
 * This example implements one routing experiment from doc/rp1_routing.txt on
 * a small, deterministic topology. The path-search algorithm stays fixed:
 * LeoSim uses the default Dijkstra route provider. The routingMetric command
 * line parameter changes only the edge cost callback, so independent runs can
 * be launched in parallel.
 *
 * Run:
 *   ./ns3 run "leosim-routing-research-example --routingMetric=distance --output=distance.csv"
 *   ./ns3 run "leosim-routing-research-example --routingMetric=congestion --output=congestion.csv"
 */

#include "ns3/core-module.h"
#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-loader.h"
#include "ns3/leosim-mobility-model.h"
#include "ns3/leosim-routing-calculator.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("LeoSimRoutingResearchExample");

namespace
{

constexpr double SPEED_OF_LIGHT_MPS = 299792458.0;

struct EdgeResearchState
{
    double remainingVisibilitySeconds = 300.0;
    double utilization = 0.1;
    double availableBandwidthMbps = 1000.0;
    double failureProbability = 0.001;
    double energyJoule = 1.0;
    double centrality = 0.1;
    double handoverPenalty = 0.0;
};

struct ExperimentWeights
{
    double distance = 0.0;
    double congestion = 0.0;
    double visibility = 0.0;
    double lifetime = 0.0;
    double failureRisk = 0.0;
    double handover = 0.0;
};

struct ExperimentConfig
{
    std::string name;
    std::string description;
    ExperimentWeights weights;
};

struct ExperimentResult
{
    std::string name;
    bool valid = false;
    uint32_t hopCount = 0;
    double distanceKm = 0.0;
    double propagationDelayMs = 0.0;
    double minSnr = 0.0;
    double bottleneckVisibilitySeconds = 0.0;
    double meanUtilization = 0.0;
    double bottleneckBandwidthMbps = 0.0;
    double cumulativeFailureProbability = 0.0;
    double handoverPenalty = 0.0;
    std::string path;
};

struct ResearchScenario
{
    NodeContainer nodes;
    Ptr<Node> ue;
    Ptr<Node> gateway;
    std::vector<Ptr<Node>> satellites;
    Ptr<LeoSimChannelModel> groundChannel;
    Ptr<LeoSimChannelModel> islChannel;
    Ptr<LeoSimRoutingCalculator> calculator;
    std::map<uint64_t, EdgeResearchState> edgeState;
};

ResearchScenario* g_activeScenario = nullptr;
const ExperimentConfig* g_activeExperiment = nullptr;

uint64_t
MakeEdgeKey(Ptr<Node> a, Ptr<Node> b)
{
    const uint32_t first = std::min(a->GetId(), b->GetId());
    const uint32_t second = std::max(a->GetId(), b->GetId());
    return (static_cast<uint64_t>(first) << 32) | second;
}

void
SetLeoSimPosition(Ptr<Node> node, double latitudeDeg, double longitudeDeg, double altitudeM, LeoSimNodeType type)
{
    Ptr<LeoSimMobilityModel> mobility = CreateObject<LeoSimMobilityModel>();
    mobility->SetNodeId(node->GetId());
    mobility->SetNodeType(type);
    mobility->SetPosition(LeoSimLoader::GeodeticToCartesian(latitudeDeg, longitudeDeg, altitudeM));
    node->AggregateObject(mobility);
}

std::string
PathToString(const LeoSimRoute& route)
{
    std::ostringstream output;
    for (size_t i = 0; i < route.path.size(); ++i)
    {
        if (i > 0)
        {
            output << "->";
        }
        output << route.path[i]->GetId();
    }
    return output.str();
}

double
NormalizeDistance(const LeoSimChannelQuality& quality)
{
    return std::max(0.001, quality.distance / 1000000.0);
}

double
NormalizeVisibility(const EdgeResearchState& state)
{
    return 1.0 / std::max(1.0, state.remainingVisibilitySeconds);
}

double
NormalizeBandwidth(const EdgeResearchState& state)
{
    return 1000.0 / std::max(1.0, state.availableBandwidthMbps);
}

double
CalculateResearchCost(const ExperimentWeights& weights,
                      const EdgeResearchState& state,
                      const LeoSimChannelQuality& quality)
{
    double cost = 0.0;
    cost += weights.distance * NormalizeDistance(quality);
    cost += weights.congestion * (1.0 + 10.0 * state.utilization + NormalizeBandwidth(state));
    cost += weights.visibility * NormalizeVisibility(state);
    cost += weights.lifetime * NormalizeVisibility(state);
    cost += weights.failureRisk * (100.0 * state.failureProbability);
    cost += weights.handover * state.handoverPenalty;
    return std::max(0.000001, cost);
}

double
ResearchCostCallback(Ptr<Node> a, Ptr<Node> b, const LeoSimChannelQuality& quality)
{
    if (g_activeScenario == nullptr || g_activeExperiment == nullptr)
    {
        return NormalizeDistance(quality);
    }

    const auto it = g_activeScenario->edgeState.find(MakeEdgeKey(a, b));
    const EdgeResearchState state = (it != g_activeScenario->edgeState.end()) ? it->second
                                                                              : EdgeResearchState();
    return CalculateResearchCost(g_activeExperiment->weights, state, quality);
}

ExperimentResult
EvaluateExperiment(const ExperimentConfig& experiment, ResearchScenario& scenario)
{
    g_activeScenario = &scenario;
    g_activeExperiment = &experiment;
    scenario.calculator->SetEdgeCostCallback(MakeCallback(&ResearchCostCallback));

    LeoSimRoute route = scenario.calculator->ComputeRoute(
        scenario.ue,
        scenario.gateway,
        LeoSimRoutingCalculator::LEOSIM_METRIC_DISTANCE,
        LeoSimRoutingCalculator::LEOSIM_PATH_ANY);

    ExperimentResult result;
    result.name = experiment.name;
    result.valid = route.valid;
    result.path = PathToString(route);

    if (!route.valid || route.path.size() < 2)
    {
        return result;
    }

    result.hopCount = route.hopCount;
    result.distanceKm = route.totalDistance / 1000.0;
    result.propagationDelayMs = route.totalDistance / SPEED_OF_LIGHT_MPS * 1000.0;
    result.minSnr = route.minSnr;
    result.bottleneckVisibilitySeconds = std::numeric_limits<double>::max();
    result.bottleneckBandwidthMbps = std::numeric_limits<double>::max();

    double utilizationSum = 0.0;
    double survivingProbability = 1.0;
    for (size_t i = 0; i + 1 < route.path.size(); ++i)
    {
        const auto it = scenario.edgeState.find(MakeEdgeKey(route.path[i], route.path[i + 1]));
        const EdgeResearchState state = (it != scenario.edgeState.end()) ? it->second
                                                                         : EdgeResearchState();
        result.bottleneckVisibilitySeconds =
            std::min(result.bottleneckVisibilitySeconds, state.remainingVisibilitySeconds);
        result.bottleneckBandwidthMbps =
            std::min(result.bottleneckBandwidthMbps, state.availableBandwidthMbps);
        result.handoverPenalty += state.handoverPenalty;
        utilizationSum += state.utilization;
        survivingProbability *= (1.0 - state.failureProbability);
    }

    result.meanUtilization = utilizationSum / route.hopCount;
    result.cumulativeFailureProbability = 1.0 - survivingProbability;
    return result;
}

void
AddIsl(ResearchScenario& scenario,
       uint32_t a,
       uint32_t b,
       double visibilitySeconds,
       double utilization,
       double bandwidthMbps,
       double failureProbability,
       double handoverPenalty)
{
    Ptr<Node> first = scenario.satellites[a];
    Ptr<Node> second = scenario.satellites[b];
    scenario.islChannel->AddIslLink(first, second);

    EdgeResearchState state;
    state.remainingVisibilitySeconds = visibilitySeconds;
    state.utilization = utilization;
    state.availableBandwidthMbps = bandwidthMbps;
    state.failureProbability = failureProbability;
    state.energyJoule = 1.0 + utilization;
    state.centrality = utilization;
    state.handoverPenalty = handoverPenalty;
    scenario.edgeState[MakeEdgeKey(first, second)] = state;
}

void
AddGroundLink(ResearchScenario& scenario,
              Ptr<Node> ground,
              Ptr<Node> satellite,
              double visibilitySeconds,
              double utilization,
              double bandwidthMbps,
              double failureProbability)
{
    scenario.groundChannel->AddLink(ground, satellite, LEOSIM_LINK_SATELLITE_TO_GROUND);

    EdgeResearchState state;
    state.remainingVisibilitySeconds = visibilitySeconds;
    state.utilization = utilization;
    state.availableBandwidthMbps = bandwidthMbps;
    state.failureProbability = failureProbability;
    scenario.edgeState[MakeEdgeKey(ground, satellite)] = state;
}

ResearchScenario
BuildScenario()
{
    ResearchScenario scenario;
    scenario.nodes.Create(8);
    scenario.ue = scenario.nodes.Get(0);
    scenario.gateway = scenario.nodes.Get(7);
    for (uint32_t i = 1; i <= 6; ++i)
    {
        scenario.satellites.push_back(scenario.nodes.Get(i));
    }

    SetLeoSimPosition(scenario.ue, 0.0, 0.0, 0.0, LEOSIM_UE);
    SetLeoSimPosition(scenario.gateway, 0.0, 14.0, 0.0, LEOSIM_GATEWAY);

    SetLeoSimPosition(scenario.satellites[0], 0.0, 0.0, 550000.0, LEOSIM_SATELLITE);
    SetLeoSimPosition(scenario.satellites[1], 0.0, 3.0, 550000.0, LEOSIM_SATELLITE);
    SetLeoSimPosition(scenario.satellites[2], 0.0, 6.0, 550000.0, LEOSIM_SATELLITE);
    SetLeoSimPosition(scenario.satellites[3], 0.0, 9.0, 550000.0, LEOSIM_SATELLITE);
    SetLeoSimPosition(scenario.satellites[4], 0.0, 12.0, 550000.0, LEOSIM_SATELLITE);
    SetLeoSimPosition(scenario.satellites[5], 5.0, 7.0, 550000.0, LEOSIM_SATELLITE);

    scenario.groundChannel = CreateObject<LeoSimChannelModel>();
    scenario.islChannel = CreateObject<LeoSimChannelModel>();
    scenario.groundChannel->SetMaxLinkDistance(2500000.0);
    scenario.groundChannel->SetMinElevationAngle(0.0);
    scenario.islChannel->SetIslMaxDistance(2500000.0);

    AddGroundLink(scenario, scenario.ue, scenario.satellites[0], 600.0, 0.20, 1000.0, 0.001);
    AddGroundLink(scenario, scenario.gateway, scenario.satellites[4], 620.0, 0.25, 1000.0, 0.001);
    AddGroundLink(scenario, scenario.gateway, scenario.satellites[5], 900.0, 0.10, 850.0, 0.002);

    // Short but congested path: sat0 -> sat1 -> sat2 -> sat3 -> sat4.
    AddIsl(scenario, 0, 1, 180.0, 0.85, 350.0, 0.020, 0.1);
    AddIsl(scenario, 1, 2, 170.0, 0.88, 320.0, 0.025, 0.1);
    AddIsl(scenario, 2, 3, 160.0, 0.90, 300.0, 0.030, 0.1);
    AddIsl(scenario, 3, 4, 150.0, 0.82, 360.0, 0.020, 0.1);

    // Longer but stable path through sat5.
    AddIsl(scenario, 0, 5, 900.0, 0.25, 950.0, 0.004, 1.0);
    AddIsl(scenario, 5, 4, 850.0, 0.20, 900.0, 0.003, 1.0);

    // Cross-link alternative with high failure risk.
    AddIsl(scenario, 2, 5, 400.0, 0.45, 700.0, 0.080, 0.6);

    scenario.groundChannel->UpdateAllLinks();
    scenario.islChannel->UpdateAllLinks();

    scenario.calculator = CreateObject<LeoSimRoutingCalculator>();
    scenario.calculator->SetChannelModel(scenario.groundChannel);
    scenario.calculator->SetIslChannelModel(scenario.islChannel);
    scenario.calculator->SetAccessAuthorityEnabled(false);
    return scenario;
}

std::vector<ExperimentConfig>
BuildExperiments(double congestionWeight)
{
    std::vector<ExperimentConfig> experiments;
    experiments.push_back({"distance", "Cost = Euclidean distance", {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    experiments.push_back({"prop-delay", "Cost = distance / speed of light", {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    experiments.push_back({"propagation-delay", "Cost = distance / speed of light", {1.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    experiments.push_back({"hop-count", "Cost = one per hop", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    experiments.push_back({"hop", "Cost = one per hop", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
    experiments.push_back({"visibility", "Cost = 1 / remaining visibility time", {0.0, 0.0, 1.0, 0.0, 0.0, 0.0}});
    experiments.push_back({"path-lifetime", "Cost = path lifetime pressure", {0.0, 0.0, 0.0, 1.0, 0.0, 0.0}});
    experiments.push_back({"lifetime", "Cost = path lifetime pressure", {0.0, 0.0, 0.0, 1.0, 0.0, 0.0}});
    experiments.push_back({"congestion", "Cost = utilization and inverse bandwidth", {0.0, 1.0, 0.0, 0.0, 0.0, 0.0}});
    experiments.push_back({"failure-aware", "Cost = predicted link failure probability", {0.0, 0.0, 0.0, 0.0, 1.0, 0.0}});
    experiments.push_back({"failure", "Cost = predicted link failure probability", {0.0, 0.0, 0.0, 0.0, 1.0, 0.0}});
    experiments.push_back({"handover-aware", "Cost = orbital-plane or handover penalty", {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}});
    experiments.push_back({"handover", "Cost = orbital-plane or handover penalty", {0.0, 0.0, 0.0, 0.0, 0.0, 1.0}});
    experiments.push_back({"multi-objective",
                           "Weighted distance, congestion, visibility, lifetime, failure, handover",
                           {0.35, congestionWeight, 0.15, 0.15, 0.20, 0.10}});
    experiments.push_back({"multi", "Weighted distance, congestion, visibility, lifetime, failure, handover", {0.35, congestionWeight, 0.15, 0.15, 0.20, 0.10}});
    return experiments;
}

ExperimentConfig
GetExperimentOrDie(const std::vector<ExperimentConfig>& experiments, const std::string& routingMetric)
{
    for (const ExperimentConfig& experiment : experiments)
    {
        if (experiment.name == routingMetric)
        {
            return experiment;
        }
    }

    std::cerr << "Unknown routingMetric='" << routingMetric << "'. Valid values:";
    for (const ExperimentConfig& experiment : experiments)
    {
        std::cerr << " " << experiment.name;
    }
    std::cerr << std::endl;
    std::exit(1);
}

void
PrintMetricList(const std::vector<ExperimentConfig>& experiments)
{
    std::cout << "Available routing metrics:\n";
    for (const ExperimentConfig& experiment : experiments)
    {
        std::cout << "  " << std::left << std::setw(20) << experiment.name
                  << experiment.description << "\n";
    }
}

void
WriteResearchPlan(const std::string& path)
{
    std::ofstream plan(path);
    plan << "LeoSim RP1 routing research plan\n"
         << "================================\n\n"
         << "Objective: compare shortest-path routing strategies while keeping Dijkstra fixed.\n"
         << "Only edge-cost definitions change between experiments.\n\n"
         << "Experiments implemented in this example:\n"
         << "1. Distance routing\n"
         << "2. Propagation-delay routing\n"
         << "3. Hop-count routing\n"
         << "4. Visibility-aware routing\n"
         << "5. Path-lifetime routing\n"
         << "6. Congestion and bandwidth-aware routing\n"
         << "7. Failure-aware routing\n"
         << "8. Handover-aware routing\n"
         << "9. Multi-objective routing\n\n"
         << "Recommended full-scale study:\n"
         << "- repeat each experiment for identical topology, traffic, duration, and random seed,\n"
         << "- sweep one weight at a time for sensitivity analysis,\n"
         << "- repeat under light, medium, heavy, and bursty traffic,\n"
         << "- repeat under 0%, 1%, 5%, and 10% satellite or ISL failures,\n"
         << "- report latency, throughput, path lifetime, reroutes, packet loss, fairness,\n"
         << "  CPU time, memory, and successful route percentage.\n";
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string routingMetric = "distance";
    std::string output = "rp1-routing-result.csv";
    std::string planOutput = "rp1-routing-plan.md";
    double congestionWeight = 0.25;
    bool writePlan = false;
    bool listMetrics = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("routingMetric",
                 "Routing metric for this run: distance, prop-delay, hop-count, visibility, "
                 "path-lifetime, congestion, failure-aware, handover-aware, multi-objective",
                 routingMetric);
    cmd.AddValue("output", "CSV file for this run's routing result", output);
    cmd.AddValue("planOutput", "Research-plan summary output file", planOutput);
    cmd.AddValue("congestionWeight", "Multi-objective congestion weight beta", congestionWeight);
    cmd.AddValue("writePlan", "Write a markdown research-plan summary for this run", writePlan);
    cmd.AddValue("listMetrics", "Print supported routingMetric values and exit", listMetrics);
    cmd.Parse(argc, argv);

    std::vector<ExperimentConfig> experiments = BuildExperiments(congestionWeight);
    if (listMetrics)
    {
        PrintMetricList(experiments);
        return 0;
    }

    ExperimentConfig experiment = GetExperimentOrDie(experiments, routingMetric);
    ResearchScenario scenario = BuildScenario();
    ExperimentResult result = EvaluateExperiment(experiment, scenario);

    std::ofstream csv(output);
    csv << "experiment,valid,hop_count,distance_km,propagation_delay_ms,min_snr_db,"
        << "bottleneck_visibility_s,mean_utilization,bottleneck_bandwidth_mbps,"
        << "cumulative_failure_probability,handover_penalty,path\n";
    csv << result.name << ","
        << result.valid << ","
        << result.hopCount << ","
        << std::fixed << std::setprecision(6)
        << result.distanceKm << ","
        << result.propagationDelayMs << ","
        << result.minSnr << ","
        << result.bottleneckVisibilitySeconds << ","
        << result.meanUtilization << ","
        << result.bottleneckBandwidthMbps << ","
        << result.cumulativeFailureProbability << ","
        << result.handoverPenalty << ","
        << result.path << "\n";

    std::cout << "LeoSim RP1 routing metric evaluation\n"
              << "routingMetric=" << result.name << "\n"
              << "description=" << experiment.description << "\n";
    std::cout << "output=" << output << "\n";
    std::cout << std::setw(18) << result.name
              << " valid=" << result.valid
              << " hops=" << result.hopCount
              << " distance_km=" << std::fixed << std::setprecision(2) << result.distanceKm
              << " visibility_s=" << result.bottleneckVisibilitySeconds
              << " failure=" << std::setprecision(4) << result.cumulativeFailureProbability
              << " path=" << result.path << "\n";

    if (writePlan)
    {
        WriteResearchPlan(planOutput);
        std::cout << "plan=" << planOutput << "\n";
    }

    Simulator::Destroy();
    return 0;
}
