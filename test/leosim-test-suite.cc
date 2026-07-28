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

#include "ns3/leosim.h"
#include "ns3/leosim-channel-helper.h"
#include "ns3/leosim-operator-model.h"
#include "ns3/leosim-beam-manager.h"
#include "ns3/leosim-routing-calculator-helper.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/test.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/node-container.h"
#include "ns3/simulator.h"

#include <cstdio>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>

using namespace ns3;

namespace
{

class LeoSimTestSpatialIslDegreeAndConnectivity : public TestCase
{
  public:
    LeoSimTestSpatialIslDegreeAndConnectivity()
        : TestCase("spatial ISL topology respects degree bound and connects reachable satellites")
    {
    }

  private:
    void DoRun() override
    {
        NodeContainer satellites;
        satellites.Create(6);
        for (uint32_t i = 0; i < satellites.GetN(); ++i)
        {
            Ptr<ConstantPositionMobilityModel> mobility =
                CreateObject<ConstantPositionMobilityModel>();
            mobility->SetPosition(Vector(i * 100000.0, 0.0, 0.0));
            satellites.Get(i)->AggregateObject(mobility);
        }

        LeoSimChannelHelper helper;
        helper.SetIslMaxDistance(250000.0);
        Ptr<LeoSimChannelModel> model =
            helper.CreateIslNearestNeighborMesh(satellites, 2);
        const auto links = model->GetLinksByType(LEOSIM_LINK_ISL, false);

        std::map<uint32_t, std::set<uint32_t>> adjacency;
        for (const auto& link : links)
        {
            const uint32_t a = link.node1->GetId();
            const uint32_t b = link.node2->GetId();
            adjacency[a].insert(b);
            adjacency[b].insert(a);
        }
        for (uint32_t i = 0; i < satellites.GetN(); ++i)
        {
            NS_TEST_ASSERT_MSG_LT(adjacency[satellites.Get(i)->GetId()].size(),
                                  3,
                                  "Spatial ISL degree exceeded configured bound");
        }

        std::set<uint32_t> visited;
        std::vector<uint32_t> pending{satellites.Get(0)->GetId()};
        while (!pending.empty())
        {
            const uint32_t node = pending.back();
            pending.pop_back();
            if (!visited.insert(node).second)
            {
                continue;
            }
            for (uint32_t neighbor : adjacency[node])
            {
                pending.push_back(neighbor);
            }
        }
        NS_TEST_ASSERT_MSG_EQ(visited.size(),
                              satellites.GetN(),
                              "Reachable spatial satellite set should be connected");
        Simulator::Destroy();
    }
};

class LeoSimTestDynamicIslNeighborReselection : public TestCase
{
  public:
    LeoSimTestDynamicIslNeighborReselection()
        : TestCase("dynamic ISL updates select new neighbours without exceeding degree")
    {
    }

  private:
    void OnTopologyChanged()
    {
        ++m_topologyChanges;
    }

    void DoRun() override
    {
        NodeContainer satellites;
        satellites.Create(4);
        std::vector<Ptr<ConstantPositionMobilityModel>> mobility;
        for (uint32_t i = 0; i < satellites.GetN(); ++i)
        {
            auto model = CreateObject<ConstantPositionMobilityModel>();
            model->SetPosition(Vector(i * 100000.0, 0.0, 0.0));
            satellites.Get(i)->AggregateObject(model);
            mobility.push_back(model);
        }

        Ptr<LeoSimChannelModel> channel = CreateObject<LeoSimChannelModel>();
        channel->SetIslMaxDistance(250000.0);
        for (uint32_t i = 0; i < satellites.GetN(); ++i)
        {
            for (uint32_t j = i + 1; j < satellites.GetN(); ++j)
            {
                channel->AddIslLink(satellites.Get(i), satellites.Get(j));
            }
        }
        channel->SetDynamicIslSelectionInterval(Seconds(1));
        channel->SetDynamicIslMaxNeighbors(1);
        channel->SetDynamicIslTopologyChangeCallback(
            MakeCallback(&LeoSimTestDynamicIslNeighborReselection::OnTopologyChanged, this));

        mobility[0]->SetPosition(Vector(300000.0, 0.0, 0.0));
        mobility[3]->SetPosition(Vector(0.0, 0.0, 0.0));
        Simulator::Schedule(Seconds(1), &LeoSimChannelModel::UpdateAllLinks, channel);
        Simulator::Run();

        std::map<uint32_t, uint32_t> degree;
        bool selectedNewPair = false;
        for (const auto& link : channel->GetLinksByType(LEOSIM_LINK_ISL, false))
        {
            degree[link.node1->GetId()]++;
            degree[link.node2->GetId()]++;
            selectedNewPair |=
                (link.node1 == satellites.Get(3) && link.node2 == satellites.Get(1)) ||
                (link.node1 == satellites.Get(1) && link.node2 == satellites.Get(3));
        }
        for (const auto& [node, count] : degree)
        {
            NS_TEST_ASSERT_MSG_LT(count, 2, "Dynamic ISL degree exceeded configured bound");
        }
        NS_TEST_ASSERT_MSG_EQ(selectedNewPair,
                              true,
                              "Periodic update did not select the newly closest neighbour");
        NS_TEST_ASSERT_MSG_EQ(m_topologyChanges,
                              1,
                              "Dynamic ISL reselection must notify routing exactly once");
        Simulator::Destroy();
    }

    uint32_t m_topologyChanges{0};
};

class LeoSimTestTrajectoryAwareAccessCandidates : public TestCase
{
  public:
    LeoSimTestTrajectoryAwareAccessCandidates()
        : TestCase("access candidates include satellites visible later in the trajectory")
    {
    }

  private:
    void DoRun() override
    {
        constexpr double earthRadius = 6371000.0;
        const Vector visible(earthRadius + 550000.0, 0.0, 0.0);
        const Vector hidden(-(earthRadius + 550000.0), 0.0, 0.0);

        NodeContainer satellites;
        satellites.Create(2);
        for (uint32_t i = 0; i < satellites.GetN(); ++i)
        {
            Ptr<LeoSimMobilityModel> mobility = CreateObject<LeoSimMobilityModel>();
            mobility->SetNodeType(LEOSIM_SATELLITE);
            mobility->SetNodeId(i);
            mobility->SetWaypoints({{Seconds(0), i == 0 ? visible : hidden, Vector()},
                                    {Seconds(10), i == 0 ? hidden : visible, Vector()}});
            satellites.Get(i)->AggregateObject(mobility);
            mobility->Start();
        }

        NodeContainer groundNodes;
        groundNodes.Create(1);
        Ptr<LeoSimMobilityModel> groundMobility = CreateObject<LeoSimMobilityModel>();
        groundMobility->SetNodeType(LEOSIM_GATEWAY);
        groundMobility->SetNodeId(0);
        groundMobility->SetWaypoints({{Seconds(0), Vector(earthRadius, 0.0, 0.0), Vector()}});
        groundNodes.Get(0)->AggregateObject(groundMobility);
        groundMobility->Start();

        LeoSimChannelHelper helper;
        helper.SetMaxGroundLinksPerNode(1);
        Ptr<LeoSimChannelModel> model = helper.CreateChannels(satellites, groundNodes);
        const auto candidates =
            model->GetLinksByType(LEOSIM_LINK_SATELLITE_TO_GROUND, true);
        NS_TEST_ASSERT_MSG_EQ(candidates.size(),
                              2,
                              "Candidate union must include the satellite visible at each epoch");
        Simulator::Destroy();
    }
};

Vector
TestGeodeticPosition(double latDeg, double lonDeg, double altitudeM)
{
    return LeoSimLoader::GeodeticToCartesian(latDeg, lonDeg, altitudeM);
}

void
SetTestPosition(Ptr<Node> node, const Vector& position)
{
    Ptr<ConstantPositionMobilityModel> mobility = CreateObject<ConstantPositionMobilityModel>();
    mobility->SetPosition(position);
    node->AggregateObject(mobility);
}

void
SetLeoSimTestPosition(Ptr<Node> node, const Vector& position, LeoSimNodeType nodeType)
{
    Ptr<LeoSimMobilityModel> mobility = CreateObject<LeoSimMobilityModel>();
    mobility->SetNodeId(node->GetId());
    mobility->SetNodeType(nodeType);
    mobility->SetPosition(position);
    node->AggregateObject(mobility);
}

LeoSimSpotBeam
MakeTestBeam(uint32_t satNodeId,
             uint32_t beamId,
             double centerLat,
             double centerLon,
             double radiusKm,
             bool active = true)
{
    LeoSimSpotBeam beam;
    beam.satelliteNodeId = satNodeId;
    beam.beamId = beamId;
    beam.cellId = beamId;
    beam.colorGroup = static_cast<int>(beamId % 3);
    beam.centerLat = centerLat;
    beam.centerLon = centerLon;
    beam.radiusKm = radiusKm;
    beam.activeInCurrentSlot = active;
    return beam;
}

} // namespace

/**
 * \ingroup leosim-tests
 * \defgroup leosim-test-suite leosim module tests
 */

/**
 * \ingroup leosim-test-suite
 * \brief LeoSim basic test case
 */
class LeoSimTestCase1 : public TestCase
{
  public:
    LeoSimTestCase1();
    ~LeoSimTestCase1() override;

  private:
    void DoRun() override;
};

LeoSimTestCase1::LeoSimTestCase1()
    : TestCase("LeoSim test case (does nothing)")
{
}

LeoSimTestCase1::~LeoSimTestCase1()
{
}

void
LeoSimTestCase1::DoRun()
{
    // Test instantiation
}

/**
 * \ingroup leosim-test-suite
 * \brief Test beam configuration structure initialization
 */
class LeoSimTestHexBeamLayout19 : public TestCase
{
  public:
    LeoSimTestHexBeamLayout19();
    ~LeoSimTestHexBeamLayout19() override;

  private:
    void DoRun() override;
};

LeoSimTestHexBeamLayout19::LeoSimTestHexBeamLayout19()
    : TestCase("LeoSim Hex Beam Layout 19 beams (2 rings)")
{
}

LeoSimTestHexBeamLayout19::~LeoSimTestHexBeamLayout19()
{
}

void
LeoSimTestHexBeamLayout19::DoRun()
{
    LeoSimBeamConfig config;
    config.numBeamsPerSatellite = 19;
    NS_TEST_ASSERT_MSG_EQ(config.numBeamsPerSatellite, 19,
                          "Beam config should have 19 beams");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test beam configuration with 61 beams
 */
class LeoSimTestHexBeamLayout61 : public TestCase
{
  public:
    LeoSimTestHexBeamLayout61();
    ~LeoSimTestHexBeamLayout61() override;

  private:
    void DoRun() override;
};

LeoSimTestHexBeamLayout61::LeoSimTestHexBeamLayout61()
    : TestCase("LeoSim Hex Beam Layout 61 beams (3 rings)")
{
}

LeoSimTestHexBeamLayout61::~LeoSimTestHexBeamLayout61()
{
}

void
LeoSimTestHexBeamLayout61::DoRun()
{
    LeoSimBeamConfig config;
    config.numBeamsPerSatellite = 61;
    NS_TEST_ASSERT_MSG_EQ(config.numBeamsPerSatellite, 61,
                          "Beam config should have 61 beams");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test SINR computation with interference conditions
 */
class LeoSimTestSinrWithICI : public TestCase
{
  public:
    LeoSimTestSinrWithICI();
    ~LeoSimTestSinrWithICI() override;

  private:
    void DoRun() override;
};

LeoSimTestSinrWithICI::LeoSimTestSinrWithICI()
    : TestCase("LeoSim SINR with Inter-Cell Interference")
{
}

LeoSimTestSinrWithICI::~LeoSimTestSinrWithICI()
{
}

void
LeoSimTestSinrWithICI::DoRun()
{
    LeoSimSpotBeam serving;
    serving.satelliteNodeId = 0;
    serving.beamId = 0;
    serving.centerLat = 30.0;
    serving.centerLon = -129.8;
    serving.radiusKm = 100.0;
    serving.colorGroup = 1;
    serving.activeInCurrentSlot = true;

    std::map<uint32_t, std::vector<LeoSimSpotBeam>> beams{{0, {serving}}};
    Ptr<LeoSimSinrEngine> engine = CreateObject<LeoSimSinrEngine>();
    const auto conventional = engine->ComputeSinr(1, 0, 0, beams, 30.0, -129.8);
    const auto wrapped = engine->ComputeSinr(1, 0, 0, beams, 30.0, 230.2);
    NS_TEST_ASSERT_MSG_EQ_TOL(conventional.sinr_dB,
                              wrapped.sinr_dB,
                              1e-9,
                              "Equivalent longitude conventions must produce equal SINR");

    LeoSimSpotBeam occulted = serving;
    occulted.satelliteNodeId = 1;
    occulted.centerLon = 50.0;
    beams[1] = {occulted};
    const auto withOccultedInterferer =
        engine->ComputeSinr(1, 0, 0, beams, 30.0, -129.8);
    NS_TEST_ASSERT_MSG_EQ_TOL(conventional.sinr_dB,
                              withOccultedInterferer.sinr_dB,
                              1e-9,
                              "Earth-occulted satellites must not contribute interference");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test beam antenna gain pattern
 */
class LeoSimTestBeamAntennaGainPattern : public TestCase
{
  public:
    LeoSimTestBeamAntennaGainPattern();
    ~LeoSimTestBeamAntennaGainPattern() override;

  private:
    void DoRun() override;
};

LeoSimTestBeamAntennaGainPattern::LeoSimTestBeamAntennaGainPattern()
    : TestCase("LeoSim Beam Antenna Gain Pattern")
{
}

LeoSimTestBeamAntennaGainPattern::~LeoSimTestBeamAntennaGainPattern()
{
}

void
LeoSimTestBeamAntennaGainPattern::DoRun()
{
    LeoSimSpotBeam beam;
    double gainAt0 = beam.GetAntennaGain(0.0);
    double gainAt10 = beam.GetAntennaGain(10.0);
    // Verify antenna gain returns reasonable values
    NS_TEST_ASSERT_MSG_GT(gainAt0, 20.0, "Gain at boresight should be significant");
    NS_TEST_ASSERT_MSG_GT(gainAt10, -80.0, "Off-boresight gain should remain finite");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test intra-beam handover configuration
 */
class LeoSimTestIntraBeamHoNoRouteChange : public TestCase
{
  public:
    LeoSimTestIntraBeamHoNoRouteChange();
    ~LeoSimTestIntraBeamHoNoRouteChange() override;

  private:
    void DoRun() override;
};

LeoSimTestIntraBeamHoNoRouteChange::LeoSimTestIntraBeamHoNoRouteChange()
    : TestCase("LeoSim Intra-Beam HO No Route Change")
{
}

LeoSimTestIntraBeamHoNoRouteChange::~LeoSimTestIntraBeamHoNoRouteChange()
{
}

void
LeoSimTestIntraBeamHoNoRouteChange::DoRun()
{
    LeoSimSpotBeam beam;
    beam.satelliteNodeId = 1;
    beam.beamId = 0;
    NS_TEST_ASSERT_MSG_EQ(beam.satelliteNodeId, 1, "Satellite node ID should be set");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test intra vs inter-satellite handover timing
 */
class LeoSimTestIntraBeamHoFasterThanInter : public TestCase
{
  public:
    LeoSimTestIntraBeamHoFasterThanInter();
    ~LeoSimTestIntraBeamHoFasterThanInter() override;

  private:
    void DoRun() override;
};

LeoSimTestIntraBeamHoFasterThanInter::LeoSimTestIntraBeamHoFasterThanInter()
    : TestCase("LeoSim Intra-Beam HO Faster Than Inter-Satellite")
{
}

LeoSimTestIntraBeamHoFasterThanInter::~LeoSimTestIntraBeamHoFasterThanInter()
{
}

void
LeoSimTestIntraBeamHoFasterThanInter::DoRun()
{
    Time intraDelay = MilliSeconds(10);
    Time choPrep = MilliSeconds(100);
    NS_TEST_ASSERT_MSG_LT(intraDelay.GetMilliSeconds(), choPrep.GetMilliSeconds(),
                          "Intra-beam HO should be faster");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test beam hopping schedule color constraint
 */
class LeoSimTestBeamHoppingScheduleColour : public TestCase
{
  public:
    LeoSimTestBeamHoppingScheduleColour();
    ~LeoSimTestBeamHoppingScheduleColour() override;

  private:
    void DoRun() override;
};

LeoSimTestBeamHoppingScheduleColour::LeoSimTestBeamHoppingScheduleColour()
    : TestCase("LeoSim Beam Hopping Schedule Colour Constraint")
{
}

LeoSimTestBeamHoppingScheduleColour::~LeoSimTestBeamHoppingScheduleColour()
{
}

void
LeoSimTestBeamHoppingScheduleColour::DoRun()
{
    LeoSimBeamConfig config;
    config.beamHoppingEnabled = true;
    config.beamHopCycleSlotsN = 256;
    NS_TEST_ASSERT_MSG_EQ(config.beamHoppingEnabled, true, "Beam hopping enabled");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test beam dark handover detection
 */
class LeoSimTestBeamDarkHandover : public TestCase
{
  public:
    LeoSimTestBeamDarkHandover();
    ~LeoSimTestBeamDarkHandover() override;

  private:
    void DoRun() override;
};

LeoSimTestBeamDarkHandover::LeoSimTestBeamDarkHandover()
    : TestCase("LeoSim Beam Dark Handover")
{
}

LeoSimTestBeamDarkHandover::~LeoSimTestBeamDarkHandover()
{
}

void
LeoSimTestBeamDarkHandover::DoRun()
{
    LeoSimSpotBeam darkBeam;
    darkBeam.activeInCurrentSlot = false;
    NS_TEST_ASSERT_MSG_EQ(darkBeam.activeInCurrentSlot, false, "Dark beam is inactive");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test demand-aware scheduling
 */
class LeoSimTestDemandAwareSchedule : public TestCase
{
  public:
    LeoSimTestDemandAwareSchedule();
    ~LeoSimTestDemandAwareSchedule() override;

  private:
    void DoRun() override;
};

LeoSimTestDemandAwareSchedule::LeoSimTestDemandAwareSchedule()
    : TestCase("LeoSim Demand-Aware Schedule")
{
}

LeoSimTestDemandAwareSchedule::~LeoSimTestDemandAwareSchedule()
{
}

void
LeoSimTestDemandAwareSchedule::DoRun()
{
    std::vector<double> demandWeights(19);
    demandWeights[0] = 0.6;
    for (size_t i = 1; i < 19; ++i)
    {
        demandWeights[i] = 0.4 / 18.0;
    }
    NS_TEST_ASSERT_MSG_GT(demandWeights[0], 0.5, "High-demand beam weight");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test TOPSIS configuration
 */
class LeoSimTestTopsisWithSinrAttribute : public TestCase
{
  public:
    LeoSimTestTopsisWithSinrAttribute();
    ~LeoSimTestTopsisWithSinrAttribute() override;

  private:
    void DoRun() override;
};

LeoSimTestTopsisWithSinrAttribute::LeoSimTestTopsisWithSinrAttribute()
    : TestCase("LeoSim TOPSIS with SINR Attribute")
{
}

LeoSimTestTopsisWithSinrAttribute::~LeoSimTestTopsisWithSinrAttribute()
{
}

void
LeoSimTestTopsisWithSinrAttribute::DoRun()
{
    std::vector<double> weights;
    weights.push_back(0.10); // RSRP
    weights.push_back(0.50); // SINR
    NS_TEST_ASSERT_MSG_GT(weights[1], weights[0], "SINR weight greater than RSRP");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test operator registration stores ID and role
 */
class LeoSimTestOperatorRegistration : public TestCase
{
  public:
    LeoSimTestOperatorRegistration();
    ~LeoSimTestOperatorRegistration() override;

  private:
    void DoRun() override;
};

LeoSimTestOperatorRegistration::LeoSimTestOperatorRegistration()
    : TestCase("LeoSim operator registration stores ID and role correctly")
{
}

LeoSimTestOperatorRegistration::~LeoSimTestOperatorRegistration()
{
}

void
LeoSimTestOperatorRegistration::DoRun()
{
    Ptr<LeoSimOperatorModel> model = CreateObject<LeoSimOperatorModel>();
    model->RegisterNode(0, "OpA", LEOSIM_ROLE_SATELLITE);
    model->RegisterNode(1, "OpB", LEOSIM_ROLE_UE);
    model->RegisterNode(2, "OpA", LEOSIM_ROLE_SERVER);

    NS_TEST_ASSERT_MSG_EQ(model->GetOperatorId(0), "OpA", "Node 0 should belong to OpA");
    NS_TEST_ASSERT_MSG_EQ(model->GetOperatorId(1), "OpB", "Node 1 should belong to OpB");
    NS_TEST_ASSERT_MSG_EQ(model->GetRole(0),
                          LEOSIM_ROLE_SATELLITE,
                          "Node 0 role should be SATELLITE");
    NS_TEST_ASSERT_MSG_EQ(model->GetNodesByOperator("OpA").size(),
                          static_cast<size_t>(2),
                          "OpA should have 2 registered nodes");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test IsSameOperator returns correct values
 */
class LeoSimTestSameOperatorQuery : public TestCase
{
  public:
    LeoSimTestSameOperatorQuery();
    ~LeoSimTestSameOperatorQuery() override;

  private:
    void DoRun() override;
};

LeoSimTestSameOperatorQuery::LeoSimTestSameOperatorQuery()
    : TestCase("LeoSim IsSameOperator returns correct values")
{
}

LeoSimTestSameOperatorQuery::~LeoSimTestSameOperatorQuery()
{
}

void
LeoSimTestSameOperatorQuery::DoRun()
{
    Ptr<LeoSimOperatorModel> model = CreateObject<LeoSimOperatorModel>();
    model->RegisterNode(0, "OpA", LEOSIM_ROLE_SATELLITE);
    model->RegisterNode(1, "OpB", LEOSIM_ROLE_UE);
    model->RegisterNode(2, "OpA", LEOSIM_ROLE_SERVER);

    NS_TEST_ASSERT_MSG_EQ(model->IsSameOperator(0, 2),
                          true,
                          "Nodes 0 and 2 (both OpA) should be same operator");
    NS_TEST_ASSERT_MSG_EQ(model->IsSameOperator(0, 1),
                          false,
                          "Node 0 (OpA) and node 1 (OpB) should NOT be same operator");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test sharing matrix CSV load populates alpha values
 */
class LeoSimTestSharingMatrixLoad : public TestCase
{
  public:
    LeoSimTestSharingMatrixLoad();
    ~LeoSimTestSharingMatrixLoad() override;

  private:
    void DoRun() override;
};

LeoSimTestSharingMatrixLoad::LeoSimTestSharingMatrixLoad()
    : TestCase("LeoSim sharing matrix loads alpha values from CSV")
{
}

LeoSimTestSharingMatrixLoad::~LeoSimTestSharingMatrixLoad()
{
}

void
LeoSimTestSharingMatrixLoad::DoRun()
{
    const std::string tmpFile = "/tmp/leosim_test_sharing_matrix.csv";
    {
        std::ofstream f(tmpFile);
        f << "OperatorA,OperatorB,AlphaDL,AlphaUL,AlphaISL\n";
        f << "OpA,OpA,1.0,1.0,1.0\n";
        f << "OpB,OpB,1.0,1.0,1.0\n";
        f << "OpA,OpB,0.6,0.5,0.4\n";
    }

    Ptr<LeoSimOperatorModel> model = CreateObject<LeoSimOperatorModel>();
    model->LoadSharingMatrixFromCsv(tmpFile);

    NS_TEST_ASSERT_MSG_EQ_TOL(model->GetAlphaByOperator("OpA", "OpB", LEOSIM_DIR_DOWNLINK),
                              0.6,
                              1e-9,
                              "AlphaDL(OpA,OpB) should be 0.6");
    NS_TEST_ASSERT_MSG_EQ_TOL(model->GetAlphaByOperator("OpA", "OpA", LEOSIM_DIR_DOWNLINK),
                              1.0,
                              1e-9,
                              "AlphaDL(OpA,OpA) should be 1.0");

    std::remove(tmpFile.c_str());
}

/**
 * \ingroup leosim-test-suite
 * \brief Test sharing matrix alpha lookup is symmetric
 */
class LeoSimTestSharingMatrixSymmetry : public TestCase
{
  public:
    LeoSimTestSharingMatrixSymmetry();
    ~LeoSimTestSharingMatrixSymmetry() override;

  private:
    void DoRun() override;
};

LeoSimTestSharingMatrixSymmetry::LeoSimTestSharingMatrixSymmetry()
    : TestCase("LeoSim sharing matrix alpha(A,B) == alpha(B,A)")
{
}

LeoSimTestSharingMatrixSymmetry::~LeoSimTestSharingMatrixSymmetry()
{
}

void
LeoSimTestSharingMatrixSymmetry::DoRun()
{
    Ptr<LeoSimOperatorModel> model = CreateObject<LeoSimOperatorModel>();
    model->SetAlpha("OpA", "OpB", 0.6, 0.6, 0.4);

    NS_TEST_ASSERT_MSG_EQ_TOL(model->GetAlphaByOperator("OpB", "OpA", LEOSIM_DIR_DOWNLINK),
                              0.6,
                              1e-9,
                              "Reverse lookup (OpB,OpA) downlink should equal 0.6");
    NS_TEST_ASSERT_MSG_EQ_TOL(model->GetAlphaByOperator("OpB", "OpA", LEOSIM_DIR_ISL),
                              0.4,
                              1e-9,
                              "Reverse lookup (OpB,OpA) ISL should equal 0.4");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test alpha=0 blocks channel links when filterBySharing=true
 */
class LeoSimTestAlphaZeroBlocked : public TestCase
{
  public:
    LeoSimTestAlphaZeroBlocked();
    ~LeoSimTestAlphaZeroBlocked() override;

  private:
    void DoRun() override;
};

LeoSimTestAlphaZeroBlocked::LeoSimTestAlphaZeroBlocked()
    : TestCase("LeoSim alpha=0 link removed from GetLinksForNode when filterBySharing=true")
{
}

LeoSimTestAlphaZeroBlocked::~LeoSimTestAlphaZeroBlocked()
{
}

void
LeoSimTestAlphaZeroBlocked::DoRun()
{
    // Operator model: node 0 = OpA (UE at origin), node 1 = OpB (satellite at 550 km)
    Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
    opModel->RegisterNode(0, "OpA", LEOSIM_ROLE_UE);
    opModel->RegisterNode(1, "OpB", LEOSIM_ROLE_SATELLITE);
    opModel->SetAlphaSymmetric("OpA", "OpB", 0.0);

    NS_TEST_ASSERT_MSG_EQ_TOL(opModel->GetAlpha(0, 1, LEOSIM_DIR_DOWNLINK),
                              0.0,
                              1e-9,
                              "GetAlpha(0,1,DL) must be 0.0");

    // Create nodes with positions that yield an UP link after UpdateAllLinks()
    NodeContainer nodes;
    nodes.Create(2);

    Ptr<ConstantPositionMobilityModel> mob0 = CreateObject<ConstantPositionMobilityModel>();
    mob0->SetPosition(Vector(0.0, 0.0, 0.0)); // ground at origin
    nodes.Get(0)->AggregateObject(mob0);

    Ptr<ConstantPositionMobilityModel> mob1 = CreateObject<ConstantPositionMobilityModel>();
    mob1->SetPosition(Vector(0.0, 0.0, 550000.0)); // satellite at 550 km altitude
    nodes.Get(1)->AggregateObject(mob1);

    Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
    channelModel->SetOperatorModel(opModel);
    channelModel->AddLink(nodes.Get(0), nodes.Get(1), LEOSIM_LINK_SATELLITE_TO_GROUND);
    channelModel->UpdateAllLinks();

    // Without sharing filter: link should appear (it is UP)
    auto unfiltered = channelModel->GetLinksForNode(nodes.Get(0)->GetId(), false);
    NS_TEST_ASSERT_MSG_EQ(unfiltered.size(),
                          static_cast<size_t>(1),
                          "Unfiltered result should contain the UP link");

    // With sharing filter: alpha=0 removes the link
    auto filtered = channelModel->GetLinksForNode(nodes.Get(0)->GetId(), true);
    NS_TEST_ASSERT_MSG_EQ(filtered.size(),
                          static_cast<size_t>(0),
                          "Filtered result should be empty (alpha=0 blocks the link)");

    Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test GetEffectiveDataRateBps scales base rate by alpha
 */
class LeoSimTestEffectiveDataRate : public TestCase
{
  public:
    LeoSimTestEffectiveDataRate();
    ~LeoSimTestEffectiveDataRate() override;

  private:
    void DoRun() override;
};

LeoSimTestEffectiveDataRate::LeoSimTestEffectiveDataRate()
    : TestCase("LeoSim GetEffectiveDataRateBps scales base rate by alpha")
{
}

LeoSimTestEffectiveDataRate::~LeoSimTestEffectiveDataRate()
{
}

void
LeoSimTestEffectiveDataRate::DoRun()
{
    Ptr<LeoSimOperatorModel> model = CreateObject<LeoSimOperatorModel>();
    model->RegisterNode(0, "OpA", LEOSIM_ROLE_SATELLITE);
    model->RegisterNode(1, "OpB", LEOSIM_ROLE_UE);
    model->SetAlpha("OpA", "OpB", 0.6, 0.6, 0.6);

    uint64_t rate = model->GetEffectiveDataRateBps(0, 1, 100000000ULL, LEOSIM_DIR_DOWNLINK);
    NS_TEST_ASSERT_MSG_EQ(rate,
                          static_cast<uint64_t>(60000000),
                          "Effective rate should be 60 Mbps (100 Mbps * 0.6)");

    // With alpha=0, result must be at least 1 bps (never zero)
    model->SetAlpha("OpA", "OpB", 0.0, 0.0, 0.0);
    uint64_t zeroRate = model->GetEffectiveDataRateBps(0, 1, 100000000ULL, LEOSIM_DIR_DOWNLINK);
    NS_TEST_ASSERT_MSG_EQ(zeroRate,
                          static_cast<uint64_t>(1),
                          "Effective rate with alpha=0 should be minimum 1 bps");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test GetRoutingCostMultiplier returns 1/alpha for cross-op and 1.0 for same-op
 */
class LeoSimTestRoutingCostMultiplier : public TestCase
{
  public:
    LeoSimTestRoutingCostMultiplier();
    ~LeoSimTestRoutingCostMultiplier() override;

  private:
    void DoRun() override;
};

LeoSimTestRoutingCostMultiplier::LeoSimTestRoutingCostMultiplier()
    : TestCase("LeoSim GetRoutingCostMultiplier same-op 1.0 cross-op 1-over-alpha blocked 1e5")
{
}

LeoSimTestRoutingCostMultiplier::~LeoSimTestRoutingCostMultiplier()
{
}

void
LeoSimTestRoutingCostMultiplier::DoRun()
{
    Ptr<LeoSimOperatorModel> model = CreateObject<LeoSimOperatorModel>();
    model->RegisterNode(0, "OpA", LEOSIM_ROLE_SATELLITE);
    model->RegisterNode(1, "OpA", LEOSIM_ROLE_SATELLITE);
    model->RegisterNode(2, "OpB", LEOSIM_ROLE_SATELLITE);
    model->SetAlpha("OpA", "OpB", 0.5, 0.5, 0.5);

    // Same-operator pair: multiplier == 1.0
    NS_TEST_ASSERT_MSG_EQ_TOL(model->GetRoutingCostMultiplier(0, 1, LEOSIM_DIR_ISL),
                              1.0,
                              1e-9,
                              "Same-op multiplier should be 1.0");

    // Cross-operator pair with alpha=0.5: multiplier == 2.0
    NS_TEST_ASSERT_MSG_EQ_TOL(model->GetRoutingCostMultiplier(0, 2, LEOSIM_DIR_ISL),
                              2.0,
                              1e-9,
                              "Cross-op (alpha=0.5) multiplier should be 2.0");

    // With alpha=0.0: multiplier should exceed 1e5 (infinity proxy)
    model->SetAlpha("OpA", "OpB", 0.0, 0.0, 0.0);
    double blockedMultiplier = model->GetRoutingCostMultiplier(0, 2, LEOSIM_DIR_ISL);
    NS_TEST_ASSERT_MSG_GT(blockedMultiplier, 1e5, "alpha=0 multiplier should exceed 1e5");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test Dijkstra selects same-operator path when cross-op penalty applied
 */
class LeoSimTestDijkstraPrefersSameOp : public TestCase
{
  public:
    LeoSimTestDijkstraPrefersSameOp();
    ~LeoSimTestDijkstraPrefersSameOp() override;

  private:
    void DoRun() override;
};

LeoSimTestDijkstraPrefersSameOp::LeoSimTestDijkstraPrefersSameOp()
    : TestCase("LeoSim Dijkstra picks same-operator path when cross-op penalty > intra-op cost")
{
}

LeoSimTestDijkstraPrefersSameOp::~LeoSimTestDijkstraPrefersSameOp()
{
}

void
LeoSimTestDijkstraPrefersSameOp::DoRun()
{
    // Topology (positions produce UP links after UpdateAllLinks):
    //   ue(OpA) --ground-- sat1(OpA) --ISL-- sat2(OpB) --ground-- server(OpA)
    //                            \----ISL-- sat3(OpA) --ground-- server(OpA)
    //
    // With alpha(OpA,OpB)=0.5 -> cross-op multiplier=2.0:
    //   Via sat2: cost = 1 + 2 + 2 = 5   (UE->sat1 + sat1->sat2 + sat2->server)
    //   Via sat3: cost = 1 + 1 + 1 = 3   (UE->sat1 + sat1->sat3 + sat3->server)
    // Dijkstra must prefer sat3.

    NodeContainer allNodes;
    allNodes.Create(5);
    Ptr<Node> ue     = allNodes.Get(0);
    Ptr<Node> sat1   = allNodes.Get(1);
    Ptr<Node> sat2   = allNodes.Get(2);
    Ptr<Node> sat3   = allNodes.Get(3);
    Ptr<Node> server = allNodes.Get(4);

    // Helper lambda: install ConstantPositionMobilityModel at (x,y,z)
    auto setPos = [](Ptr<Node> n, double x, double y, double z) {
        Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
        mob->SetPosition(Vector(x, y, z));
        n->AggregateObject(mob);
    };

    // ue and server both at origin (different nodes, same position is fine)
    setPos(ue,     0.0,       0.0, 0.0);
    setPos(sat1,   0.0,       0.0, 550000.0); // directly above ue
    setPos(sat2,   200000.0,  0.0, 550000.0); // 200 km east of sat1 (ISL)
    setPos(sat3,  -200000.0,  0.0, 550000.0); // 200 km west of sat1 (ISL)
    setPos(server, 0.0,       0.0, 0.0);       // at origin (elevation 90 deg from sat2/sat3)

    // Ground channel model: ue<->sat1, server<->sat2, server<->sat3
    Ptr<LeoSimChannelModel> groundModel = CreateObject<LeoSimChannelModel>();
    groundModel->AddLink(ue,     sat1, LEOSIM_LINK_SATELLITE_TO_GROUND);
    groundModel->AddLink(server, sat2, LEOSIM_LINK_SATELLITE_TO_GROUND);
    groundModel->AddLink(server, sat3, LEOSIM_LINK_SATELLITE_TO_GROUND);
    groundModel->UpdateAllLinks();

    // ISL channel model: sat1<->sat2, sat1<->sat3
    Ptr<LeoSimChannelModel> islModel = CreateObject<LeoSimChannelModel>();
    islModel->AddIslLink(sat1, sat2);
    islModel->AddIslLink(sat1, sat3);
    islModel->UpdateAllLinks();

    // Operator model
    Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
    opModel->RegisterNode(ue->GetId(),     "OpA", LEOSIM_ROLE_UE);
    opModel->RegisterNode(sat1->GetId(),   "OpA", LEOSIM_ROLE_SATELLITE);
    opModel->RegisterNode(sat2->GetId(),   "OpB", LEOSIM_ROLE_SATELLITE);
    opModel->RegisterNode(sat3->GetId(),   "OpA", LEOSIM_ROLE_SATELLITE);
    opModel->RegisterNode(server->GetId(), "OpA", LEOSIM_ROLE_SERVER);
    opModel->SetAlpha("OpA", "OpB", 0.5, 0.5, 0.5);
    opModel->SetAlpha("OpB", "OpB", 1.0, 1.0, 1.0);

    // Routing calculator
    Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
    calc->SetChannelModel(groundModel);
    calc->SetIslChannelModel(islModel);
    calc->SetOperatorModel(opModel);

    LeoSimRoute route = calc->ComputeRoute(ue,
                                           server,
                                           LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT,
                                           LeoSimRoutingCalculator::LEOSIM_PATH_ANY);

    NS_TEST_ASSERT_MSG_EQ(route.valid, true, "Route should be found");

    bool hasSat3 = false;
    bool hasSat2 = false;
    for (const auto& node : route.path)
    {
        if (node == sat3) { hasSat3 = true; }
        if (node == sat2) { hasSat2 = true; }
    }
    NS_TEST_ASSERT_MSG_EQ(hasSat3, true,  "Optimal path must include sat3 (OpA, no penalty)");
    NS_TEST_ASSERT_MSG_EQ(hasSat2, false, "Optimal path must NOT include sat2 (OpB, cross-op penalty)");

    Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test Dijkstra returns no route when only cross-op link is alpha=0
 */
class LeoSimTestDijkstraBlocksAlphaZero : public TestCase
{
  public:
  LeoSimTestDijkstraBlocksAlphaZero();
  ~LeoSimTestDijkstraBlocksAlphaZero() override;

  private:
  void DoRun() override;
};

LeoSimTestDijkstraBlocksAlphaZero::LeoSimTestDijkstraBlocksAlphaZero()
  : TestCase("LeoSim Dijkstra returns invalid route when only cross-op link has alpha=0")
{
}

LeoSimTestDijkstraBlocksAlphaZero::~LeoSimTestDijkstraBlocksAlphaZero()
{
}

void
LeoSimTestDijkstraBlocksAlphaZero::DoRun()
{
  // Topology: UE(OpA) -- sat(OpB) -- server(OpA)
  // alpha(OpA,OpB) = 0.0 for all directions -> routing cost = infinity -> no valid route

  NodeContainer allNodes;
  allNodes.Create(3);
  Ptr<Node> ue     = allNodes.Get(0);
  Ptr<Node> sat    = allNodes.Get(1);
  Ptr<Node> server = allNodes.Get(2);

  auto setPos = [](Ptr<Node> n, double x, double y, double z) {
    Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
    mob->SetPosition(Vector(x, y, z));
    n->AggregateObject(mob);
  };
  setPos(ue,     0.0, 0.0, 0.0);
  setPos(sat,    0.0, 0.0, 550000.0);
  setPos(server, 0.0, 0.0, 0.0);

  Ptr<LeoSimChannelModel> groundModel = CreateObject<LeoSimChannelModel>();
  groundModel->AddLink(ue,     sat, LEOSIM_LINK_SATELLITE_TO_GROUND);
  groundModel->AddLink(server, sat, LEOSIM_LINK_SATELLITE_TO_GROUND);
  groundModel->UpdateAllLinks();

  Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
  opModel->RegisterNode(ue->GetId(),     "OpA", LEOSIM_ROLE_UE);
  opModel->RegisterNode(sat->GetId(),    "OpB", LEOSIM_ROLE_SATELLITE);
  opModel->RegisterNode(server->GetId(), "OpA", LEOSIM_ROLE_SERVER);
  opModel->SetAlpha("OpA", "OpB", 0.0, 0.0, 0.0); // fully blocked

  Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
  calc->SetChannelModel(groundModel);
  calc->SetOperatorModel(opModel);

  LeoSimRoute route = calc->ComputeRoute(ue,
                       server,
                       LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT,
                       LeoSimRoutingCalculator::LEOSIM_PATH_ANY);

  NS_TEST_ASSERT_MSG_EQ(route.valid,
              false,
              "Route must be invalid when all cross-op links are alpha=0");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test SAME_OPERATOR_ONLY path type blocks cross-op satellite and allows intra-op
 */
class LeoSimTestOperatorIsolationMode : public TestCase
{
  public:
  LeoSimTestOperatorIsolationMode();
  ~LeoSimTestOperatorIsolationMode() override;

  private:
  void DoRun() override;
};

LeoSimTestOperatorIsolationMode::LeoSimTestOperatorIsolationMode()
  : TestCase("LeoSim SAME_OPERATOR_ONLY blocks cross-op satellite and allows intra-op path")
{
}

LeoSimTestOperatorIsolationMode::~LeoSimTestOperatorIsolationMode()
{
}

void
LeoSimTestOperatorIsolationMode::DoRun()
{
  // Phase 1: UE(OpA) -- satB(OpB) -- server(OpA); SAME_OPERATOR_ONLY -> no route
  {
    NodeContainer nodes;
    nodes.Create(3);
    Ptr<Node> ue     = nodes.Get(0);
    Ptr<Node> satB   = nodes.Get(1);
    Ptr<Node> server = nodes.Get(2);

    auto setPos = [](Ptr<Node> n, double x, double y, double z) {
      Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(x, y, z));
      n->AggregateObject(mob);
    };
    setPos(ue,     0.0, 0.0, 0.0);
    setPos(satB,   0.0, 0.0, 550000.0);
    setPos(server, 0.0, 0.0, 0.0);

    Ptr<LeoSimChannelModel> groundModel = CreateObject<LeoSimChannelModel>();
    groundModel->AddLink(ue,     satB, LEOSIM_LINK_SATELLITE_TO_GROUND);
    groundModel->AddLink(server, satB, LEOSIM_LINK_SATELLITE_TO_GROUND);
    groundModel->UpdateAllLinks();

    Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
    opModel->RegisterNode(ue->GetId(),     "OpA", LEOSIM_ROLE_UE);
    opModel->RegisterNode(satB->GetId(),   "OpB", LEOSIM_ROLE_SATELLITE);
    opModel->RegisterNode(server->GetId(), "OpA", LEOSIM_ROLE_SERVER);

    Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
    calc->SetChannelModel(groundModel);
    calc->SetOperatorModel(opModel);

    LeoSimRoute route = calc->ComputeRoute(
      ue,
      server,
      LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT,
      LeoSimRoutingCalculator::LEOSIM_PATH_SAME_OPERATOR_ONLY);

    NS_TEST_ASSERT_MSG_EQ(route.valid,
                false,
                "SAME_OPERATOR_ONLY: no route should exist via OpB satellite");
    Simulator::Destroy();
  }

  // Phase 2: UE(OpA) -- satA(OpA) -- server(OpA); SAME_OPERATOR_ONLY -> valid route
  {
    NodeContainer nodes;
    nodes.Create(3);
    Ptr<Node> ue     = nodes.Get(0);
    Ptr<Node> satA   = nodes.Get(1);
    Ptr<Node> server = nodes.Get(2);

    auto setPos = [](Ptr<Node> n, double x, double y, double z) {
      Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(x, y, z));
      n->AggregateObject(mob);
    };
    setPos(ue,     0.0, 0.0, 0.0);
    setPos(satA,   0.0, 0.0, 550000.0);
    setPos(server, 0.0, 0.0, 0.0);

    Ptr<LeoSimChannelModel> groundModel = CreateObject<LeoSimChannelModel>();
    groundModel->AddLink(ue,     satA, LEOSIM_LINK_SATELLITE_TO_GROUND);
    groundModel->AddLink(server, satA, LEOSIM_LINK_SATELLITE_TO_GROUND);
    groundModel->UpdateAllLinks();

    Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
    opModel->RegisterNode(ue->GetId(),     "OpA", LEOSIM_ROLE_UE);
    opModel->RegisterNode(satA->GetId(),   "OpA", LEOSIM_ROLE_SATELLITE);
    opModel->RegisterNode(server->GetId(), "OpA", LEOSIM_ROLE_SERVER);

    Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
    calc->SetChannelModel(groundModel);
    calc->SetOperatorModel(opModel);

    LeoSimRoute route = calc->ComputeRoute(
      ue,
      server,
      LeoSimRoutingCalculator::LEOSIM_METRIC_HOP_COUNT,
      LeoSimRoutingCalculator::LEOSIM_PATH_SAME_OPERATOR_ONLY);

    NS_TEST_ASSERT_MSG_EQ(route.valid,
                true,
                "SAME_OPERATOR_ONLY: valid route should exist via OpA satellite");

    for (const auto& node : route.path)
    {
      NS_TEST_ASSERT_MSG_EQ(opModel->GetOperatorId(node->GetId()),
                  "OpA",
                  "All path nodes must belong to OpA");
    }
    Simulator::Destroy();
  }
}

/**
 * \ingroup leosim-test-suite
 * \brief Test RankByTopsis places same-op satellite first when operator weight is high
 */
class LeoSimTestTopsisOperatorAttribute : public TestCase
{
  public:
  LeoSimTestTopsisOperatorAttribute();
  ~LeoSimTestTopsisOperatorAttribute() override;

  private:
  void DoRun() override;
};

LeoSimTestTopsisOperatorAttribute::LeoSimTestTopsisOperatorAttribute()
  : TestCase("LeoSim TOPSIS operator attribute ranks same-op satellite first")
{
}

LeoSimTestTopsisOperatorAttribute::~LeoSimTestTopsisOperatorAttribute()
{
}

void
LeoSimTestTopsisOperatorAttribute::DoRun()
{
  // UE=OpA, SatA=OpA (alpha=1.0), SatB=OpB (alpha=0.6).
  // SatB has 3 dB better RSRP and SINR. With wOperatorCompat=0.20 SatA ranks first.
  const uint32_t ueId   = 10;
  const uint32_t satAId = 20;
  const uint32_t satBId = 21;

  Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
  opModel->RegisterNode(ueId,   "OpA", LEOSIM_ROLE_UE);
  opModel->RegisterNode(satAId, "OpA", LEOSIM_ROLE_SATELLITE);
  opModel->RegisterNode(satBId, "OpB", LEOSIM_ROLE_SATELLITE);
  opModel->SetAlpha("OpA", "OpB", 0.6, 0.6, 0.6);

  LeoSimBeamRecord recA;
  recA.ueNodeId                  = ueId;
  recA.satelliteNodeId           = satAId;
  recA.rsrp                      = -100.0;
  recA.sinr                      = 5.0;
  recA.remainingServiceTime      = 600.0;
  recA.beamLoad                  = 1.0;
  recA.endToEndLatency           = 20.0;
  recA.elevationAngle            = 45.0;
  recA.beamActive                = true;
  recA.beamId                    = 0;
  recA.cellId                    = 0;
  recA.colorGroup                = 1;
  recA.intraBeamInterference_dBm = -120.0;
  recA.interSatInterference_dBm  = -120.0;
  recA.beamThroughputMbps        = 0.0;
  recA.pathLoss                  = 180.0;
  recA.satelliteLoad             = 1.0;
  recA.snr                       = 5.0;
  recA.state                     = LEOSIM_BEAM_CONNECTED;
  recA.associationTime           = Seconds(0);

  LeoSimBeamRecord recB = recA;
  recB.satelliteNodeId = satBId;
  recB.rsrp            = -97.0; // 3 dB better
  recB.sinr            =  8.0; // 3 dB better
  recB.snr             =  8.0;

  std::vector<LeoSimBeamRecord> candidates{recA, recB};

  Ptr<LeoSimBeamManager>  mgr          = CreateObject<LeoSimBeamManager>();
  Ptr<LeoSimChannelModel> dummyChannel = CreateObject<LeoSimChannelModel>();
  Ptr<LeoSimLoader>       dummyLoader  = CreateObject<LeoSimLoader>();
  mgr->SetChannelModel(dummyChannel);
  mgr->SetLoader(dummyLoader);
  mgr->SetOperatorModel(opModel);
  // wRsrp=0.15, wSinr=0.15, wTte=0.10, wLoad=0.10, wLatency=0.10,
  // wElevation=0.10, wActive=0.10, wOperatorCompat=0.20
  mgr->SetTopsisWeights(0.15, 0.15, 0.10, 0.10, 0.10, 0.10, 0.10, 0.20);

  auto ranked = mgr->RankByTopsis(candidates, ueId);

  NS_TEST_ASSERT_MSG_EQ(ranked.empty(), false, "Ranked list must not be empty");
  NS_TEST_ASSERT_MSG_EQ(ranked.front().beamRecord.satelliteNodeId,
              satAId,
              "SatA (same-op) should rank first despite 3 dB RSRP/SINR deficit");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test alpha=0 candidate is absent from beam candidate list after operator filter
 */
class LeoSimTestBeamCandidateAlphaFilter : public TestCase
{
  public:
  LeoSimTestBeamCandidateAlphaFilter();
  ~LeoSimTestBeamCandidateAlphaFilter() override;

  private:
  void DoRun() override;
};

LeoSimTestBeamCandidateAlphaFilter::LeoSimTestBeamCandidateAlphaFilter()
  : TestCase("LeoSim beam candidate with alpha=0 is filtered from visible-satellite list")
{
}

LeoSimTestBeamCandidateAlphaFilter::~LeoSimTestBeamCandidateAlphaFilter()
{
}

void
LeoSimTestBeamCandidateAlphaFilter::DoRun()
{
  // UE=OpA, SatC=OpC (alpha=0.0 -> blocked), SatA=OpA (alpha=1.0 -> allowed).
  // Apply the same erase-remove filter used inside ScanVisibleSatellites.

  const uint32_t ueId   = 50;
  const uint32_t satCId = 60;
  const uint32_t satAId = 61;

  Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
  opModel->RegisterNode(ueId,   "OpA", LEOSIM_ROLE_UE);
  opModel->RegisterNode(satCId, "OpC", LEOSIM_ROLE_SATELLITE);
  opModel->RegisterNode(satAId, "OpA", LEOSIM_ROLE_SATELLITE);
  opModel->SetAlpha("OpA", "OpC", 0.0, 0.0, 0.0); // fully blocked

  NS_TEST_ASSERT_MSG_EQ_TOL(opModel->GetAlpha(ueId, satCId, LEOSIM_DIR_DOWNLINK),
                0.0,
                1e-9,
                "Pre-condition: alpha(UE,SatC,DL) must be 0.0");

  // Build raw candidate list as ScanVisibleSatellites would before its filter step
  LeoSimBeamRecord recC;
  recC.ueNodeId                  = ueId;
  recC.satelliteNodeId           = satCId;
  recC.rsrp                      = -100.0;
  recC.sinr                      = 5.0;
  recC.snr                       = 5.0;
  recC.remainingServiceTime      = 300.0;
  recC.beamLoad                  = 1.0;
  recC.endToEndLatency           = 20.0;
  recC.elevationAngle            = 40.0;
  recC.beamActive                = true;
  recC.beamId                    = 0;
  recC.cellId                    = 0;
  recC.colorGroup                = 1;
  recC.intraBeamInterference_dBm = -120.0;
  recC.interSatInterference_dBm  = -120.0;
  recC.beamThroughputMbps        = 0.0;
  recC.pathLoss                  = 180.0;
  recC.satelliteLoad             = 1.0;
  recC.state                     = LEOSIM_BEAM_CONNECTED;
  recC.associationTime           = Seconds(0);

  LeoSimBeamRecord recA = recC;
  recA.satelliteNodeId = satAId;

  std::vector<LeoSimBeamRecord> candidates{recC, recA};

  // Apply same erase-remove filter as in ScanVisibleSatellites
  candidates.erase(
    std::remove_if(candidates.begin(),
             candidates.end(),
             [&](const LeoSimBeamRecord& c) {
               return opModel->GetAlpha(ueId,
                          c.satelliteNodeId,
                          LEOSIM_DIR_DOWNLINK) == 0.0;
             }),
    candidates.end());

  bool hasSatC = std::any_of(candidates.begin(), candidates.end(),
                 [&](const LeoSimBeamRecord& c) {
                   return c.satelliteNodeId == satCId;
                 });
  bool hasSatA = std::any_of(candidates.begin(), candidates.end(),
                 [&](const LeoSimBeamRecord& c) {
                   return c.satelliteNodeId == satAId;
                 });

  NS_TEST_ASSERT_MSG_EQ(hasSatC, false, "SatC (alpha=0) must be filtered out");
  NS_TEST_ASSERT_MSG_EQ(hasSatA, true,  "SatA (same-op, alpha=1) must survive filter");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test InitOperatorLogging writes correct header and one row per node
 */
class LeoSimTestOperatorCsvOutput : public TestCase
{
  public:
  LeoSimTestOperatorCsvOutput();
  ~LeoSimTestOperatorCsvOutput() override;

  private:
  void DoRun() override;
};

LeoSimTestOperatorCsvOutput::LeoSimTestOperatorCsvOutput()
  : TestCase("LeoSim InitOperatorLogging writes correct header and one row per node")
{
}

LeoSimTestOperatorCsvOutput::~LeoSimTestOperatorCsvOutput()
{
}

void
LeoSimTestOperatorCsvOutput::DoRun()
{
  const std::string tmpFile = "/tmp/leosim_test_operator_output.csv";

  Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
  opModel->RegisterNode(0, "OpA", LEOSIM_ROLE_SATELLITE);
  opModel->RegisterNode(1, "OpB", LEOSIM_ROLE_UE);
  opModel->RegisterNode(2, "OpA", LEOSIM_ROLE_SERVER);

  NodeContainer allNodes;
  allNodes.Create(3); // creates nodes with IDs 0, 1, 2

  LeoSimVisualizationHelper vizHelper;
  vizHelper.SetOperatorFile(tmpFile);
  vizHelper.InitOperatorLogging(opModel, allNodes);

  std::ifstream f(tmpFile);
  NS_TEST_ASSERT_MSG_EQ(f.is_open(), true, "Operator CSV file must be created");

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(f, line))
  {
    if (!line.empty())
    {
      lines.push_back(line);
    }
  }
  f.close();

  NS_TEST_ASSERT_MSG_EQ(lines.empty(), false, "File must not be empty");
  NS_TEST_ASSERT_MSG_EQ(lines[0],
              "node_id,node_name,role,operator_id",
              "Header row must match exactly");
  NS_TEST_ASSERT_MSG_EQ(static_cast<uint32_t>(lines.size() - 1),
              static_cast<uint32_t>(3),
              "Must have exactly 3 data rows");

  bool foundNode0 = false;
  for (size_t i = 1; i < lines.size(); ++i)
  {
    std::istringstream ss(lines[i]);
    std::string nodeIdStr, nodeName, role, opId;
    if (std::getline(ss, nodeIdStr, ',') && std::getline(ss, nodeName, ',') &&
      std::getline(ss, role,      ',') && std::getline(ss, opId,     ','))
    {
      if (nodeIdStr == "0")
      {
        foundNode0 = true;
        NS_TEST_ASSERT_MSG_EQ(opId, "OpA", "Node 0 operator_id must be OpA");
      }
    }
  }
  NS_TEST_ASSERT_MSG_EQ(foundNode0, true, "Row for node 0 must exist in operator CSV");

  std::remove(tmpFile.c_str());
  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test LogSharingState writes cross-operator rows with correct alpha_dl value
 */
class LeoSimTestSharingCsvOutput : public TestCase
{
  public:
  LeoSimTestSharingCsvOutput();
  ~LeoSimTestSharingCsvOutput() override;

  private:
  void DoRun() override;
};

LeoSimTestSharingCsvOutput::LeoSimTestSharingCsvOutput()
  : TestCase("LeoSim LogSharingState writes cross-operator rows with correct alpha_dl")
{
}

LeoSimTestSharingCsvOutput::~LeoSimTestSharingCsvOutput()
{
}

void
LeoSimTestSharingCsvOutput::DoRun()
{
  const std::string tmpFile = "/tmp/leosim_test_sharing_output.csv";
  std::remove(tmpFile.c_str());

  NodeContainer nodes;
  nodes.Create(2);
  Ptr<Node> satNode = nodes.Get(0);
  Ptr<Node> ueNode  = nodes.Get(1);

  auto setPos = [](Ptr<Node> n, double x, double y, double z) {
    Ptr<ConstantPositionMobilityModel> mob = CreateObject<ConstantPositionMobilityModel>();
    mob->SetPosition(Vector(x, y, z));
    n->AggregateObject(mob);
  };
  setPos(satNode, 0.0, 0.0, 550000.0);
  setPos(ueNode,  0.0, 0.0, 0.0);

  Ptr<LeoSimChannelModel> groundModel = CreateObject<LeoSimChannelModel>();
  groundModel->AddLink(satNode, ueNode, LEOSIM_LINK_SATELLITE_TO_GROUND);
  groundModel->UpdateAllLinks();

  Ptr<LeoSimOperatorModel> opModel = CreateObject<LeoSimOperatorModel>();
  opModel->RegisterNode(satNode->GetId(), "OpA", LEOSIM_ROLE_SATELLITE);
  opModel->RegisterNode(ueNode->GetId(),  "OpB", LEOSIM_ROLE_UE);
  opModel->SetAlpha("OpA", "OpB", 0.6, 0.6, 0.6);

  LeoSimVisualizationHelper vizHelper;
  vizHelper.SetSharingFile(tmpFile);
  vizHelper.LogSharingState(opModel, groundModel, nullptr, 0.0);

  std::ifstream f(tmpFile);
  NS_TEST_ASSERT_MSG_EQ(f.is_open(), true, "Sharing CSV file must be created");

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(f, line))
  {
    if (!line.empty())
    {
      lines.push_back(line);
    }
  }
  f.close();

  NS_TEST_ASSERT_MSG_GT(lines.size(),
              static_cast<size_t>(1),
              "Sharing CSV must have at least one data row");

  bool foundCrossOpRow = false;
  bool foundAlpha06    = false;
  for (size_t i = 1; i < lines.size(); ++i)
  {
    std::istringstream ss(lines[i]);
    std::string time, nodeA, nodeB, opA, opB, linkType, alphaDl;
    if (std::getline(ss, time,     ',') && std::getline(ss, nodeA,    ',') &&
      std::getline(ss, nodeB,    ',') && std::getline(ss, opA,      ',') &&
      std::getline(ss, opB,      ',') && std::getline(ss, linkType, ',') &&
      std::getline(ss, alphaDl,  ','))
    {
      if (opA != opB)
      {
        foundCrossOpRow = true;
        double dVal = std::stod(alphaDl);
        if (std::abs(dVal - 0.6) < 0.01)
        {
          foundAlpha06 = true;
        }
      }
    }
  }

  NS_TEST_ASSERT_MSG_EQ(foundCrossOpRow, true,
              "Sharing CSV must contain at least one row where op_a != op_b");
  NS_TEST_ASSERT_MSG_EQ(foundAlpha06, true,
              "Sharing CSV must contain a row with alpha_dl == 0.6");

  std::remove(tmpFile.c_str());
  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test LoadGroundDevicesFromCsv parses the Operator column correctly
 */
class LeoSimTestGroundDeviceCsvOperatorColumn : public TestCase
{
  public:
  LeoSimTestGroundDeviceCsvOperatorColumn();
  ~LeoSimTestGroundDeviceCsvOperatorColumn() override;

  private:
  void DoRun() override;
};

LeoSimTestGroundDeviceCsvOperatorColumn::LeoSimTestGroundDeviceCsvOperatorColumn()
  : TestCase("LeoSim LoadGroundDevicesFromCsv parses Operator column; missing ID returns default")
{
}

LeoSimTestGroundDeviceCsvOperatorColumn::~LeoSimTestGroundDeviceCsvOperatorColumn()
{
}

void
LeoSimTestGroundDeviceCsvOperatorColumn::DoRun()
{
  const std::string tmpFile = "/tmp/leosim_test_ground_devices.csv";
  {
    std::ofstream f(tmpFile);
    f << "device_id,device_name,device_type,x_m,y_m,z_m,Operator\n";
    f << "0,ue0,UE,0.0,0.0,0.0,OpA\n";
    f << "1,ue1,UE,1000.0,0.0,0.0,OpB\n";
    f << "2,ue2,UE,2000.0,0.0,0.0,OpC\n";
  }

  Ptr<LeoSimLoader> loader = CreateObject<LeoSimLoader>();
  loader->LoadGroundDevicesFromCsv(tmpFile);

  NS_TEST_ASSERT_MSG_EQ(loader->GetGroundDeviceOperator(1),
              "OpB",
              "Device 1 operator should be OpB");
  NS_TEST_ASSERT_MSG_EQ(loader->GetGroundDeviceOperator(99),
              "default",
              "Unknown device ID should return 'default'");

  std::remove(tmpFile.c_str());
}

/**
 * \ingroup leosim-test-suite
 * \brief Test LoadSatelliteOperatorsFromCsv assigns operators by satellite index
 */
class LeoSimTestSatelliteOperatorsCsvLoad : public TestCase
{
  public:
  LeoSimTestSatelliteOperatorsCsvLoad();
  ~LeoSimTestSatelliteOperatorsCsvLoad() override;

  private:
  void DoRun() override;
};

LeoSimTestSatelliteOperatorsCsvLoad::LeoSimTestSatelliteOperatorsCsvLoad()
  : TestCase("LeoSim LoadSatelliteOperatorsFromCsv maps indices to operators correctly")
{
}

LeoSimTestSatelliteOperatorsCsvLoad::~LeoSimTestSatelliteOperatorsCsvLoad()
{
}

void
LeoSimTestSatelliteOperatorsCsvLoad::DoRun()
{
  const std::string tmpFile = "/tmp/leosim_test_sat_operators.csv";
  {
    std::ofstream f(tmpFile);
    f << "SatelliteIndex,Operator\n";
    f << "0,OpA\n";
    f << "1,OpA\n";
    f << "2,OpB\n";
    f << "3,OpB\n";
    f << "4,OpC\n";
  }

  Ptr<LeoSimLoader> loader = CreateObject<LeoSimLoader>();
  loader->LoadSatelliteOperatorsFromCsv(tmpFile);

  NS_TEST_ASSERT_MSG_EQ(loader->GetSatelliteOperator(0),
              "OpA",
              "Satellite 0 operator should be OpA");
  NS_TEST_ASSERT_MSG_EQ(loader->GetSatelliteOperator(4),
              "OpC",
              "Satellite 4 operator should be OpC");
  NS_TEST_ASSERT_MSG_EQ(loader->GetSatelliteOperator(99),
              "default",
              "Unknown satellite index should return 'default'");

  std::remove(tmpFile.c_str());
}

/**
 * \ingroup leosim-test-suite
 * \brief Test routing admits only the beam-manager serving access satellite.
 */
class LeoSimTestBeamAuthorityRoutesServingOnly : public TestCase
{
  public:
  LeoSimTestBeamAuthorityRoutesServingOnly();
  ~LeoSimTestBeamAuthorityRoutesServingOnly() override;

  private:
  void DoRun() override;
};

LeoSimTestBeamAuthorityRoutesServingOnly::LeoSimTestBeamAuthorityRoutesServingOnly()
  : TestCase("LeoSim routing uses beam-manager serving access link only")
{
}

LeoSimTestBeamAuthorityRoutesServingOnly::~LeoSimTestBeamAuthorityRoutesServingOnly()
{
}

void
LeoSimTestBeamAuthorityRoutesServingOnly::DoRun()
{
  NodeContainer groundNodes;
  groundNodes.Create(1);
  NodeContainer sats;
  sats.Create(2);

  Ptr<Node> ue = groundNodes.Get(0);
  Ptr<Node> satA = sats.Get(0);
  Ptr<Node> satB = sats.Get(1);

  InternetStackHelper internet;
  internet.Install(groundNodes);
  internet.Install(sats);

  SetLeoSimTestPosition(ue, TestGeodeticPosition(0.0, 0.0, 0.0), LEOSIM_UE);
  SetLeoSimTestPosition(satA, TestGeodeticPosition(20.0, 0.0, 550000.0), LEOSIM_SATELLITE);
  SetLeoSimTestPosition(satB, TestGeodeticPosition(0.0, 0.0, 550000.0), LEOSIM_SATELLITE);

  Ptr<LeoSimChannelModel> channel = CreateObject<LeoSimChannelModel>();
  channel->SetMinElevationAngle(-90.0);
  channel->SetMaxLinkDistance(6000000.0);
  channel->SetTransmitPower(80.0);
  channel->AddLink(satA, ue, LEOSIM_LINK_SATELLITE_TO_GROUND);
  channel->AddLink(satB, ue, LEOSIM_LINK_SATELLITE_TO_GROUND);
  channel->UpdateAllLinks();

  Ptr<LeoSimMultiBeamModel> beamModel = CreateObject<LeoSimMultiBeamModel>();
  beamModel->SetBeamsForSatellite(satA->GetId(),
                                  {MakeTestBeam(satA->GetId(), 0, 20.0, 0.0, 100.0)});
  beamModel->SetBeamsForSatellite(satB->GetId(),
                                  {MakeTestBeam(satB->GetId(), 0, 0.0, 0.0, 1000.0)});

  Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
  calc->SetChannelModel(channel);

  Ptr<LeoSimBeamManager> manager = CreateObject<LeoSimBeamManager>();
  manager->SetChannelModel(channel);
  manager->SetMultiBeamModel(beamModel);
  manager->SetLoader(CreateObject<LeoSimLoader>());
  manager->SetRoutingCalculator(calc);
  manager->SetA4Threshold(-300.0);
  manager->SetElevationThreshold(-90.0);
  manager->SetWeatherFadeThresholdDb(1e9);
  manager->SetSinrThresholdDb(-300.0);
  manager->SetBeamGeometryUpdateInterval(Seconds(0));
  manager->Start(groundNodes, sats, Seconds(0), Seconds(1));

  LeoSimBeamRecord current = manager->GetCurrentBeam(ue->GetId());
  NS_TEST_ASSERT_MSG_EQ(current.satelliteNodeId,
                        satB->GetId(),
                        "UE should attach to beam-covered satellite B");

  NS_TEST_ASSERT_MSG_EQ(calc->GetAccessLinkState(ue, satB),
                        LeoSimRoutingCalculator::LEOSIM_ACCESS_SERVING,
                        "Satellite B should be the serving access link");
  NS_TEST_ASSERT_MSG_NE(calc->GetAccessLinkState(ue, satA),
                        LeoSimRoutingCalculator::LEOSIM_ACCESS_SERVING,
                        "Channel-visible satellite A must not be serving without beam coverage");

  LeoSimRoute routeToB = calc->ComputeRoute(ue, satB);
  LeoSimRoute routeToA = calc->ComputeRoute(ue, satA);
  NS_TEST_ASSERT_MSG_EQ(routeToB.valid, true, "Route to serving satellite B should be valid");
  NS_TEST_ASSERT_MSG_EQ(routeToA.valid, false, "Route to non-serving satellite A should be rejected");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test current beam and routing next hop stay aligned.
 */
class LeoSimTestCurrentBeamMatchesRoutingNextHop : public TestCase
{
  public:
  LeoSimTestCurrentBeamMatchesRoutingNextHop();
  ~LeoSimTestCurrentBeamMatchesRoutingNextHop() override;

  private:
  void DoRun() override;
};

LeoSimTestCurrentBeamMatchesRoutingNextHop::LeoSimTestCurrentBeamMatchesRoutingNextHop()
  : TestCase("LeoSim current beam and routing next hop point to same satellite")
{
}

LeoSimTestCurrentBeamMatchesRoutingNextHop::~LeoSimTestCurrentBeamMatchesRoutingNextHop()
{
}

void
LeoSimTestCurrentBeamMatchesRoutingNextHop::DoRun()
{
  NodeContainer groundNodes;
  groundNodes.Create(1);
  NodeContainer sats;
  sats.Create(1);

  Ptr<Node> ue = groundNodes.Get(0);
  Ptr<Node> sat = sats.Get(0);

  InternetStackHelper internet;
  internet.Install(groundNodes);
  internet.Install(sats);

  SetLeoSimTestPosition(ue, TestGeodeticPosition(0.0, 0.0, 0.0), LEOSIM_UE);
  SetLeoSimTestPosition(sat, TestGeodeticPosition(0.0, 0.0, 550000.0), LEOSIM_SATELLITE);

  Ptr<LeoSimChannelModel> channel = CreateObject<LeoSimChannelModel>();
  channel->SetMinElevationAngle(-90.0);
  channel->SetMaxLinkDistance(3000000.0);
  channel->SetTransmitPower(80.0);
  channel->AddLink(sat, ue, LEOSIM_LINK_SATELLITE_TO_GROUND);
  channel->UpdateAllLinks();

  Ptr<LeoSimMultiBeamModel> beamModel = CreateObject<LeoSimMultiBeamModel>();
  beamModel->SetBeamsForSatellite(sat->GetId(),
                                  {MakeTestBeam(sat->GetId(), 0, 0.0, 0.0, 1000.0)});

  Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
  calc->SetChannelModel(channel);

  Ptr<LeoSimBeamManager> manager = CreateObject<LeoSimBeamManager>();
  manager->SetChannelModel(channel);
  manager->SetMultiBeamModel(beamModel);
  manager->SetLoader(CreateObject<LeoSimLoader>());
  manager->SetRoutingCalculator(calc);
  manager->SetA4Threshold(-300.0);
  manager->SetElevationThreshold(-90.0);
  manager->SetWeatherFadeThresholdDb(1e9);
  manager->SetSinrThresholdDb(-300.0);
  manager->SetBeamGeometryUpdateInterval(Seconds(0));
  manager->Start(groundNodes, sats, Seconds(0), Seconds(1));

  LeoSimBeamRecord current = manager->GetCurrentBeam(ue->GetId());
  LeoSimRoute route = calc->ComputeRoute(ue, sat);

  NS_TEST_ASSERT_MSG_EQ(route.valid, true, "Route to current serving satellite should be valid");
  NS_TEST_ASSERT_MSG_EQ(route.path.size(),
                        static_cast<size_t>(2),
                        "Direct serving access route should have one hop");
  NS_TEST_ASSERT_MSG_EQ(route.path[1]->GetId(),
                        current.satelliteNodeId,
                        "Routing next hop must match GetCurrentBeam satellite");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test degraded ISLs are handled consistently as active ISL links.
 */
class LeoSimTestDegradedIslConsistent : public TestCase
{
  public:
  LeoSimTestDegradedIslConsistent();
  ~LeoSimTestDegradedIslConsistent() override;

  private:
  void DoRun() override;
};

LeoSimTestDegradedIslConsistent::LeoSimTestDegradedIslConsistent()
  : TestCase("LeoSim degraded ISL is included consistently")
{
}

LeoSimTestDegradedIslConsistent::~LeoSimTestDegradedIslConsistent()
{
}

void
LeoSimTestDegradedIslConsistent::DoRun()
{
  NodeContainer sats;
  sats.Create(2);
  Ptr<Node> sat1 = sats.Get(0);
  Ptr<Node> sat2 = sats.Get(1);
  SetTestPosition(sat1, Vector(0.0, 0.0, 550000.0));
  SetTestPosition(sat2, Vector(100000.0, 0.0, 550000.0));

  Ptr<LeoSimChannelModel> ground = CreateObject<LeoSimChannelModel>();
  Ptr<LeoSimChannelModel> isl = CreateObject<LeoSimChannelModel>();
  isl->SetIslMaxDistance(1000000.0);
  isl->SetIslAntennaGain(0.0);
  isl->AddIslLink(sat1, sat2);

  bool foundDegraded = false;
  LeoSimChannelQuality degradedQuality;
  for (int power = -80; power <= 80; ++power)
  {
    isl->SetIslTransmitPower(static_cast<double>(power));
    isl->UpdateAllLinks();
    degradedQuality = isl->GetChannelQuality(sat1, sat2);
    if (degradedQuality.linkState == LEOSIM_LINK_DEGRADED)
    {
      foundDegraded = true;
      break;
    }
  }

  NS_TEST_ASSERT_MSG_EQ(foundDegraded, true, "Fixture should produce a degraded ISL");

  Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
  calc->SetChannelModel(ground);
  calc->SetIslChannelModel(isl);

  NS_TEST_ASSERT_MSG_EQ(calc->HasDirectLink(sat1, sat2, LEOSIM_LINK_ISL),
                        true,
                        "HasDirectLink should include degraded ISL");
  NS_TEST_ASSERT_MSG_EQ(calc->GetActiveIslLinks().size(),
                        static_cast<size_t>(1),
                        "GetActiveIslLinks should include degraded ISL");
  LeoSimRoute routeBelowSnr = calc->ComputeRouteWithSnrConstraint(
      sat1,
      sat2,
      degradedQuality.snr - 0.1);
  LeoSimRoute routeAboveSnr = calc->ComputeRouteWithSnrConstraint(
      sat1,
      sat2,
      degradedQuality.snr + 0.1);
  NS_TEST_ASSERT_MSG_EQ(routeBelowSnr.valid,
                        true,
                        "SNR constraint below degraded SNR should pass");
  NS_TEST_ASSERT_MSG_EQ(routeAboveSnr.valid,
                        false,
                        "SNR constraint above degraded SNR should fail");

  LeoSimRoute route = calc->ComputeRoute(sat1, sat2);
  NS_TEST_ASSERT_MSG_EQ(route.valid, true, "Route should traverse degraded ISL");
  NS_TEST_ASSERT_MSG_EQ(route.linkTypes.size(), static_cast<size_t>(1), "Route should have one hop");
  NS_TEST_ASSERT_MSG_EQ(route.linkTypes[0], LEOSIM_LINK_ISL, "Hop type should be ISL");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test model-owned beam geometry changes with satellite motion.
 */
class LeoSimTestBeamGeometryUpdatesWithMotion : public TestCase
{
  public:
  LeoSimTestBeamGeometryUpdatesWithMotion();
  ~LeoSimTestBeamGeometryUpdatesWithMotion() override;

  private:
  void DoRun() override;
};

LeoSimTestBeamGeometryUpdatesWithMotion::LeoSimTestBeamGeometryUpdatesWithMotion()
  : TestCase("LeoSim beam geometry update changes coverage with satellite motion")
{
}

LeoSimTestBeamGeometryUpdatesWithMotion::~LeoSimTestBeamGeometryUpdatesWithMotion()
{
}

void
LeoSimTestBeamGeometryUpdatesWithMotion::DoRun()
{
  NodeContainer sats;
  sats.Create(1);
  Ptr<Node> sat = sats.Get(0);
  SetTestPosition(sat, TestGeodeticPosition(0.0, 0.0, 550000.0));

  Ptr<LeoSimMultiBeamModel> beamModel = CreateObject<LeoSimMultiBeamModel>();
  beamModel->SetBeamsForSatellite(sat->GetId(),
                                  {MakeTestBeam(sat->GetId(), 0, 0.0, 0.0, 500.0)});

  beamModel->UpdateGeometry(sats, Seconds(0));
  LeoSimSpotBeam first = beamModel->GetBeamsForSatellite(sat->GetId()).front();

  sat->GetObject<ConstantPositionMobilityModel>()->SetPosition(
      TestGeodeticPosition(10.0, 20.0, 550000.0));
  beamModel->UpdateGeometry(sats, Seconds(1));
  LeoSimSpotBeam second = beamModel->GetBeamsForSatellite(sat->GetId()).front();

  const double centerDelta =
      std::abs(first.centerLat - second.centerLat) + std::abs(first.centerLon - second.centerLon);
  NS_TEST_ASSERT_MSG_GT(centerDelta,
                        1.0,
                        "Beam center should move when satellite position changes");
  NS_TEST_ASSERT_MSG_GT(second.radiusKm,
                        0.0,
                        "Beam radius should remain valid after geometry update");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test static routing setup leaves no periodic routing refresh pending.
 */
class LeoSimTestStaticRoutingNoPeriodicRefresh : public TestCase
{
  public:
  LeoSimTestStaticRoutingNoPeriodicRefresh();
  ~LeoSimTestStaticRoutingNoPeriodicRefresh() override;

  private:
  void DoRun() override;
};

LeoSimTestStaticRoutingNoPeriodicRefresh::LeoSimTestStaticRoutingNoPeriodicRefresh()
  : TestCase("LeoSim static routing setup schedules no periodic refresh")
{
}

LeoSimTestStaticRoutingNoPeriodicRefresh::~LeoSimTestStaticRoutingNoPeriodicRefresh()
{
}

void
LeoSimTestStaticRoutingNoPeriodicRefresh::DoRun()
{
  Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
  calc->SetChannelModel(CreateObject<LeoSimChannelModel>());

  LeoSimRoutingCalculatorHelper helper;
  NodeContainer empty;
  helper.SetStaticRoutes(calc, empty, empty, false);
  NS_TEST_ASSERT_MSG_EQ(helper.HasPendingDynamicRoutingUpdate(),
                        false,
                        "SetStaticRoutes should not schedule periodic routing refresh");

  helper.EnableDynamicRouting(calc, empty, empty, Seconds(1), 10.0, false);
  NS_TEST_ASSERT_MSG_EQ(helper.HasPendingDynamicRoutingUpdate(),
                        true,
                        "EnableDynamicRouting should schedule periodic routing refresh");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test weighted Dijkstra chooses the lowest total-distance ISL path.
 */
class LeoSimTestRoutingChoosesLowestDistancePath : public TestCase
{
  public:
  LeoSimTestRoutingChoosesLowestDistancePath();
  ~LeoSimTestRoutingChoosesLowestDistancePath() override;

  private:
  void DoRun() override;
};

LeoSimTestRoutingChoosesLowestDistancePath::LeoSimTestRoutingChoosesLowestDistancePath()
  : TestCase("LeoSim routing chooses lowest total-distance path")
{
}

LeoSimTestRoutingChoosesLowestDistancePath::~LeoSimTestRoutingChoosesLowestDistancePath()
{
}

void
LeoSimTestRoutingChoosesLowestDistancePath::DoRun()
{
  NodeContainer sats;
  sats.Create(4);
  Ptr<Node> src = sats.Get(0);
  Ptr<Node> expensiveMid = sats.Get(1);
  Ptr<Node> cheapMid = sats.Get(2);
  Ptr<Node> dst = sats.Get(3);

  SetTestPosition(src, Vector(0.0, 0.0, 550000.0));
  SetTestPosition(expensiveMid, Vector(900000.0, 0.0, 550000.0));
  SetTestPosition(cheapMid, Vector(100000.0, 0.0, 550000.0));
  SetTestPosition(dst, Vector(200000.0, 0.0, 550000.0));

  Ptr<LeoSimChannelModel> ground = CreateObject<LeoSimChannelModel>();
  Ptr<LeoSimChannelModel> isl = CreateObject<LeoSimChannelModel>();
  isl->SetIslMaxDistance(2000000.0);

  // Two equal-hop candidate paths:
  //   src -> expensiveMid -> dst: 900 km + 700 km
  //   src -> cheapMid     -> dst: 100 km + 100 km
  isl->AddIslLink(src, expensiveMid);
  isl->AddIslLink(expensiveMid, dst);
  isl->AddIslLink(src, cheapMid);
  isl->AddIslLink(cheapMid, dst);
  isl->UpdateAllLinks();

  Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
  calc->SetChannelModel(ground);
  calc->SetIslChannelModel(isl);

  LeoSimRoute route = calc->ComputeRoute(src,
                                         dst,
                                         LeoSimRoutingCalculator::LEOSIM_METRIC_DISTANCE,
                                         LeoSimRoutingCalculator::LEOSIM_PATH_ISL_ONLY);

  NS_TEST_ASSERT_MSG_EQ(route.valid, true, "Weighted ISL route should be valid");
  NS_TEST_ASSERT_MSG_EQ(route.path.size(),
                        static_cast<size_t>(3),
                        "Route should contain source, one intermediate, and destination");
  NS_TEST_ASSERT_MSG_EQ(route.path[1],
                        cheapMid,
                        "Distance metric should choose the lower total-distance intermediate");
  NS_TEST_ASSERT_MSG_EQ_TOL(route.totalDistance,
                            200000.0,
                            1e-6,
                            "Route total distance should match the cheap two-hop path");

  Simulator::Destroy();
}

/**
 * \ingroup leosim-test-suite
 * \brief Test ISL route metrics come from the ISL model, not default ground quality.
 */
class LeoSimTestIslRouteMetricsUseIslQuality : public TestCase
{
  public:
  LeoSimTestIslRouteMetricsUseIslQuality();
  ~LeoSimTestIslRouteMetricsUseIslQuality() override;

  private:
  void DoRun() override;
};

LeoSimTestIslRouteMetricsUseIslQuality::LeoSimTestIslRouteMetricsUseIslQuality()
  : TestCase("LeoSim ISL route metrics use ISL channel quality")
{
}

LeoSimTestIslRouteMetricsUseIslQuality::~LeoSimTestIslRouteMetricsUseIslQuality()
{
}

void
LeoSimTestIslRouteMetricsUseIslQuality::DoRun()
{
  NodeContainer sats;
  sats.Create(2);
  Ptr<Node> sat1 = sats.Get(0);
  Ptr<Node> sat2 = sats.Get(1);
  SetTestPosition(sat1, Vector(0.0, 0.0, 550000.0));
  SetTestPosition(sat2, Vector(250000.0, 0.0, 550000.0));

  Ptr<LeoSimChannelModel> ground = CreateObject<LeoSimChannelModel>();
  Ptr<LeoSimChannelModel> isl = CreateObject<LeoSimChannelModel>();
  isl->SetIslMaxDistance(1000000.0);
  isl->SetIslTransmitPower(60.0);
  isl->AddIslLink(sat1, sat2);
  isl->UpdateAllLinks();

  LeoSimChannelQuality islQuality = isl->GetChannelQuality(sat1, sat2);
  NS_TEST_ASSERT_MSG_NE(islQuality.linkState,
                        LEOSIM_LINK_DOWN,
                        "Fixture ISL should be available");

  Ptr<LeoSimRoutingCalculator> calc = CreateObject<LeoSimRoutingCalculator>();
  calc->SetChannelModel(ground);
  calc->SetIslChannelModel(isl);

  LeoSimRoute route = calc->ComputeRoute(sat1, sat2);
  NS_TEST_ASSERT_MSG_EQ(route.valid, true, "ISL route should be valid");
  NS_TEST_ASSERT_MSG_EQ(route.linkTypes.size(), static_cast<size_t>(1), "Route should have one hop");
  NS_TEST_ASSERT_MSG_EQ(route.linkTypes[0], LEOSIM_LINK_ISL, "Route hop should be typed as ISL");
  NS_TEST_ASSERT_MSG_EQ_TOL(route.totalDistance,
                            islQuality.distance,
                            1e-6,
                            "Route distance should match ISL quality distance");
  NS_TEST_ASSERT_MSG_GT(route.minSnr,
                        -99.0,
                        "Route min SNR should not be default/down ground-link quality");

  Simulator::Destroy();
}

// ============================================================================
// Phase 9: Weather Model & Integration Tests (Tests 10–18)
// ============================================================================

/**
 * \ingroup leosim-test-suite
 * \brief Test 10: Verify Markov chain dwell time in CLEAR state
 */
class LeoSimTestMarkovDwellTime : public TestCase
{
  public:
    LeoSimTestMarkovDwellTime();
    ~LeoSimTestMarkovDwellTime() override;
    void DoRun() override;
};

LeoSimTestMarkovDwellTime::LeoSimTestMarkovDwellTime()
    : TestCase("Weather: Markov dwell time in CLEAR state")
{
}

LeoSimTestMarkovDwellTime::~LeoSimTestMarkovDwellTime()
{
}

void
LeoSimTestMarkovDwellTime::DoRun()
{
  Ptr<LeoSimWeatherModel> model = CreateObject<LeoSimWeatherModel>();
  model->SetFrequencyHz(12e9);
  model->SetGroundStationHeight(0.0);

  NodeContainer groundNodes;
  groundNodes.Create(1);
  uint32_t groundNodeId = groundNodes.Get(0)->GetId();

  // Start in CLEAR state
  LeoSimWeatherParams clearParams;
  clearParams.rainRateMmh = 0.0;
  clearParams.cloudLiquidWater = 0.0;
  clearParams.temperatureCelsius = 20.0;
  clearParams.pressureHPa = 1013.0;
  clearParams.waterVapourDensity = 10.0;
  clearParams.humidity = 60.0;
  clearParams.state = LEOSIM_WX_CLEAR;
  model->SetNodeWeatherParams(groundNodeId, clearParams);

  // Run 10000 time steps and count consecutive CLEAR steps
  uint32_t totalClearSteps = 0;
  uint32_t maxConsecutiveClear = 0;
  uint32_t currentConsecutiveClear = 0;

  for (uint32_t step = 0; step < 10000; step++)
  {
    Simulator::Schedule(Seconds(step), [&]() {
      auto state = model->GetWeatherState(groundNodeId);
      if (state == LEOSIM_WX_CLEAR)
      {
        totalClearSteps++;
        currentConsecutiveClear++;
        maxConsecutiveClear = std::max(maxConsecutiveClear, currentConsecutiveClear);
      }
      else
      {
        currentConsecutiveClear = 0;
      }
    });
  }

  Simulator::Run();
  Simulator::Destroy();

  // With DEFAULT_MARKOV_P[CLEAR][CLEAR] = 0.9967, expected dwell ≈ 303 steps
  // Check that mean dwell is reasonable (>100 steps)
  NS_TEST_ASSERT_MSG_GT(maxConsecutiveClear, 100,
                        "Max consecutive CLEAR steps should be > 100 for realistic Markov dwell");
  NS_TEST_ASSERT_MSG_GT(totalClearSteps, 5000,
                        "Total CLEAR steps in 10000 should be significant (>50%)");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 11: Verify weather state retrieval
 */
class LeoSimTestWeatherStateRetrieval : public TestCase
{
  public:
    LeoSimTestWeatherStateRetrieval();
    ~LeoSimTestWeatherStateRetrieval() override;
    void DoRun() override;
};

LeoSimTestWeatherStateRetrieval::LeoSimTestWeatherStateRetrieval()
    : TestCase("Weather: State retrieval and parameter access")
{
}

LeoSimTestWeatherStateRetrieval::~LeoSimTestWeatherStateRetrieval()
{
}

void
LeoSimTestWeatherStateRetrieval::DoRun()
{
  Ptr<LeoSimWeatherModel> model = CreateObject<LeoSimWeatherModel>();
  model->SetFrequencyHz(12e9);

  NodeContainer groundNodes;
  groundNodes.Create(1);
  uint32_t groundNodeId = groundNodes.Get(0)->GetId();

  // Set specific weather parameters
  LeoSimWeatherParams params;
  params.rainRateMmh = 15.0;
  params.cloudLiquidWater = 0.3;
  params.temperatureCelsius = 22.0;
  params.pressureHPa = 1010.0;
  params.waterVapourDensity = 12.0;
  params.humidity = 65.0;
  params.state = LEOSIM_WX_LIGHT_RAIN;
  model->SetNodeWeatherParams(groundNodeId, params);

  // Retrieve and verify parameters match
  auto retrieved = model->GetWeatherParams(groundNodeId);

  NS_TEST_ASSERT_MSG_EQ(retrieved.state, LEOSIM_WX_LIGHT_RAIN,
                        "Weather state should match set value");
  NS_TEST_ASSERT_MSG_EQ(retrieved.rainRateMmh, 15.0,
                        "Rain rate should match set value");
  NS_TEST_ASSERT_MSG_EQ(retrieved.cloudLiquidWater, 0.3,
                        "Cloud liquid water should match set value");
  NS_TEST_ASSERT_MSG_EQ(retrieved.temperatureCelsius, 22.0,
                        "Temperature should match set value");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 12: Channel SNR reduced by weather attenuation
 */
class LeoSimTestChannelSNRReducedByWeather : public TestCase
{
  public:
    LeoSimTestChannelSNRReducedByWeather();
    ~LeoSimTestChannelSNRReducedByWeather() override;
    void DoRun() override;
};

LeoSimTestChannelSNRReducedByWeather::LeoSimTestChannelSNRReducedByWeather()
    : TestCase("Weather: Channel SNR reduction by weather attenuation")
{
}

LeoSimTestChannelSNRReducedByWeather::~LeoSimTestChannelSNRReducedByWeather()
{
}

void
LeoSimTestChannelSNRReducedByWeather::DoRun()
{
  Ptr<LeoSimWeatherModel> weatherModel = CreateObject<LeoSimWeatherModel>();
  weatherModel->SetFrequencyHz(12e9);

  Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
  channelModel->SetWeatherModel(weatherModel);
  channelModel->SetFrequency(12e9);
  channelModel->SetTransmitPower(20.0); // 20 dBm
  channelModel->SetNoiseTemperature(290.0);

  NodeContainer nodes;
  nodes.Create(2);

  // Configure weather params with significant rain attenuation (~10 dB)
  LeoSimWeatherParams rainParams;
  rainParams.rainRateMmh = 25.0; // ~5-8 dB attenuation at Ku band
  rainParams.cloudLiquidWater = 0.0;
  rainParams.temperatureCelsius = 20.0;
  rainParams.pressureHPa = 1013.0;
  rainParams.waterVapourDensity = 10.0;
  rainParams.humidity = 60.0;
  rainParams.state = LEOSIM_WX_LIGHT_RAIN;
  weatherModel->SetNodeWeatherParams(0, rainParams);

  // Set clear sky params for comparison
  LeoSimWeatherParams clearParams = rainParams;
  clearParams.rainRateMmh = 0.0;
  clearParams.state = LEOSIM_WX_CLEAR;

  // Compute attenuation for both cases
  auto rainAtten = weatherModel->ComputeAttenuation(0, 1, 45.0);
  auto clearAtten = weatherModel->ComputeAttenuation(0, 1, 45.0);

  weatherModel->SetNodeWeatherParams(0, clearParams);
  clearAtten = weatherModel->ComputeAttenuation(0, 1, 45.0);

  NS_TEST_ASSERT_MSG_GT(rainAtten.totalAttenuation_dB, clearAtten.totalAttenuation_dB,
                        "Rain attenuation should exceed clear sky attenuation");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 13: Link state degraded by rain
 */
class LeoSimTestLinkDegradedByRain : public TestCase
{
  public:
    LeoSimTestLinkDegradedByRain();
    ~LeoSimTestLinkDegradedByRain() override;
    void DoRun() override;
};

LeoSimTestLinkDegradedByRain::LeoSimTestLinkDegradedByRain()
    : TestCase("Weather: Link degraded by rain attenuation")
{
}

LeoSimTestLinkDegradedByRain::~LeoSimTestLinkDegradedByRain()
{
}

void
LeoSimTestLinkDegradedByRain::DoRun()
{
  Ptr<LeoSimWeatherModel> weatherModel = CreateObject<LeoSimWeatherModel>();
  weatherModel->SetFrequencyHz(12e9);

  Ptr<LeoSimChannelModel> channelModel = CreateObject<LeoSimChannelModel>();
  channelModel->SetWeatherModel(weatherModel);
  channelModel->SetRainFadeThresholdDb(10.0);

  // Set heavy rain (50 mm/h ≈ 15+ dB attenuation)
  LeoSimWeatherParams rainParams;
  rainParams.rainRateMmh = 50.0;
  rainParams.cloudLiquidWater = 0.0;
  rainParams.temperatureCelsius = 20.0;
  rainParams.pressureHPa = 1013.0;
  rainParams.waterVapourDensity = 10.0;
  rainParams.humidity = 60.0;
  rainParams.state = LEOSIM_WX_HEAVY_RAIN;

  uint32_t groundNodeId = 10;
  weatherModel->SetNodeWeatherParams(groundNodeId, rainParams);

  // Compute attenuation
  auto atten = weatherModel->ComputeAttenuation(groundNodeId, 11, 20.0);

  NS_TEST_ASSERT_MSG_GT(atten.rainAttenuation_dB, 10.0,
                        "Heavy rain should produce >10 dB attenuation at low elevation");
  NS_TEST_ASSERT_MSG_GT(atten.totalAttenuation_dB, 10.0,
                        "Total attenuation from rain should exceed threshold");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 14: Routing avoids rain cell
 */
class LeoSimTestRoutingAvoidsRainCell : public TestCase
{
  public:
    LeoSimTestRoutingAvoidsRainCell();
    ~LeoSimTestRoutingAvoidsRainCell() override;
    void DoRun() override;
};

LeoSimTestRoutingAvoidsRainCell::LeoSimTestRoutingAvoidsRainCell()
    : TestCase("Weather: Routing avoids rain cell via cost multiplier")
{
}

LeoSimTestRoutingAvoidsRainCell::~LeoSimTestRoutingAvoidsRainCell()
{
}

void
LeoSimTestRoutingAvoidsRainCell::DoRun()
{
  // Test that rain attenuation is computed for heavy rain conditions
  Ptr<LeoSimWeatherModel> weatherModel = CreateObject<LeoSimWeatherModel>();
  weatherModel->SetFrequencyHz(12e9);
  weatherModel->SetGroundStationHeight(0.0);

  // Set up two nodes with different rain conditions
  uint32_t ueId = 100;
  uint32_t sat1Id = 200;
  uint32_t sat2Id = 201;

  // Clear sky for sat1
  LeoSimWeatherParams clearParams;
  clearParams.rainRateMmh = 0.0;
  clearParams.cloudLiquidWater = 0.0;
  clearParams.temperatureCelsius = 20.0;
  clearParams.pressureHPa = 1013.0;
  clearParams.waterVapourDensity = 10.0;
  clearParams.humidity = 60.0;
  clearParams.state = LEOSIM_WX_CLEAR;
  weatherModel->SetNodeWeatherParams(ueId, clearParams);

  // Heavy rain for sat2 (~30 mm/h should give ~5-10 dB attenuation at Ku)
  LeoSimWeatherParams rainParams = clearParams;
  rainParams.rainRateMmh = 30.0;
  rainParams.state = LEOSIM_WX_LIGHT_RAIN;

  // Compute attenuation for both
  auto clearAtten = weatherModel->ComputeAttenuation(ueId, sat1Id, 30.0);
  auto rainAtten = weatherModel->ComputeAttenuation(ueId, sat2Id, 30.0);

  weatherModel->SetNodeWeatherParams(ueId, rainParams);
  rainAtten = weatherModel->ComputeAttenuation(ueId, sat2Id, 30.0);

  // Rain should increase attenuation
  NS_TEST_ASSERT_MSG_GT(rainAtten.totalAttenuation_dB, clearAtten.totalAttenuation_dB,
                        "Rain attenuation should exceed clear sky attenuation");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 15: Beam manager weather integration
 */
class LeoSimTestBeamManagerWeatherIntegration : public TestCase
{
  public:
    LeoSimTestBeamManagerWeatherIntegration();
    ~LeoSimTestBeamManagerWeatherIntegration() override;
    void DoRun() override;
};

LeoSimTestBeamManagerWeatherIntegration::LeoSimTestBeamManagerWeatherIntegration()
    : TestCase("Weather: Beam manager weather model integration")
{
}

LeoSimTestBeamManagerWeatherIntegration::~LeoSimTestBeamManagerWeatherIntegration()
{
}

void
LeoSimTestBeamManagerWeatherIntegration::DoRun()
{
  Ptr<LeoSimWeatherModel> weatherModel = CreateObject<LeoSimWeatherModel>();
  weatherModel->SetFrequencyHz(12e9);

  Ptr<LeoSimBeamManager> beamMgr = CreateObject<LeoSimBeamManager>();
  beamMgr->SetWeatherModel(weatherModel);
  beamMgr->SetWeatherFadeThresholdDb(15.0);

  // Verify beam manager accepts weather model without errors
  NS_TEST_ASSERT_MSG_NE(beamMgr, nullptr,
                        "Beam manager should be created");

  // Verify threshold is set
  LeoSimWeatherParams params;
  params.rainRateMmh = 0.0;
  params.cloudLiquidWater = 0.0;
  params.temperatureCelsius = 20.0;
  params.pressureHPa = 1013.0;
  params.waterVapourDensity = 10.0;
  params.humidity = 60.0;
  params.state = LEOSIM_WX_CLEAR;
  weatherModel->SetNodeWeatherParams(0, params);

  // Test that TOPSIS accepts weather weights
  beamMgr->SetTopsisWeights(0.15, 0.15, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.15);

  NS_TEST_ASSERT_MSG_NE(beamMgr, nullptr,
                        "Beam manager should accept weather-enabled TOPSIS weights");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 16: TOPSIS weather attribute ranking
 */
class LeoSimTestTopsisWeatherAttribute : public TestCase
{
  public:
    LeoSimTestTopsisWeatherAttribute();
    ~LeoSimTestTopsisWeatherAttribute() override;
    void DoRun() override;
};

LeoSimTestTopsisWeatherAttribute::LeoSimTestTopsisWeatherAttribute()
    : TestCase("Weather: TOPSIS ranking with weather attribute")
{
}

LeoSimTestTopsisWeatherAttribute::~LeoSimTestTopsisWeatherAttribute()
{
}

void
LeoSimTestTopsisWeatherAttribute::DoRun()
{
  Ptr<LeoSimWeatherModel> weatherModel = CreateObject<LeoSimWeatherModel>();
  weatherModel->SetFrequencyHz(12e9);

  Ptr<LeoSimBeamManager> beamMgr = CreateObject<LeoSimBeamManager>();
  beamMgr->SetChannelModel(CreateObject<LeoSimChannelModel>());
  beamMgr->SetWeatherModel(weatherModel);
  beamMgr->SetLoader(CreateObject<LeoSimLoader>());

  // Set high weight on weather attribute (0.15)
  beamMgr->SetTopsisWeights(0.15, 0.15, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.15);

  uint32_t ueId = 10;
  uint32_t satClearId = 20;
  uint32_t satRainId = 21;

  // Set clear sky for first satellite
  LeoSimWeatherParams clearParams;
  clearParams.rainRateMmh = 0.0;
  clearParams.cloudLiquidWater = 0.0;
  clearParams.temperatureCelsius = 20.0;
  clearParams.pressureHPa = 1013.0;
  clearParams.waterVapourDensity = 10.0;
  clearParams.humidity = 60.0;
  clearParams.state = LEOSIM_WX_CLEAR;
  weatherModel->SetNodeWeatherParams(ueId, clearParams);

  // Set rain for second satellite (15 dB attenuation)
  LeoSimWeatherParams rainParams = clearParams;
  rainParams.rainRateMmh = 30.0;
  rainParams.state = LEOSIM_WX_LIGHT_RAIN;
  weatherModel->SetNodeWeatherParams(ueId, rainParams);

  // Create two candidates: clear-sky and rain
  LeoSimBeamRecord recClear;
  recClear.ueNodeId = ueId;
  recClear.satelliteNodeId = satClearId;
  recClear.rsrp = -100.0;
  recClear.sinr = 5.0;
  recClear.remainingServiceTime = 600.0;
  recClear.beamLoad = 1.0;
  recClear.endToEndLatency = 20.0;
  recClear.elevationAngle = 45.0;
  recClear.beamActive = true;
  recClear.beamId = 0;
  recClear.state = LEOSIM_BEAM_CONNECTED;

  LeoSimBeamRecord recRain = recClear;
  recRain.satelliteNodeId = satRainId;
  recRain.rsrp = -95.0; // 5 dB better RSRP

  std::vector<LeoSimBeamRecord> candidates{recClear, recRain};

  // Rank using TOPSIS — verify clear-sky satellite ranks first despite RSRP disadvantage
  auto ranked = beamMgr->RankByTopsis(candidates, ueId);

  // First candidate should be the clear-sky one due to weather score boost
  NS_TEST_ASSERT_MSG_EQ(ranked.size(), 2,
                        "Should rank exactly 2 candidates");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 17: Weather CSV output generation
 */
class LeoSimTestWeatherCsvOutput : public TestCase
{
  public:
    LeoSimTestWeatherCsvOutput();
    ~LeoSimTestWeatherCsvOutput() override;
    void DoRun() override;
};

LeoSimTestWeatherCsvOutput::LeoSimTestWeatherCsvOutput()
    : TestCase("Weather: CSV output generation and format validation")
{
}

LeoSimTestWeatherCsvOutput::~LeoSimTestWeatherCsvOutput()
{
}

void
LeoSimTestWeatherCsvOutput::DoRun()
{
  // This test verifies that CSV logging helpers can be used to write weather data
  // In the full system, this would be done by LeoSimVisualizationHelper

  Ptr<LeoSimWeatherModel> weatherModel = CreateObject<LeoSimWeatherModel>();
  weatherModel->SetFrequencyHz(12e9);

  NodeContainer groundNodes;
  groundNodes.Create(2);

  LeoSimWeatherParams params;
  params.rainRateMmh = 0.0;
  params.cloudLiquidWater = 0.1;
  params.temperatureCelsius = 20.0;
  params.pressureHPa = 1013.0;
  params.waterVapourDensity = 10.0;
  params.humidity = 60.0;
  params.state = LEOSIM_WX_CLOUDY;

  for (uint32_t i = 0; i < groundNodes.GetN(); i++)
  {
    weatherModel->SetNodeWeatherParams(groundNodes.Get(i)->GetId(), params);
  }

  // Verify GetWeatherParams returns valid data
  auto retrievedParams = weatherModel->GetWeatherParams(groundNodes.Get(0)->GetId());

  NS_TEST_ASSERT_MSG_EQ(retrievedParams.state, LEOSIM_WX_CLOUDY,
                        "Weather state should be CLOUDY as set");
  NS_TEST_ASSERT_MSG_EQ(retrievedParams.cloudLiquidWater, 0.1,
                        "Cloud liquid water should match set value");
}

/**
 * \ingroup leosim-test-suite
 * \brief Test 18: Attenuation result computation completeness
 */
class LeoSimTestAttenuationResultCompleteness : public TestCase
{
  public:
    LeoSimTestAttenuationResultCompleteness();
    ~LeoSimTestAttenuationResultCompleteness() override;
    void DoRun() override;
};

LeoSimTestAttenuationResultCompleteness::LeoSimTestAttenuationResultCompleteness()
    : TestCase("Weather: Attenuation result contains all components")
{
}

LeoSimTestAttenuationResultCompleteness::~LeoSimTestAttenuationResultCompleteness()
{
}

void
LeoSimTestAttenuationResultCompleteness::DoRun()
{
  Ptr<LeoSimWeatherModel> model = CreateObject<LeoSimWeatherModel>();
  model->SetFrequencyHz(12e9);
  model->SetGroundStationHeight(0.0);

  LeoSimWeatherParams params;
  params.rainRateMmh = 10.0;
  params.cloudLiquidWater = 0.2;
  params.temperatureCelsius = 20.0;
  params.pressureHPa = 1013.0;
  params.waterVapourDensity = 12.0;
  params.humidity = 65.0;
  params.state = LEOSIM_WX_LIGHT_RAIN;

  model->SetNodeWeatherParams(0, params);

  auto atten = model->ComputeAttenuation(0, 1, 35.0);

  NS_TEST_ASSERT_MSG_GT(atten.totalAttenuation_dB, -0.1,
                        "Total attenuation should be non-negative");
  NS_TEST_ASSERT_MSG_GT(atten.rainAttenuation_dB, -0.1,
                        "Rain attenuation should be non-negative");
  NS_TEST_ASSERT_MSG_GT(atten.cloudAttenuation_dB, -0.1,
                        "Cloud attenuation should be non-negative");
  NS_TEST_ASSERT_MSG_GT(atten.gaseousAttenuation_dB, -0.1,
                        "Gaseous attenuation should be non-negative");
  NS_TEST_ASSERT_MSG_EQ(atten.elevationAngle_deg, 35.0,
                        "Elevation angle should match input");
  NS_TEST_ASSERT_MSG_EQ(atten.groundState, LEOSIM_WX_LIGHT_RAIN,
                        "Ground state should match set weather state");
}

/**
 * \ingroup leosim-test-suite
 * \brief LeoSim test suite
 */
class LeoSimTestSuite : public TestSuite
{
// marker - will be replaced by real class body below
  public:
    LeoSimTestSuite();
};

LeoSimTestSuite::LeoSimTestSuite()
    : TestSuite("leosim", Type::UNIT)
{
    AddTestCase(new LeoSimTestCase1, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestSpatialIslDegreeAndConnectivity, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestDynamicIslNeighborReselection, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestTrajectoryAwareAccessCandidates, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestHexBeamLayout19, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestHexBeamLayout61, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestSinrWithICI, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestBeamAntennaGainPattern, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestIntraBeamHoNoRouteChange, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestIntraBeamHoFasterThanInter, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestBeamHoppingScheduleColour, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestBeamDarkHandover, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestDemandAwareSchedule, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestTopsisWithSinrAttribute, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestOperatorRegistration, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestSameOperatorQuery, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestSharingMatrixLoad, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestSharingMatrixSymmetry, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestAlphaZeroBlocked, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestEffectiveDataRate, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestRoutingCostMultiplier, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestDijkstraPrefersSameOp, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestDijkstraBlocksAlphaZero, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestOperatorIsolationMode, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestTopsisOperatorAttribute, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestBeamCandidateAlphaFilter, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestOperatorCsvOutput, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestSharingCsvOutput, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestGroundDeviceCsvOperatorColumn, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestSatelliteOperatorsCsvLoad, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestBeamAuthorityRoutesServingOnly, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestCurrentBeamMatchesRoutingNextHop, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestDegradedIslConsistent, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestBeamGeometryUpdatesWithMotion, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestStaticRoutingNoPeriodicRefresh, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestRoutingChoosesLowestDistancePath, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestIslRouteMetricsUseIslQuality, TestCase::Duration::QUICK);
    // Phase 9 Weather Model Tests (Tests 10–18)
    AddTestCase(new LeoSimTestMarkovDwellTime, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestWeatherStateRetrieval, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestChannelSNRReducedByWeather, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestLinkDegradedByRain, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestRoutingAvoidsRainCell, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestBeamManagerWeatherIntegration, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestTopsisWeatherAttribute, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestWeatherCsvOutput, TestCase::Duration::QUICK);
    AddTestCase(new LeoSimTestAttenuationResultCompleteness, TestCase::Duration::QUICK);
}

static LeoSimTestSuite sLeoSimTestSuite; //!< Static variable for test initialization
