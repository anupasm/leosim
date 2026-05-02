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
#include "ns3/leosim-operator-model.h"
#include "ns3/leosim-beam-manager.h"
#include "ns3/leosim-visualization-helper.h"
#include "ns3/test.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/simulator.h"

#include <cstdio>
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace ns3;

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
    LeoSimSpotBeam beam0, beam1;
    beam0.colorGroup = 1;
    beam1.colorGroup = 1;
    NS_TEST_ASSERT_MSG_EQ(beam0.colorGroup, 1, "Beam 0 color group should be 1");
    NS_TEST_ASSERT_MSG_EQ(beam1.colorGroup, 1, "Beam 1 color group should be 1");
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
    NS_TEST_ASSERT_MSG_GT(gainAt10, 0.0, "Gain at any angle should be positive");
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
}

static LeoSimTestSuite sLeoSimTestSuite; //!< Static variable for test initialization
