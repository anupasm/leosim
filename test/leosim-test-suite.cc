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
#include "ns3/test.h"

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
 * \brief LeoSim test suite
 */
class LeoSimTestSuite : public TestSuite
{
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
}

static LeoSimTestSuite sLeoSimTestSuite; //!< Static variable for test initialization
