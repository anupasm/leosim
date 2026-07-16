/* SPDX-License-Identifier: GPL-2.0-only */
#include "ns3/leosim-statistics-helper.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace ns3;

class LeoSimRunningStatisticsTest : public TestCase
{
  public:
    LeoSimRunningStatisticsTest() : TestCase("LeoSim numerically stable running statistics") {}
    void DoRun() override
    {
        LeoSimRunningStatistics stats;
        stats.Add(1.0); stats.Add(2.0); stats.Add(3.0); stats.Add(NAN);
        NS_TEST_ASSERT_MSG_EQ(stats.count, 3, "non-finite samples must be ignored");
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.mean, 2.0, 1e-12, "incorrect mean");
        NS_TEST_ASSERT_MSG_EQ_TOL(stats.GetVariance(), 1.0, 1e-12, "incorrect sample variance");
        NS_TEST_ASSERT_MSG_EQ(stats.min, 1.0, "incorrect minimum");
        NS_TEST_ASSERT_MSG_EQ(stats.max, 3.0, "incorrect maximum");
    }
};

class LeoSimStatisticsSnapshotTest : public TestCase
{
  public:
    LeoSimStatisticsSnapshotTest() : TestCase("LeoSim satellite KPI snapshot and JSON output") {}
    void DoRun() override
    {
        Ptr<LeoSimStatisticsHelper> helper = CreateObject<LeoSimStatisticsHelper>();
        helper->RecordSnr(8.0); helper->RecordSnr(12.0);
        helper->RecordDoppler(-2500.0); helper->RecordPathLoss(168.0);
        helper->RecordLinkState(LEOSIM_LINK_UP);
        helper->RecordLinkState(LEOSIM_LINK_DEGRADED);
        const auto snapshot = helper->GetSnapshot(false);
        NS_TEST_ASSERT_MSG_EQ(snapshot.snrDb.count, 2, "SNR samples not captured");
        NS_TEST_ASSERT_MSG_EQ_TOL(snapshot.snrDb.mean, 10.0, 1e-12, "incorrect SNR mean");
        NS_TEST_ASSERT_MSG_EQ(snapshot.linkUpEvents, 1, "link-up event not captured");
        NS_TEST_ASSERT_MSG_EQ(snapshot.linkDegradedEvents, 1, "degraded event not captured");

        const std::string path = "/tmp/leosim-statistics-test.json";
        helper->WriteSummary(path);
        std::ifstream input(path);
        std::stringstream contents; contents << input.rdbuf();
        NS_TEST_ASSERT_MSG_NE(contents.str().find("leosim.statistics.v1"), std::string::npos,
                              "summary schema marker missing");
        NS_TEST_ASSERT_MSG_NE(contents.str().find("\"snr_db\""), std::string::npos,
                              "satellite KPI section missing");
        std::remove(path.c_str());
        Simulator::Destroy();
    }
};

class LeoSimStatisticsTestSuite : public TestSuite
{
  public:
    LeoSimStatisticsTestSuite() : TestSuite("leosim-statistics", Type::UNIT)
    {
        AddTestCase(new LeoSimRunningStatisticsTest, TestCase::Duration::QUICK);
        AddTestCase(new LeoSimStatisticsSnapshotTest, TestCase::Duration::QUICK);
    }
};

static LeoSimStatisticsTestSuite g_leoSimStatisticsTestSuite;
