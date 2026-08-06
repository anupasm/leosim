/* SPDX-License-Identifier: GPL-2.0-only */
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/leosim-statistics-helper.h"
#include "ns3/point-to-point-module.h"
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

class LeoSimApplicationGoodputTest : public TestCase
{
  public:
    LeoSimApplicationGoodputTest()
        : TestCase("LeoSim records exact PacketSink payload goodput and fairness")
    {
    }

    void DoRun() override
    {
        NodeContainer nodes;
        nodes.Create(2);
        PointToPointHelper pointToPoint;
        pointToPoint.SetDeviceAttribute("DataRate", StringValue("10Mbps"));
        pointToPoint.SetChannelAttribute("Delay", StringValue("1ms"));
        NetDeviceContainer devices = pointToPoint.Install(nodes);

        InternetStackHelper internet;
        internet.Install(nodes);
        Ipv4AddressHelper addresses;
        addresses.SetBase("10.100.0.0", "255.255.255.0");
        Ipv4InterfaceContainer interfaces = addresses.Assign(devices);

        const uint16_t port = 9000;
        PacketSinkHelper sinkHelper("ns3::UdpSocketFactory",
                                    InetSocketAddress(Ipv4Address::GetAny(), port));
        ApplicationContainer sinkApps = sinkHelper.Install(nodes.Get(1));
        sinkApps.Start(Seconds(0.0));
        sinkApps.Stop(Seconds(1.1));
        Ptr<PacketSink> sink = DynamicCast<PacketSink>(sinkApps.Get(0));

        OnOffHelper source("ns3::UdpSocketFactory",
                           InetSocketAddress(interfaces.GetAddress(1), port));
        source.SetAttribute("DataRate", StringValue("1Mbps"));
        source.SetAttribute("PacketSize", UintegerValue(100));
        source.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        source.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer sourceApps = source.Install(nodes.Get(0));
        sourceApps.Start(Seconds(0.1));
        sourceApps.Stop(Seconds(1.1));

        Ptr<LeoSimStatisticsHelper> helper = CreateObject<LeoSimStatisticsHelper>();
        helper->AddApplicationSink(sink);
        helper->SetApplicationMeasurementWindow(Seconds(0.1), Seconds(1.1));
        const std::string csv = "/tmp/leosim-goodput-test.csv";
        const std::string json = "/tmp/leosim-goodput-test.json";
        helper->StartPeriodicSampling(Seconds(0.25), csv);

        Simulator::Stop(Seconds(1.2));
        Simulator::Run();
        helper->WriteFinalSample();
        helper->StopPeriodicSampling();

        const auto snapshot = helper->GetSnapshot(false);
        NS_TEST_ASSERT_MSG_GT(sink->GetTotalRx(), 0, "test sink received no payload");
        NS_TEST_ASSERT_MSG_EQ(snapshot.applicationRxBytes,
                              sink->GetTotalRx(),
                              "snapshot did not use exact PacketSink payload bytes");
        NS_TEST_ASSERT_MSG_EQ_TOL(snapshot.applicationMeasurementSeconds,
                                  1.0,
                                  1e-12,
                                  "application measurement window is incorrect");
        NS_TEST_ASSERT_MSG_EQ_TOL(snapshot.applicationGoodputMbps,
                                  sink->GetTotalRx() * 8.0 / 1e6,
                                  1e-12,
                                  "application goodput is incorrect");
        NS_TEST_ASSERT_MSG_EQ(snapshot.applicationSinkCount, 1, "sink count is incorrect");
        NS_TEST_ASSERT_MSG_EQ(snapshot.applicationSinksWithRx,
                              1,
                              "count of sinks with received payload is incorrect");
        NS_TEST_ASSERT_MSG_EQ_TOL(snapshot.sinkGoodputJainFairness,
                                  1.0,
                                  1e-12,
                                  "single-sink Jain fairness must be one");

        helper->WriteSummary(json, false);
        std::ifstream jsonInput(json);
        std::stringstream jsonContents;
        jsonContents << jsonInput.rdbuf();
        NS_TEST_ASSERT_MSG_NE(jsonContents.str().find("\"application\""),
                              std::string::npos,
                              "summary omitted application statistics");
        NS_TEST_ASSERT_MSG_NE(jsonContents.str().find("\"goodput_mbps\""),
                              std::string::npos,
                              "summary omitted exact goodput");

        std::ifstream csvInput(csv);
        std::stringstream csvContents;
        csvContents << csvInput.rdbuf();
        NS_TEST_ASSERT_MSG_NE(csvContents.str().find("app_rx_bytes"),
                              std::string::npos,
                              "periodic CSV omitted application payload bytes");
        NS_TEST_ASSERT_MSG_NE(csvContents.str().find("goodput_mbps"),
                              std::string::npos,
                              "periodic CSV omitted application goodput");
        NS_TEST_ASSERT_MSG_NE(csvContents.str().find("interval_goodput_mbps"),
                              std::string::npos,
                              "periodic CSV omitted interval application goodput");

        std::remove(csv.c_str());
        std::remove(json.c_str());
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
        AddTestCase(new LeoSimApplicationGoodputTest, TestCase::Duration::QUICK);
    }
};

static LeoSimStatisticsTestSuite g_leoSimStatisticsTestSuite;
