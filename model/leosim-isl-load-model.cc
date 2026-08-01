/*
 * Copyright (c) 2026 LeoSim contributors
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "leosim-isl-load-model.h"

#include "ns3/assert.h"
#include "ns3/log.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/node.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <fstream>
#include <iomanip>
#include <stdexcept>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimIslLoadModel");
NS_OBJECT_ENSURE_REGISTERED(LeoSimIslLoadModel);

namespace
{

uint64_t
Mix64(uint64_t value)
{
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31);
}

uint64_t
SaturatingAdd(uint64_t left, uint64_t right)
{
    return right > std::numeric_limits<uint64_t>::max() - left
               ? std::numeric_limits<uint64_t>::max()
               : left + right;
}

} // namespace

TypeId
LeoSimIslLoadModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimIslLoadModel")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimIslLoadModel>();
    return tid;
}

void
LeoSimIslLoadModel::SetSeed(uint64_t seed)
{
    m_seed = seed;
}

void
LeoSimIslLoadModel::SetLoadRange(double minimum, double maximum)
{
    NS_ABORT_MSG_IF(!std::isfinite(minimum) || !std::isfinite(maximum),
                    "Synthetic ISL load bounds must be finite");
    NS_ABORT_MSG_IF(minimum < 0.0 || maximum > 1.0 || minimum > maximum,
                    "Synthetic ISL load range must satisfy 0 <= min <= max <= 1");
    m_minimumLoad = minimum;
    m_maximumLoad = maximum;
}

void
LeoSimIslLoadModel::SetDistribution(const std::string& distribution)
{
    NS_ABORT_MSG_IF(distribution != "uniform",
                    "Unsupported synthetic ISL load distribution: " << distribution);
    m_distribution = distribution;
}

void
LeoSimIslLoadModel::RegisterSatellite(uint32_t satelliteId, uint64_t maximumCapacityBps)
{
    LeoSimSatelliteIslCapacity& state = m_satellites[satelliteId];
    state.satelliteId = satelliteId;
    state.maximumCapacityBps = maximumCapacityBps;
}

void
LeoSimIslLoadModel::RegisterDirectedIsl(uint32_t sourceSatelliteId,
                                        uint32_t destinationSatelliteId,
                                        uint64_t physicalCapacityBps)
{
    NS_ABORT_MSG_IF(sourceSatelliteId == destinationSatelliteId,
                    "A synthetic ISL must connect different satellites");
    LeoSimIslLoadRecord& record = m_links[{sourceSatelliteId, destinationSatelliteId}];
    record.sourceSatelliteId = sourceSatelliteId;
    record.destinationSatelliteId = destinationSatelliteId;
    record.active = true;
    record.physicalCapacityBps = physicalCapacityBps;
}

double
LeoSimIslLoadModel::GenerateUniform(uint32_t sourceSatelliteId,
                                    uint32_t destinationSatelliteId,
                                    uint64_t epoch) const
{
    uint64_t hash = Mix64(m_seed);
    hash = Mix64(hash ^ static_cast<uint64_t>(sourceSatelliteId));
    hash = Mix64(hash ^ (static_cast<uint64_t>(destinationSatelliteId) << 1));
    hash = Mix64(hash ^ epoch);
    const double unit = static_cast<double>(hash >> 11) * (1.0 / 9007199254740992.0);
    return m_minimumLoad + unit * (m_maximumLoad - m_minimumLoad);
}

void
LeoSimIslLoadModel::Generate(uint64_t epoch)
{
    for (auto& [key, record] : m_links)
    {
        (void)key;
        record.requestedSyntheticLoad =
            GenerateUniform(record.sourceSatelliteId, record.destinationSatelliteId, epoch);
        record.requestedBackgroundBps = static_cast<uint64_t>(std::llround(
            static_cast<long double>(record.physicalCapacityBps) *
            record.requestedSyntheticLoad));

    }

    RecalculateCapacity();
}

void
LeoSimIslLoadModel::AttachChannelModel(Ptr<LeoSimChannelModel> channelModel,
                                       const NetDeviceContainer& devices)
{
    m_channelModel = channelModel;
    m_devices = devices;
    if (!m_channelModel)
    {
        return;
    }
    for (auto& [key, record] : m_links)
    {
        (void)key;
        const LeoSimLinkState state = m_channelModel->GetLinkState(
            record.sourceSatelliteId,
            record.destinationSatelliteId);
        record.active = state == LEOSIM_LINK_UP || state == LEOSIM_LINK_DEGRADED;
    }
    m_channelModel->TraceConnectWithoutContext(
        "LinkStateChange",
        MakeCallback(&LeoSimIslLoadModel::OnLinkStateChange, this));
}

void
LeoSimIslLoadModel::RecalculateCapacity()
{
    std::map<uint32_t, uint64_t> requestedBySatellite;
    for (auto& [satelliteId, state] : m_satellites)
    {
        (void)satelliteId;
        state.backgroundBps = 0;
        state.droppedBackgroundBps = 0;
        state.remainingCapacityBps = state.maximumCapacityBps;
        state.utilization = 0.0;
    }
    for (const auto& [key, record] : m_links)
    {
        (void)key;
        if (record.active)
        {
            requestedBySatellite[record.sourceSatelliteId] = SaturatingAdd(
                requestedBySatellite[record.sourceSatelliteId],
                record.requestedBackgroundBps);
        }
    }

    std::map<uint32_t, double> admissionScale;
    for (const auto& [satelliteId, state] : m_satellites)
    {
        const uint64_t requested = requestedBySatellite[satelliteId];
        const double scale = requested == 0
                                 ? 1.0
                                 : std::min(1.0,
                                            static_cast<double>(state.maximumCapacityBps) /
                                                static_cast<double>(requested));
        admissionScale[satelliteId] = scale;
    }

    for (auto& [key, record] : m_links)
    {
        (void)key;
        if (!record.active)
        {
            record.admittedBackgroundBps = 0;
            record.droppedBackgroundBps = 0;
            record.remainingCapacityBps = record.physicalCapacityBps;
            record.effectiveCapacityBps = 0;
            record.effectiveUtilization = 0.0;
            continue;
        }
        const double scale = admissionScale[record.sourceSatelliteId];
        record.admittedBackgroundBps = static_cast<uint64_t>(std::floor(
            static_cast<long double>(record.requestedBackgroundBps) * scale));
        record.droppedBackgroundBps =
            record.requestedBackgroundBps - record.admittedBackgroundBps;
        record.remainingCapacityBps =
            record.physicalCapacityBps - record.admittedBackgroundBps;
        record.effectiveUtilization = record.physicalCapacityBps == 0
                                          ? 0.0
                                          : static_cast<double>(record.admittedBackgroundBps) /
                                                static_cast<double>(record.physicalCapacityBps);
        LeoSimSatelliteIslCapacity& satellite = m_satellites[record.sourceSatelliteId];
        satellite.backgroundBps =
            SaturatingAdd(satellite.backgroundBps, record.admittedBackgroundBps);
        satellite.droppedBackgroundBps =
            SaturatingAdd(satellite.droppedBackgroundBps, record.droppedBackgroundBps);
    }

    for (auto& [satelliteId, state] : m_satellites)
    {
        (void)satelliteId;
        state.remainingCapacityBps = state.backgroundBps >= state.maximumCapacityBps
                                         ? 0
                                         : state.maximumCapacityBps - state.backgroundBps;
        state.utilization = state.maximumCapacityBps == 0
                                ? 0.0
                                : static_cast<double>(state.backgroundBps) /
                                      static_cast<double>(state.maximumCapacityBps);
    }

    // Share each satellite's residual forwarding budget among its outgoing
    // ISLs in proportion to their link-level residual capacities.
    std::map<uint32_t, uint64_t> outgoingResidualTotal;
    for (const auto& [key, record] : m_links)
    {
        (void)key;
        if (record.active)
        {
            outgoingResidualTotal[record.sourceSatelliteId] = SaturatingAdd(
                outgoingResidualTotal[record.sourceSatelliteId],
                record.remainingCapacityBps);
        }
    }
    std::map<uint32_t, std::vector<LeoSimIslLoadRecord*>> outgoingLinks;
    for (auto& [key, record] : m_links)
    {
        (void)key;
        const auto satIt = m_satellites.find(record.sourceSatelliteId);
        const uint64_t total = outgoingResidualTotal[record.sourceSatelliteId];
        if (!record.active || satIt == m_satellites.end() || total == 0)
        {
            record.effectiveCapacityBps = 0;
            continue;
        }
        if (satIt->second.remainingCapacityBps >= total)
        {
            record.effectiveCapacityBps = record.remainingCapacityBps;
        }
        else
        {
            const long double share =
                static_cast<long double>(satIt->second.remainingCapacityBps) *
                static_cast<long double>(record.remainingCapacityBps) /
                static_cast<long double>(total);
            record.effectiveCapacityBps = std::min(
                record.remainingCapacityBps,
                static_cast<uint64_t>(std::floor(share)));
        }
        outgoingLinks[record.sourceSatelliteId].push_back(&record);
    }

    // Distribute proportional-allocation remainder bits deterministically in
    // link-key order instead of silently losing them to integer rounding.
    for (auto& [satelliteId, links] : outgoingLinks)
    {
        const uint64_t target = std::min(m_satellites[satelliteId].remainingCapacityBps,
                                         outgoingResidualTotal[satelliteId]);
        uint64_t allocated = 0;
        for (const auto* link : links)
        {
            allocated = SaturatingAdd(allocated, link->effectiveCapacityBps);
        }
        uint64_t remainder = target > allocated ? target - allocated : 0;
        for (auto* link : links)
        {
            if (remainder == 0)
            {
                break;
            }
            if (link->effectiveCapacityBps < link->remainingCapacityBps)
            {
                ++link->effectiveCapacityBps;
                --remainder;
            }
        }
    }
}

void
LeoSimIslLoadModel::OnLinkStateChange(Ptr<Node> first,
                                      Ptr<Node> second,
                                      LeoSimLinkState state)
{
    if (!first || !second)
    {
        return;
    }
    const bool active = state == LEOSIM_LINK_UP || state == LEOSIM_LINK_DEGRADED;
    auto forward = m_links.find({first->GetId(), second->GetId()});
    if (forward != m_links.end())
    {
        forward->second.active = active;
    }
    auto reverse = m_links.find({second->GetId(), first->GetId()});
    if (reverse != m_links.end())
    {
        reverse->second.active = active;
    }
    if (!m_topologyUpdateEvent.IsPending())
    {
        m_topologyUpdateEvent = Simulator::ScheduleNow(
            &LeoSimIslLoadModel::ApplyPendingTopologyUpdate,
            this);
    }
}

void
LeoSimIslLoadModel::ApplyPendingTopologyUpdate()
{
    RecalculateCapacity();
    ApplyToIslDevices(m_devices);
    if (!m_csvFilename.empty())
    {
        WriteCsv(m_csvFilename, Simulator::Now().GetSeconds(), true);
    }
}

bool
LeoSimIslLoadModel::GetIslLoad(uint32_t sourceSatelliteId,
                               uint32_t destinationSatelliteId,
                               LeoSimIslLoadRecord& record) const
{
    auto it = m_links.find({sourceSatelliteId, destinationSatelliteId});
    if (it == m_links.end())
    {
        return false;
    }
    record = it->second;
    return true;
}

bool
LeoSimIslLoadModel::GetSatelliteCapacity(uint32_t satelliteId,
                                         LeoSimSatelliteIslCapacity& capacity) const
{
    auto it = m_satellites.find(satelliteId);
    if (it == m_satellites.end())
    {
        return false;
    }
    capacity = it->second;
    return true;
}

std::vector<LeoSimIslLoadRecord>
LeoSimIslLoadModel::GetAllIslLoads() const
{
    std::vector<LeoSimIslLoadRecord> records;
    records.reserve(m_links.size());
    for (const auto& [key, record] : m_links)
    {
        (void)key;
        records.push_back(record);
    }
    return records;
}

std::vector<LeoSimSatelliteIslCapacity>
LeoSimIslLoadModel::GetAllSatelliteCapacities() const
{
    std::vector<LeoSimSatelliteIslCapacity> records;
    records.reserve(m_satellites.size());
    for (const auto& [satelliteId, record] : m_satellites)
    {
        (void)satelliteId;
        records.push_back(record);
    }
    return records;
}

double
LeoSimIslLoadModel::GetRoutingCost(uint32_t sourceSatelliteId,
                                   uint32_t destinationSatelliteId) const
{
    LeoSimIslLoadRecord load;
    if (!GetIslLoad(sourceSatelliteId, destinationSatelliteId, load))
    {
        return 1.0;
    }
    if (load.physicalCapacityBps == 0 || load.effectiveCapacityBps == 0)
    {
        return 1.0e12;
    }
    return 1.0 / std::max(static_cast<double>(load.effectiveCapacityBps) /
                              static_cast<double>(load.physicalCapacityBps),
                          1.0e-12);
}

void
LeoSimIslLoadModel::WriteCsv(const std::string& filename,
                             double timeSeconds,
                             bool append) const
{
    std::ofstream out(filename, append ? std::ios::app : std::ios::trunc);
    if (!out)
    {
        throw std::runtime_error("cannot open synthetic ISL load CSV: " + filename);
    }
    if (!append)
    {
        out << "time_s,source_satellite,destination_satellite,active,physical_capacity_bps,"
               "requested_load,requested_background_bps,admitted_background_bps,"
               "dropped_background_bps,effective_utilization,link_remaining_bps,"
               "satellite_capacity_bps,satellite_background_bps,"
               "satellite_dropped_background_bps,satellite_remaining_bps,"
               "effective_capacity_bps,routing_cost\n";
    }
    out << std::setprecision(12);
    for (const auto& [key, load] : m_links)
    {
        (void)key;
        LeoSimSatelliteIslCapacity satellite;
        GetSatelliteCapacity(load.sourceSatelliteId, satellite);
        out << timeSeconds << ',' << load.sourceSatelliteId << ','
            << load.destinationSatelliteId << ',' << (load.active ? 1 : 0) << ','
            << load.physicalCapacityBps << ','
            << load.requestedSyntheticLoad << ',' << load.requestedBackgroundBps << ','
            << load.admittedBackgroundBps << ',' << load.droppedBackgroundBps << ','
            << load.effectiveUtilization << ',' << load.remainingCapacityBps << ','
            << satellite.maximumCapacityBps << ',' << satellite.backgroundBps << ','
            << satellite.droppedBackgroundBps << ',' << satellite.remainingCapacityBps << ','
            << load.effectiveCapacityBps << ','
            << GetRoutingCost(load.sourceSatelliteId, load.destinationSatelliteId) << '\n';
    }
}

void
LeoSimIslLoadModel::EnableCsvOutput(const std::string& filename)
{
    m_csvFilename = filename;
    WriteCsv(m_csvFilename, Simulator::Now().GetSeconds(), false);
}

void
LeoSimIslLoadModel::ApplyToIslDevices(NetDeviceContainer& devices) const
{
    for (uint32_t i = 0; i + 1 < devices.GetN(); i += 2)
    {
        Ptr<PointToPointNetDevice> deviceA = DynamicCast<PointToPointNetDevice>(devices.Get(i));
        Ptr<PointToPointNetDevice> deviceB =
            DynamicCast<PointToPointNetDevice>(devices.Get(i + 1));
        if (!deviceA || !deviceB || !deviceA->GetNode() || !deviceB->GetNode())
        {
            continue;
        }

        LeoSimIslLoadRecord forward;
        if (GetIslLoad(deviceA->GetNode()->GetId(), deviceB->GetNode()->GetId(), forward))
        {
            // ns-3 DataRate must be positive. One bit/s behaves as a closed
            // link for ordinary packets while allowing its finite queue to drop.
            deviceA->SetDataRate(DataRate(std::max<uint64_t>(1, forward.effectiveCapacityBps)));
        }
        LeoSimIslLoadRecord reverse;
        if (GetIslLoad(deviceB->GetNode()->GetId(), deviceA->GetNode()->GetId(), reverse))
        {
            deviceB->SetDataRate(DataRate(std::max<uint64_t>(1, reverse.effectiveCapacityBps)));
        }
    }
}

} // namespace ns3
