/*
 * Copyright (c) 2026 Anupa De Silva
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include "leosim-beam-capacity-manager.h"

#include "leosim-beam-manager.h"

#include "ns3/abort.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/queue.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimBeamCapacityManager");
NS_OBJECT_ENSURE_REGISTERED(LeoSimBeamCapacityManager);

TypeId
LeoSimBeamCapacityManager::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimBeamCapacityManager")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimBeamCapacityManager>();
    return tid;
}

LeoSimBeamCapacityManager::LeoSimBeamCapacityManager()
    : m_uplinkBeamCapacity("100Mbps"),
      m_downlinkBeamCapacity("100Mbps"),
      m_defaultUplinkPackageRate("100Mbps"),
      m_defaultDownlinkPackageRate("100Mbps"),
      m_satelliteUplinkCapacity(std::numeric_limits<uint64_t>::max()),
      m_satelliteDownlinkCapacity(std::numeric_limits<uint64_t>::max()),
      m_updateInterval(MilliSeconds(100)),
      m_activeUserTimeout(MilliSeconds(200)),
      m_scheduler(LEOSIM_BEAM_SCHEDULER_PROPORTIONAL_FAIR),
      m_alphaFairness(1.0),
      m_queueDelayWeight(1.0),
      m_demandAware(true),
      m_running(false)
{
}

LeoSimBeamCapacityManager::~LeoSimBeamCapacityManager()
{
    Stop();
}

bool
LeoSimBeamCapacityManager::LinkKey::operator<(const LinkKey& other) const
{
    return groundNodeId < other.groundNodeId ||
           (groundNodeId == other.groundNodeId && satelliteNodeId < other.satelliteNodeId);
}

bool
LeoSimBeamCapacityManager::BeamKey::operator<(const BeamKey& other) const
{
    return satelliteNodeId < other.satelliteNodeId ||
           (satelliteNodeId == other.satelliteNodeId && beamId < other.beamId);
}

void
LeoSimBeamCapacityManager::SetBeamManager(Ptr<LeoSimBeamManager> beamManager)
{
    m_beamManager = beamManager;
}

void
LeoSimBeamCapacityManager::SetBeamCapacity(DataRate capacity)
{
    NS_ABORT_MSG_IF(capacity.GetBitRate() == 0, "Beam capacity must be greater than zero");
    m_uplinkBeamCapacity = capacity;
    m_downlinkBeamCapacity = capacity;
}

void
LeoSimBeamCapacityManager::SetUplinkBeamCapacity(DataRate capacity)
{
    NS_ABORT_MSG_IF(capacity.GetBitRate() == 0, "Uplink beam capacity must be positive");
    m_uplinkBeamCapacity = capacity;
}

void
LeoSimBeamCapacityManager::SetDownlinkBeamCapacity(DataRate capacity)
{
    NS_ABORT_MSG_IF(capacity.GetBitRate() == 0, "Downlink beam capacity must be positive");
    m_downlinkBeamCapacity = capacity;
}

void
LeoSimBeamCapacityManager::SetDefaultUplinkPackageRate(DataRate rate)
{
    NS_ABORT_MSG_IF(rate.GetBitRate() == 0, "Uplink package rate must be positive");
    m_defaultUplinkPackageRate = rate;
}

void
LeoSimBeamCapacityManager::SetDefaultDownlinkPackageRate(DataRate rate)
{
    NS_ABORT_MSG_IF(rate.GetBitRate() == 0, "Downlink package rate must be positive");
    m_defaultDownlinkPackageRate = rate;
}

void
LeoSimBeamCapacityManager::SetSatelliteUplinkCapacity(DataRate capacity)
{
    NS_ABORT_MSG_IF(capacity.GetBitRate() == 0, "Satellite uplink capacity must be positive");
    m_satelliteUplinkCapacity = capacity;
}

void
LeoSimBeamCapacityManager::SetSatelliteDownlinkCapacity(DataRate capacity)
{
    NS_ABORT_MSG_IF(capacity.GetBitRate() == 0, "Satellite downlink capacity must be positive");
    m_satelliteDownlinkCapacity = capacity;
}

void
LeoSimBeamCapacityManager::SetAlphaFairness(double alpha)
{
    NS_ABORT_MSG_IF(alpha < 0.0, "Alpha fairness must be non-negative");
    m_alphaFairness = alpha;
}

void
LeoSimBeamCapacityManager::SetQueueDelayWeight(double weight)
{
    NS_ABORT_MSG_IF(weight < 0.0, "Queue weight must be non-negative");
    m_queueDelayWeight = weight;
}

void
LeoSimBeamCapacityManager::SetScheduler(LeoSimBeamScheduler scheduler)
{
    m_scheduler = scheduler;
}

void
LeoSimBeamCapacityManager::SetDemandAware(bool enabled)
{
    m_demandAware = enabled;
}

void
LeoSimBeamCapacityManager::SetActiveUserTimeout(Time timeout)
{
    NS_ABORT_MSG_IF(timeout < Time(0), "Active-user timeout cannot be negative");
    m_activeUserTimeout = timeout;
}

void
LeoSimBeamCapacityManager::SetUpdateInterval(Time interval)
{
    NS_ABORT_MSG_IF(interval <= Time(0), "Beam capacity update interval must be positive");
    m_updateInterval = interval;
}

void
LeoSimBeamCapacityManager::SetUeServiceProfile(uint32_t groundNodeId,
                                                const LeoSimUeServiceProfile& profile)
{
    NS_ABORT_MSG_IF(profile.weight <= 0.0, "UE service weight must be positive");
    NS_ABORT_MSG_IF(profile.uplinkMinimum.GetBitRate() > profile.uplinkPeak.GetBitRate() ||
                        profile.downlinkMinimum.GetBitRate() > profile.downlinkPeak.GetBitRate(),
                    "UE minimum rate cannot exceed its package peak");
    m_profiles[groundNodeId] = profile;
    for (auto& item : m_links)
    {
        if (item.first.groundNodeId == groundNodeId)
        {
            item.second.uplinkPackageCapBps =
                std::min(item.second.baselineUplinkBps, profile.uplinkPeak.GetBitRate());
            item.second.downlinkPackageCapBps =
                std::min(item.second.baselineDownlinkBps, profile.downlinkPeak.GetBitRate());
            item.second.uplinkMinimumBps = profile.uplinkMinimum.GetBitRate();
            item.second.downlinkMinimumBps = profile.downlinkMinimum.GetBitRate();
            item.second.serviceWeight = profile.weight;
        }
    }
}

bool
LeoSimBeamCapacityManager::LoadUeServiceProfiles(const std::string& filename)
{
    std::ifstream input(filename);
    if (!input.is_open())
    {
        return false;
    }
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        std::stringstream stream(line);
        std::string nodeId;
        std::string downlinkPeak;
        std::string downlinkMinimum;
        std::string uplinkPeak;
        std::string uplinkMinimum;
        std::string weight;
        if (!std::getline(stream, nodeId, ',') || !std::getline(stream, downlinkPeak, ',') ||
            !std::getline(stream, downlinkMinimum, ',') || !std::getline(stream, uplinkPeak, ',') ||
            !std::getline(stream, uplinkMinimum, ',') || !std::getline(stream, weight, ','))
        {
            return false;
        }
        try
        {
            LeoSimUeServiceProfile profile;
            profile.downlinkPeak = DataRate(downlinkPeak);
            profile.downlinkMinimum = DataRate(downlinkMinimum);
            profile.uplinkPeak = DataRate(uplinkPeak);
            profile.uplinkMinimum = DataRate(uplinkMinimum);
            profile.weight = std::stod(weight);
            SetUeServiceProfile(static_cast<uint32_t>(std::stoul(nodeId)), profile);
        }
        catch (const std::exception&)
        {
            return false;
        }
    }
    return true;
}

bool
LeoSimBeamCapacityManager::RegisterAccessLink(Ptr<Node> groundNode,
                                               Ptr<Node> satelliteNode,
                                               Ptr<NetDevice> groundDevice,
                                               Ptr<NetDevice> satelliteDevice)
{
    Ptr<PointToPointNetDevice> groundP2p = DynamicCast<PointToPointNetDevice>(groundDevice);
    Ptr<PointToPointNetDevice> satelliteP2p =
        DynamicCast<PointToPointNetDevice>(satelliteDevice);
    if (!groundNode || !satelliteNode || !groundP2p || !satelliteP2p)
    {
        NS_LOG_WARN("Ignoring invalid or non-point-to-point access link");
        return false;
    }

    DataRateValue uplink;
    DataRateValue downlink;
    groundP2p->GetAttribute("DataRate", uplink);
    satelliteP2p->GetAttribute("DataRate", downlink);
    const LinkKey key{groundNode->GetId(), satelliteNode->GetId()};
    const uint64_t baselineUplink = uplink.Get().GetBitRate();
    const uint64_t baselineDownlink = downlink.Get().GetBitRate();
    const uint64_t uplinkCap = std::min(baselineUplink,
                                        m_defaultUplinkPackageRate.GetBitRate());
    const uint64_t downlinkCap = std::min(baselineDownlink,
                                          m_defaultDownlinkPackageRate.GetBitRate());
    const auto profileIt = m_profiles.find(groundNode->GetId());
    const uint64_t profileUplinkCap = profileIt == m_profiles.end()
                                          ? uplinkCap
                                          : std::min(baselineUplink,
                                                     profileIt->second.uplinkPeak.GetBitRate());
    const uint64_t profileDownlinkCap = profileIt == m_profiles.end()
                                            ? downlinkCap
                                            : std::min(baselineDownlink,
                                                       profileIt->second.downlinkPeak.GetBitRate());
    m_links[key] = {groundP2p,
                    satelliteP2p,
                    baselineUplink,
                    baselineDownlink,
                    profileUplinkCap,
                    profileDownlinkCap,
                    profileIt == m_profiles.end()
                        ? 0
                        : profileIt->second.uplinkMinimum.GetBitRate(),
                    profileIt == m_profiles.end()
                        ? 0
                        : profileIt->second.downlinkMinimum.GetBitRate(),
                    profileIt == m_profiles.end() ? 1.0 : profileIt->second.weight,
                    Simulator::Now(),
                    Simulator::Now(),
                    1.0,
                    1.0};

    const std::string uplinkContext = "uplink:" + std::to_string(groundNode->GetId()) + ":" +
                                      std::to_string(satelliteNode->GetId());
    const std::string downlinkContext = "downlink:" + std::to_string(groundNode->GetId()) + ":" +
                                        std::to_string(satelliteNode->GetId());
    m_traceContexts[uplinkContext] = {key, true};
    m_traceContexts[downlinkContext] = {key, false};
    groundP2p->TraceConnect("MacTx",
                            uplinkContext,
                            MakeCallback(&LeoSimBeamCapacityManager::NotifyDeviceTx, this));
    satelliteP2p->TraceConnect("MacTx",
                               downlinkContext,
                               MakeCallback(&LeoSimBeamCapacityManager::NotifyDeviceTx, this));
    return true;
}

void
LeoSimBeamCapacityManager::NotifyDeviceTx(std::string context, Ptr<const Packet> packet)
{
    (void)packet;
    auto trace = m_traceContexts.find(context);
    if (trace == m_traceContexts.end())
    {
        return;
    }
    if (trace->second.second)
    {
        MarkUplinkDemand(trace->second.first.groundNodeId,
                         trace->second.first.satelliteNodeId);
    }
    else
    {
        MarkDownlinkDemand(trace->second.first.groundNodeId,
                           trace->second.first.satelliteNodeId);
    }
}

void
LeoSimBeamCapacityManager::MarkUplinkDemand(uint32_t groundNodeId, uint32_t satelliteNodeId)
{
    auto link = m_links.find({groundNodeId, satelliteNodeId});
    if (link != m_links.end())
    {
        const bool wasActive = Simulator::Now() - link->second.lastUplinkDemand <=
                               m_activeUserTimeout;
        link->second.lastUplinkDemand = Simulator::Now();
        if (m_running && !wasActive)
        {
            if (m_updateEvent.IsPending())
            {
                Simulator::Cancel(m_updateEvent);
            }
            m_updateEvent = Simulator::ScheduleNow(&LeoSimBeamCapacityManager::UpdateNow, this);
        }
    }
}

void
LeoSimBeamCapacityManager::MarkDownlinkDemand(uint32_t groundNodeId, uint32_t satelliteNodeId)
{
    auto link = m_links.find({groundNodeId, satelliteNodeId});
    if (link != m_links.end())
    {
        const bool wasActive = Simulator::Now() - link->second.lastDownlinkDemand <=
                               m_activeUserTimeout;
        link->second.lastDownlinkDemand = Simulator::Now();
        if (m_running && !wasActive)
        {
            if (m_updateEvent.IsPending())
            {
                Simulator::Cancel(m_updateEvent);
            }
            m_updateEvent = Simulator::ScheduleNow(&LeoSimBeamCapacityManager::UpdateNow, this);
        }
    }
}

void
LeoSimBeamCapacityManager::Start()
{
    if (m_running)
    {
        return;
    }
    m_running = true;
    UpdateNow();
}

void
LeoSimBeamCapacityManager::Stop()
{
    m_running = false;
    if (m_updateEvent.IsPending())
    {
        Simulator::Cancel(m_updateEvent);
    }
    if (m_csv.is_open())
    {
        m_csv.close();
    }
}

void
LeoSimBeamCapacityManager::UpdateNow()
{
    std::vector<LeoSimBeamCapacityAssociation> associations;
    if (m_beamManager)
    {
        std::set<uint32_t> visitedGroundNodes;
        for (const auto& item : m_links)
        {
            const uint32_t groundId = item.first.groundNodeId;
            if (!visitedGroundNodes.insert(groundId).second)
            {
                continue;
            }
            const LeoSimBeamRecord beam = m_beamManager->GetCurrentBeam(groundId);
            if (beam.state != LEOSIM_BEAM_SEARCHING && beam.beamActive &&
                beam.satelliteNodeId != std::numeric_limits<uint32_t>::max() &&
                m_beamManager->IsServingAccessLink(groundId, beam.satelliteNodeId))
            {
                const LinkKey key{groundId, beam.satelliteNodeId};
                const AccessLink& link = m_links.at(key);
                const Time now = Simulator::Now();
                const uint32_t uplinkQueueBytes = link.groundDevice->GetQueue()->GetNBytes();
                const uint32_t downlinkQueueBytes = link.satelliteDevice->GetQueue()->GetNBytes();
                associations.push_back({groundId,
                                        beam.satelliteNodeId,
                                        beam.beamId,
                                        beam.sinr,
                                        !m_demandAware || uplinkQueueBytes > 0 ||
                                            now - link.lastUplinkDemand <= m_activeUserTimeout,
                                        !m_demandAware || downlinkQueueBytes > 0 ||
                                            now - link.lastDownlinkDemand <= m_activeUserTimeout,
                                        uplinkQueueBytes,
                                        downlinkQueueBytes});
            }
        }
    }
    ApplyAssociations(associations);
    if (m_running)
    {
        ScheduleNextUpdate();
    }
}

void
LeoSimBeamCapacityManager::ApplyAssociations(
    const std::vector<LeoSimBeamCapacityAssociation>& associations)
{
    std::map<BeamKey, std::vector<LeoSimBeamCapacityAssociation>> groups;
    std::set<LinkKey> seenLinks;
    for (const auto& association : associations)
    {
        LinkKey link{association.groundNodeId, association.satelliteNodeId};
        if (m_links.find(link) != m_links.end() && seenLinks.insert(link).second)
        {
            groups[{association.satelliteNodeId, association.beamId}].push_back(association);
        }
    }

    for (const auto& previous : m_allocatedLinks)
    {
        if (seenLinks.find(previous.first) == seenLinks.end())
        {
            RestoreLink(previous.first);
        }
    }

    m_allocations.clear();
    m_allocatedLinks.clear();
    const auto uplinkBudgets = AllocateBeamBudgets(groups, true);
    const auto downlinkBudgets = AllocateBeamBudgets(groups, false);
    for (const auto& group : groups)
    {
        const uint32_t count = static_cast<uint32_t>(group.second.size());
        const auto uplinkRates = AllocateDirection(group.second,
                                                   true,
                                                   uplinkBudgets.at(group.first));
        const auto downlinkRates = AllocateDirection(group.second,
                                                     false,
                                                     downlinkBudgets.at(group.first));
        uint32_t activeUplinks = 0;
        uint32_t activeDownlinks = 0;
        for (const auto& association : group.second)
        {
            activeUplinks += association.uplinkActive ? 1 : 0;
            activeDownlinks += association.downlinkActive ? 1 : 0;
        }
        for (const auto& association : group.second)
        {
            LinkKey key{association.groundNodeId, association.satelliteNodeId};
            AccessLink& link = m_links.at(key);
            const uint64_t uplink = uplinkRates.at(key);
            const uint64_t downlink = downlinkRates.at(key);
            // An idle link retains its subscription cap so its first packet can reveal
            // new demand; active links are constrained by the scheduler allocation.
            link.groundDevice->SetDataRate(
                DataRate(uplink > 0 ? uplink : link.uplinkPackageCapBps));
            link.satelliteDevice->SetDataRate(
                DataRate(downlink > 0 ? downlink : link.downlinkPackageCapBps));
            m_allocatedLinks[key] = true;
            m_allocations.push_back({association.groundNodeId,
                                     association.satelliteNodeId,
                                     association.beamId,
                                     count,
                                     activeUplinks,
                                     activeDownlinks,
                                     downlinkBudgets.at(group.first),
                                     uplinkBudgets.at(group.first),
                                     downlinkBudgets.at(group.first),
                                     link.uplinkPackageCapBps,
                                     link.downlinkPackageCapBps,
                                     link.uplinkMinimumBps,
                                     link.downlinkMinimumBps,
                                     association.uplinkQueuedBytes,
                                     association.downlinkQueuedBytes,
                                     uplink,
                                     downlink,
                                     association.sinrDb,
                                     m_scheduler == LEOSIM_BEAM_SCHEDULER_EQUAL
                                         ? "equal"
                                         : (m_scheduler == LEOSIM_BEAM_SCHEDULER_ALPHA_FAIR
                                                ? "alpha-fair"
                                                : "pf"),
                                     GetLimitReason(link,
                                                    association,
                                                    true,
                                                    uplink,
                                                    uplinkBudgets.at(group.first)),
                                     GetLimitReason(link,
                                                    association,
                                                    false,
                                                    downlink,
                                                    downlinkBudgets.at(group.first))});
        }
    }
    WriteCsvRows();
}

std::map<LeoSimBeamCapacityManager::LinkKey, uint64_t>
LeoSimBeamCapacityManager::AllocateDirection(
    const std::vector<LeoSimBeamCapacityAssociation>& associations,
    bool uplink,
    uint64_t capacity)
{
    std::map<LinkKey, uint64_t> result;
    std::map<LinkKey, double> weights;
    std::map<LinkKey, uint64_t> caps;
    std::map<LinkKey, uint64_t> minimums;
    std::set<LinkKey> remaining;
    uint64_t minimumTotal = 0;
    for (const auto& association : associations)
    {
        const LinkKey key{association.groundNodeId, association.satelliteNodeId};
        AccessLink& link = m_links.at(key);
        const bool active = uplink ? association.uplinkActive : association.downlinkActive;
        result[key] = 0;
        if (!active)
        {
            continue;
        }
        const uint64_t cap = uplink ? link.uplinkPackageCapBps : link.downlinkPackageCapBps;
        const uint64_t minimum = std::min(cap,
                                          uplink ? link.uplinkMinimumBps
                                                 : link.downlinkMinimumBps);
        const double average = uplink ? link.averageUplinkBps : link.averageDownlinkBps;
        const uint32_t queuedBytes = uplink ? association.uplinkQueuedBytes
                                            : association.downlinkQueuedBytes;
        const double epochDemandBytes = std::max(
            1.0,
            static_cast<double>(cap) * m_updateInterval.GetSeconds() / 8.0);
        const double queuePressure = 1.0 + m_queueDelayWeight *
                                               std::min(10.0,
                                                        queuedBytes / epochDemandBytes);
        const double alpha = m_scheduler == LEOSIM_BEAM_SCHEDULER_ALPHA_FAIR
                                 ? m_alphaFairness
                                 : 1.0;
        caps[key] = cap;
        minimums[key] = minimum;
        result[key] = minimum;
        minimumTotal += minimum;
        weights[key] = m_scheduler == LEOSIM_BEAM_SCHEDULER_EQUAL
                           ? 1.0
                           : link.serviceWeight * queuePressure *
                                 SpectralEfficiency(association.sinrDb) /
                                 std::pow(std::max(1.0, average), alpha);
        if (minimum < cap)
        {
            remaining.insert(key);
        }
    }

    NS_ABORT_MSG_IF(capacity < caps.size(),
                    "Directional beam capacity in bps is smaller than active-user count");

    if (minimumTotal > capacity)
    {
        uint64_t assigned = 0;
        for (auto& item : result)
        {
            item.second = static_cast<uint64_t>(
                static_cast<long double>(capacity) * minimums.at(item.first) / minimumTotal);
            assigned += item.second;
        }
        uint64_t remainder = capacity - assigned;
        for (auto& item : result)
        {
            if (remainder == 0)
            {
                break;
            }
            if (caps.find(item.first) != caps.end())
            {
                ++item.second;
                --remainder;
            }
        }
        remaining.clear();
    }

    uint64_t budget = minimumTotal <= capacity ? capacity - minimumTotal : 0;
    while (!remaining.empty() && budget > 0)
    {
        double totalWeight = 0.0;
        for (const auto& key : remaining)
        {
            totalWeight += weights.at(key);
        }
        bool cappedAny = false;
        for (auto it = remaining.begin(); it != remaining.end();)
        {
            const LinkKey key = *it;
            const uint64_t weightedShare = static_cast<uint64_t>(
                static_cast<long double>(budget) * weights.at(key) / totalWeight);
            const uint64_t needed = caps.at(key) - result.at(key);
            if (weightedShare >= needed)
            {
                result[key] += needed;
                budget -= needed;
                it = remaining.erase(it);
                cappedAny = true;
            }
            else
            {
                ++it;
            }
        }
        if (!cappedAny)
        {
            uint64_t assigned = 0;
            for (const auto& key : remaining)
            {
                const uint64_t share = static_cast<uint64_t>(
                    static_cast<long double>(budget) * weights.at(key) / totalWeight);
                result[key] += share;
                assigned += share;
            }
            budget -= std::min(budget, assigned);
            break;
        }
    }

    // Preserve the exact capacity invariant despite integer division, without
    // exceeding a subscription ceiling.
    while (budget > 0)
    {
        bool assigned = false;
        for (auto& item : result)
        {
            auto cap = caps.find(item.first);
            if (cap != caps.end() && item.second < cap->second)
            {
                ++item.second;
                --budget;
                assigned = true;
                if (budget == 0)
                {
                    break;
                }
            }
        }
        if (!assigned)
        {
            break;
        }
    }

    for (const auto& association : associations)
    {
        const LinkKey key{association.groundNodeId, association.satelliteNodeId};
        AccessLink& link = m_links.at(key);
        double& average = uplink ? link.averageUplinkBps : link.averageDownlinkBps;
        average = 0.9 * average + 0.1 * static_cast<double>(result.at(key));
    }
    return result;
}

std::map<LeoSimBeamCapacityManager::BeamKey, uint64_t>
LeoSimBeamCapacityManager::AllocateBeamBudgets(
    const std::map<BeamKey, std::vector<LeoSimBeamCapacityAssociation>>& groups,
    bool uplink) const
{
    std::map<BeamKey, uint64_t> budgets;
    std::map<uint32_t, std::vector<BeamKey>> satelliteBeams;
    for (const auto& group : groups)
    {
        uint64_t demand = 0;
        for (const auto& association : group.second)
        {
            const bool active = uplink ? association.uplinkActive : association.downlinkActive;
            if (!active)
            {
                continue;
            }
            const AccessLink& link =
                m_links.at({association.groundNodeId, association.satelliteNodeId});
            demand += uplink ? link.uplinkPackageCapBps : link.downlinkPackageCapBps;
        }
        const uint64_t beamCap = uplink ? m_uplinkBeamCapacity.GetBitRate()
                                        : m_downlinkBeamCapacity.GetBitRate();
        budgets[group.first] = std::min(beamCap, demand);
        satelliteBeams[group.first.satelliteNodeId].push_back(group.first);
    }

    const uint64_t satelliteCap = uplink ? m_satelliteUplinkCapacity.GetBitRate()
                                         : m_satelliteDownlinkCapacity.GetBitRate();
    for (const auto& satellite : satelliteBeams)
    {
        uint64_t requested = 0;
        for (const auto& beam : satellite.second)
        {
            requested += budgets.at(beam);
        }
        if (requested <= satelliteCap)
        {
            continue;
        }
        uint64_t assigned = 0;
        for (const auto& beam : satellite.second)
        {
            budgets[beam] = static_cast<uint64_t>(
                static_cast<long double>(satelliteCap) * budgets.at(beam) / requested);
            assigned += budgets.at(beam);
        }
        uint64_t remainder = satelliteCap - assigned;
        for (const auto& beam : satellite.second)
        {
            if (remainder == 0)
            {
                break;
            }
            ++budgets[beam];
            --remainder;
        }
    }
    return budgets;
}

std::string
LeoSimBeamCapacityManager::GetLimitReason(
    const AccessLink& link,
    const LeoSimBeamCapacityAssociation& association,
    bool uplink,
    uint64_t allocation,
    uint64_t beamBudget) const
{
    const bool active = uplink ? association.uplinkActive : association.downlinkActive;
    if (!active)
    {
        return "idle";
    }
    const uint64_t minimum = uplink ? link.uplinkMinimumBps : link.downlinkMinimumBps;
    const uint64_t package = uplink ? link.uplinkPackageCapBps : link.downlinkPackageCapBps;
    if (allocation < minimum)
    {
        return "minimum-rate-shortfall";
    }
    if (allocation >= package)
    {
        return "package";
    }
    const uint64_t configuredBeam = uplink ? m_uplinkBeamCapacity.GetBitRate()
                                           : m_downlinkBeamCapacity.GetBitRate();
    if (beamBudget < configuredBeam)
    {
        return "satellite-capacity";
    }
    return "beam-capacity";
}

double
LeoSimBeamCapacityManager::SpectralEfficiency(double sinrDb)
{
    const double shannon = std::log2(1.0 + std::pow(10.0, sinrDb / 10.0));
    return std::clamp(shannon, 0.1, 7.4);
}

void
LeoSimBeamCapacityManager::EnableCsvOutput(const std::string& filename)
{
    if (m_csv.is_open())
    {
        m_csv.close();
    }
    m_csv.open(filename, std::ios::out | std::ios::trunc);
    NS_ABORT_MSG_IF(!m_csv.is_open(), "Cannot open beam capacity CSV: " << filename);
    m_csv << "time_s,satellite_id,beam_id,ground_id,associated_nodes,active_uplink_nodes,"
             "active_downlink_nodes,uplink_beam_capacity_bps,downlink_beam_capacity_bps,"
             "uplink_package_cap_bps,downlink_package_cap_bps,uplink_minimum_bps,"
             "downlink_minimum_bps,uplink_queue_bytes,downlink_queue_bytes,uplink_bps,"
             "downlink_bps,sinr_db,scheduler,uplink_limit_reason,downlink_limit_reason\n";
}

const std::vector<LeoSimBeamCapacityAllocation>&
LeoSimBeamCapacityManager::GetAllocations() const
{
    return m_allocations;
}

void
LeoSimBeamCapacityManager::ScheduleNextUpdate()
{
    m_updateEvent = Simulator::Schedule(m_updateInterval,
                                        &LeoSimBeamCapacityManager::UpdateNow,
                                        this);
}

void
LeoSimBeamCapacityManager::RestoreLink(const LinkKey& key)
{
    auto it = m_links.find(key);
    if (it == m_links.end())
    {
        return;
    }
    it->second.groundDevice->SetDataRate(DataRate(it->second.baselineUplinkBps));
    it->second.satelliteDevice->SetDataRate(DataRate(it->second.baselineDownlinkBps));
}

void
LeoSimBeamCapacityManager::WriteCsvRows()
{
    if (!m_csv.is_open())
    {
        return;
    }
    for (const auto& allocation : m_allocations)
    {
        m_csv << Simulator::Now().GetSeconds() << ',' << allocation.satelliteNodeId << ','
              << allocation.beamId << ',' << allocation.groundNodeId << ','
              << allocation.associatedNodes << ',' << allocation.activeUplinkNodes << ','
              << allocation.activeDownlinkNodes << ',' << allocation.uplinkBeamCapacityBps << ','
              << allocation.downlinkBeamCapacityBps << ',' << allocation.uplinkPackageCapBps << ','
              << allocation.downlinkPackageCapBps << ',' << allocation.uplinkMinimumBps << ','
              << allocation.downlinkMinimumBps << ',' << allocation.uplinkQueuedBytes << ','
              << allocation.downlinkQueuedBytes << ',' << allocation.uplinkBps << ','
              << allocation.downlinkBps << ',' << allocation.sinrDb << ','
              << allocation.scheduler << ',' << allocation.uplinkLimitReason << ','
              << allocation.downlinkLimitReason << '\n';
    }
    m_csv.flush();
}

} // namespace ns3
