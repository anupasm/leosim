/*
 * Copyright (c) 2026 Anupa De Silva
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef LEOSIM_BEAM_CAPACITY_MANAGER_H
#define LEOSIM_BEAM_CAPACITY_MANAGER_H

#include "ns3/data-rate.h"
#include "ns3/event-id.h"
#include "ns3/net-device.h"
#include "ns3/node.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <vector>

namespace ns3
{

class LeoSimBeamManager;
class Packet;
class PointToPointNetDevice;

/** A ground-node association used by the beam capacity allocator. */
struct LeoSimBeamCapacityAssociation
{
    uint32_t groundNodeId;
    uint32_t satelliteNodeId;
    uint32_t beamId;
    double sinrDb{20.0};
    bool uplinkActive{true};
    bool downlinkActive{true};
    uint32_t uplinkQueuedBytes{0};
    uint32_t downlinkQueuedBytes{0};
};

struct LeoSimUeServiceProfile
{
    DataRate uplinkPeak{DataRate("100Mbps")};
    DataRate downlinkPeak{DataRate("100Mbps")};
    DataRate uplinkMinimum{DataRate(0)};
    DataRate downlinkMinimum{DataRate(0)};
    double weight{1.0};
};

/** Result of one per-beam capacity allocation. */
struct LeoSimBeamCapacityAllocation
{
    uint32_t groundNodeId;
    uint32_t satelliteNodeId;
    uint32_t beamId;
    uint32_t associatedNodes;
    uint32_t activeUplinkNodes;
    uint32_t activeDownlinkNodes;
    uint64_t beamCapacityBps;
    uint64_t uplinkBeamCapacityBps;
    uint64_t downlinkBeamCapacityBps;
    uint64_t uplinkPackageCapBps;
    uint64_t downlinkPackageCapBps;
    uint64_t uplinkMinimumBps;
    uint64_t downlinkMinimumBps;
    uint32_t uplinkQueuedBytes;
    uint32_t downlinkQueuedBytes;
    uint64_t uplinkBps;
    uint64_t downlinkBps;
    double sinrDb;
    std::string scheduler;
    std::string uplinkLimitReason;
    std::string downlinkLimitReason;
};

enum LeoSimBeamScheduler
{
    LEOSIM_BEAM_SCHEDULER_EQUAL = 0,
    LEOSIM_BEAM_SCHEDULER_PROPORTIONAL_FAIR = 1,
    LEOSIM_BEAM_SCHEDULER_ALPHA_FAIR = 2
};

/**
 * Enforces a finite, per-direction capacity budget for every satellite beam.
 * Associated ground nodes receive an equal share, capped by the configured
 * data rate of their access link.
 */
class LeoSimBeamCapacityManager : public Object
{
  public:
    static TypeId GetTypeId();

    LeoSimBeamCapacityManager();
    ~LeoSimBeamCapacityManager() override;

    void SetBeamManager(Ptr<LeoSimBeamManager> beamManager);
    /** Compatibility setter: applies the same capacity to uplink and downlink. */
    void SetBeamCapacity(DataRate capacity);
    void SetUplinkBeamCapacity(DataRate capacity);
    void SetDownlinkBeamCapacity(DataRate capacity);
    void SetDefaultUplinkPackageRate(DataRate rate);
    void SetDefaultDownlinkPackageRate(DataRate rate);
    void SetSatelliteUplinkCapacity(DataRate capacity);
    void SetSatelliteDownlinkCapacity(DataRate capacity);
    void SetAlphaFairness(double alpha);
    void SetQueueDelayWeight(double weight);
    void SetScheduler(LeoSimBeamScheduler scheduler);
    void SetDemandAware(bool enabled);
    void SetActiveUserTimeout(Time timeout);
    void SetUpdateInterval(Time interval);
    void SetUeServiceProfile(uint32_t groundNodeId, const LeoSimUeServiceProfile& profile);
    bool LoadUeServiceProfiles(const std::string& filename);

    bool RegisterAccessLink(Ptr<Node> groundNode,
                            Ptr<Node> satelliteNode,
                            Ptr<NetDevice> groundDevice,
                            Ptr<NetDevice> satelliteDevice);

    void Start();
    void Stop();
    void UpdateNow();

    /** Public deterministic entry point, primarily useful for experiments and tests. */
    void ApplyAssociations(const std::vector<LeoSimBeamCapacityAssociation>& associations);

    /** Explicit demand hooks for applications that know when data becomes backlogged. */
    void MarkUplinkDemand(uint32_t groundNodeId, uint32_t satelliteNodeId);
    void MarkDownlinkDemand(uint32_t groundNodeId, uint32_t satelliteNodeId);

    void EnableCsvOutput(const std::string& filename);
    const std::vector<LeoSimBeamCapacityAllocation>& GetAllocations() const;

  private:
    struct LinkKey
    {
        uint32_t groundNodeId;
        uint32_t satelliteNodeId;

        bool operator<(const LinkKey& other) const;
    };

    struct BeamKey
    {
        uint32_t satelliteNodeId;
        uint32_t beamId;

        bool operator<(const BeamKey& other) const;
    };

    struct AccessLink
    {
        Ptr<PointToPointNetDevice> groundDevice;
        Ptr<PointToPointNetDevice> satelliteDevice;
        uint64_t baselineUplinkBps;
        uint64_t baselineDownlinkBps;
        uint64_t uplinkPackageCapBps;
        uint64_t downlinkPackageCapBps;
        uint64_t uplinkMinimumBps;
        uint64_t downlinkMinimumBps;
        double serviceWeight;
        Time lastUplinkDemand;
        Time lastDownlinkDemand;
        double averageUplinkBps;
        double averageDownlinkBps;
    };

    void NotifyDeviceTx(std::string context, Ptr<const Packet> packet);
    void ScheduleNextUpdate();
    void RestoreLink(const LinkKey& key);
    void WriteCsvRows();
    std::map<LinkKey, uint64_t> AllocateDirection(
        const std::vector<LeoSimBeamCapacityAssociation>& associations,
        bool uplink,
        uint64_t capacity);
    static double SpectralEfficiency(double sinrDb);
    std::map<BeamKey, uint64_t> AllocateBeamBudgets(
        const std::map<BeamKey, std::vector<LeoSimBeamCapacityAssociation>>& groups,
        bool uplink) const;
    std::string GetLimitReason(const AccessLink& link,
                               const LeoSimBeamCapacityAssociation& association,
                               bool uplink,
                               uint64_t allocation,
                               uint64_t beamBudget) const;

    Ptr<LeoSimBeamManager> m_beamManager;
    DataRate m_uplinkBeamCapacity;
    DataRate m_downlinkBeamCapacity;
    DataRate m_defaultUplinkPackageRate;
    DataRate m_defaultDownlinkPackageRate;
    DataRate m_satelliteUplinkCapacity;
    DataRate m_satelliteDownlinkCapacity;
    Time m_updateInterval;
    Time m_activeUserTimeout;
    LeoSimBeamScheduler m_scheduler;
    double m_alphaFairness;
    double m_queueDelayWeight;
    bool m_demandAware;
    EventId m_updateEvent;
    bool m_running;
    std::map<LinkKey, AccessLink> m_links;
    std::map<uint32_t, LeoSimUeServiceProfile> m_profiles;
    std::map<std::string, std::pair<LinkKey, bool>> m_traceContexts;
    std::vector<LeoSimBeamCapacityAllocation> m_allocations;
    std::map<LinkKey, bool> m_allocatedLinks;
    std::ofstream m_csv;
};

} // namespace ns3

#endif /* LEOSIM_BEAM_CAPACITY_MANAGER_H */
