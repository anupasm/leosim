/*
 * Copyright (c) 2026 LeoSim contributors
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef LEOSIM_ISL_LOAD_MODEL_H
#define LEOSIM_ISL_LOAD_MODEL_H

#include "ns3/object.h"
#include "ns3/net-device-container.h"
#include "ns3/event-id.h"
#include "leosim-channel-model.h"

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

class Node;

/** State of one directed inter-satellite link. */
struct LeoSimIslLoadRecord
{
    uint32_t sourceSatelliteId{0};
    uint32_t destinationSatelliteId{0};
    bool active{false};
    uint64_t physicalCapacityBps{0};
    double requestedSyntheticLoad{0.0};
    uint64_t requestedBackgroundBps{0};
    uint64_t admittedBackgroundBps{0};
    uint64_t droppedBackgroundBps{0};
    uint64_t remainingCapacityBps{0};
    uint64_t effectiveCapacityBps{0};
    double effectiveUtilization{0.0};
};

/** Aggregate ISL forwarding state of one satellite. */
struct LeoSimSatelliteIslCapacity
{
    uint32_t satelliteId{0};
    uint64_t maximumCapacityBps{0};
    uint64_t backgroundBps{0};
    uint64_t droppedBackgroundBps{0};
    uint64_t remainingCapacityBps{0};
    double utilization{0.0};
};

/**
 * Deterministically assigns synthetic background utilization to directed ISLs.
 *
 * Requested background load above a satellite's aggregate outgoing capacity is
 * proportionally rejected and recorded as dropped load. The model does not
 * change device rates or affect routing.
 */
class LeoSimIslLoadModel : public Object
{
  public:
    static TypeId GetTypeId();

    void SetSeed(uint64_t seed);
    void SetLoadRange(double minimum, double maximum);
    void SetDistribution(const std::string& distribution);

    void RegisterSatellite(uint32_t satelliteId, uint64_t maximumCapacityBps);
    void RegisterDirectedIsl(uint32_t sourceSatelliteId,
                             uint32_t destinationSatelliteId,
                             uint64_t physicalCapacityBps);

    /** Track active channel state and update capacities when the ISL topology changes. */
    void AttachChannelModel(Ptr<LeoSimChannelModel> channelModel,
                            const NetDeviceContainer& devices);

    /** Generate load for an epoch. Repeating an epoch produces identical state. */
    void Generate(uint64_t epoch = 0);

    bool GetIslLoad(uint32_t sourceSatelliteId,
                    uint32_t destinationSatelliteId,
                    LeoSimIslLoadRecord& record) const;
    bool GetSatelliteCapacity(uint32_t satelliteId,
                              LeoSimSatelliteIslCapacity& capacity) const;
    std::vector<LeoSimIslLoadRecord> GetAllIslLoads() const;
    std::vector<LeoSimSatelliteIslCapacity> GetAllSatelliteCapacities() const;

    /** Return the exact routing cost derived from effective residual capacity. */
    double GetRoutingCost(uint32_t sourceSatelliteId,
                          uint32_t destinationSatelliteId) const;

    /** Write one auditable row per directed ISL. */
    void WriteCsv(const std::string& filename,
                  double timeSeconds = 0.0,
                  bool append = false) const;

    /** Enable initial and topology-change load snapshots in one CSV. */
    void EnableCsvOutput(const std::string& filename);

    /** Apply effective directional capacities to point-to-point ISL devices. */
    void ApplyToIslDevices(NetDeviceContainer& devices) const;

  private:
    using LinkKey = std::pair<uint32_t, uint32_t>;

    double GenerateUniform(uint32_t sourceSatelliteId,
                           uint32_t destinationSatelliteId,
                           uint64_t epoch) const;
    void RecalculateCapacity();
    void OnLinkStateChange(Ptr<Node> first, Ptr<Node> second, LeoSimLinkState state);
    void ApplyPendingTopologyUpdate();

    uint64_t m_seed{12345};
    double m_minimumLoad{0.1};
    double m_maximumLoad{0.9};
    std::string m_distribution{"uniform"};
    std::map<LinkKey, LeoSimIslLoadRecord> m_links;
    std::map<uint32_t, LeoSimSatelliteIslCapacity> m_satellites;
    Ptr<LeoSimChannelModel> m_channelModel;
    NetDeviceContainer m_devices;
    EventId m_topologyUpdateEvent;
    std::string m_csvFilename;
};

} // namespace ns3

#endif // LEOSIM_ISL_LOAD_MODEL_H
