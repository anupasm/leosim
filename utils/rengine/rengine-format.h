/*
 * LeoSim external routing engine binary format.
 *
 * This header intentionally has no ns-3 dependencies. Keep it usable by both
 * the simulator-side exporter and the standalone process workers.
 */

#ifndef LEOSIM_RENGINE_FORMAT_H
#define LEOSIM_RENGINE_FORMAT_H

#include <cstdint>

namespace leosim_rengine
{

static constexpr uint32_t GRAPH_MAGIC = 0x4C524731;   // LRG1
static constexpr uint32_t REQUEST_MAGIC = 0x4C525131; // LRQ1
static constexpr uint32_t RESULT_MAGIC = 0x4C525331;  // LRS1
static constexpr uint16_t FORMAT_VERSION = 1;
static constexpr uint32_t INVALID_NODE = 0xffffffffu;

enum class RoutingMode : uint32_t
{
    PAIR = 1,
    DESTINATION_TREE = 2,
};

enum class RoutingMetric : uint32_t
{
    WEIGHT = 1,
    HOP_COUNT = 2,
};

#pragma pack(push, 1)

struct GraphHeader
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint64_t snapshotId;
    double simTimeSeconds;
    uint32_t nodeCount;
    uint32_t edgeCount;
    uint64_t rowOffsetCount;
    uint64_t colIndexCount;
    uint64_t weightCount;
};

struct RequestHeader
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint64_t snapshotId;
    uint32_t requestCount;
    uint32_t mode;
    uint32_t metric;
    uint32_t reserved2;
};

struct RouteRequest
{
    uint32_t src;
    uint32_t dst;
};

struct ResultHeader
{
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    uint64_t snapshotId;
    uint32_t resultCount;
    uint32_t mode;
    uint32_t metric;
    uint32_t workerId;
};

struct RouteResult
{
    uint32_t src;
    uint32_t dst;
    uint32_t nextHop;
    float cost;
    uint16_t hopCount;
    uint8_t valid;
    uint8_t reserved;
};

#pragma pack(pop)

} // namespace leosim_rengine

#endif // LEOSIM_RENGINE_FORMAT_H
