/*
 * Copyright (c) 2026 LeoSim contributors
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

#include "leosim-beam-load-balancer.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimBeamLoadBalancer");

void
LeoSimBeamLoadBalancer::Update(const std::map<uint32_t, LeoSimBeamRecord>& ueBeamMap,
                               std::map<uint32_t, std::vector<LeoSimSpotBeam>>& allBeams)
{
    NS_LOG_FUNCTION(this << ueBeamMap.size() << allBeams.size());

    m_beamMetrics.clear();

    for (auto& [satId, beams] : allBeams)
    {
        std::vector<LeoSimBeamLoadMetrics> metrics;
        metrics.reserve(beams.size());

        std::map<uint32_t, std::size_t> beamIdToIndex;
        for (std::size_t i = 0; i < beams.size(); ++i)
        {
            LeoSimBeamLoadMetrics m{};
            m.beamId = beams[i].beamId;
            m.satelliteNodeId = satId;
            m.activeUeCount = 0;
            m.utilisation = 0.0;
            m.throughputMbps = beams[i].currentThroughputMbps;
            m.avgSinrDb = std::numeric_limits<double>::quiet_NaN();
            m.demandWeight = 0.0;
            metrics.push_back(m);
            beamIdToIndex[beams[i].beamId] = i;
        }

        std::vector<double> sinrSum(metrics.size(), 0.0);
        for (const auto& [ueId, rec] : ueBeamMap)
        {
            (void)ueId;
            if (rec.satelliteNodeId != satId)
            {
                continue;
            }

            auto idxIt = beamIdToIndex.find(rec.beamId);
            if (idxIt == beamIdToIndex.end())
            {
                continue;
            }

            std::size_t idx = idxIt->second;
            metrics[idx].activeUeCount++;
            sinrSum[idx] += rec.sinr;
        }

        uint32_t totalUe = 0;
        for (std::size_t i = 0; i < metrics.size(); ++i)
        {
            totalUe += metrics[i].activeUeCount;
            if (metrics[i].activeUeCount > 0)
            {
                metrics[i].avgSinrDb = sinrSum[i] / static_cast<double>(metrics[i].activeUeCount);
            }
        }

        double denominator = static_cast<double>(std::max<uint32_t>(1, totalUe));
        if (totalUe == 0 && !metrics.empty())
        {
            const double equalWeight = 1.0 / static_cast<double>(metrics.size());
            for (auto& m : metrics)
            {
                m.demandWeight = equalWeight;
                m.utilisation = 0.0;
            }
        }
        else
        {
            for (auto& m : metrics)
            {
                m.demandWeight = static_cast<double>(m.activeUeCount) / denominator;
                m.utilisation = m.demandWeight;
            }
        }

        m_beamMetrics[satId] = std::move(metrics);
    }
}

int32_t
LeoSimBeamLoadBalancer::FindLeastLoadedBeam(uint32_t ueNodeId, uint32_t satId, double minSinrDb) const
{
    NS_LOG_FUNCTION(this << ueNodeId << satId << minSinrDb);

    auto satIt = m_beamMetrics.find(satId);
    if (satIt == m_beamMetrics.end())
    {
        return -1;
    }

    int32_t bestBeamId = -1;
    uint32_t minLoad = std::numeric_limits<uint32_t>::max();

    for (const auto& m : satIt->second)
    {
        const bool sinrKnown = !std::isnan(m.avgSinrDb);
        if (sinrKnown && m.avgSinrDb < minSinrDb)
        {
            continue;
        }

        if (m.activeUeCount < minLoad)
        {
            minLoad = m.activeUeCount;
            bestBeamId = static_cast<int32_t>(m.beamId);
        }
    }

    return bestBeamId;
}

std::vector<double>
LeoSimBeamLoadBalancer::GetDemandWeights(uint32_t satId) const
{
    NS_LOG_FUNCTION(this << satId);

    std::vector<double> weights;

    auto satIt = m_beamMetrics.find(satId);
    if (satIt == m_beamMetrics.end() || satIt->second.empty())
    {
        return weights;
    }

    uint32_t maxBeamId = 0;
    for (const auto& m : satIt->second)
    {
        maxBeamId = std::max(maxBeamId, m.beamId);
    }

    weights.assign(maxBeamId + 1, 0.0);

    uint32_t totalUe = 0;
    for (const auto& m : satIt->second)
    {
        totalUe += m.activeUeCount;
    }

    if (totalUe == 0)
    {
        const double equalWeight = 1.0 / static_cast<double>(satIt->second.size());
        for (const auto& m : satIt->second)
        {
            weights[m.beamId] = equalWeight;
        }
        return weights;
    }

    const double denom = static_cast<double>(totalUe);
    for (const auto& m : satIt->second)
    {
        weights[m.beamId] = static_cast<double>(m.activeUeCount) / denom;
    }

    return weights;
}

bool
LeoSimBeamLoadBalancer::ShouldTriggerLoadBalanceHo(uint32_t srcBeamId,
                                                    uint32_t tgtBeamId,
                                                    uint32_t satId,
                                                    uint32_t threshold) const
{
    NS_LOG_FUNCTION(this << srcBeamId << tgtBeamId << satId << threshold);

    auto satIt = m_beamMetrics.find(satId);
    if (satIt == m_beamMetrics.end())
    {
        return false;
    }

    bool srcFound = false;
    bool tgtFound = false;
    int srcCount = 0;
    int tgtCount = 0;

    for (const auto& m : satIt->second)
    {
        if (m.beamId == srcBeamId)
        {
            srcCount = static_cast<int>(m.activeUeCount);
            srcFound = true;
        }
        else if (m.beamId == tgtBeamId)
        {
            tgtCount = static_cast<int>(m.activeUeCount);
            tgtFound = true;
        }
    }

    if (!srcFound || !tgtFound)
    {
        return false;
    }

    return (srcCount - tgtCount) > static_cast<int>(threshold);
}

} // namespace ns3
