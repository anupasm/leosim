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

#include "leosim-beam-hopping-manager.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <numeric>
#include <queue>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimBeamHoppingManager");

LeoSimBeamHoppingManager::LeoSimBeamHoppingManager(uint32_t numBeams,
                                                   uint32_t cycleSlots,
                                                   uint32_t slotMs,
                                                   uint32_t reuseColors)
    : m_numBeams(numBeams),
      m_cycleSlots(std::max<uint32_t>(1, cycleSlots)),
      m_reuseColors(std::max<uint32_t>(1, reuseColors)),
      m_slotDuration(MilliSeconds(std::max<uint32_t>(1, slotMs)))
{
    m_schedule.resize(m_cycleSlots);
    m_beamColorGroup.resize(m_numBeams);
    for (uint32_t b = 0; b < m_numBeams; ++b)
    {
        m_beamColorGroup[b] = static_cast<int>(b % m_reuseColors);
    }
}

void
LeoSimBeamHoppingManager::GenerateSchedule(const std::vector<double>& demandWeights)
{
    NS_LOG_FUNCTION(this << demandWeights.size());

    std::vector<double> weights(m_numBeams, 1.0);
    for (uint32_t b = 0; b < m_numBeams && b < demandWeights.size(); ++b)
    {
        weights[b] = std::max(0.0, demandWeights[b]);
    }

    double weightSum = std::accumulate(weights.begin(), weights.end(), 0.0);
    if (weightSum <= 0.0)
    {
        weightSum = static_cast<double>(m_numBeams);
        std::fill(weights.begin(), weights.end(), 1.0 / weightSum);
    }
    else
    {
        for (double& w : weights)
        {
            w /= weightSum;
        }
    }

    std::vector<int> slotsPerBeam(m_numBeams, 1);
    for (uint32_t b = 0; b < m_numBeams; ++b)
    {
        slotsPerBeam[b] = std::max(1, static_cast<int>(std::lround(weights[b] * m_cycleSlots)));
    }

    m_schedule.assign(m_cycleSlots, {});

    using BeamNeed = std::pair<int, uint32_t>;
    auto cmp = [](const BeamNeed& a, const BeamNeed& b) {
        if (a.first == b.first)
        {
            return a.second > b.second;
        }
        return a.first < b.first;
    };

    std::priority_queue<BeamNeed, std::vector<BeamNeed>, decltype(cmp)> pq(cmp);
    for (uint32_t b = 0; b < m_numBeams; ++b)
    {
        pq.push({slotsPerBeam[b], b});
    }

    for (uint32_t slot = 0; slot < m_cycleSlots; ++slot)
    {
        bool madeProgress = true;
        while (madeProgress && !pq.empty())
        {
            madeProgress = false;
            std::vector<BeamNeed> skipped;

            while (!pq.empty())
            {
                BeamNeed top = pq.top();
                pq.pop();

                int remaining = top.first;
                uint32_t beamId = top.second;

                if (remaining <= 0)
                {
                    continue;
                }

                if (CanAddBeam(m_schedule[slot], beamId))
                {
                    m_schedule[slot].push_back(beamId);
                    --remaining;
                    if (remaining > 0)
                    {
                        skipped.push_back({remaining, beamId});
                    }
                    madeProgress = true;
                    break;
                }

                skipped.push_back(top);
            }

            for (const auto& item : skipped)
            {
                pq.push(item);
            }
        }
    }

    std::vector<uint32_t> occurrences(m_numBeams, 0);
    for (const auto& slotBeams : m_schedule)
    {
        for (uint32_t beamId : slotBeams)
        {
            if (beamId < m_numBeams)
            {
                occurrences[beamId]++;
            }
        }
    }

    for (uint32_t b = 0; b < m_numBeams; ++b)
    {
        NS_ASSERT_MSG(occurrences[b] > 0, "Beam " << b << " must appear in at least one slot");
    }
}

std::vector<uint32_t>
LeoSimBeamHoppingManager::GetActiveBeams(uint32_t slotIndex) const
{
    if (m_schedule.empty())
    {
        return {};
    }

    const uint32_t slot = slotIndex % m_cycleSlots;
    return m_schedule[slot];
}

bool
LeoSimBeamHoppingManager::CanAddBeam(const std::vector<uint32_t>& slotBeams, uint32_t beamId) const
{
    if (beamId >= m_beamColorGroup.size())
    {
        return false;
    }

    for (uint32_t existingBeam : slotBeams)
    {
        if (existingBeam < m_beamColorGroup.size() &&
            m_beamColorGroup[existingBeam] == m_beamColorGroup[beamId])
        {
            return false;
        }
    }

    return true;
}

bool
LeoSimBeamHoppingManager::IsBeamActive(uint32_t beamId, Time simTime) const
{
    if (m_schedule.empty() || m_cycleSlots == 0 || m_slotDuration.IsZero())
    {
        return false;
    }

    uint32_t slot = (simTime.GetMilliSeconds() / m_slotDuration.GetMilliSeconds()) % m_cycleSlots;
    const auto& activeBeams = m_schedule[slot];
    return std::find(activeBeams.begin(), activeBeams.end(), beamId) != activeBeams.end();
}

void
LeoSimBeamHoppingManager::Start(std::vector<LeoSimSpotBeam>& beams, Time simTime)
{
    NS_LOG_FUNCTION(this << beams.size() << simTime.GetSeconds());

    for (auto& beam : beams)
    {
        beam.activeInCurrentSlot = IsBeamActive(beam.beamId, simTime);
    }

    Simulator::Schedule(m_slotDuration,
                        &LeoSimBeamHoppingManager::AdvanceSlot,
                        this,
                        std::ref(beams));
}

void
LeoSimBeamHoppingManager::AdvanceSlot(std::vector<LeoSimSpotBeam>& beams)
{
    NS_LOG_FUNCTION(this << beams.size());

    Time now = Simulator::Now();
    for (auto& beam : beams)
    {
        beam.activeInCurrentSlot = IsBeamActive(beam.beamId, now);
    }

    Simulator::Schedule(m_slotDuration,
                        &LeoSimBeamHoppingManager::AdvanceSlot,
                        this,
                        std::ref(beams));
}

} // namespace ns3
