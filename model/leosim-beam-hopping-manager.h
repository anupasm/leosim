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

#ifndef LEOSIM_BEAM_HOPPING_MANAGER_H
#define LEOSIM_BEAM_HOPPING_MANAGER_H

#include "leosim-multi-beam-model.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Beam hopping scheduler for multi-beam satellites.
 */
class LeoSimBeamHoppingManager : public Object
{
  public:
    LeoSimBeamHoppingManager(uint32_t numBeams,
                             uint32_t cycleSlots,
                             uint32_t slotMs,
                             uint32_t reuseColors);

    /**
     * \brief Build one hopping cycle schedule from demand weights.
     * \param demandWeights Per-beam demand weights.
     *
     * Builds a time-slot illumination schedule for one hopping cycle,
     * weighted by demand, while ensuring no two same-colour beams are
     * assigned to the same slot.
     */
    void GenerateSchedule(const std::vector<double>& demandWeights);

    /**
     * \brief Get active beam IDs for a slot in the hopping cycle.
     * \param slotIndex Slot index in the cycle.
     * \return Beam IDs active in that slot.
     */
    std::vector<uint32_t> GetActiveBeams(uint32_t slotIndex) const;

    /**
     * \brief Check whether a beam is active at simulation time.
     * \param beamId Beam identifier.
     * \param simTime Simulation time.
     * \return True if the beam is active at the given time.
     */
    bool IsBeamActive(uint32_t beamId, Time simTime) const;

    /**
     * \brief Start hopping schedule updates.
     * \param beams Beam list whose active flags are updated.
     * \param simTime Simulation start time.
     *
     * Schedules the first slot tick using Simulator::Schedule.
     */
    void Start(std::vector<LeoSimSpotBeam>& beams, Time simTime);

  private:
    /**
     * \brief Validate adding a beam to a slot with colour conflict checks.
     * \param slotBeams Beam IDs already present in the slot.
     * \param beamId Candidate beam ID.
     * \return True if beam can be added without colour conflict.
     */
    bool CanAddBeam(const std::vector<uint32_t>& slotBeams, uint32_t beamId) const;

    /**
     * \brief Advance to next slot and reschedule following tick.
     * \param beams Beam list whose active flags are updated.
     */
    void AdvanceSlot(std::vector<LeoSimSpotBeam>& beams);

  private:
    uint32_t m_numBeams;
    uint32_t m_cycleSlots;
    uint32_t m_reuseColors;
    Time m_slotDuration;
    std::vector<std::vector<uint32_t>> m_schedule;
    std::vector<int> m_beamColorGroup;
};

} // namespace ns3

#endif // LEOSIM_BEAM_HOPPING_MANAGER_H
