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

#ifndef LEOSIM_BEAM_LOAD_BALANCER_H
#define LEOSIM_BEAM_LOAD_BALANCER_H

#include "leosim-beam-manager.h"
#include "leosim-multi-beam-model.h"
#include "ns3/object.h"

#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Aggregated load and quality metrics for one beam.
 */
struct LeoSimBeamLoadMetrics
{
    uint32_t beamId;            //!< Beam identifier.
    uint32_t satelliteNodeId;   //!< Satellite node ID owning this beam.
    uint32_t activeUeCount;     //!< Number of active UEs currently on this beam.
    double utilisation;         //!< Beam utilization ratio in [0.0, 1.0].
    double throughputMbps;      //!< Aggregate beam throughput in Mbps.
    double avgSinrDb;           //!< Average SINR of UEs on the beam (dB).
    double demandWeight;        //!< Relative demand weight for scheduler/load balancing.
};

/**
 * \ingroup leosim
 * \brief Beam-level load balancing helper for LeoSim.
 */
class LeoSimBeamLoadBalancer : public Object
{
  public:
    /**
     * \brief Update per-beam metrics from UE-beam mappings.
     * \param ueBeamMap Mapping from UE ID to current beam record.
     * \param allBeams Beam map indexed by satellite node ID.
     *
     * Collects per-beam UE counts and updates internal beam load metrics.
     */
    void Update(const std::map<uint32_t, LeoSimBeamRecord>& ueBeamMap,
                std::map<uint32_t, std::vector<LeoSimSpotBeam>>& allBeams);

    /**
     * \brief Find least loaded candidate beam on a satellite meeting SINR floor.
     * \param ueNodeId UE node ID.
     * \param satId Satellite node ID.
     * \param minSinrDb Minimum acceptable SINR in dB.
     * \return Candidate beam ID, or -1 if no candidate satisfies constraints.
     */
    int32_t FindLeastLoadedBeam(uint32_t ueNodeId, uint32_t satId, double minSinrDb) const;

    /**
     * \brief Get demand weights for all beams on one satellite.
     * \param satId Satellite node ID.
     * \return Weight vector indexed by beamId, proportional to UE counts.
     */
    std::vector<double> GetDemandWeights(uint32_t satId) const;

    /**
     * \brief Check whether load imbalance justifies handover.
     * \param srcBeamId Source beam ID.
     * \param tgtBeamId Target beam ID.
     * \param satId Satellite node ID.
     * \param threshold Minimum UE-count delta to trigger balancing handover.
     * \return True if src-target UE count difference exceeds threshold.
     */
    bool ShouldTriggerLoadBalanceHo(uint32_t srcBeamId,
                                    uint32_t tgtBeamId,
                                    uint32_t satId,
                                    uint32_t threshold) const;

  private:
    std::map<uint32_t, std::vector<LeoSimBeamLoadMetrics>> m_beamMetrics;
};

} // namespace ns3

#endif // LEOSIM_BEAM_LOAD_BALANCER_H
