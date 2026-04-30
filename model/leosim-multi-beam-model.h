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

#ifndef LEOSIM_MULTI_BEAM_MODEL_H
#define LEOSIM_MULTI_BEAM_MODEL_H

#include "ns3/object.h"
#include "ns3/vector.h"

#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Global configuration for satellite multi-beam operation.
 */
struct LeoSimBeamConfig
{
    uint32_t numBeamsPerSatellite = 19;   //!< Number of spot beams generated per satellite.
    double beamRadiusKm = 100.0;          //!< Nominal beam footprint radius (km).
    uint32_t frequencyReuseColors = 3;    //!< Number of frequency reuse colors.
    double beamBandwidthMHz = 250.0;      //!< Per-beam bandwidth (MHz).
    double beamEIRP_dBW = 60.0;           //!< Beam EIRP in dBW.
    double beamGT_dBK = 15.0;             //!< Beam G/T in dB/K.
    bool beamHoppingEnabled = false;      //!< Enables beam hopping scheduler when true.
    uint32_t beamHopCycleSlotsN = 256;    //!< Number of slots in one beam hopping cycle.
    uint32_t beamHopSlotMs = 10;          //!< Beam hopping slot duration (ms).
};

/**
 * \ingroup leosim
 * \brief Representation of a single spot beam on a satellite.
 */
struct LeoSimSpotBeam
{
    uint32_t beamId = 0;                  //!< Unique beam ID within simulation context.
    uint32_t satelliteNodeId = 0;         //!< ns-3 node ID of the owning satellite.
    uint32_t cellId = 0;                  //!< Logical cell identifier for the beam.
    int colorGroup = 0;                   //!< Frequency reuse colour index in [0, reuseColors).
    double centerLat = 0.0;               //!< Beam center latitude (degrees).
    double centerLon = 0.0;               //!< Beam center longitude (degrees).
    double radiusKm = 0.0;                //!< Beam footprint radius (km).
    double currentLoad = 0.0;             //!< Current active UE count carried by the beam.
    double currentThroughputMbps = 0.0;   //!< Aggregate beam throughput (Mbps).
    bool activeInCurrentSlot = false;     //!< True when beam is active in the current hop slot.

    /**
     * \brief Compute antenna gain for an off-boresight angle.
     * \param offBoresightDeg Off-boresight angle in degrees.
     * \return Antenna gain in dBi for the specified angle.
     *
     * Uses an envelope pattern aligned with common satellite spot-beam
     * modeling practice in ITU-R S.580 and 3GPP TR 38.821 style studies.
     */
    double GetAntennaGain(double offBoresightDeg) const;
};

/**
 * \ingroup leosim
 * \brief Placeholder object wrapper for multi-beam state/model ownership.
 */
class LeoSimMultiBeamModel : public Object
{
  public:
    static TypeId GetTypeId();

    /**
     * \brief Set beam list for one satellite.
     * \param satId Satellite node ID.
     * \param beams Beam vector for this satellite.
     */
    void SetBeamsForSatellite(uint32_t satId, const std::vector<LeoSimSpotBeam>& beams)
    {
        m_beamsBySatellite[satId] = beams;
    }

    /**
     * \brief Get beam list for one satellite.
     * \param satId Satellite node ID.
     * \return Const reference to beam vector, or empty vector if missing.
     */
    const std::vector<LeoSimSpotBeam>& GetBeamsForSatellite(uint32_t satId) const
    {
        auto it = m_beamsBySatellite.find(satId);
        if (it != m_beamsBySatellite.end())
        {
            return it->second;
        }
        static const std::vector<LeoSimSpotBeam> kEmpty;
        return kEmpty;
    }

    /**
     * \brief Get all satellite beam lists.
     * \return Beam map keyed by satellite node ID.
     */
    const std::map<uint32_t, std::vector<LeoSimSpotBeam>>& GetAllBeams() const
    {
        return m_beamsBySatellite;
    }

    /**
     * \brief Get mutable access to all satellite beam lists.
     * \return Beam map keyed by satellite node ID.
     */
    std::map<uint32_t, std::vector<LeoSimSpotBeam>>& GetAllBeamsMutable()
    {
        return m_beamsBySatellite;
    }

  private:
    std::map<uint32_t, std::vector<LeoSimSpotBeam>> m_beamsBySatellite;
};

} // namespace ns3

#endif // LEOSIM_MULTI_BEAM_MODEL_H
