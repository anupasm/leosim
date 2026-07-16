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
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/vector.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <utility>
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
    bool singleBestLinkMode = false;      //!< Enable single best link per satellite (state-of-the-art selection).
};

/**
 * \ingroup leosim
 * \brief Configuration parameters for a phased array antenna.
 *
 * Models a Uniform Rectangular Array (URA) whose beam can be steered
 * electronically toward any ground node within the maximum steering cone.
 */
struct LeoSimPhasedArrayConfig
{
    uint32_t numElementsAz = 8;           //!< Number of elements along the azimuth axis.
    uint32_t numElementsEl = 8;           //!< Number of elements along the elevation axis.
    double elementSpacingLambda = 0.5;    //!< Element spacing as fraction of wavelength (d/λ).
    double operatingFrequencyGHz = 20.0;  //!< Carrier frequency in GHz.
    double maxSteeringAngleDeg = 60.0;    //!< Maximum off-nadir steering angle (degrees).
    double elementGainDbi = 5.0;          //!< Single-element gain (dBi).
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
 * \brief Manages satellite multi-beam state and phased array steering.
 *
 * Owns the per-satellite spot-beam list and exposes a Uniform Rectangular
 * Array (URA) phased array antenna model.  Call SteerBeam() each time the
 * satellite or a ground node moves, then query CalculateGain() for link
 * budget calculations.
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
        for (const auto& beam : beams)
        {
            m_nominalBeamRadiusKm[{satId, beam.beamId}] = beam.radiusKm;
        }
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

    // -----------------------------------------------------------------------
    // Phased array configuration
    // -----------------------------------------------------------------------

    /**
     * \brief Replace the phased array configuration.
     * \param config New configuration struct.
     */
    void SetPhasedArrayConfig(const LeoSimPhasedArrayConfig& config);

    /**
     * \brief Retrieve the current phased array configuration.
     */
    const LeoSimPhasedArrayConfig& GetPhasedArrayConfig() const;

    // -----------------------------------------------------------------------
    // Dynamic beam steering
    // -----------------------------------------------------------------------

    /**
     * \brief Steer the phased array beam toward a target position.
     * \param antennaPosition ECEF position of the satellite antenna (metres).
     * \param targetPosition  ECEF position of the target ground node (metres).
     *
     * Computes the off-nadir elevation and azimuth angles required to point
     * the URA boresight at the target and stores them as the current steering
     * state.  The off-nadir angle is clamped to
     * LeoSimPhasedArrayConfig::maxSteeringAngleDeg.
     */
    void SteerBeam(const Vector& antennaPosition, const Vector& targetPosition);

    /**
     * \brief Calculate array gain in a given direction.
     * \param azimuthDeg   Azimuth of the query direction (degrees, 0 = North).
     * \param elevationDeg Off-nadir angle of the query direction (degrees).
     * \return Antenna gain (dBi) toward the requested direction.
     *
     * Uses a separable sinc-envelope URA pattern referenced to the current
     * steering state set by SteerBeam().  Returns peak gain when the query
     * direction matches the steering direction.
     */
    double CalculateGain(double azimuthDeg, double elevationDeg) const;

    /**
     * \brief Return the current beam steering azimuth (degrees, 0 = North).
     */
    double GetSteeringAzimuthDeg() const;

    /**
     * \brief Return the current beam steering off-nadir elevation (degrees).
     */
    double GetSteeringElevationDeg() const;

    /**
     * \brief Synchronize beam footprints with current satellite positions.
     * \param satellites Satellite nodes whose beam geometry should be updated.
     * \param now Current simulation time, used for logging and trace context.
     *
     * Updates each configured satellite's spot-beam centers through
     * LeoSimBeamLayoutEngine::UpdateBeamPositions(), then refreshes the
     * footprint radii for altitude and off-nadir projection effects.
     */
    void UpdateGeometry(NodeContainer satellites, Time now);

  private:
    std::map<uint32_t, std::vector<LeoSimSpotBeam>> m_beamsBySatellite;
    std::map<std::pair<uint32_t, uint32_t>, double> m_nominalBeamRadiusKm;

    // Phased array antenna parameters
    LeoSimPhasedArrayConfig m_phasedArrayConfig; //!< URA configuration.
    double m_steeringAzimuthDeg{0.0};            //!< Current steering azimuth (deg).
    double m_steeringElevationDeg{0.0};          //!< Current off-nadir steering angle (deg).
};

} // namespace ns3

#endif // LEOSIM_MULTI_BEAM_MODEL_H
