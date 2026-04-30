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

#ifndef LEOSIM_SINR_ENGINE_H
#define LEOSIM_SINR_ENGINE_H

#include "leosim-multi-beam-model.h"
#include "ns3/object.h"

#include <cstdint>
#include <map>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Detailed SINR decomposition for a UE-beam association.
 */
struct LeoSimSinrResult
{
    double sinr_dB;                        //!< End-to-end SINR in dB.
    double signal_dBm;                     //!< Serving beam received power in dBm.
    double intraBeamInterference_dBm;      //!< Intra-beam interference component in dBm.
    double interSatInterference_dBm;       //!< Inter-satellite/co-channel interference in dBm.
    double thermalNoise_dBm;               //!< Thermal noise power in dBm.
    uint32_t servingBeamId;                //!< Selected serving beam ID.
    uint32_t servingSatId;                 //!< Selected serving satellite node ID.
};

/**
 * \ingroup leosim
 * \brief SINR computation engine for LeoSim multi-beam links.
 */
class LeoSimSinrEngine : public Object
{
  public:
    /**
     * \brief Get TypeId for this class.
     * \return Object TypeId.
     */
    static TypeId GetTypeId();

    /**
     * \brief Set global multi-beam link-budget configuration.
     * \param cfg Beam configuration parameters.
     */
    void SetBeamConfig(const LeoSimBeamConfig& cfg);

    /**
     * \brief Compute SINR for a UE against a given serving beam.
     * \param ueNodeId UE node ID.
     * \param servingSatId Serving satellite node ID.
     * \param servingBeamId Serving beam ID.
     * \param allBeams Beam map indexed by satellite node ID.
     * \param ueLat UE latitude in degrees.
     * \param ueLon UE longitude in degrees.
     * \return Full SINR result structure.
     */
    LeoSimSinrResult ComputeSinr(uint32_t ueNodeId,
                                 uint32_t servingSatId,
                                 uint32_t servingBeamId,
                                 const std::map<uint32_t, std::vector<LeoSimSpotBeam>>& allBeams,
                                 double ueLat,
                                 double ueLon) const;

  private:
    /**
     * \brief Compute received power using link-budget inputs.
     * \param distanceKm Path distance in kilometers.
     * \param gainDbi Antenna gain in dBi.
     * \param eirpDbW EIRP in dBW.
     * \param freqHz Carrier frequency in Hz.
     * \return Received power in dBm.
     */
    double ComputeReceivedPower_dBm(double distanceKm, double gainDbi, double eirpDbW, double freqHz) const;

    /**
     * \brief Compute off-boresight angle of UE relative to beam center.
     * \param beamCenterLat Beam center latitude in degrees.
     * \param beamCenterLon Beam center longitude in degrees.
     * \param ueLat UE latitude in degrees.
     * \param ueLon UE longitude in degrees.
     * \param satLat Satellite sub-point latitude in degrees.
     * \param satLon Satellite sub-point longitude in degrees.
     * \return Off-boresight angle in degrees.
     */
    double ComputeOffBoresightAngle(double beamCenterLat,
                                    double beamCenterLon,
                                    double ueLat,
                                    double ueLon,
                                    double satLat,
                                    double satLon) const;

    /**
     * \brief Check whether two beams are co-channel interferers.
     * \param a First beam.
     * \param b Second beam.
     * \return True if both beams share the same frequency color.
     */
    bool IsCochannelBeam(const LeoSimSpotBeam& a, const LeoSimSpotBeam& b) const;

    /**
     * \brief Compute thermal noise power for the specified bandwidth.
     * \param bandwidthHz Channel bandwidth in Hz.
     * \return Thermal noise power in dBm.
     */
    double ThermalNoisePower_dBm(double bandwidthHz) const;

  private:
    LeoSimBeamConfig m_cfg;
};

} // namespace ns3

#endif // LEOSIM_SINR_ENGINE_H
