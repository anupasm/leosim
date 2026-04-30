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

#ifndef LEOSIM_BEAM_LAYOUT_ENGINE_H
#define LEOSIM_BEAM_LAYOUT_ENGINE_H

#include "leosim-multi-beam-model.h"
#include "ns3/vector.h"

#include <cstdint>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Static utility for generating and maintaining satellite spot-beam layouts.
 *
 * This class provides pure static helpers and is not intended for instantiation.
 */
class LeoSimBeamLayoutEngine
{
  public:
    LeoSimBeamLayoutEngine() = delete;

    /**
     * \brief Generate a hexagonal lattice of spot beams around satellite nadir.
     * \param satelliteNodeId ns-3 node ID of the satellite owning the beams.
     * \param satPos Satellite ECEF/Cartesian position vector.
     * \param numRings Number of hex rings around the center beam.
     * \param beamRadiusKm Beam footprint radius in kilometers.
     * \param frequencyReuseColors Number of reuse color groups.
     * \param cellIdBase Base cell ID assigned to the first generated beam.
     * \return Vector of generated spot beams.
     *
     * Ring counts follow standard centered hex numbers:
     * numRings=1 gives 7 beams, numRings=2 gives 19 beams,
     * and numRings=3 gives 61 beams.
     */
    static std::vector<LeoSimSpotBeam> GenerateHexLayout(uint32_t satelliteNodeId,
                                                          const Vector& satPos,
                                                          uint32_t numRings,
                                                          double beamRadiusKm,
                                                          uint32_t frequencyReuseColors,
                                                          uint32_t cellIdBase);

    /**
     * \brief Update beam center coordinates after satellite movement.
     * \param beams Spot beams to update.
     * \param newSatPos Updated satellite position vector.
     *
     * Beam geometry remains satellite-fixed, i.e., beam centers remain locked
     * to the current satellite nadir reference as the satellite moves.
     */
    static void UpdateBeamPositions(std::vector<LeoSimSpotBeam>& beams,
                                    const Vector& newSatPos);

    /**
     * \brief Find the nearest serving beam for a geographic position.
     * \param beams Candidate spot beams.
     * \param lat Query latitude in degrees.
     * \param lon Query longitude in degrees.
     * \return Beam ID of the nearest beam center, or -1 if none is within
     *         1.5 * beamRadiusKm.
     */
    static int32_t FindBeamForPosition(const std::vector<LeoSimSpotBeam>& beams,
                                       double lat,
                                       double lon);

  private:
    /**
     * \brief Convert axial hex coordinates to planar offset.
     * \param q Axial q coordinate.
     * \param r Axial r coordinate.
     * \param beamRadiusKm Beam footprint radius in kilometers.
     * \return 2D planar offset from nadir in kilometers.
     */
    static ns3::Vector2D AxialToOffset(int q, int r, double beamRadiusKm);

    /**
     * \brief Project local planar offsets to geographic coordinates.
     * \param satPos Current satellite position vector.
     * \param offsetKmX East-west local offset in kilometers.
     * \param offsetKmY North-south local offset in kilometers.
     * \param outLat Output latitude in degrees.
     * \param outLon Output longitude in degrees.
     */
    static void ProjectToGround(const Vector& satPos,
                                double offsetKmX,
                                double offsetKmY,
                                double& outLat,
                                double& outLon);
};

} // namespace ns3

#endif // LEOSIM_BEAM_LAYOUT_ENGINE_H
