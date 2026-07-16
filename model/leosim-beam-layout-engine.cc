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

#include "leosim-beam-layout-engine.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimBeamLayoutEngine");

namespace
{

constexpr double EARTH_RADIUS_KM = 6371.0;
constexpr double KM_PER_DEGREE = 111.32;
constexpr double PI = 3.14159265358979323846;
// Slightly tighter than mathematically tangent packing to avoid visible gaps
// from spherical projection and floating-point rounding.
constexpr double HEX_CENTER_SPACING_FACTOR = 0.96;

inline int
PositiveModulo(int value, int mod)
{
    return ((value % mod) + mod) % mod;
}

inline void
IndexToAxial(uint32_t index, int& q, int& r)
{
    if (index == 0)
    {
        q = 0;
        r = 0;
        return;
    }

    // Axial directions (pointy-top convention)
    static const int dirQ[6] = {1, 1, 0, -1, -1, 0};
    static const int dirR[6] = {0, -1, -1, 0, 1, 1};

    uint32_t cursor = 1;
    for (uint32_t ring = 1;; ++ring)
    {
        const uint32_t ringCount = 6 * ring;
        if (index < cursor + ringCount)
        {
            // Start each ring at axial coordinate (-ring, +ring)
            q = -static_cast<int>(ring);
            r = static_cast<int>(ring);
            uint32_t local = index - cursor;
            for (uint32_t d = 0; d < 6; ++d)
            {
                for (uint32_t step = 0; step < ring; ++step)
                {
                    if (local == 0)
                    {
                        return;
                    }
                    q += dirQ[d];
                    r += dirR[d];
                    --local;
                }
            }
            return;
        }
        cursor += ringCount;
    }
}

} // namespace

std::vector<LeoSimSpotBeam>
LeoSimBeamLayoutEngine::GenerateHexLayout(uint32_t satelliteNodeId,
                                          const Vector& satPos,
                                          uint32_t numRings,
                                          double beamRadiusKm,
                                          uint32_t frequencyReuseColors,
                                          uint32_t cellIdBase)
{
    NS_LOG_FUNCTION(satelliteNodeId << satPos << numRings << beamRadiusKm << frequencyReuseColors
                                    << cellIdBase);

    std::vector<LeoSimSpotBeam> beams;
    if (numRings == 0 || beamRadiusKm <= 0.0)
    {
        return beams;
    }

    const uint32_t totalBeams = 1 + 3 * numRings * (numRings + 1);
    beams.reserve(totalBeams);

    const int reuse = static_cast<int>(std::max<uint32_t>(1, frequencyReuseColors));

    auto appendBeam = [&](int q, int r, uint32_t beamIndex) {
        const Vector2D offset = AxialToOffset(q, r, beamRadiusKm);

        double lat = 0.0;
        double lon = 0.0;
        ProjectToGround(satPos, offset.x, offset.y, lat, lon);

        const int qMod = PositiveModulo(q, reuse);
        const int rMod = PositiveModulo(r, reuse);
        const int colorGroup = (qMod + rMod) % reuse;

        LeoSimSpotBeam beam;
        beam.beamId = beamIndex;
        beam.satelliteNodeId = satelliteNodeId;
        beam.cellId = cellIdBase + beamIndex;
        beam.colorGroup = colorGroup;
        beam.centerLat = lat;
        beam.centerLon = lon;
        beam.radiusKm = beamRadiusKm;
        beam.currentLoad = 0.0;
        beam.currentThroughputMbps = 0.0;
        beam.activeInCurrentSlot = true;
        beams.push_back(beam);
    };

    uint32_t beamIndex = 0;
    appendBeam(0, 0, beamIndex++);

    // Axial directions (pointy-top convention)
    static const int dirQ[6] = {1, 1, 0, -1, -1, 0};
    static const int dirR[6] = {0, -1, -1, 0, 1, 1};

    for (uint32_t ring = 1; ring <= numRings; ++ring)
    {
        // Start each ring at axial coordinate (-ring, +ring)
        int q = -static_cast<int>(ring);
        int r = static_cast<int>(ring);

        for (uint32_t d = 0; d < 6; ++d)
        {
            for (uint32_t step = 0; step < ring; ++step)
            {
                appendBeam(q, r, beamIndex++);
                q += dirQ[d];
                r += dirR[d];
            }
        }
    }

    return beams;
}

void
LeoSimBeamLayoutEngine::UpdateBeamPositions(std::vector<LeoSimSpotBeam>& beams,
                                            const Vector& newSatPos)
{
    NS_LOG_FUNCTION(&beams << newSatPos);

    for (uint32_t i = 0; i < beams.size(); ++i)
    {
        int q = 0;
        int r = 0;
        IndexToAxial(i, q, r);

        const Vector2D offset = AxialToOffset(q, r, beams[i].radiusKm);
        ProjectToGround(newSatPos, offset.x, offset.y, beams[i].centerLat, beams[i].centerLon);
    }
}

int32_t
LeoSimBeamLayoutEngine::FindBeamForPosition(const std::vector<LeoSimSpotBeam>& beams,
                                            double lat,
                                            double lon)
{
    NS_LOG_FUNCTION(&beams << lat << lon);

    auto toRad = [](double deg) { return deg * PI / 180.0; };

    const double latRad = toRad(lat);
    const double lonRad = toRad(lon);

    int32_t bestBeamId = -1;
    double bestDistanceKm = std::numeric_limits<double>::max();

    for (const auto& beam : beams)
    {
        const double beamLatRad = toRad(beam.centerLat);
        const double beamLonRad = toRad(beam.centerLon);

        const double dLat = beamLatRad - latRad;
        const double dLon = beamLonRad - lonRad;

        const double sinHalfDLat = std::sin(dLat * 0.5);
        const double sinHalfDLon = std::sin(dLon * 0.5);
        const double a = sinHalfDLat * sinHalfDLat +
                         std::cos(latRad) * std::cos(beamLatRad) * sinHalfDLon * sinHalfDLon;
        const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
        const double distanceKm = EARTH_RADIUS_KM * c;

        if (distanceKm < bestDistanceKm)
        {
            bestDistanceKm = distanceKm;
            bestBeamId = static_cast<int32_t>(beam.beamId);
        }
    }

    if (bestBeamId < 0)
    {
        return -1;
    }

    for (const auto& beam : beams)
    {
        if (static_cast<int32_t>(beam.beamId) == bestBeamId)
        {
            return (bestDistanceKm <= beam.radiusKm) ? bestBeamId : -1;
        }
    }

    return -1;
}

ns3::Vector2D
LeoSimBeamLayoutEngine::AxialToOffset(int q, int r, double beamRadiusKm)
{
    NS_LOG_FUNCTION(q << r << beamRadiusKm);

    const double effectiveRadiusKm = beamRadiusKm * HEX_CENTER_SPACING_FACTOR;
    const double sqrt3 = std::sqrt(3.0);
    const double x = effectiveRadiusKm * (3.0 / 2.0) * static_cast<double>(q);
    const double y = effectiveRadiusKm * ((sqrt3 / 2.0) * static_cast<double>(q) +
                                          sqrt3 * static_cast<double>(r));

    return Vector2D(x, y);
}

void
LeoSimBeamLayoutEngine::ProjectToGround(const Vector& satPos,
                                        double offsetKmX,
                                        double offsetKmY,
                                        double& outLat,
                                        double& outLon)
{
    NS_LOG_FUNCTION(satPos << offsetKmX << offsetKmY << &outLat << &outLon);

    const double radius = std::sqrt(satPos.x * satPos.x + satPos.y * satPos.y + satPos.z * satPos.z);
    if (radius <= 0.0)
    {
        outLat = 0.0;
        outLon = 0.0;
        return;
    }

    const double subLat = std::asin(satPos.z / radius) * 180.0 / PI;
    const double subLon = std::atan2(satPos.y, satPos.x) * 180.0 / PI;

    const double deltaLat = offsetKmY / KM_PER_DEGREE;

    double cosLat = std::cos(subLat * PI / 180.0);
    if (std::abs(cosLat) < 1e-9)
    {
        cosLat = (cosLat >= 0.0) ? 1e-9 : -1e-9;
    }
    const double deltaLon = offsetKmX / (KM_PER_DEGREE * cosLat);

    outLat = subLat + deltaLat;
    outLon = subLon + deltaLon;
}

} // namespace ns3
