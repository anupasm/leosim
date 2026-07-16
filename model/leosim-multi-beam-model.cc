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

#include "leosim-multi-beam-model.h"

#include "leosim-beam-layout-engine.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimMultiBeamModel");

NS_OBJECT_ENSURE_REGISTERED(LeoSimMultiBeamModel);

namespace
{
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kRefAltitudeKm = 550.0;

std::array<double, 3>
GeodeticToEcefUnit(double latDeg, double lonDeg)
{
    const double latRad = latDeg * M_PI / 180.0;
    const double lonRad = lonDeg * M_PI / 180.0;

    const double clat = std::cos(latRad);
    return {clat * std::cos(lonRad), clat * std::sin(lonRad), std::sin(latRad)};
}

void
UpdateBeamRadiiForGeometry(std::vector<LeoSimSpotBeam>& beams,
                           const Vector& satPos,
                           const std::map<std::pair<uint32_t, uint32_t>, double>& nominalRadii)
{
    const double satNorm =
        std::sqrt(satPos.x * satPos.x + satPos.y * satPos.y + satPos.z * satPos.z);
    if (satNorm <= 1.0)
    {
        return;
    }

    const double altitudeKm = std::max(0.0, (satNorm - kEarthRadiusM) / 1000.0);
    double altitudeScale = altitudeKm / kRefAltitudeKm;
    altitudeScale = std::clamp(altitudeScale, 0.6, 2.0);

    const double nadirX = -satPos.x / satNorm;
    const double nadirY = -satPos.y / satNorm;
    const double nadirZ = -satPos.z / satNorm;

    for (auto& beam : beams)
    {
        auto nominalIt = nominalRadii.find({beam.satelliteNodeId, beam.beamId});
        const double nominalRadiusKm =
            (nominalIt != nominalRadii.end()) ? nominalIt->second : beam.radiusKm;

        auto u = GeodeticToEcefUnit(beam.centerLat, beam.centerLon);
        const double groundX = kEarthRadiusM * u[0];
        const double groundY = kEarthRadiusM * u[1];
        const double groundZ = kEarthRadiusM * u[2];

        double dx = groundX - satPos.x;
        double dy = groundY - satPos.y;
        double dz = groundZ - satPos.z;
        const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist <= 1.0)
        {
            beam.radiusKm = nominalRadiusKm * altitudeScale;
            continue;
        }
        dx /= dist;
        dy /= dist;
        dz /= dist;

        const double dot = std::clamp(dx * nadirX + dy * nadirY + dz * nadirZ, -1.0, 1.0);
        const double offNadirRad = std::acos(dot);

        // Projection stretches footprint away from nadir: r_eff = r0 / cos(theta).
        const double projectionScale = 1.0 / std::max(0.35, std::cos(offNadirRad));

        beam.radiusKm = nominalRadiusKm * altitudeScale * projectionScale;
    }
}
} // namespace

// ============================================================================
// LeoSimSpotBeam
// ============================================================================

double
LeoSimSpotBeam::GetAntennaGain(double offBoresightDeg) const
{
    // Derive the half-power beamwidth from the beam footprint radius.
    // Assume a typical LEO altitude of 550 km for the geometry.
    constexpr double kAltitudeKm = 550.0;
    double halfBeamwidthDeg = std::atan(radiusKm / kAltitudeKm) * 180.0 / M_PI;
    halfBeamwidthDeg = std::max(halfBeamwidthDeg, 0.05); // guard against degenerate beams

    // Peak gain from the standard pencil-beam approximation:
    //   G_max ≈ 10 * log10(30000 / (θ_az * θ_el))   (both angles in degrees)
    // For a symmetric circular beam θ_az = θ_el = 2 * halfBeamwidthDeg.
    double fullBeamwidthDeg = 2.0 * halfBeamwidthDeg;
    double peakGainDbi =
        10.0 * std::log10(30000.0 / (fullBeamwidthDeg * fullBeamwidthDeg));

    // Normalised off-boresight ratio.
    double ratio = offBoresightDeg / halfBeamwidthDeg;

    if (ratio <= 1.0)
    {
        // Main lobe: Gaussian envelope – -3 dB at half-beamwidth.
        return peakGainDbi - 12.0 * ratio * ratio;
    }
    else if (ratio <= 2.58)
    {
        // Near side-lobe region: plateau at -12 dB below peak.
        return peakGainDbi - 12.0;
    }
    else
    {
        // Far side-lobes: ITU-R S.580-style log-envelope roll-off.
        return peakGainDbi - 40.0 * std::log10(ratio);
    }
}

// ============================================================================
// LeoSimMultiBeamModel – TypeId
// ============================================================================

TypeId
LeoSimMultiBeamModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LeoSimMultiBeamModel")
            .SetParent<Object>()
            .SetGroupName("LeoSim")
            .AddConstructor<LeoSimMultiBeamModel>();
    return tid;
}

// ============================================================================
// LeoSimMultiBeamModel – Phased array configuration
// ============================================================================

void
LeoSimMultiBeamModel::SetPhasedArrayConfig(const LeoSimPhasedArrayConfig& config)
{
    NS_LOG_FUNCTION(this);
    m_phasedArrayConfig = config;
}

const LeoSimPhasedArrayConfig&
LeoSimMultiBeamModel::GetPhasedArrayConfig() const
{
    return m_phasedArrayConfig;
}

// ============================================================================
// LeoSimMultiBeamModel – Dynamic beam steering
// ============================================================================

void
LeoSimMultiBeamModel::SteerBeam(const Vector& antennaPosition, const Vector& targetPosition)
{
    NS_LOG_FUNCTION(this << antennaPosition << targetPosition);

    // Vector from antenna to target, normalised.
    double dx = targetPosition.x - antennaPosition.x;
    double dy = targetPosition.y - antennaPosition.y;
    double dz = targetPosition.z - antennaPosition.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < 1.0)
    {
        NS_LOG_WARN("SteerBeam: antenna and target are co-located – steering unchanged.");
        return;
    }
    dx /= dist;
    dy /= dist;
    dz /= dist;

    // Nadir direction: unit vector from satellite toward Earth centre.
    double satR = std::sqrt(antennaPosition.x * antennaPosition.x +
                            antennaPosition.y * antennaPosition.y +
                            antennaPosition.z * antennaPosition.z);
    if (satR < 1.0)
    {
        NS_LOG_WARN("SteerBeam: satellite is at origin – steering unchanged.");
        return;
    }
    double nadirX = -antennaPosition.x / satR;
    double nadirY = -antennaPosition.y / satR;
    double nadirZ = -antennaPosition.z / satR;

    // Off-nadir angle: angle between nadir and the direction to target.
    double dotNadir =
        std::max(-1.0, std::min(1.0, dx * nadirX + dy * nadirY + dz * nadirZ));
    double offNadirDeg = std::acos(dotNadir) * 180.0 / M_PI;

    // Clamp to the array's maximum steering cone.
    offNadirDeg = std::min(offNadirDeg, m_phasedArrayConfig.maxSteeringAngleDeg);

    // Build a local North–East frame at the satellite to derive azimuth.
    // Local East  = cross(globalZ, nadir),  then normalise.
    // Local North = cross(nadir, localEast).
    double globalZx = 0.0, globalZy = 0.0, globalZz = 1.0;
    double eastX = globalZy * nadirZ - globalZz * nadirY; // = -nadirY
    double eastY = globalZz * nadirX - globalZx * nadirZ; //=  nadirX
    double eastZ = globalZx * nadirY - globalZy * nadirX; //=  0

    double eastMag = std::sqrt(eastX * eastX + eastY * eastY + eastZ * eastZ);

    double azimuthDeg = 0.0;
    if (eastMag > 1e-6)
    {
        eastX /= eastMag;
        eastY /= eastMag;
        eastZ /= eastMag;

        // Local North = cross(nadir, localEast).
        double northX = nadirY * eastZ - nadirZ * eastY;
        double northY = nadirZ * eastX - nadirX * eastZ;
        double northZ = nadirX * eastY - nadirY * eastX;

        double eastComp = dx * eastX + dy * eastY + dz * eastZ;
        double northComp = dx * northX + dy * northY + dz * northZ;

        azimuthDeg = std::atan2(eastComp, northComp) * 180.0 / M_PI;
        if (azimuthDeg < 0.0)
        {
            azimuthDeg += 360.0;
        }
    }

    m_steeringAzimuthDeg = azimuthDeg;
    m_steeringElevationDeg = offNadirDeg;

    NS_LOG_DEBUG("SteerBeam: azimuth=" << m_steeringAzimuthDeg
                                       << " deg, off-nadir=" << m_steeringElevationDeg
                                       << " deg");
}

double
LeoSimMultiBeamModel::CalculateGain(double azimuthDeg, double elevationDeg) const
{
    NS_LOG_FUNCTION(this << azimuthDeg << elevationDeg);

    const double pi = M_PI;
    double d = m_phasedArrayConfig.elementSpacingLambda; // d/λ
    uint32_t Naz = m_phasedArrayConfig.numElementsAz;
    uint32_t Nel = m_phasedArrayConfig.numElementsEl;

    // Angular differences between query direction and current steering.
    double deltaAzRad = (azimuthDeg - m_steeringAzimuthDeg) * pi / 180.0;
    double deltaElRad = (elevationDeg - m_steeringElevationDeg) * pi / 180.0;

    // Direction-cosine differences (u-space) for each axis.
    double uAz = std::sin(deltaAzRad);
    double uEl = std::sin(deltaElRad);

    // Separable URA array factor: AF = sin(N*π*d*u) / (N*sin(π*d*u))
    // Returns 1.0 (maximum) when u ≈ 0 (on boresight).
    auto arrayFactor = [pi](uint32_t N, double dOverLambda, double u) -> double {
        double phi = pi * dOverLambda * u;
        if (std::abs(phi) < 1e-9)
        {
            return 1.0;
        }
        double sinPhi = std::sin(phi);
        if (std::abs(sinPhi) < 1e-12)
        {
            return 1.0;
        }
        return std::sin(static_cast<double>(N) * phi) /
               (static_cast<double>(N) * sinPhi);
    };

    double afAz = arrayFactor(Naz, d, uAz);
    double afEl = arrayFactor(Nel, d, uEl);

    // Total gain (dBi):
    //   G = 10*log10(Naz*Nel) + G_element + 20*log10|AF_az| + 20*log10|AF_el|
    double totalElements = static_cast<double>(Naz * Nel);
    double gainDbi = 10.0 * std::log10(totalElements) +
                     m_phasedArrayConfig.elementGainDbi +
                     20.0 * std::log10(std::abs(afAz) + 1e-12) +
                     20.0 * std::log10(std::abs(afEl) + 1e-12);

    NS_LOG_DEBUG("CalculateGain: az=" << azimuthDeg << " el=" << elevationDeg
                                      << " -> " << gainDbi << " dBi");
    return gainDbi;
}

double
LeoSimMultiBeamModel::GetSteeringAzimuthDeg() const
{
    return m_steeringAzimuthDeg;
}

double
LeoSimMultiBeamModel::GetSteeringElevationDeg() const
{
    return m_steeringElevationDeg;
}

void
LeoSimMultiBeamModel::UpdateGeometry(NodeContainer satellites, Time now)
{
    NS_LOG_FUNCTION(this << satellites.GetN() << now);

    for (uint32_t i = 0; i < satellites.GetN(); ++i)
    {
        Ptr<Node> sat = satellites.Get(i);
        if (!sat)
        {
            continue;
        }

        auto it = m_beamsBySatellite.find(sat->GetId());
        if (it == m_beamsBySatellite.end() || it->second.empty())
        {
            continue;
        }

        Ptr<MobilityModel> mobility = sat->GetObject<MobilityModel>();
        if (!mobility)
        {
            NS_LOG_DEBUG("Skipping beam geometry update for satellite " << sat->GetId()
                                                                        << ": no mobility model");
            continue;
        }

        for (auto& beam : it->second)
        {
            auto nominalIt = m_nominalBeamRadiusKm.find({sat->GetId(), beam.beamId});
            if (nominalIt == m_nominalBeamRadiusKm.end())
            {
                m_nominalBeamRadiusKm[{sat->GetId(), beam.beamId}] = beam.radiusKm;
            }
            else
            {
                beam.radiusKm = nominalIt->second;
            }
        }

        const Vector satPos = mobility->GetPosition();
        LeoSimBeamLayoutEngine::UpdateBeamPositions(it->second, satPos);
        UpdateBeamRadiiForGeometry(it->second, satPos, m_nominalBeamRadiusKm);
    }
}

} // namespace ns3
