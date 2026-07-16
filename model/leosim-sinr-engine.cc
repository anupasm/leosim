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

#include "leosim-sinr-engine.h"

#include "ns3/log.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimSinrEngine");
NS_OBJECT_ENSURE_REGISTERED(LeoSimSinrEngine);

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double EARTH_RADIUS_KM = 6371.0;
constexpr double DEFAULT_SAT_ALTITUDE_KM = 550.0;
constexpr double DEFAULT_CARRIER_FREQ_HZ = 20.0e9;

inline double
WrapLongitudeDelta(double deltaDeg)
{
	deltaDeg = std::fmod(deltaDeg + 180.0, 360.0);
	if (deltaDeg < 0.0)
	{
		deltaDeg += 360.0;
	}
	return deltaDeg - 180.0;
}

inline double
DegToRad(double deg)
{
	return deg * PI / 180.0;
}

inline double
GreatCircleDistanceKm(double lat1, double lon1, double lat2, double lon2)
{
	const double lat1r = DegToRad(lat1);
	const double lon1r = DegToRad(lon1);
	const double lat2r = DegToRad(lat2);
	const double lon2r = DegToRad(lon2);

	const double dLat = lat2r - lat1r;
	const double dLon = lon2r - lon1r;

	const double sinHalfLat = std::sin(dLat * 0.5);
	const double sinHalfLon = std::sin(dLon * 0.5);
	const double a = sinHalfLat * sinHalfLat +
					 std::cos(lat1r) * std::cos(lat2r) * sinHalfLon * sinHalfLon;
	const double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
	return EARTH_RADIUS_KM * c;
}

inline double
dBmToMilliwatt(double pDbm)
{
	return std::pow(10.0, pDbm / 10.0);
}

inline double
MilliwattToDbm(double pMw)
{
	if (pMw <= 0.0)
	{
		return -std::numeric_limits<double>::infinity();
	}
	return 10.0 * std::log10(pMw);
}

inline std::pair<double, double>
EstimateSatelliteSubpoint(const std::vector<LeoSimSpotBeam>& beams)
{
	if (beams.empty())
	{
		return {0.0, 0.0};
	}

	for (const auto& beam : beams)
	{
		if (beam.beamId == 0)
		{
			return {beam.centerLat, beam.centerLon};
		}
	}

	double latAcc = 0.0;
	double lonAcc = 0.0;
	for (const auto& beam : beams)
	{
		latAcc += beam.centerLat;
		lonAcc += beam.centerLon;
	}
	return {latAcc / beams.size(), lonAcc / beams.size()};
}

} // namespace

TypeId
LeoSimSinrEngine::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimSinrEngine")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimSinrEngine>();
    return tid;
}

void
LeoSimSinrEngine::SetBeamConfig(const LeoSimBeamConfig& cfg)
{
    NS_LOG_FUNCTION(this << &cfg);
    m_cfg = cfg;
}

LeoSimSinrResult
LeoSimSinrEngine::ComputeSinr(uint32_t ueNodeId,
                              uint32_t servingSatId,
                              uint32_t servingBeamId,
                              const std::map<uint32_t, std::vector<LeoSimSpotBeam>>& allBeams,
                              double ueLat,
                              double ueLon) const
{
    NS_LOG_FUNCTION(this << ueNodeId << servingSatId << servingBeamId << ueLat << ueLon);

    LeoSimSinrResult result{};
    result.servingBeamId = servingBeamId;
    result.servingSatId = servingSatId;
    result.sinr_dB = -std::numeric_limits<double>::infinity();
    result.signal_dBm = -std::numeric_limits<double>::infinity();
    result.intraBeamInterference_dBm = -std::numeric_limits<double>::infinity();
    result.interSatInterference_dBm = -std::numeric_limits<double>::infinity();
    result.thermalNoise_dBm = -std::numeric_limits<double>::infinity();

    const auto satIt = allBeams.find(servingSatId);
    if (satIt == allBeams.end() || servingBeamId >= satIt->second.size())
    {
        NS_LOG_WARN("Serving beam not found for requested satellite/beam index");
        return result;
    }

    const auto& servingSatBeams = satIt->second;
    const LeoSimSpotBeam& servingBeam = servingSatBeams[servingBeamId];
    const auto servingSubpoint = EstimateSatelliteSubpoint(servingSatBeams);
    const double servingSatLat = servingSubpoint.first;
    const double servingSatLon = servingSubpoint.second;

    const double groundDistanceKm = GreatCircleDistanceKm(ueLat, ueLon, servingSatLat, servingSatLon);
    const double slantDistanceKm =
        std::sqrt(groundDistanceKm * groundDistanceKm + DEFAULT_SAT_ALTITUDE_KM * DEFAULT_SAT_ALTITUDE_KM);

    const double servingOffBoresight = ComputeOffBoresightAngle(servingBeam.centerLat,
                                                                servingBeam.centerLon,
                                                                ueLat,
                                                                ueLon,
                                                                servingSatLat,
                                                                servingSatLon);
    const double servingGainDbi = servingBeam.GetAntennaGain(servingOffBoresight);

    const double signalDbm = ComputeReceivedPower_dBm(
        slantDistanceKm, servingGainDbi, m_cfg.beamEIRP_dBW, DEFAULT_CARRIER_FREQ_HZ);

    const double noiseDbm = ThermalNoisePower_dBm(m_cfg.beamBandwidthMHz * 1e6);
    const double noiseMw = dBmToMilliwatt(noiseDbm);

    double intraInterferenceMw = 0.0;
    for (const auto& beam : servingSatBeams)
    {
        if (beam.beamId == servingBeam.beamId)
        {
            continue;
        }
        if (!beam.activeInCurrentSlot)
        {
            continue;
        }
        if (!IsCochannelBeam(servingBeam, beam))
        {
            continue;
        }

        const double offBoresight = ComputeOffBoresightAngle(beam.centerLat,
                                                              beam.centerLon,
                                                              ueLat,
                                                              ueLon,
                                                              servingSatLat,
                                                              servingSatLon);
        const double gainDbi = beam.GetAntennaGain(offBoresight);
        const double interfDbm =
            ComputeReceivedPower_dBm(slantDistanceKm, gainDbi, m_cfg.beamEIRP_dBW, DEFAULT_CARRIER_FREQ_HZ);
        intraInterferenceMw += dBmToMilliwatt(interfDbm);
    }

    double interSatInterferenceMw = 0.0;
    for (const auto& satEntry : allBeams)
    {
        if (satEntry.first == servingSatId)
        {
            continue;
        }

        const auto& satBeams = satEntry.second;
        const auto satSubpoint = EstimateSatelliteSubpoint(satBeams);
        const double satLat = satSubpoint.first;
        const double satLon = satSubpoint.second;

        const double satGroundDistanceKm = GreatCircleDistanceKm(ueLat, ueLon, satLat, satLon);
        // A satellite below the geometric radio horizon is occulted by Earth and
        // cannot contribute co-channel interference at this ground terminal.
        const double horizonCentralAngleRad =
            std::acos(EARTH_RADIUS_KM / (EARTH_RADIUS_KM + DEFAULT_SAT_ALTITUDE_KM));
        const double horizonGroundDistanceKm = EARTH_RADIUS_KM * horizonCentralAngleRad;
        if (satGroundDistanceKm > horizonGroundDistanceKm)
        {
            continue;
        }
        const double satSlantDistanceKm =
            std::sqrt(satGroundDistanceKm * satGroundDistanceKm +
                      DEFAULT_SAT_ALTITUDE_KM * DEFAULT_SAT_ALTITUDE_KM);

        for (const auto& beam : satBeams)
        {
            if (!beam.activeInCurrentSlot)
            {
                continue;
            }
            if (!IsCochannelBeam(servingBeam, beam))
            {
                continue;
            }

            const double offBoresight =
                ComputeOffBoresightAngle(beam.centerLat, beam.centerLon, ueLat, ueLon, satLat, satLon);
            const double gainDbi = beam.GetAntennaGain(offBoresight);
            const double interfDbm = ComputeReceivedPower_dBm(
                satSlantDistanceKm, gainDbi, m_cfg.beamEIRP_dBW, DEFAULT_CARRIER_FREQ_HZ);
            interSatInterferenceMw += dBmToMilliwatt(interfDbm);
        }
    }

    const double signalMw = dBmToMilliwatt(signalDbm);
    const double denominatorMw = intraInterferenceMw + interSatInterferenceMw + noiseMw;
    const double sinrLinear = (denominatorMw > 0.0) ? signalMw / denominatorMw : 0.0;

    result.signal_dBm = signalDbm;
    result.intraBeamInterference_dBm = MilliwattToDbm(intraInterferenceMw);
    result.interSatInterference_dBm = MilliwattToDbm(interSatInterferenceMw);
    result.thermalNoise_dBm = noiseDbm;
    result.sinr_dB = (sinrLinear > 0.0) ? 10.0 * std::log10(sinrLinear)
                                        : -std::numeric_limits<double>::infinity();
    return result;
}

double
LeoSimSinrEngine::ComputeReceivedPower_dBm(double distanceKm,
                                           double gainDbi,
                                           double eirpDbW,
                                           double freqHz) const
{
    NS_LOG_FUNCTION(this << distanceKm << gainDbi << eirpDbW << freqHz);

    const double dKm = std::max(distanceKm, 1e-6);
    const double fMHz = std::max(freqHz / 1e6, 1e-6);

    const double fsplDb = 32.44 + 20.0 * std::log10(dKm) + 20.0 * std::log10(fMHz);
    return eirpDbW + 30.0 + gainDbi - fsplDb;
}

double
LeoSimSinrEngine::ComputeOffBoresightAngle(double beamCenterLat,
                                           double beamCenterLon,
                                           double ueLat,
                                           double ueLon,
                                           double satLat,
                                           double satLon) const
{
    NS_LOG_FUNCTION(this << beamCenterLat << beamCenterLon << ueLat << ueLon << satLat << satLon);

    const double satLatRad = DegToRad(satLat);
    const double kmPerDegLat = 111.32;
    const double kmPerDegLon = kmPerDegLat * std::max(1e-6, std::cos(satLatRad));

    const double beamX = WrapLongitudeDelta(beamCenterLon - satLon) * kmPerDegLon;
    const double beamY = (beamCenterLat - satLat) * kmPerDegLat;
    const double ueX = WrapLongitudeDelta(ueLon - satLon) * kmPerDegLon;
    const double ueY = (ueLat - satLat) * kmPerDegLat;

    const double beamNorm = std::sqrt(beamX * beamX + beamY * beamY);
    const double ueNorm = std::sqrt(ueX * ueX + ueY * ueY);

    if (beamNorm < 1e-9 || ueNorm < 1e-9)
    {
        return 0.0;
    }

    double cosAngle = (beamX * ueX + beamY * ueY) / (beamNorm * ueNorm);
    cosAngle = std::clamp(cosAngle, -1.0, 1.0);
    return std::acos(cosAngle) * 180.0 / PI;
}

bool
LeoSimSinrEngine::IsCochannelBeam(const LeoSimSpotBeam& a, const LeoSimSpotBeam& b) const
{
    NS_LOG_FUNCTION(this << &a << &b);
    return a.colorGroup == b.colorGroup;
}

double
LeoSimSinrEngine::ThermalNoisePower_dBm(double bandwidthHz) const
{
    NS_LOG_FUNCTION(this << bandwidthHz);

    const double bHz = std::max(bandwidthHz, 1.0);
    const double k = 1.38e-23;
    const double t = 290.0;
    const double noiseW = k * t * bHz;
    return 10.0 * std::log10(noiseW) + 30.0;
}

} // namespace ns3
