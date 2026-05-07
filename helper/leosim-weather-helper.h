/*
 * Copyright (c) 2026 Anupa De Silva
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

#ifndef LEOSIM_WEATHER_HELPER_H
#define LEOSIM_WEATHER_HELPER_H

#include "../model/leosim-weather-model.h"

#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/ptr.h"

#include <string>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Helper for configuring and installing LeoSimWeatherModel
 */
class LeoSimWeatherHelper
{
  public:
    LeoSimWeatherHelper();

    void SetFrequency(double freqHz);
    void SetGroundStationHeight(double hS_km);
    void SetRainHeight(double hR_km);
    void SetUpdateInterval(Time t);

    // Input source (choose one)
    void LoadWeatherTrace(const std::string& csvFile);
    void SetUniformClearSky();
    void SetUniformWeather(LeoSimWeatherState state,
                           double rainRateMmh = 0,
                           double cloudLwc = 0,
                           double tempC = 20,
                           double pressHPa = 1013,
                           double vapourDens = 10,
                           double humidity = 60);
    void SetMarkovMatrix(const std::string& matrixCsvFile);

    // Channel/beam integration thresholds (stored for caller wiring)
    void SetRainFadeThreshold(double db);
    void SetSnrFloor(double db);
    void SetWeatherFadeHoThreshold(double db);

    Ptr<LeoSimWeatherModel> Install(const NodeContainer& groundNodes, Time simTime);

    void SetVerbose(bool v);

  private:
    bool ParseMarkovMatrixCsv(const std::string& matrixCsvFile, double outP[4][4]) const;

    double m_frequencyHz;
    double m_groundHeightKm;
    double m_rainHeightKm;
    Time m_updateInterval;

    std::string m_weatherTraceFile;
    bool m_hasUniformWeather;
    LeoSimWeatherParams m_uniformWeather;

    bool m_hasMarkovMatrix;
    double m_markovP[4][4];

    double m_rainFadeThresholdDb;
    double m_snrFloorDb;
    double m_weatherFadeHoThresholdDb;

    bool m_verbose;
};

} // namespace ns3

#endif /* LEOSIM_WEATHER_HELPER_H */
