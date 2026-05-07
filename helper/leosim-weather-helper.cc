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

#include "leosim-weather-helper.h"

#include "ns3/log.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <vector>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimWeatherHelper");

LeoSimWeatherHelper::LeoSimWeatherHelper()
    : m_frequencyHz(12.0e9),
      m_groundHeightKm(0.0),
      m_rainHeightKm(-1.0),
      m_updateInterval(Seconds(1.0)),
      m_hasUniformWeather(false),
      m_hasMarkovMatrix(false),
      m_rainFadeThresholdDb(10.0),
      m_snrFloorDb(-5.0),
      m_weatherFadeHoThresholdDb(15.0),
      m_verbose(false)
{
    m_uniformWeather.rainRateMmh = 0.0;
    m_uniformWeather.cloudLiquidWater = 0.0;
    m_uniformWeather.temperatureCelsius = 20.0;
    m_uniformWeather.pressureHPa = 1013.0;
    m_uniformWeather.waterVapourDensity = 10.0;
    m_uniformWeather.humidity = 60.0;
    m_uniformWeather.state = LEOSIM_WX_CLEAR;

    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            m_markovP[i][j] = LeoSimWeatherModel::DEFAULT_MARKOV_P[i][j];
        }
    }
}

void
LeoSimWeatherHelper::SetFrequency(double freqHz)
{
    m_frequencyHz = freqHz;
}

void
LeoSimWeatherHelper::SetGroundStationHeight(double hS_km)
{
    m_groundHeightKm = hS_km;
}

void
LeoSimWeatherHelper::SetRainHeight(double hR_km)
{
    m_rainHeightKm = hR_km;
}

void
LeoSimWeatherHelper::SetUpdateInterval(Time t)
{
    m_updateInterval = t;
}

void
LeoSimWeatherHelper::LoadWeatherTrace(const std::string& csvFile)
{
    m_weatherTraceFile = csvFile;
}

void
LeoSimWeatherHelper::SetUniformClearSky()
{
    SetUniformWeather(LEOSIM_WX_CLEAR, 0.0, 0.0, 20.0, 1013.0, 10.0, 60.0);
}

void
LeoSimWeatherHelper::SetUniformWeather(LeoSimWeatherState state,
                                       double rainRateMmh,
                                       double cloudLwc,
                                       double tempC,
                                       double pressHPa,
                                       double vapourDens,
                                       double humidity)
{
    m_uniformWeather.rainRateMmh = rainRateMmh;
    m_uniformWeather.cloudLiquidWater = cloudLwc;
    m_uniformWeather.temperatureCelsius = tempC;
    m_uniformWeather.pressureHPa = pressHPa;
    m_uniformWeather.waterVapourDensity = vapourDens;
    m_uniformWeather.humidity = humidity;
    m_uniformWeather.state = state;
    m_hasUniformWeather = true;
}

void
LeoSimWeatherHelper::SetMarkovMatrix(const std::string& matrixCsvFile)
{
    m_hasMarkovMatrix = ParseMarkovMatrixCsv(matrixCsvFile, m_markovP);
    NS_ABORT_MSG_IF(!m_hasMarkovMatrix,
                    "LeoSimWeatherHelper: invalid Markov matrix CSV: " << matrixCsvFile);
}

void
LeoSimWeatherHelper::SetRainFadeThreshold(double db)
{
    m_rainFadeThresholdDb = db;
}

void
LeoSimWeatherHelper::SetSnrFloor(double db)
{
    m_snrFloorDb = db;
}

void
LeoSimWeatherHelper::SetWeatherFadeHoThreshold(double db)
{
    m_weatherFadeHoThresholdDb = db;
}

Ptr<LeoSimWeatherModel>
LeoSimWeatherHelper::Install(const NodeContainer& groundNodes, Time simTime)
{
    Ptr<LeoSimWeatherModel> weather = CreateObject<LeoSimWeatherModel>();

    weather->SetVerbose(m_verbose);
    weather->SetFrequencyHz(m_frequencyHz);
    weather->SetGroundStationHeight(m_groundHeightKm);
    if (m_rainHeightKm > 0.0)
    {
        weather->SetRainHeightKm(m_rainHeightKm);
    }
    weather->SetUpdateInterval(m_updateInterval);

    if (m_hasMarkovMatrix)
    {
        weather->SetMarkovMatrix(m_markovP);
    }

    if (!m_weatherTraceFile.empty())
    {
        weather->LoadWeatherTraceFromCsv(m_weatherTraceFile);
    }

    if (m_hasUniformWeather)
    {
        weather->SetUniformWeatherParams(m_uniformWeather);
    }

    weather->Start(groundNodes, simTime);

    if (m_verbose)
    {
        NS_LOG_INFO("Installed weather model for " << groundNodes.GetN()
                                                    << " ground nodes"
                                                    << ", rainFadeThreshold="
                                                    << m_rainFadeThresholdDb
                                                    << " dB, snrFloor=" << m_snrFloorDb
                                                    << " dB, weatherHoThreshold="
                                                    << m_weatherFadeHoThresholdDb << " dB");
    }

    return weather;
}

void
LeoSimWeatherHelper::SetVerbose(bool v)
{
    m_verbose = v;
}

bool
LeoSimWeatherHelper::ParseMarkovMatrixCsv(const std::string& matrixCsvFile, double outP[4][4]) const
{
    std::ifstream in(matrixCsvFile);
    if (!in.is_open())
    {
        return false;
    }

    std::string line;
    uint32_t row = 0;

    while (std::getline(in, line))
    {
        if (line.empty() || line[0] == '#')
        {
            continue;
        }
        if (line.find("FROM") != std::string::npos)
        {
            continue;
        }

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> cols;
        while (std::getline(ss, token, ','))
        {
            cols.push_back(token);
        }

        if (cols.size() < 5)
        {
            continue;
        }

        try
        {
            outP[row][0] = std::stod(cols[1]);
            outP[row][1] = std::stod(cols[2]);
            outP[row][2] = std::stod(cols[3]);
            outP[row][3] = std::stod(cols[4]);
        }
        catch (...)
        {
            continue;
        }

        ++row;
        if (row == 4)
        {
            break;
        }
    }

    if (row != 4)
    {
        return false;
    }

    for (uint32_t i = 0; i < 4; ++i)
    {
        double s = outP[i][0] + outP[i][1] + outP[i][2] + outP[i][3];
        if (s <= 0.0)
        {
            return false;
        }
        outP[i][0] /= s;
        outP[i][1] /= s;
        outP[i][2] /= s;
        outP[i][3] /= s;
    }

    return true;
}

} // namespace ns3
