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

#ifndef LEOSIM_WEATHER_MODEL_H
#define LEOSIM_WEATHER_MODEL_H

#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"
#include "ns3/object.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Four atmospheric states per ground location
 *
 * Used to drive the Markov chain transition at each simulation second.
 */
enum LeoSimWeatherState
{
    LEOSIM_WX_CLEAR,      //!< Clear sky, gaseous attenuation only
    LEOSIM_WX_CLOUDY,     //!< Overcast, cloud + gaseous attenuation
    LEOSIM_WX_LIGHT_RAIN, //!< Light rain (R <= 5 mm/h)
    LEOSIM_WX_HEAVY_RAIN  //!< Heavy/convective rain (R > 5 mm/h)
};

/**
 * \ingroup leosim
 * \brief Weather parameters for a single ground location at one time step
 */
struct LeoSimWeatherParams
{
    double rainRateMmh;        //!< Rain rate R (mm/h), 0 if clear/cloudy
    double cloudLiquidWater;   //!< Liquid water content L (kg/m^2) for P.840
    double temperatureCelsius; //!< Surface temperature (C)
    double pressureHPa;        //!< Surface pressure (hPa)
    double waterVapourDensity; //!< Surface water vapour density (g/m^3)
    double humidity;           //!< Relative humidity (%)
    LeoSimWeatherState state;  //!< Current weather state
};

/**
 * \ingroup leosim
 * \brief Computed per-link attenuation breakdown (all values in dB)
 */
struct LeoSimAttenuationResult
{
    double rainAttenuation_dB;         //!< ITU-R P.618 / P.838 rain attenuation
    double cloudAttenuation_dB;        //!< ITU-R P.840 cloud attenuation
    double gaseousAttenuation_dB;      //!< ITU-R P.676 (O2 + H2O) attenuation
    double scintillationAmplitude_dB;  //!< ITU-R P.618 1-sigma scintillation
    double scintillationSample_dB;     //!< Instantaneous scintillation draw
    double totalAttenuation_dB;        //!< Sum of all attenuation components
    double elevationAngle_deg;         //!< Elevation at time of computation
    LeoSimWeatherState groundState;    //!< Ground weather state during compute
    Time computedAt;                   //!< Simulation timestamp of computation
};

/**
 * \ingroup leosim
 * \brief Weather-driven atmospheric attenuation model for LEO ground-to-satellite links.
 *
 * LeoSimWeatherModel implements a practical ITU-R P.618-14 attenuation workflow by
 * combining rain (P.838/P.618), cloud (P.840), gaseous (P.676), and scintillation
 * terms into a per-link attenuation result. Ground weather evolves over time via a
 * discrete-state Markov chain (CLEAR, CLOUDY, LIGHT_RAIN, HEAVY_RAIN) using a
 * configurable transition matrix, or optionally from time-series CSV traces.
 *
 * The model provides runtime configuration, per-node weather assignment, periodic
 * state updates, attenuation queries used by channel/routing components, and a
 * callback for weather state transitions.
 */
class LeoSimWeatherModel : public Object
{
    public:
        /**
         * \brief Default Markov transition matrix P[from][to] for weather states.
         *
         * Rows correspond to source states (CLEAR, CLOUDY, LIGHT_RAIN, HEAVY_RAIN)
         * and columns correspond to destination states in the same order.
         */
        static const double DEFAULT_MARKOV_P[4][4];

        static TypeId GetTypeId();

        // Configuration
        void SetFrequencyHz(double freqHz);         // link frequency for ITU-R formulas
        void SetGroundStationHeight(double hS_km);  // station altitude above sea level
        void SetRainHeightKm(double hR_km);         // override ITU-R P.839 rain height
        void SetMarkovMatrix(const double P[4][4]); // override default transition matrix
        void SetUpdateInterval(Time t);             // how often to advance Markov state

        // Input modes
        void LoadWeatherTraceFromCsv(const std::string& file);
        // CSV: time_s, node_id, rain_rate_mmh, cloud_lwc, temp_c, pressure_hpa,
        //       water_vapour_density, humidity
        void SetUniformWeatherParams(const LeoSimWeatherParams& params);
        void SetNodeWeatherParams(uint32_t groundNodeId,
                                                            const LeoSimWeatherParams& params);

        // Lifecycle
        void Start(const NodeContainer& groundNodes, Time simTime);
        void Stop();

        // Core computation, called by LeoSimChannelModel each update
        LeoSimAttenuationResult ComputeAttenuation(uint32_t groundNodeId,
                                                                                             uint32_t satNodeId,
                                                                                             double elevationAngle_deg) const;

        // State queries
        LeoSimWeatherState GetWeatherState(uint32_t groundNodeId) const;
        LeoSimWeatherParams GetWeatherParams(uint32_t groundNodeId) const;
        double GetTotalAttenuation(uint32_t groundNodeId, uint32_t satNodeId) const;

        // Callback: fired whenever any ground node's weather state changes
        void SetWeatherChangeCallback(
                Callback<void, uint32_t, LeoSimWeatherState, LeoSimWeatherState> cb);

        void SetVerbose(bool v);

    private:
        void AdvanceMarkovStates();
        void DrawWeatherParams(uint32_t nodeId, LeoSimWeatherState newState);
        void AdvanceFromTrace(uint32_t nodeId);

        // ITU-R computation engines
        double ComputeRainAttenuation(const LeoSimWeatherParams& wx,
                                                                    double elev_deg) const;
        double ComputeCloudAttenuation(const LeoSimWeatherParams& wx,
                                                                     double elev_deg) const;
        double ComputeCloudSpecificAttenuation(double freqGHz, double T_K) const;
        double ComputeGaseousAttenuation(const LeoSimWeatherParams& wx,
                                                                         double elev_deg) const;
        double ComputeOxygenAttenuation(double freqGHz, double pressureHPa, double temperatureK) const;
        double ComputeWaterVapourAttenuation(double freqGHz,
                             double pressureHPa,
                             double temperatureK,
                             double waterVapourDensity) const;
        void GetP838Coefficients(double freqGHz, double elevDeg, double& k, double& alpha) const;
        double ComputeScintillationSigma(const LeoSimWeatherParams& wx,
                                                                         double elev_deg) const;
        double DrawScintillationSample(double sigma_dB) const;

        std::map<uint32_t, LeoSimWeatherState> m_states;
        std::map<uint32_t, LeoSimWeatherParams> m_params;
        std::map<uint32_t, std::vector<std::pair<double, LeoSimWeatherParams>>> m_traces; // time-indexed traces
        double m_freqHz = 12.0e9;
        double m_groundHeightKm = 0.0;
        double m_rainHeightKm = -1.0; // -1 = use P.839 formula
        double m_markovP[4][4];
        Time m_updateInterval = Seconds(1.0);
        bool m_useTrace = false;
        bool m_verbose = false;
        EventId m_updateEvent;
        Callback<void, uint32_t, LeoSimWeatherState, LeoSimWeatherState> m_weatherChangeCallback;
        mutable std::default_random_engine m_rng;
        mutable std::normal_distribution<double> m_normalDist{0.0, 1.0};
};

} // namespace ns3

#endif /* LEOSIM_WEATHER_MODEL_H */
