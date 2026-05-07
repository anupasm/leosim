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

#include "leosim-weather-model.h"

#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimWeatherModel");
NS_OBJECT_ENSURE_REGISTERED(LeoSimWeatherModel);

// Default 4x4 Markov transition matrix P[from][to].
// Each row sums to 1.0 and entries are per-second transition probabilities.
// Values are tuned to produce realistic average dwell times:
// CLEAR ~30 min, CLOUDY ~20 min, LIGHT_RAIN ~10 min, HEAVY_RAIN ~5 min.
const double LeoSimWeatherModel::DEFAULT_MARKOV_P[4][4] = {
    // TO:  CLEAR   CLOUDY  L_RAIN  H_RAIN
    /* CLEAR    */ {0.9967, 0.0030, 0.0003, 0.0000},
    /* CLOUDY   */ {0.0020, 0.9950, 0.0025, 0.0005},
    /* L_RAIN   */ {0.0005, 0.0100, 0.9800, 0.0095},
    /* H_RAIN   */ {0.0000, 0.0010, 0.0200, 0.9790}
};

TypeId
LeoSimWeatherModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LeoSimWeatherModel").SetParent<Object>().SetGroupName("LeoSim");
    return tid;
}

void
LeoSimWeatherModel::SetFrequencyHz(double freqHz)
{
    NS_LOG_FUNCTION(this << freqHz);
    m_freqHz = freqHz;
}

void
LeoSimWeatherModel::SetGroundStationHeight(double hS_km)
{
    NS_LOG_FUNCTION(this << hS_km);
    m_groundHeightKm = hS_km;
}

void
LeoSimWeatherModel::SetRainHeightKm(double hR_km)
{
    NS_LOG_FUNCTION(this << hR_km);
    m_rainHeightKm = hR_km;
}

void
LeoSimWeatherModel::SetMarkovMatrix(const double P[4][4])
{
    NS_LOG_FUNCTION(this);
    for (int i = 0; i < 4; ++i)
    {
        double rowSum = 0.0;
        for (int j = 0; j < 4; ++j)
        {
            rowSum += P[i][j];
        }
        if (rowSum <= 0.0)
        {
            for (int j = 0; j < 4; ++j)
            {
                m_markovP[i][j] = DEFAULT_MARKOV_P[i][j];
            }
            continue;
        }

        for (int j = 0; j < 4; ++j)
        {
            m_markovP[i][j] = P[i][j] / rowSum;
        }
    }
}

void
LeoSimWeatherModel::SetUpdateInterval(Time t)
{
    NS_LOG_FUNCTION(this << t);
    m_updateInterval = t;
}

void
LeoSimWeatherModel::SetUniformWeatherParams(const LeoSimWeatherParams& params)
{
    NS_LOG_FUNCTION(this);

    for (auto& [nodeId, p] : m_params)
    {
        (void)(nodeId);
        p = params;
    }

    for (auto& [nodeId, state] : m_states)
    {
        state = params.state;
        if (m_params.find(nodeId) == m_params.end())
        {
            m_params[nodeId] = params;
        }
    }
}

void
LeoSimWeatherModel::SetNodeWeatherParams(uint32_t groundNodeId,
                                         const LeoSimWeatherParams& params)
{
    NS_LOG_FUNCTION(this << groundNodeId);
    m_params[groundNodeId] = params;
    m_states[groundNodeId] = params.state;
}

void
LeoSimWeatherModel::Start(const NodeContainer& groundNodes, Time simTime)
{
    NS_LOG_FUNCTION(this << groundNodes.GetN() << simTime);

    bool markovInitialized = false;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            if (m_markovP[i][j] > 0.0)
            {
                markovInitialized = true;
                break;
            }
        }
        if (markovInitialized)
        {
            break;
        }
    }
    if (!markovInitialized)
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                m_markovP[i][j] = DEFAULT_MARKOV_P[i][j];
            }
        }
    }

    for (uint32_t i = 0; i < groundNodes.GetN(); ++i)
    {
        Ptr<Node> node = groundNodes.Get(i);
        if (!node)
        {
            continue;
        }
        uint32_t nodeId = node->GetId();

        if (m_states.find(nodeId) == m_states.end())
        {
            m_states[nodeId] = LEOSIM_WX_CLEAR;
        }

        if (m_params.find(nodeId) == m_params.end())
        {
            LeoSimWeatherParams p{};
            p.rainRateMmh = 0.0;
            p.cloudLiquidWater = 0.0;
            p.temperatureCelsius = 20.0;
            p.pressureHPa = 1013.0;
            p.waterVapourDensity = 10.0;
            p.humidity = 60.0;
            p.state = m_states[nodeId];

            auto traceIt = m_traces.find(nodeId);
            if (m_useTrace && traceIt != m_traces.end() && !traceIt->second.empty())
            {
                p = traceIt->second.front().second;
                m_states[nodeId] = p.state;
            }

            m_params[nodeId] = p;
        }
    }

    if (!m_updateEvent.IsExpired())
    {
        Simulator::Cancel(m_updateEvent);
    }
    m_updateEvent = Simulator::Schedule(
        m_updateInterval, &LeoSimWeatherModel::AdvanceMarkovStates, this);
}

void
LeoSimWeatherModel::Stop()
{
    NS_LOG_FUNCTION(this);
    if (!m_updateEvent.IsExpired())
    {
        Simulator::Cancel(m_updateEvent);
    }
}

LeoSimWeatherState
LeoSimWeatherModel::GetWeatherState(uint32_t groundNodeId) const
{
    NS_LOG_FUNCTION(this << groundNodeId);
    auto it = m_states.find(groundNodeId);
    if (it != m_states.end())
    {
        return it->second;
    }
    return LEOSIM_WX_CLEAR;
}

LeoSimWeatherParams
LeoSimWeatherModel::GetWeatherParams(uint32_t groundNodeId) const
{
    NS_LOG_FUNCTION(this << groundNodeId);
    auto it = m_params.find(groundNodeId);
    if (it != m_params.end())
    {
        return it->second;
    }

    LeoSimWeatherParams p{};
    p.rainRateMmh = 0.0;
    p.cloudLiquidWater = 0.0;
    p.temperatureCelsius = 20.0;
    p.pressureHPa = 1013.0;
    p.waterVapourDensity = 10.0;
    p.humidity = 60.0;
    p.state = LEOSIM_WX_CLEAR;
    return p;
}

double
LeoSimWeatherModel::GetTotalAttenuation(uint32_t groundNodeId, uint32_t satNodeId) const
{
    NS_LOG_FUNCTION(this << groundNodeId << satNodeId);
    if (m_params.find(groundNodeId) == m_params.end())
    {
        return 0.0;
    }
    return ComputeAttenuation(groundNodeId, satNodeId, 90.0).totalAttenuation_dB;
}

void
LeoSimWeatherModel::SetWeatherChangeCallback(
    Callback<void, uint32_t, LeoSimWeatherState, LeoSimWeatherState> cb)
{
    NS_LOG_FUNCTION(this);
    m_weatherChangeCallback = cb;
}

void
LeoSimWeatherModel::SetVerbose(bool v)
{
    NS_LOG_FUNCTION(this << v);
    m_verbose = v;
}

double
LeoSimWeatherModel::ComputeRainAttenuation(const LeoSimWeatherParams& wx, double elev_deg) const
{
    if (wx.rainRateMmh <= 0.0)
    {
        return 0.0;
    }
    if (elev_deg <= 0.0)
    {
        return 0.0;
    }

    const double R001 = wx.rainRateMmh; // rain rate exceeded 0.01% of time
    const double freqGHz = m_freqHz / 1e9;

    // Step 1: Rain height h_R (ITU-R P.839-4)
    // h_R ~= h_0 + 0.36 km, where h_0 is 0 degC isotherm height.
    const double hR = (m_rainHeightKm > 0.0) ? m_rainHeightKm : (3.0 + 0.36);

    // Step 2: Slant path length below rain height.
    const double hS = m_groundHeightKm;
    const double elev_rad = elev_deg * M_PI / 180.0;
    double Ls = 0.0;
    if (elev_deg >= 5.0)
    {
        Ls = (hR - hS) / std::sin(elev_rad);
    }
    else
    {
        Ls = 2.0 * (hR - hS) /
             (std::sqrt(std::sin(elev_rad) * std::sin(elev_rad) + 2 * (hR - hS) / 8500.0) +
              std::sin(elev_rad));
    }

    if (Ls <= 0.0)
    {
        return 0.0;
    }

    // Step 3: Horizontal projection.
    const double Lg = Ls * std::cos(elev_rad);

    // Step 4: Specific attenuation gamma_R (ITU-R P.838-3), gamma_R = k * R^alpha.
    double k = 0.0;
    double alpha_coeff = 0.0;
    GetP838Coefficients(freqGHz, elev_deg, k, alpha_coeff);
    const double gammaR = k * std::pow(R001, alpha_coeff);

    // Step 5: Horizontal reduction factor r_001.
    const double r001 =
        1.0 /
        (1.0 + 0.78 * std::sqrt(Lg * gammaR / freqGHz) - 0.38 * (1.0 - std::exp(-2.0 * Lg)));

    // Step 6: Vertical adjustment factor v_001.
    const double chi =
        (elev_deg < 36.0) ? std::abs(std::atan(0.01 * (hR - hS) / (Lg * r001 - 0.36 * std::cos(elev_rad))))
                          : elev_rad;
    const double Lr =
        (Lg * r001 / std::cos(chi)) < ((hR - hS) / std::sin(elev_rad))
            ? (Lg * r001 / std::cos(chi))
            : ((hR - hS) / std::sin(elev_rad));

    const double v001 =
        1.0 /
        (1.0 + std::sqrt(std::sin(elev_rad)) *
                   (31.0 * (1.0 - std::exp(-elev_deg / (1.0 + chi))) * std::sqrt(Lr * gammaR) /
                        (freqGHz * freqGHz) -
                    0.45));

    // Step 7: Effective path length.
    const double Le = Lr * v001;

    // Step 8: Attenuation at 0.01% of time.
    const double A001 = gammaR * Le;

    if (m_verbose)
    {
        NS_LOG_DEBUG("[Weather] Rain attenuation A001=" << A001 << " dB"
                     << " (R001=" << R001 << " mm/h, elev=" << elev_deg << " deg)");
    }

    return A001;
}

void
LeoSimWeatherModel::GetP838Coefficients(double freqGHz,
                                         double elevDeg,
                                         double& k,
                                         double& alpha) const
{
    (void)(elevDeg);

    NS_ASSERT_MSG(freqGHz >= 1.0 && freqGHz <= 1000.0,
                  "Frequency out of ITU-R P.838 range");

    // ITU-R P.838-3 Table 1 — key frequency points (log-linear interpolation)
    // Horizontal polarisation coefficients
    static const double freqs[] = {1.0, 2.0, 4.0, 6.0, 7.0, 8.0, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0};
    static const double kH[] = {0.0000387, 0.000154, 0.000650, 0.00175, 0.00301,
                                0.00454, 0.0101, 0.0188, 0.0367, 0.0751, 0.124,
                                0.187, 0.263, 0.350};
    static const double alphaH[] = {0.912, 0.963, 1.121, 1.308, 1.332, 1.327, 1.276,
                                    1.217, 1.154, 1.099, 1.061, 1.021, 0.979, 0.939};

    const int n = static_cast<int>(sizeof(freqs) / sizeof(freqs[0]));
    int idx = 0;

    if (freqGHz <= freqs[0])
    {
        idx = 0;
    }
    else if (freqGHz >= freqs[n - 1])
    {
        idx = n - 2;
    }
    else
    {
        for (int i = 0; i < n - 1; ++i)
        {
            if (freqGHz >= freqs[i] && freqGHz <= freqs[i + 1])
            {
                idx = i;
                break;
            }
        }
    }

    const double logF = std::log10(freqGHz);
    const double logF0 = std::log10(freqs[idx]);
    const double logF1 = std::log10(freqs[idx + 1]);
    const double t = (logF - logF0) / (logF1 - logF0);

    const double logK = (1.0 - t) * std::log10(kH[idx]) + t * std::log10(kH[idx + 1]);
    k = std::pow(10.0, logK);
    alpha = (1.0 - t) * alphaH[idx] + t * alphaH[idx + 1];
}

double
LeoSimWeatherModel::ComputeCloudAttenuation(const LeoSimWeatherParams& wx, double elev_deg) const
{
    if (wx.cloudLiquidWater <= 0.0 ||
        wx.state == LEOSIM_WX_CLEAR || wx.state == LEOSIM_WX_LIGHT_RAIN)
    {
        return 0.0;
    }

    if (elev_deg <= 0.0)
    {
        return 0.0;
    }

    const double freqGHz = m_freqHz / 1e9;

    // Specific attenuation coefficient K_l (dB/km per kg/m^2)
    // ITU-R P.840-9 Annex 1 (double-Debye + Rayleigh approximation).
    const double T_K = wx.temperatureCelsius + 273.15;
    const double Kl = ComputeCloudSpecificAttenuation(freqGHz, T_K);

    const double elev_rad = elev_deg * M_PI / 180.0;
    const double AC = (Kl * wx.cloudLiquidWater) / std::sin(elev_rad);
    return AC;
}

double
LeoSimWeatherModel::ComputeCloudSpecificAttenuation(double freqGHz, double T_K) const
{
    NS_ASSERT_MSG(T_K > 0.0, "Temperature must be positive in Kelvin");

    // ITU-R P.840-9 Annex 1 double-Debye model for liquid water permittivity.
    const double theta = 300.0 / T_K;
    const double dtheta = theta - 1.0;

    // Relaxation frequencies (GHz).
    const double fp = 20.09 - 142.4 * dtheta + 294.0 * dtheta * dtheta;
    const double fs = 590.0 - 1500.0 * dtheta;

    // Static/high-frequency permittivity terms.
    const double eps0 = 77.66 + 103.3 * dtheta;
    const double eps1 = 5.48;
    const double eps2 = 3.51;

    // Real and imaginary parts of complex permittivity (P.840-9 eqs. 3-4).
    const double ratioP = freqGHz / fp;
    const double ratioS = freqGHz / fs;
    const double denomP = 1.0 + ratioP * ratioP;
    const double denomS = 1.0 + ratioS * ratioS;

    const double epsr = eps0 - (eps0 - eps1) / denomP - (eps1 - eps2) / denomS;
    const double epsi = freqGHz * (eps0 - eps1) / (fp * denomP) +
                        freqGHz * (eps1 - eps2) / (fs * denomS);

    // Rayleigh approximation:
    // Kl = 0.819 * f * eps'' / ( (eps'^2 + eps''^2) * term ),
    // where term = ((eps' + 2)^2 + eps''^2) / (eps'^2 + eps''^2).
    // This is algebraically equivalent to dividing by ((eps' + 2)^2 + eps''^2).
    const double epsMagSq = epsr * epsr + epsi * epsi;
    const double term = (((epsr + 2.0) * (epsr + 2.0)) + (epsi * epsi)) / epsMagSq;
    return 0.819 * freqGHz * epsi / (epsMagSq * term);
}

double
LeoSimWeatherModel::ComputeGaseousAttenuation(const LeoSimWeatherParams& wx, double elev_deg) const
{
    if (elev_deg <= 0.0)
    {
        return 0.0;
    }

    const double freqGHz = m_freqHz / 1e9;
    const double T_K = wx.temperatureCelsius + 273.15;
    const double P = wx.pressureHPa;
    const double rho = wx.waterVapourDensity;

    const double gamma_o = ComputeOxygenAttenuation(freqGHz, P, T_K);
    const double gamma_w = ComputeWaterVapourAttenuation(freqGHz, P, T_K, rho);

    const double elev_rad = elev_deg * M_PI / 180.0;
    const double A_G = (gamma_o + gamma_w) / std::sin(elev_rad);
    return A_G;
}

double
LeoSimWeatherModel::ComputeOxygenAttenuation(double freqGHz,
                                              double pressureHPa,
                                              double temperatureK) const
{
    NS_ASSERT_MSG(temperatureK > 0.0, "Temperature must be positive in Kelvin");

    // Simplified empirical approximation (P.676 Annex 2 style, valid ~1-100 GHz).
    const double f2 = freqGHz * freqGHz;
    const double pressureScale = pressureHPa / 1013.0;
    const double tempScale = std::pow(273.0 / temperatureK, 0.5);
    const double gamma_o = (7.2 * (f2 / (f2 + 0.36)) + 0.005 * f2) * pressureScale * tempScale;
    return std::max(0.0, gamma_o);
}

double
LeoSimWeatherModel::ComputeWaterVapourAttenuation(double freqGHz,
                                                   double pressureHPa,
                                                   double temperatureK,
                                                   double waterVapourDensity) const
{
    (void)(pressureHPa);
    (void)(temperatureK);

    // Simplified empirical approximation (P.676 Annex 2 style, valid ~1-100 GHz).
    const double f2 = freqGHz * freqGHz;
    const double d = freqGHz - 22.2;
    const double lineShape = 3.6 * d * d / (f2 + 8.5 * d * d);
    const double gamma_w =
        (0.05 + 0.0021 * waterVapourDensity + lineShape) * waterVapourDensity * (f2 / (f2 + 25.0));
    return std::max(0.0, gamma_w);
}

double
LeoSimWeatherModel::ComputeScintillationSigma(const LeoSimWeatherParams& wx, double elev_deg) const
{
    // ITU-R P.618-14 section 2.4 - standard deviation of scintillation
    double freqGHz = m_freqHz / 1e9;

    // Wet term of refractivity N_wet
    double T_K = wx.temperatureCelsius + 273.15;
    double e = wx.humidity * 0.01 * 6.105 *
               std::exp(25.22 * (T_K - 273.16) / T_K - 5.31 * std::log(T_K / 273.16));
    double N_wet = 3.732e5 * e / (T_K * T_K);

    // Sigma reference (1 GHz, 1 m diameter antenna, Cn^2 model)
    double sigma_ref = 3.6e-3 + 1e-4 * N_wet;

    // Frequency scaling: f^(7/12)
    double f_scale = std::pow(freqGHz, 7.0 / 12.0);

    // Elevation scaling: 1/sin^(11/12)(elev)
    double elev_rad = elev_deg * M_PI / 180.0;
    double elev_scale = 1.0 / std::pow(std::sin(elev_rad), 11.0 / 12.0);

    // Effective antenna diameter correction (D = 1m default)
    double D = 1.0; // configurable per ground station
    double ga = std::sqrt(1.0 + 1.0 / std::pow(D * std::sqrt(freqGHz) / 0.4, 7.0 / 3.0));

    return sigma_ref * f_scale * elev_scale * ga;
}

double
LeoSimWeatherModel::DrawScintillationSample(double sigma_dB) const
{
    // Draw from zero-mean Gaussian with std = sigma_dB
    // Clamp at +/-3 sigma to avoid extreme outliers
    double sample = m_normalDist(m_rng) * sigma_dB;
    return std::max(-3.0 * sigma_dB, std::min(3.0 * sigma_dB, sample));
}

LeoSimAttenuationResult
LeoSimWeatherModel::ComputeAttenuation(uint32_t groundNodeId,
                                       uint32_t satNodeId,
                                       double elevAngle_deg) const
{
    (void)(satNodeId);

    LeoSimAttenuationResult result{};
    result.elevationAngle_deg = elevAngle_deg;
    result.computedAt = Simulator::Now();
    result.groundState = LEOSIM_WX_CLEAR;

    auto it = m_params.find(groundNodeId);
    if (it == m_params.end())
    {
        return result;
    }

    const auto& wx = it->second;
    result.groundState = wx.state;

    result.rainAttenuation_dB = ComputeRainAttenuation(wx, elevAngle_deg);
    result.cloudAttenuation_dB = ComputeCloudAttenuation(wx, elevAngle_deg);
    result.gaseousAttenuation_dB = ComputeGaseousAttenuation(wx, elevAngle_deg);
    double sigma = ComputeScintillationSigma(wx, elevAngle_deg);
    result.scintillationAmplitude_dB = sigma;
    result.scintillationSample_dB = DrawScintillationSample(sigma);

    // ITU-R P.618-14 total: A_T = A_G + sqrt((A_R + A_C)^2 + A_S^2)
    double AR = result.rainAttenuation_dB;
    double AC = result.cloudAttenuation_dB;
    double AG = result.gaseousAttenuation_dB;
    double AS = result.scintillationSample_dB;

    result.totalAttenuation_dB = AG + std::sqrt((AR + AC) * (AR + AC) + AS * AS);

    if (m_verbose)
    {
        NS_LOG_DEBUG("[Weather] Attenuation components: "
                     << "rain=" << result.rainAttenuation_dB << " dB, "
                     << "cloud=" << result.cloudAttenuation_dB << " dB, "
                     << "gas=" << result.gaseousAttenuation_dB << " dB, "
                     << "scint_sigma=" << result.scintillationAmplitude_dB << " dB, "
                     << "scint_sample=" << result.scintillationSample_dB << " dB, "
                     << "total=" << result.totalAttenuation_dB << " dB");
    }

    return result;
}

void
LeoSimWeatherModel::AdvanceMarkovStates()
{
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    for (auto& [nodeId, state] : m_states)
    {
        if (m_useTrace)
        {
            AdvanceFromTrace(nodeId);
            continue;
        }

        double r = unif(m_rng);
        double cumProb = 0.0;
        LeoSimWeatherState newState = state;
        for (int j = 0; j < 4; j++)
        {
            cumProb += m_markovP[(int)state][j];
            if (r < cumProb)
            {
                newState = static_cast<LeoSimWeatherState>(j);
                break;
            }
        }

        if (newState != state)
        {
            LeoSimWeatherState oldState = state;
            state = newState;
            DrawWeatherParams(nodeId, newState);

            if (m_verbose)
                NS_LOG_INFO("[Weather] Node " << nodeId << " state: "
                    << oldState << " -> " << newState
                    << " at t=" << Simulator::Now().GetSeconds() << "s");

            if (!m_weatherChangeCallback.IsNull())
                m_weatherChangeCallback(nodeId, oldState, newState);
        }
    }

    m_updateEvent = Simulator::Schedule(
        m_updateInterval, &LeoSimWeatherModel::AdvanceMarkovStates, this);
}

void
LeoSimWeatherModel::DrawWeatherParams(uint32_t nodeId, LeoSimWeatherState state)
{
    auto& p = m_params[nodeId];
    p.state = state;

    switch (state) {
    case LEOSIM_WX_CLEAR:
        p.rainRateMmh      = 0.0;
        p.cloudLiquidWater = 0.0;
        break;
    case LEOSIM_WX_CLOUDY:
        p.rainRateMmh      = 0.0;
        // L in [0.05, 0.5] kg/m² log-normally distributed
        p.cloudLiquidWater = std::max(0.05,
            std::min(0.5, std::exp(m_normalDist(m_rng) * 0.5 - 2.0)));
        break;
    case LEOSIM_WX_LIGHT_RAIN:
        // R in (0, 5] mm/h — log-normal
        p.rainRateMmh      = std::max(0.5,
            std::min(5.0, std::exp(m_normalDist(m_rng) * 0.6 + 0.5)));
        p.cloudLiquidWater = 0.3;
        break;
    case LEOSIM_WX_HEAVY_RAIN:
        // R > 5 mm/h — log-normal, mean ~25, tail to 150
        p.rainRateMmh      = std::max(5.0,
            std::min(150.0, std::exp(m_normalDist(m_rng) * 0.7 + 3.2)));
        p.cloudLiquidWater = 1.0;
        break;
    }
    // Temperature, pressure, humidity stay as configured (vary slowly)
}

void
LeoSimWeatherModel::LoadWeatherTraceFromCsv(const std::string& file)
{
    std::ifstream ifs(file);
    NS_ABORT_MSG_IF(!ifs.is_open(), "LeoSimWeatherModel: cannot open trace file: " << file);

    std::string line;
    // Skip header line
    std::getline(ifs, line);

    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#')
            continue;
        std::istringstream ss(line);
        std::string tok;
        double t, rain, lwc, temp, press, vapour, hum;
        uint32_t nodeId;

        // Columns: time_s,node_id,rain_rate_mmh,cloud_lwc,temp_c,pressure_hpa,
        //          water_vapour_density,humidity
        std::getline(ss, tok, ','); t      = std::stod(tok);
        std::getline(ss, tok, ','); nodeId = static_cast<uint32_t>(std::stoul(tok));
        std::getline(ss, tok, ','); rain   = std::stod(tok);
        std::getline(ss, tok, ','); lwc    = std::stod(tok);
        std::getline(ss, tok, ','); temp   = std::stod(tok);
        std::getline(ss, tok, ','); press  = std::stod(tok);
        std::getline(ss, tok, ','); vapour = std::stod(tok);
        std::getline(ss, tok, ','); hum    = std::stod(tok);

        LeoSimWeatherParams p;
        p.rainRateMmh        = rain;
        p.cloudLiquidWater   = lwc;
        p.temperatureCelsius = temp;
        p.pressureHPa        = press;
        p.waterVapourDensity = vapour;
        p.humidity           = hum;
        // Derive state from rain rate and liquid water
        if (rain <= 0.0 && lwc <= 0.0)
            p.state = LEOSIM_WX_CLEAR;
        else if (rain <= 0.0)
            p.state = LEOSIM_WX_CLOUDY;
        else if (rain <= 5.0)
            p.state = LEOSIM_WX_LIGHT_RAIN;
        else
            p.state = LEOSIM_WX_HEAVY_RAIN;

        m_traces[nodeId].emplace_back(t, p);
    }

    // Sort each node's trace by time
    for (auto& [nodeId, trace] : m_traces)
        std::sort(trace.begin(), trace.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

    m_useTrace = true;
    NS_LOG_INFO("[Weather] Loaded trace from " << file
        << " for " << m_traces.size() << " node(s)");
}

void
LeoSimWeatherModel::AdvanceFromTrace(uint32_t nodeId)
{
    auto it = m_traces.find(nodeId);
    if (it == m_traces.end() || it->second.empty()) {
        // No trace for this node — fall back to Markov
        std::uniform_real_distribution<double> unif(0.0, 1.0);
        double r = unif(m_rng);
        auto& state = m_states[nodeId];
        double cumProb = 0.0;
        LeoSimWeatherState newState = state;
        for (int j = 0; j < 4; j++) {
            cumProb += m_markovP[(int)state][j];
            if (r < cumProb) {
                newState = static_cast<LeoSimWeatherState>(j);
                break;
            }
        }
        if (newState != state) {
            state = newState;
            DrawWeatherParams(nodeId, newState);
        }
        return;
    }

    const auto& trace = it->second;
    double now = Simulator::Now().GetSeconds();

    // Clamp to last entry
    if (now >= trace.back().first) {
        m_params[nodeId] = trace.back().second;
        m_states[nodeId] = trace.back().second.state;
        return;
    }

    // Clamp to first entry
    if (now <= trace.front().first) {
        m_params[nodeId] = trace.front().second;
        m_states[nodeId] = trace.front().second.state;
        return;
    }

    // Find bracketing entries and interpolate linearly
    std::size_t hi = 1;
    while (hi < trace.size() && trace[hi].first < now)
        ++hi;

    const double t0 = trace[hi - 1].first;
    const double t1 = trace[hi].first;
    const LeoSimWeatherParams& p0 = trace[hi - 1].second;
    const LeoSimWeatherParams& p1 = trace[hi].second;
    double alpha = (now - t0) / (t1 - t0);

    LeoSimWeatherParams interp;
    interp.rainRateMmh        = p0.rainRateMmh        + alpha * (p1.rainRateMmh        - p0.rainRateMmh);
    interp.cloudLiquidWater   = p0.cloudLiquidWater   + alpha * (p1.cloudLiquidWater   - p0.cloudLiquidWater);
    interp.temperatureCelsius = p0.temperatureCelsius + alpha * (p1.temperatureCelsius - p0.temperatureCelsius);
    interp.pressureHPa        = p0.pressureHPa        + alpha * (p1.pressureHPa        - p0.pressureHPa);
    interp.waterVapourDensity = p0.waterVapourDensity + alpha * (p1.waterVapourDensity - p0.waterVapourDensity);
    interp.humidity           = p0.humidity           + alpha * (p1.humidity           - p0.humidity);
    // State: use nearest neighbour (discrete — no interpolation)
    interp.state = (alpha < 0.5) ? p0.state : p1.state;

    LeoSimWeatherState prevState = m_states[nodeId];
    m_params[nodeId] = interp;
    m_states[nodeId] = interp.state;

    if (interp.state != prevState) {
        if (m_verbose)
            NS_LOG_INFO("[Weather] Node " << nodeId << " trace state: "
                << prevState << " -> " << interp.state
                << " at t=" << now << "s");
        if (!m_weatherChangeCallback.IsNull())
            m_weatherChangeCallback(nodeId, prevState, interp.state);
    }
}

} // namespace ns3
