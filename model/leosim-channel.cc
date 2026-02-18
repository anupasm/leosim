/*
 * Copyright (c) 2024 Anupa De Silva
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

#include "leosim-channel.h"

#include "ns3/boolean.h"
#include "ns3/double.h"
#include "ns3/log.h"
#include "ns3/mobility-model.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/uinteger.h"

#include <algorithm>
#include <cmath>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimChannel");
NS_OBJECT_ENSURE_REGISTERED(LeoSimChannel);

TypeId
LeoSimChannel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LeoSimChannel")
            .SetParent<Channel>()
            .SetGroupName("LeoSim")
            .AddConstructor<LeoSimChannel>()
            .AddAttribute("CarrierFrequencyHz",
                          "Carrier frequency used to compute Doppler (Hz).",
                          DoubleValue(12.0e9),
                          MakeDoubleAccessor(&LeoSimChannel::m_carrierFrequencyHz),
                          MakeDoubleChecker<double>())
            .AddAttribute("TxPowerDbm",
                          "Transmit power (dBm).",
                          DoubleValue(40.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_txPowerDbm),
                          MakeDoubleChecker<double>())
            .AddAttribute("TxAntennaGainDb",
                          "Transmit antenna gain (dB).",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_txAntennaGainDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("RxAntennaGainDb",
                          "Receive antenna gain (dB).",
                          DoubleValue(20.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_rxAntennaGainDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("NoiseFigureDb",
                          "Receiver noise figure (dB).",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_noiseFigureDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("NoiseBandwidthHz",
                          "Noise bandwidth (Hz).",
                          DoubleValue(1.0e6),
                          MakeDoubleAccessor(&LeoSimChannel::m_noiseBandwidthHz),
                          MakeDoubleChecker<double>())
            .AddAttribute("MinSnrDb",
                          "Minimum SNR (dB) for a link to be up.",
                          DoubleValue(3.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_minSnrDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("AtmosphericLossDb",
                          "Atmospheric attenuation (dB).",
                          DoubleValue(2.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_atmosphericLossDb),
                          MakeDoubleChecker<double>())
            .AddAttribute("PropagationSpeed",
                          "Propagation speed (m/s).",
                          DoubleValue(299792458.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_propagationSpeed),
                          MakeDoubleChecker<double>())
            .AddAttribute("EnableBeamHopping",
                          "Enable beam hopping scheduler.",
                          BooleanValue(false),
                          MakeBooleanAccessor(&LeoSimChannel::m_enableBeamHopping),
                          MakeBooleanChecker())
            .AddAttribute("BeamHopPeriod",
                          "Total beam hop period (s).",
                          DoubleValue(1.0),
                          MakeDoubleAccessor(&LeoSimChannel::m_beamHopPeriod),
                          MakeDoubleChecker<double>())
            .AddAttribute("BeamDwellTime",
                          "Beam dwell time (s).",
                          DoubleValue(0.1),
                          MakeDoubleAccessor(&LeoSimChannel::m_beamDwellTime),
                          MakeDoubleChecker<double>())
            .AddAttribute("BeamsPerSatellite",
                          "Number of beams per satellite.",
                          UintegerValue(8),
                          MakeUintegerAccessor(&LeoSimChannel::m_beamsPerSatellite),
                          MakeUintegerChecker<uint32_t>())
            .AddAttribute("EnableDoppler",
                          "Enable Doppler computation.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&LeoSimChannel::m_enableDoppler),
                          MakeBooleanChecker())
            .AddAttribute("EnableLinkBudget",
                          "Enable link budget and SNR gating.",
                          BooleanValue(true),
                          MakeBooleanAccessor(&LeoSimChannel::m_enableLinkBudget),
                          MakeBooleanChecker())
            .AddTraceSource("SnrDb",
                            "Trace computed SNR for each delivered packet.",
                            MakeTraceSourceAccessor(&LeoSimChannel::m_snrTrace),
                            "ns3::TracedCallback")
            .AddTraceSource("DopplerHz",
                            "Trace Doppler shift (Hz) for each delivered packet.",
                            MakeTraceSourceAccessor(&LeoSimChannel::m_dopplerTrace),
                            "ns3::TracedCallback")
            .AddTraceSource("BeamId",
                            "Trace beam ID used for each delivered packet.",
                            MakeTraceSourceAccessor(&LeoSimChannel::m_beamTrace),
                            "ns3::TracedCallback")
            .AddTraceSource("LinkState",
                            "Trace link availability decisions.",
                            MakeTraceSourceAccessor(&LeoSimChannel::m_linkStateTrace),
                            "ns3::TracedCallback");
    return tid;
}

LeoSimChannel::LeoSimChannel()
    : m_carrierFrequencyHz(12.0e9),
      m_txPowerDbm(40.0),
      m_txAntennaGainDb(20.0),
      m_rxAntennaGainDb(20.0),
      m_noiseFigureDb(3.0),
      m_noiseBandwidthHz(1.0e6),
      m_minSnrDb(3.0),
      m_atmosphericLossDb(2.0),
      m_propagationSpeed(299792458.0),
      m_enableBeamHopping(false),
      m_beamHopPeriod(1.0),
      m_beamDwellTime(0.1),
      m_beamsPerSatellite(8),
      m_enableDoppler(true),
      m_enableLinkBudget(true)
{
    NS_LOG_FUNCTION(this);
}

void
LeoSimChannel::Add(Ptr<NetDevice> device)
{
    NS_LOG_FUNCTION(this << device);
    m_devices.push_back(device);
}

void
LeoSimChannel::Send(Ptr<Packet> p,
                    uint16_t protocol,
                    Mac48Address to,
                    Mac48Address from,
                    Ptr<NetDevice> sender)
{
    NS_LOG_FUNCTION(this << p << protocol << to << from << sender);

    // Note: LeoSimNetDevice functionality has been removed from this project.
    // This is now a stub implementation that preserves the class interface.
    // For actual packet transmission, use standard ns3 channels or your custom implementation.

    for (auto i = m_devices.begin(); i != m_devices.end(); ++i)
    {
        Ptr<NetDevice> rxDevice = *i;
        if (rxDevice == sender)
        {
            continue;
        }
        // Minimal handling - do not forward packets in stub mode
    }
}

std::size_t
LeoSimChannel::GetNDevices() const
{
    return m_devices.size();
}

Ptr<NetDevice>
LeoSimChannel::GetDevice(std::size_t i) const
{
    return m_devices[i];
}

Time
LeoSimChannel::CalculatePropagationDelay(double distance) const
{
    if (m_propagationSpeed <= 0.0)
    {
        return Seconds(0);
    }
    return Seconds(distance / m_propagationSpeed);
}

double
LeoSimChannel::CalculateDistance(Ptr<Node> a, Ptr<Node> b) const
{
    if (!a || !b)
    {
        return 0.0;
    }

    Ptr<MobilityModel> ma = a->GetObject<MobilityModel>();
    Ptr<MobilityModel> mb = b->GetObject<MobilityModel>();
    if (!ma || !mb)
    {
        return 0.0;
    }

    Vector pa = ma->GetPosition();
    Vector pb = mb->GetPosition();
    Vector d(pa.x - pb.x, pa.y - pb.y, pa.z - pb.z);

    return std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
}

double
LeoSimChannel::CalculateDopplerHz(Ptr<Node> tx, Ptr<Node> rx, double distance) const
{
    if (!m_enableDoppler || m_carrierFrequencyHz <= 0.0)
    {
        return 0.0;
    }

    if (!tx || !rx)
    {
        return 0.0;
    }

    Ptr<MobilityModel> mtx = tx->GetObject<MobilityModel>();
    Ptr<MobilityModel> mrx = rx->GetObject<MobilityModel>();
    if (!mtx || !mrx || distance <= 0.0)
    {
        return 0.0;
    }

    Vector ptx = mtx->GetPosition();
    Vector prx = mrx->GetPosition();
    Vector vtx = mtx->GetVelocity();
    Vector vrx = mrx->GetVelocity();

    Vector los(prx.x - ptx.x, prx.y - ptx.y, prx.z - ptx.z);
    double invDist = 1.0 / distance;
    Vector losUnit(los.x * invDist, los.y * invDist, los.z * invDist);

    Vector vrel(vtx.x - vrx.x, vtx.y - vrx.y, vtx.z - vrx.z);
    double radial = vrel.x * losUnit.x + vrel.y * losUnit.y + vrel.z * losUnit.z;

    return -(radial / m_propagationSpeed) * m_carrierFrequencyHz;
}

double
LeoSimChannel::CalculateSnrDb(double distance) const
{
    if (!m_enableLinkBudget || distance <= 0.0)
    {
        return 0.0;
    }

    double wavelength = m_propagationSpeed / m_carrierFrequencyHz;
    double fsplDb = 0.0;
    if (wavelength > 0.0)
    {
        fsplDb = 20.0 * std::log10(4.0 * M_PI * distance / wavelength);
    }

    double rxPowerDbm = m_txPowerDbm + m_txAntennaGainDb + m_rxAntennaGainDb - fsplDb -
                        m_atmosphericLossDb;

    double noiseDbm = -174.0;
    if (m_noiseBandwidthHz > 0.0)
    {
        noiseDbm += 10.0 * std::log10(m_noiseBandwidthHz);
    }
    noiseDbm += m_noiseFigureDb;

    return rxPowerDbm - noiseDbm;
}

bool
LeoSimChannel::IsBeamActive(Ptr<Node> satellite, Ptr<Node> ground) const
{
    if (!m_enableBeamHopping || m_beamDwellTime <= 0.0 || m_beamHopPeriod <= 0.0 ||
        m_beamsPerSatellite == 0)
    {
        return true;
    }

    double t = Simulator::Now().GetSeconds();
    double phase = std::fmod(t, m_beamHopPeriod);
    uint32_t activeBeam = static_cast<uint32_t>(phase / m_beamDwellTime) % m_beamsPerSatellite;
    uint32_t groundBeam = ground->GetId() % m_beamsPerSatellite;

    (void)satellite;

    return activeBeam == groundBeam;
}

bool
LeoSimChannel::IsLinkAvailable(Ptr<Node> satellite,
                               Ptr<Node> ground,
                               double& snrDb,
                               double& dopplerHz) const
{
    double distance = CalculateDistance(satellite, ground);
    snrDb = CalculateSnrDb(distance);
    dopplerHz = CalculateDopplerHz(satellite, ground, distance);

    if (!IsBeamActive(satellite, ground))
    {
        return false;
    }

    if (m_enableLinkBudget && snrDb < m_minSnrDb)
    {
        return false;
    }

    return true;
}

} // namespace ns3
