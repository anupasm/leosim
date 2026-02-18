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

#ifndef LEOSIM_CHANNEL_H
#define LEOSIM_CHANNEL_H

#include "ns3/channel.h"
#include "ns3/mac48-address.h"
#include "ns3/nstime.h"
#include "ns3/traced-callback.h"

#include <vector>

namespace ns3
{

class Packet;
class MobilityModel;
class Node;

/**
 * \ingroup leosim
 * \brief LEO satellite channel with Doppler, beam hopping, and link budget.
 */
class LeoSimChannel : public Channel
{
  public:
    static TypeId GetTypeId();
    LeoSimChannel();

    void Add(Ptr<NetDevice> device);

    void Send(Ptr<Packet> p,
              uint16_t protocol,
              Mac48Address to,
              Mac48Address from,
              Ptr<NetDevice> sender);

    std::size_t GetNDevices() const override;
    Ptr<NetDevice> GetDevice(std::size_t i) const override;

  private:
    double CalculateDistance(Ptr<Node> a, Ptr<Node> b) const;
    double CalculateDopplerHz(Ptr<Node> tx, Ptr<Node> rx, double distance) const;
    double CalculateSnrDb(double distance) const;
    bool IsBeamActive(Ptr<Node> satellite, Ptr<Node> ground) const;
    bool IsLinkAvailable(Ptr<Node> satellite, Ptr<Node> ground, double& snrDb, double& dopplerHz) const;

    Time CalculatePropagationDelay(double distance) const;

    std::vector<Ptr<NetDevice>> m_devices;

    double m_carrierFrequencyHz;
    double m_txPowerDbm;
    double m_txAntennaGainDb;
    double m_rxAntennaGainDb;
    double m_noiseFigureDb;
    double m_noiseBandwidthHz;
    double m_minSnrDb;
    double m_atmosphericLossDb;
    double m_propagationSpeed;

    bool m_enableBeamHopping;
    double m_beamHopPeriod;
    double m_beamDwellTime;
    uint32_t m_beamsPerSatellite;

    bool m_enableDoppler;
    bool m_enableLinkBudget;

    TracedCallback<Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, double> m_snrTrace;
    TracedCallback<Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, double> m_dopplerTrace;
    TracedCallback<Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, uint32_t> m_beamTrace;
    TracedCallback<Ptr<const Packet>, Ptr<const NetDevice>, Ptr<const NetDevice>, bool> m_linkStateTrace;
};

} // namespace ns3

#endif /* LEOSIM_CHANNEL_H */
