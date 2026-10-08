/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef LEOSIM_TCP_TRAFFIC_APPLICATION_H
#define LEOSIM_TCP_TRAFFIC_APPLICATION_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/data-rate.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/traced-callback.h"

namespace ns3
{

class Socket;
class Packet;
class LeoSimBeamManager;
class TcpHeader;
class TcpSocketBase;

/**
 * Rate-controlled LeoSim TCP source that treats transient no-route connection
 * failures as retryable network events rather than fatal simulation errors.
 */
class LeoSimTcpTrafficApplication : public Application
{
  public:
    static TypeId GetTypeId();

    void Configure(const Address& remote,
                   const DataRate& dataRate,
                   uint32_t packetSize,
                   Time retryInterval = Seconds(1));

    void SetHandoverManager(Ptr<LeoSimBeamManager> manager);

  protected:
    void StartApplication() override;
    void StopApplication() override;

  private:
    void Connect();
    void ConnectionSucceeded(Ptr<Socket> socket);
    void ConnectionFailed(Ptr<Socket> socket);
    void ConnectionClosed(Ptr<Socket> socket);
    void SendPacket();
    void SendPendingPacket();
    void SendReady(Ptr<Socket> socket, uint32_t availableBytes);
    void ScheduleNextPacket();
    void ScheduleRetry();
    void CongestionWindowChanged(uint32_t oldValue, uint32_t newValue);
    void PacketRetransmitted(Ptr<const Packet> packet,
                             const TcpHeader& header,
                             const Address& localAddress,
                             const Address& peerAddress,
                             Ptr<const TcpSocketBase> socket);

    Address m_remote;
    DataRate m_dataRate{0};
    uint32_t m_packetSize{1024};
    Time m_retryInterval{Seconds(1)};
    Ptr<Socket> m_socket;
    Ptr<Packet> m_pendingPacket;
    Ptr<LeoSimBeamManager> m_handoverManager;
    EventId m_sendEvent;
    EventId m_retryEvent;
    bool m_running{false};
    bool m_connected{false};
    TracedCallback<uint32_t, uint32_t> m_congestionWindowTrace;
    TracedCallback<Ptr<const Packet>> m_retransmissionTrace;
};

} // namespace ns3

#endif
