/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef LEOSIM_TCP_TRAFFIC_APPLICATION_H
#define LEOSIM_TCP_TRAFFIC_APPLICATION_H

#include "ns3/address.h"
#include "ns3/application.h"
#include "ns3/data-rate.h"
#include "ns3/event-id.h"
#include "ns3/ptr.h"

namespace ns3
{

class Socket;

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

  protected:
    void StartApplication() override;
    void StopApplication() override;

  private:
    void Connect();
    void ConnectionSucceeded(Ptr<Socket> socket);
    void ConnectionFailed(Ptr<Socket> socket);
    void ConnectionClosed(Ptr<Socket> socket);
    void SendPacket();
    void ScheduleRetry();

    Address m_remote;
    DataRate m_dataRate{0};
    uint32_t m_packetSize{1024};
    Time m_retryInterval{Seconds(1)};
    Ptr<Socket> m_socket;
    EventId m_sendEvent;
    EventId m_retryEvent;
    bool m_running{false};
    bool m_connected{false};
};

} // namespace ns3

#endif
