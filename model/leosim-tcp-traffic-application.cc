/* SPDX-License-Identifier: GPL-2.0-only */
#include "leosim-tcp-traffic-application.h"

#include "ns3/inet-socket-address.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"
#include "ns3/socket.h"
#include "ns3/tcp-socket-factory.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimTcpTrafficApplication");
NS_OBJECT_ENSURE_REGISTERED(LeoSimTcpTrafficApplication);

TypeId
LeoSimTcpTrafficApplication::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimTcpTrafficApplication")
                            .SetParent<Application>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimTcpTrafficApplication>();
    return tid;
}

void
LeoSimTcpTrafficApplication::Configure(const Address& remote,
                                       const DataRate& dataRate,
                                       uint32_t packetSize,
                                       Time retryInterval)
{
    NS_ABORT_MSG_IF(dataRate.GetBitRate() == 0, "LeoSim TCP traffic rate must be positive");
    NS_ABORT_MSG_IF(packetSize == 0, "LeoSim TCP packet size must be positive");
    NS_ABORT_MSG_IF(retryInterval.IsNegative(), "TCP retry interval cannot be negative");
    m_remote = remote;
    m_dataRate = dataRate;
    m_packetSize = packetSize;
    m_retryInterval = retryInterval;
}

void
LeoSimTcpTrafficApplication::StartApplication()
{
    m_running = true;
    Connect();
}

void
LeoSimTcpTrafficApplication::StopApplication()
{
    m_running = false;
    m_connected = false;
    Simulator::Cancel(m_sendEvent);
    Simulator::Cancel(m_retryEvent);
    if (m_socket)
    {
        m_socket->SetConnectCallback(MakeNullCallback<void, Ptr<Socket>>(),
                                     MakeNullCallback<void, Ptr<Socket>>());
        m_socket->SetCloseCallbacks(MakeNullCallback<void, Ptr<Socket>>(),
                                    MakeNullCallback<void, Ptr<Socket>>());
        m_socket->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
        m_socket->Close();
        m_socket = nullptr;
    }
    m_pendingPacket = nullptr;
}

void
LeoSimTcpTrafficApplication::Connect()
{
    if (!m_running || m_connected || m_socket)
    {
        return;
    }
    m_socket = Socket::CreateSocket(GetNode(), TcpSocketFactory::GetTypeId());
    m_socket->Bind();
    m_socket->SetConnectCallback(
        MakeCallback(&LeoSimTcpTrafficApplication::ConnectionSucceeded, this),
        MakeCallback(&LeoSimTcpTrafficApplication::ConnectionFailed, this));
    m_socket->SetCloseCallbacks(
        MakeCallback(&LeoSimTcpTrafficApplication::ConnectionClosed, this),
        MakeCallback(&LeoSimTcpTrafficApplication::ConnectionClosed, this));
    m_socket->SetSendCallback(MakeCallback(&LeoSimTcpTrafficApplication::SendReady, this));
    m_socket->Connect(m_remote);
    m_socket->ShutdownRecv();
}

void
LeoSimTcpTrafficApplication::ConnectionSucceeded(Ptr<Socket> socket)
{
    if (!m_running || socket != m_socket)
    {
        return;
    }
    m_connected = true;
    Simulator::Cancel(m_retryEvent);
    SendPacket();
}

void
LeoSimTcpTrafficApplication::ConnectionFailed(Ptr<Socket> socket)
{
    NS_LOG_WARN("TCP connection failed at " << Simulator::Now().GetSeconds()
                                             << "s; scheduling LeoSim retry");
    if (socket == m_socket)
    {
        socket->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
        socket->Close();
        m_socket = nullptr;
    }
    m_connected = false;
    Simulator::Cancel(m_sendEvent);
    ScheduleRetry();
}

void
LeoSimTcpTrafficApplication::ConnectionClosed(Ptr<Socket> socket)
{
    if (socket == m_socket)
    {
        socket->SetSendCallback(MakeNullCallback<void, Ptr<Socket>, uint32_t>());
        m_socket = nullptr;
    }
    m_connected = false;
    Simulator::Cancel(m_sendEvent);
    ScheduleRetry();
}

void
LeoSimTcpTrafficApplication::SendPacket()
{
    if (!m_running || !m_connected || !m_socket)
    {
        return;
    }
    if (!m_pendingPacket)
    {
        m_pendingPacket = Create<Packet>(m_packetSize);
    }
    SendPendingPacket();
}

void
LeoSimTcpTrafficApplication::SendPendingPacket()
{
    if (!m_running || !m_connected || !m_socket || !m_pendingPacket)
    {
        return;
    }

    const int sent = m_socket->Send(m_pendingPacket);
    if (sent < 0)
    {
        // TCP has no send-buffer space.  Keep the packet and wait for SendReady().
        return;
    }

    const uint32_t bytesSent = static_cast<uint32_t>(sent);
    if (bytesSent < m_pendingPacket->GetSize())
    {
        m_pendingPacket = m_pendingPacket->CreateFragment(bytesSent,
                                                          m_pendingPacket->GetSize() - bytesSent);
        return;
    }

    m_pendingPacket = nullptr;
    ScheduleNextPacket();
}

void
LeoSimTcpTrafficApplication::SendReady(Ptr<Socket> socket, uint32_t availableBytes)
{
    if (socket == m_socket && availableBytes > 0)
    {
        SendPendingPacket();
    }
}

void
LeoSimTcpTrafficApplication::ScheduleNextPacket()
{
    if (!m_running || m_sendEvent.IsPending())
    {
        return;
    }
    const Time interval = Seconds(static_cast<double>(m_packetSize) * 8.0 /
                                  static_cast<double>(m_dataRate.GetBitRate()));
    m_sendEvent = Simulator::Schedule(interval,
                                      &LeoSimTcpTrafficApplication::SendPacket,
                                      this);
}

void
LeoSimTcpTrafficApplication::ScheduleRetry()
{
    if (m_running && m_retryInterval.IsStrictlyPositive() && !m_retryEvent.IsPending())
    {
        m_retryEvent = Simulator::Schedule(m_retryInterval,
                                           &LeoSimTcpTrafficApplication::Connect,
                                           this);
    }
}

} // namespace ns3
