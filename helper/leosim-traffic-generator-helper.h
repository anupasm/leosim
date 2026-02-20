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

#ifndef LEOSIM_TRAFFIC_GENERATOR_HELPER_H
#define LEOSIM_TRAFFIC_GENERATOR_HELPER_H

#include "ns3/application-container.h"
#include "ns3/ipv4-address.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/nstime.h"

#include <string>
#include <vector>

namespace ns3
{

class Node;

/**
 * \ingroup leosim
 * \brief Helper for generating traffic between nodes in LeoSim simulations
 *
 * This helper provides convenient methods for installing traffic generation applications
 * between nodes, such as ping (ICMPv4 echo) traffic for testing connectivity and latency.
 * It supports flexible configuration of traffic parameters and can be extended to support
 * other traffic types (UDP, TCP, etc.).
 */
class LeoSimTrafficGeneratorHelper
{
  public:
    /**
     * \brief Constructor
     */
    LeoSimTrafficGeneratorHelper();

    /**
     * \brief Destructor
     */
    ~LeoSimTrafficGeneratorHelper();

    /**
     * \brief Set the ping interval (time between consecutive ping packets)
     * \param interval Time between pings
     */
    void SetPingInterval(Time interval);

    /**
     * \brief Get the current ping interval
     * \return Time between pings
     */
    Time GetPingInterval() const;

    /**
     * \brief Set the data size for ping packets
     * \param size Size of ping payload in bytes
     */
    void SetPingDataSize(uint32_t size);

    /**
     * \brief Get the current ping data size
     * \return Size of ping payload in bytes
     */
    uint32_t GetPingDataSize() const;

    /**
     * \brief Set the verbose flag for detailed logging
     * \param verbose Enable/disable verbose output
     */
    void SetVerbose(bool verbose);

    /**
     * \brief Install a ping client on a node targeting a specific IP address
     * \param sourceNode The node that will run the ping client
     * \param destAddress The IP address to ping
     * \param startTime Time at which the client starts (relative to simulation start)
     * \param stopTime Time at which the client stops
     * \return ApplicationContainer containing the installed application
     */
    ApplicationContainer InstallPingClient(Ptr<Node> sourceNode,
                                           Ipv4Address destAddress,
                                           Time startTime,
                                           Time stopTime);

    /**
     * \brief Install ping clients from multiple source nodes to a destination address
     * \param sourceNodes Container of nodes that will run ping clients
     * \param destAddress The IP address to ping
     * \param startTime Time at which clients start
     * \param stopTime Time at which clients stop
     * \return ApplicationContainer containing all installed clients
     */
    ApplicationContainer InstallPingClients(NodeContainer sourceNodes,
                                            Ipv4Address destAddress,
                                            Time startTime,
                                            Time stopTime);

    /**
     * \brief Install bidirectional ping between two nodes
     *
     * Creates ping traffic flowing in both directions between the two nodes.
     *
     * \param node1 First node (will ping node2)
     * \param node2 Second node (will ping node1)
     * \param addr1 IP address of node1
     * \param addr2 IP address of node2
     * \param startTime Time at which pings start
     * \param stopTime Time at which pings stop
     * \return ApplicationContainer containing all installed applications
     */
    ApplicationContainer InstallBidirectionalPing(Ptr<Node> node1,
                                                  Ptr<Node> node2,
                                                  Ipv4Address addr1,
                                                  Ipv4Address addr2,
                                                  Time startTime,
                                                  Time stopTime);

    /**
     * \brief Install ping traffic from UEs to a server
     *
     * Creates ping clients on all UE nodes targeting the server's IP address.
     *
     * \param ueNodes Container of UE nodes
     * \param serverNode The server node
     * \param serverAddress IP address of the server
     * \param startTime Time at which pings start
     * \param stopTime Time at which pings stop
     * \return ApplicationContainer containing all installed ping clients
     */
    ApplicationContainer InstallUeToServerPing(NodeContainer ueNodes,
                                               Ptr<Node> serverNode,
                                               Ipv4Address serverAddress,
                                               Time startTime,
                                               Time stopTime);

    /**
     * \brief Install bidirectional ping traffic between all UEs and a server
     *
     * Creates bidirectional ping traffic between each UE and the server.
     *
     * \param ueNodes Container of UE nodes
     * \param serverNode The server node
     * \param ueAddresses Vector of IP addresses for UE nodes (must match ueNodes size)
     * \param serverAddress IP address of the server
     * \param startTime Time at which pings start
     * \param stopTime Time at which pings stop
     * \return ApplicationContainer containing all installed applications
     */
    ApplicationContainer InstallBidirectionalUeToServerPing(NodeContainer ueNodes,
                                                            Ptr<Node> serverNode,
                                                            std::vector<Ipv4Address> ueAddresses,
                                                            Ipv4Address serverAddress,
                                                            Time startTime,
                                                            Time stopTime);

  private:
    Time m_pingInterval;       //!< Interval between ping packets
    uint32_t m_pingDataSize;   //!< Data size in ping payload
    bool m_verbose;            //!< Verbose output flag
};

} // namespace ns3

#endif /* LEOSIM_TRAFFIC_GENERATOR_HELPER_H */
