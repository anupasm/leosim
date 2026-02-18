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

#ifndef LEOSIM_ISL_ROUTING_MODEL_H
#define LEOSIM_ISL_ROUTING_MODEL_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"

#include <map>
#include <set>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief ISL routing model for dynamic LEO satellite topology
 *
 * This model maintains a graph of Inter-Satellite Links (ISLs) and computes
 * paths between satellites considering the dynamic topology. It uses Dijkstra's
 * algorithm to find shortest paths based on hop count.
 */
class LeoSimISLRoutingModel : public Object
{
  public:
    /**
     * \brief Get the type ID
     * \return The TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief Constructor
     */
    LeoSimISLRoutingModel();

    /**
     * \brief Destructor
     */
    ~LeoSimISLRoutingModel();

    /**
     * \brief Update ISL topology when a link changes state
     * \param sat1 First satellite node
     * \param sat2 Second satellite node
     * \param isConnected True if link is UP, False if DOWN
     */
    void UpdateIslLink(Ptr<Node> sat1, Ptr<Node> sat2, bool isConnected);

    /**
     * \brief Compute shortest path between two nodes using Dijkstra's algorithm
     * \param source Source satellite node
     * \param destination Destination satellite node
     * \return Vector of nodes representing the path (empty if no path exists)
     */
    std::vector<Ptr<Node>> ComputePath(Ptr<Node> source, Ptr<Node> destination);

    /**
     * \brief Get all ISL neighbors of a satellite
     * \param satellite Satellite node
     * \return Set of neighbor satellite nodes connected via ISL
     */
    std::set<Ptr<Node>> GetIslNeighbors(Ptr<Node> satellite) const;

    /**
     * \brief Check if two satellites are directly connected via ISL
     * \param sat1 First satellite node
     * \param sat2 Second satellite node
     * \return True if directly connected, False otherwise
     */
    bool IsDirectlyConnected(Ptr<Node> sat1, Ptr<Node> sat2) const;

    /**
     * \brief Get the current ISL topology as an adjacency list
     * \return Map of satellite node to set of neighbor nodes
     */
    std::map<Ptr<Node>, std::set<Ptr<Node>>> GetTopology() const;

    /**
     * \brief Get number of active ISL links
     * \return Number of active ISL connections
     */
    uint32_t GetNumActiveLinks() const;

    /**
     * \brief Clear all topology information
     */
    void ClearTopology();

  private:
    /**
     * \brief Internal adjacency list representation of ISL topology
     *
     * Maps each satellite node to a set of directly connected neighbor nodes
     */
    std::map<Ptr<Node>, std::set<Ptr<Node>>> islTopology_;

    /**
     * \brief Helper function for Dijkstra's algorithm
     * \param source Source node
     * \param destination Destination node
     * \param distance Output map of node distances
     * \param previous Output map for path reconstruction
     */
    void DijkstrasAlgorithm(Ptr<Node> source,
                             Ptr<Node> destination,
                             std::map<Ptr<Node>, uint32_t>& distance,
                             std::map<Ptr<Node>, Ptr<Node>>& previous);
};

} // namespace ns3

#endif /* LEOSIM_ISL_ROUTING_MODEL_H */
