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

#include "leosim-isl-routing-model.h"
#include "ns3/log.h"
#include "ns3/assert.h"

#include <algorithm>
#include <limits>
#include <queue>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimISLRoutingModel");
NS_OBJECT_ENSURE_REGISTERED(LeoSimISLRoutingModel);

TypeId
LeoSimISLRoutingModel::GetTypeId()
{
    static TypeId tid =
        TypeId("ns3::LeoSimISLRoutingModel")
            .SetParent<Object>()
            .SetGroupName("LeoSim")
            .AddConstructor<LeoSimISLRoutingModel>();
    return tid;
}

LeoSimISLRoutingModel::LeoSimISLRoutingModel()
{
    NS_LOG_FUNCTION(this);
}

LeoSimISLRoutingModel::~LeoSimISLRoutingModel()
{
    NS_LOG_FUNCTION(this);
    ClearTopology();
}

void
LeoSimISLRoutingModel::UpdateIslLink(Ptr<Node> sat1, Ptr<Node> sat2, bool isConnected)
{
    NS_LOG_FUNCTION(this << sat1->GetId() << sat2->GetId() << isConnected);

    if (!sat1 || !sat2)
    {
        NS_LOG_ERROR("Invalid node pointers");
        return;
    }

    if (isConnected)
    {
        // Add bidirectional link
        islTopology_[sat1].insert(sat2);
        islTopology_[sat2].insert(sat1);
        NS_LOG_DEBUG("ISL link added: " << sat1->GetId() << " <-> " << sat2->GetId());
    }
    else
    {
        // Remove bidirectional link
        islTopology_[sat1].erase(sat2);
        islTopology_[sat2].erase(sat1);
        NS_LOG_DEBUG("ISL link removed: " << sat1->GetId() << " <-> " << sat2->GetId());
    }
}

std::vector<Ptr<Node>>
LeoSimISLRoutingModel::ComputePath(Ptr<Node> source, Ptr<Node> destination)
{
    NS_LOG_FUNCTION(this << source->GetId() << destination->GetId());

    if (!source || !destination)
    {
        NS_LOG_ERROR("Invalid node pointers");
        return std::vector<Ptr<Node>>();
    }

    if (source == destination)
    {
        NS_LOG_DEBUG("Source and destination are the same");
        return std::vector<Ptr<Node>>{source};
    }

    std::map<Ptr<Node>, uint32_t> distance;
    std::map<Ptr<Node>, Ptr<Node>> previous;

    DijkstrasAlgorithm(source, destination, distance, previous);

    // Reconstruct path
    std::vector<Ptr<Node>> path;
    Ptr<Node> current = destination;

    if (previous.find(destination) == previous.end() && destination != source)
    {
        // No path exists
        NS_LOG_WARN("No path exists from node " << source->GetId() << " to node "
                                                 << destination->GetId());
        return std::vector<Ptr<Node>>();
    }

    while (current != nullptr)
    {
        path.insert(path.begin(), current);
        if (current == source)
            break;
        current = previous[current];
    }

    NS_LOG_DEBUG("Path found with " << path.size() << " hops");
    return path;
}

std::set<Ptr<Node>>
LeoSimISLRoutingModel::GetIslNeighbors(Ptr<Node> satellite) const
{
    NS_LOG_FUNCTION(this << satellite->GetId());

    auto it = islTopology_.find(satellite);
    if (it != islTopology_.end())
    {
        return it->second;
    }
    return std::set<Ptr<Node>>();
}

bool
LeoSimISLRoutingModel::IsDirectlyConnected(Ptr<Node> sat1, Ptr<Node> sat2) const
{
    NS_LOG_FUNCTION(this << sat1->GetId() << sat2->GetId());

    auto neighbors = GetIslNeighbors(sat1);
    return neighbors.find(sat2) != neighbors.end();
}

std::map<Ptr<Node>, std::set<Ptr<Node>>>
LeoSimISLRoutingModel::GetTopology() const
{
    NS_LOG_FUNCTION(this);
    return islTopology_;
}

uint32_t
LeoSimISLRoutingModel::GetNumActiveLinks() const
{
    NS_LOG_FUNCTION(this);

    uint32_t numLinks = 0;
    for (const auto& entry : islTopology_)
    {
        numLinks += entry.second.size();
    }
    // Each link is counted twice (bidirectional), so divide by 2
    return numLinks / 2;
}

void
LeoSimISLRoutingModel::ClearTopology()
{
    NS_LOG_FUNCTION(this);
    islTopology_.clear();
}

void
LeoSimISLRoutingModel::DijkstrasAlgorithm(Ptr<Node> source,
                                           Ptr<Node> destination,
                                           std::map<Ptr<Node>, uint32_t>& distance,
                                           std::map<Ptr<Node>, Ptr<Node>>& previous)
{
    NS_LOG_FUNCTION(this);

    // Initialize distances
    for (const auto& node : islTopology_)
    {
        distance[node.first] = std::numeric_limits<uint32_t>::max();
    }
    distance[source] = 0;

    // Priority queue: (distance, node)
    std::priority_queue<std::pair<uint32_t, Ptr<Node>>,
                        std::vector<std::pair<uint32_t, Ptr<Node>>>,
                        std::greater<std::pair<uint32_t, Ptr<Node>>>>
        pq;

    pq.push({0, source});

    while (!pq.empty())
    {
        auto [currentDistance, current] = pq.top();
        pq.pop();

        if (currentDistance > distance[current])
        {
            continue;
        }

        // Early exit if we reached destination
        if (current == destination)
        {
            break;
        }

        // Explore neighbors
        auto neighbors = GetIslNeighbors(current);
        for (Ptr<Node> neighbor : neighbors)
        {
            uint32_t newDistance = distance[current] + 1;
            if (newDistance < distance[neighbor])
            {
                distance[neighbor] = newDistance;
                previous[neighbor] = current;
                pq.push({newDistance, neighbor});
            }
        }
    }
}

} // namespace ns3
