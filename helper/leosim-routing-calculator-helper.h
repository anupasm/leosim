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

#ifndef LEOSIM_ROUTING_CALCULATOR_HELPER_H
#define LEOSIM_ROUTING_CALCULATOR_HELPER_H

#include "ns3/ptr.h"
#include "ns3/object.h"
#include "ns3/node-container.h"
#include "ns3/nstime.h"

namespace ns3
{

class LeoSimChannelModel;
class LeoSimRoutingCalculator;
class EventId;

/**
 * \ingroup leosim
 * \brief Helper class for setting up and managing routing calculators
 *
 * This helper simplifies the creation and configuration of routing calculators
 * for LEO satellite networks. It handles the integration with channel models
 * and provides convenience methods for creating optimized routing calculators.
 */
class LeoSimRoutingCalculatorHelper
{
  public:
    /**
     * \brief Constructor
     */
    LeoSimRoutingCalculatorHelper();

    /**
     * \brief Destructor
     */
    ~LeoSimRoutingCalculatorHelper();

    /**
     * \brief Create a routing calculator attached to a channel model
     * \param channelModel The channel model containing link information
     * \return Pointer to the created routing calculator
     */
    Ptr<LeoSimRoutingCalculator> CreateRoutingCalculator(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Create a routing calculator with verbose output for debugging
     * \param channelModel The channel model containing link information
     * \param verbose Enable verbose logging
     * \return Pointer to the created routing calculator
     */
    Ptr<LeoSimRoutingCalculator> CreateRoutingCalculator(Ptr<LeoSimChannelModel> channelModel,
                                                         bool verbose);

    /**
     * \brief Create a unified routing calculator with both ground and ISL links
     * \param groundChannelModel The ground channel model (satellite-to-ground links)
     * \param islChannelModel The ISL channel model (inter-satellite links)
     * \param verbose Enable verbose logging
     * \return Pointer to the created unified routing calculator
     *
     * This creates a single calculator that can route through both ground and ISL links
     * in a unified network. This is used when all links are in the same network address space.
     */
    Ptr<LeoSimRoutingCalculator> CreateUnifiedRoutingCalculator(Ptr<LeoSimChannelModel> groundChannelModel,
                                                                 Ptr<LeoSimChannelModel> islChannelModel,
                                                                 bool verbose = false);

    /**
     * \brief Set computed routes into static routing tables of nodes
     * \param calculator The routing calculator to use for computing routes
     * \param sources Source nodes (UEs, servers, etc.)
     * \param destinations Destination nodes (satellites, servers, etc.)
     * \param verbose Enable verbose output
     * 
     * This method computes routes between all source-destination pairs and
     * sets them into the static routing tables of the nodes. Routes are
     * computed using the current topology from the channel model.
     */
    void SetStaticRoutes(Ptr<LeoSimRoutingCalculator> calculator,
                             const NodeContainer& sources,
                             const NodeContainer& destinations,
                             bool verbose = false);

    /**
     * \brief Enable dynamic periodic routing updates
     * \param calculator The routing calculator to use for computing routes
     * \param sources Source nodes (UEs, servers, etc.)
     * \param destinations Destination nodes (satellites, servers, etc.)
     * \param updateInterval Time interval between routing updates
     * \param stopTime Stop time for routing updates (0.0 means run until simulation ends)
     * \param verbose Enable verbose output
     *
     * This method enables periodic recalculation and installation of routes
     * based on the dynamic topology changes in the satellite network. Routes
     * are recomputed and updated at the specified interval, allowing the network
     * to adapt as satellites move and links appear/disappear.
     */
    void EnableDynamicRouting(Ptr<LeoSimRoutingCalculator> calculator,
                              const NodeContainer& sources,
                              const NodeContainer& destinations,
                              Time updateInterval,
                              double stopTime = 0.0,
                              bool verbose = false);

  private:
    /**
     * \brief Internal method to perform routing update and reschedule
     */
    void UpdateRoutesAndReschedule(Ptr<LeoSimRoutingCalculator> calculator,
                                   const NodeContainer& sources,
                                   const NodeContainer& destinations,
                                   Time updateInterval,
                                   double stopTime,
                                   bool verbose);
};

} // namespace ns3

#endif /* LEOSIM_ROUTING_CALCULATOR_HELPER_H */
