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

#ifndef LEOSIM_HELPER_H
#define LEOSIM_HELPER_H

#include "ns3/object.h"

#include <string>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Helper class for LeoSim module
 *
 * This helper provides convenience methods for setting up LEO satellite simulations.
 * Use specialized helpers like LeoSimMobilityHelper, LeoSimChannelHelper, etc.
 */
class LeoSimHelper
{
  public:
    /**
     * \brief Constructor
     */
    LeoSimHelper();

    /**
     * \brief Destructor
     */
    ~LeoSimHelper();

  private:
};

} // namespace ns3

#endif /* LEOSIM_HELPER_H */
