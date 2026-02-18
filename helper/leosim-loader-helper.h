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

#ifndef LEOSIM_LOADER_HELPER_H
#define LEOSIM_LOADER_HELPER_H

#include "ns3/leosim-loader.h"
#include "ns3/node-container.h"

#include <string>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Helper class for loading satellite and ground device data
 *
 * This helper simplifies the process of loading satellite position traces
 * and ground device locations, creating nodes, and setting up mobility models.
 */
class LeoSimLoaderHelper
{
  public:
    /**
     * \brief Constructor
     */
    LeoSimLoaderHelper();

    /**
     * \brief Destructor
     */
    ~LeoSimLoaderHelper();

    /**
     * \brief Load satellite positions from CSV file
     * \param filename Path to CSV file
     * \return True if successful
     */
    bool LoadSatellitesFromCsv(const std::string& filename);

    /**
     * \brief Load satellite positions from ns-3 mobility trace file
     * \param filename Path to mobility trace file
     * \return True if successful
     */
    bool LoadSatellitesFromTrace(const std::string& filename);

    /**
     * \brief Load ground devices from CSV file
     * \param filename Path to CSV file
     * \return True if successful
     */
    bool LoadGroundDevicesFromCsv(const std::string& filename);

    /**
     * \brief Create satellite nodes with mobility
     * \return NodeContainer with satellite nodes
     */
    NodeContainer CreateSatelliteNodes();

    /**
     * \brief Create ground device nodes with positions
     * \return NodeContainer with ground device nodes
     */
    NodeContainer CreateGroundDeviceNodes();

    /**
     * \brief Apply satellite mobility to existing nodes
     * \param nodes NodeContainer to apply mobility to
     * \return True if successful
     */
    bool ApplySatelliteMobility(NodeContainer& nodes);

    /**
     * \brief Apply ground device positions to existing nodes
     * \param nodes NodeContainer to apply positions to
     * \return True if successful
     */
    bool ApplyGroundDevicePositions(NodeContainer& nodes);

    /**
     * \brief Get the number of satellites loaded
     * \return Number of satellites
     */
    uint32_t GetNumSatellites() const;

    /**
     * \brief Get the number of ground devices loaded
     * \return Number of ground devices
     */
    uint32_t GetNumGroundDevices() const;

    /**
     * \brief Get satellite name by ID
     * \param satId Satellite ID
     * \return Satellite name
     */
    std::string GetSatelliteName(uint32_t satId) const;

    /**
     * \brief Get ground device name by ID
     * \param deviceId Device ID
     * \return Device name
     */
    std::string GetGroundDeviceName(uint32_t deviceId) const;

    /**
     * \brief Get the loader object
     * \return Pointer to LeoSimLoader
     */
    Ptr<LeoSimLoader> GetLoader() const;

    /**
     * \brief Set whether to enable verbose logging
     * \param verbose True to enable verbose logging
     */
    void SetVerbose(bool verbose);

  private:
    Ptr<LeoSimLoader> m_loader; //!< LeoSim loader object
    bool m_verbose;             //!< Verbose logging flag
};

} // namespace ns3

#endif /* LEOSIM_LOADER_HELPER_H */
