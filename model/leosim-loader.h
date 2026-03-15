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

#ifndef LEOSIM_LOADER_H
#define LEOSIM_LOADER_H

#include "ns3/mobility-model.h"
#include "ns3/node-container.h"
#include "ns3/object.h"
#include "ns3/vector.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Structure to hold satellite position data
 */
struct SatellitePosition
{
    uint32_t satId;          //!< Satellite ID
    std::string satName;     //!< Satellite name
    uint32_t timestep;       //!< Timestep number
    double time;             //!< Time in seconds
    Vector position;         //!< Position (x, y, z) in meters or (lat, lon, alt)
    bool isGeodetic;         //!< True if position is geodetic (lat, lon, alt)
};

/**
 * \ingroup leosim
 * \brief Structure to hold ground device data
 */
struct GroundDevice
{
    uint32_t deviceId;       //!< Device ID
    std::string deviceName;  //!< Device name
    std::string deviceType;  //!< Device type ("SERVER" or "UE")
    Vector position;         //!< Position (x, y, z) in meters or (lat, lon, alt)
    bool isGeodetic;         //!< True if position is geodetic (lat, lon, alt)
};

/**
 * \ingroup leosim
 * \brief LeoSim Loader for satellite and ground device data
 *
 * This class loads satellite position traces and ground device locations,
 * and applies them to ns-3 mobility models.
 */
class LeoSimLoader : public Object
{
  public:
    /**
     * \brief Get the type ID.
     * \return the object TypeId
     */
    static TypeId GetTypeId();

    /**
     * \brief Constructor
     */
    LeoSimLoader();

    /**
     * \brief Destructor
     */
    ~LeoSimLoader() override;

    /**
     * \brief Load satellite positions from CSV file
     * \param filename Path to CSV file
     * \return Number of satellites loaded
     */
    uint32_t LoadSatellitesFromCsv(const std::string& filename);

    /**
     * \brief Load satellite positions from ns-3 mobility trace file
     * \param filename Path to mobility trace file
     * \return Number of satellites loaded
     */
    uint32_t LoadSatellitesFromTrace(const std::string& filename);

    /**
     * \brief Load ground devices from CSV file
     * \param filename Path to CSV file
     * \param clearExisting Whether to clear any previously loaded devices
     * \return Number of ground devices loaded
     */
    uint32_t LoadGroundDevicesFromCsv(const std::string& filename, bool clearExisting = true);

    /**
     * \brief Load server devices from CSV file
     * \param filename Path to CSV file
     * \return Number of ground devices loaded
     */
    uint32_t LoadServersFromCsv(const std::string& filename);

    /**
     * \brief Load UE devices from CSV file
     * \param filename Path to CSV file
     * \return Number of ground devices loaded
     */
    uint32_t LoadUEsFromCsv(const std::string& filename);

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
     * \brief Get ground device type by ID
     * \param deviceId Device ID
     * \return Device type
     */
    std::string GetGroundDeviceType(uint32_t deviceId) const;

    /**
     * \brief Get ground device IDs matching a device type
     * \param deviceType Device type string
     * \return Vector of device IDs
     */
    std::vector<uint32_t> GetGroundDeviceIdsByType(const std::string& deviceType) const;

    /**
     * \brief Apply satellite mobility to nodes
     * \param nodes NodeContainer for satellites
     * \return True if successful
     */
    bool ApplySatelliteMobility(NodeContainer& nodes);

    /**
     * \brief Apply ground device positions to nodes
     * \param nodes NodeContainer for ground devices
     * \return True if successful
     */
    bool ApplyGroundDevicePositions(NodeContainer& nodes);

    /**
     * \brief Get all positions for a specific satellite
     * \param satId Satellite ID
     * \return Vector of positions
     */
    std::vector<SatellitePosition> GetSatellitePositions(uint32_t satId) const;

    /**
     * \brief Get ground device position
     * \param deviceId Device ID
     * \return Position vector
     */
    Vector GetGroundDevicePosition(uint32_t deviceId) const;

    /**
     * \brief Convert geodetic coordinates to Cartesian (ECEF)
     * \param latitude Latitude in degrees
     * \param longitude Longitude in degrees
     * \param altitude Altitude in meters
     * \return Cartesian position vector (x, y, z)
     */
    static Vector GeodeticToCartesian(double latitude, double longitude, double altitude);

    /**
     * \brief Convert Cartesian (ECEF) coordinates to geodetic
     * \param position Cartesian position (x, y, z)
     * \return Vector with (latitude, longitude, altitude)
     */
    static Vector CartesianToGeodetic(const Vector& position);

    /**
     * \brief Get satellite position at a specific simulation time with interpolation
     * \param satId Satellite identifier
     * \param t Simulation time
     * \return 3D Cartesian position at time t, interpolated from trace data.
     *         Returns first position if t is before first entry, extrapolates linearly
     *         if t is after last entry.
     */
    Vector GetSatellitePositionAt(uint32_t satId, Time t) const;

    /**
     * \brief Get orbit plane index for a satellite
     * \param satId Satellite identifier
     * \return Orbit plane index derived from satId / m_satellitesPerPlane
     */
    uint32_t GetOrbitPlane(uint32_t satId) const;

    /**
     * \brief Set the number of satellites per orbital plane
     * \param count Number of satellites per plane (default: 20)
     */
    void SetSatellitesPerPlane(uint32_t count);

    /**
     * \brief Get the number of satellites per orbital plane
     * \return Number of satellites per plane
     */
    uint32_t GetSatellitesPerPlane() const;

  private:
    /**
     * \brief Schedule position update for a satellite node
     * \param node Node pointer
     * \param satId Satellite ID
     */
    void ScheduleSatellitePositions(Ptr<Node> node, uint32_t satId);

    /**
     * \brief Update node position
     * \param node Node pointer
     * \param position New position
     */
    static void UpdateNodePosition(Ptr<Node> node, Vector position);

    /**
     * \brief Parse CSV line
     * \param line CSV line
     * \return Vector of parsed values
     */
    std::vector<std::string> ParseCsvLine(const std::string& line) const;

    /**
     * \brief Internal ground device CSV loader
     * \param filename Path to CSV file
     * \param clearExisting Whether to clear any previously loaded devices
     * \param defaultDeviceType Default device type when CSV lacks device_type column
     * \return Number of ground devices loaded
     */
    uint32_t LoadGroundDevicesFromCsvInternal(const std::string& filename,
                          bool clearExisting,
                          const std::string& defaultDeviceType);

    std::map<uint32_t, std::vector<SatellitePosition>> m_satelliteData; //!< Satellite position data
    std::map<uint32_t, std::string> m_satelliteNames;                   //!< Satellite names
    std::map<uint32_t, GroundDevice> m_groundDevices;                   //!< Ground device data
    uint32_t m_numSatellites;                                           //!< Number of satellites
    uint32_t m_numGroundDevices;                                        //!< Number of ground devices
    uint32_t m_satellitesPerPlane = 20;                                 //!< Satellites per orbital plane (default: 20)
};

} // namespace ns3

#endif /* LEOSIM_LOADER_H */
