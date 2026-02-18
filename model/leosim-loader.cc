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

#include "leosim-loader.h"

#include "ns3/constant-position-mobility-model.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimLoader");

NS_OBJECT_ENSURE_REGISTERED(LeoSimLoader);

// WGS84 ellipsoid constants
static const double WGS84_A = 6378137.0;               // Semi-major axis (m)
static const double WGS84_B = 6356752.314245;          // Semi-minor axis (m)
static const double WGS84_E2 = 0.00669437999014;       // First eccentricity squared

TypeId
LeoSimLoader::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimLoader")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimLoader>();
    return tid;
}

LeoSimLoader::LeoSimLoader()
    : m_numSatellites(0),
      m_numGroundDevices(0)
{
    NS_LOG_FUNCTION(this);
}

LeoSimLoader::~LeoSimLoader()
{
    NS_LOG_FUNCTION(this);
}

std::vector<std::string>
LeoSimLoader::ParseCsvLine(const std::string& line) const
{
    std::vector<std::string> result;
    std::stringstream ss(line);
    std::string item;

    while (std::getline(ss, item, ','))
    {
        // Trim whitespace
        size_t start = item.find_first_not_of(" \t\r\n");
        size_t end = item.find_last_not_of(" \t\r\n");
        if (start != std::string::npos && end != std::string::npos)
        {
            result.push_back(item.substr(start, end - start + 1));
        }
        else
        {
            result.push_back("");
        }
    }

    return result;
}

uint32_t
LeoSimLoader::LoadSatellitesFromCsv(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);

    std::ifstream file(filename);
    if (!file.is_open())
    {
        NS_LOG_ERROR("Failed to open file: " << filename);
        return 0;
    }

    std::string line;
    // Skip header
    std::getline(file, line);
    
    // Check if file contains geodetic or cartesian data
    bool isGeodetic = (line.find("latitude") != std::string::npos);

    m_satelliteData.clear();
    m_satelliteNames.clear();

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        auto fields = ParseCsvLine(line);
        
        if (isGeodetic && fields.size() >= 7)
        {
            // Format: sat_id, sat_name, timestep, time_s, latitude_deg, longitude_deg, altitude_m
            SatellitePosition pos;
            pos.satId = std::stoul(fields[0]);
            pos.satName = fields[1];
            pos.timestep = std::stoul(fields[2]);
            pos.time = std::stod(fields[3]);
            pos.isGeodetic = true;
            
            double lat = std::stod(fields[4]);
            double lon = std::stod(fields[5]);
            double alt = std::stod(fields[6]);
            
            // Convert to Cartesian
            pos.position = GeodeticToCartesian(lat, lon, alt);
            
            m_satelliteData[pos.satId].push_back(pos);
            m_satelliteNames[pos.satId] = pos.satName;
        }
        else if (!isGeodetic && fields.size() >= 7)
        {
            // Format: sat_id, sat_name, timestep, time_s, x_m, y_m, z_m
            SatellitePosition pos;
            pos.satId = std::stoul(fields[0]);
            pos.satName = fields[1];
            pos.timestep = std::stoul(fields[2]);
            pos.time = std::stod(fields[3]);
            pos.position.x = std::stod(fields[4]);
            pos.position.y = std::stod(fields[5]);
            pos.position.z = std::stod(fields[6]);
            pos.isGeodetic = false;
            
            m_satelliteData[pos.satId].push_back(pos);
            m_satelliteNames[pos.satId] = pos.satName;
        }
    }

    file.close();

    m_numSatellites = m_satelliteData.size();
    NS_LOG_INFO("Loaded " << m_numSatellites << " satellites from " << filename);

    return m_numSatellites;
}

uint32_t
LeoSimLoader::LoadSatellitesFromTrace(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);

    std::ifstream file(filename);
    if (!file.is_open())
    {
        NS_LOG_ERROR("Failed to open file: " << filename);
        return 0;
    }

    m_satelliteData.clear();
    m_satelliteNames.clear();

    std::string line;
    std::map<uint32_t, SatellitePosition> currentPositions;

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        // Parse format: $ns_ at <time> "$node_(<id>) set X_ <value>"
        std::stringstream ss(line);
        std::string token;
        double time;
        uint32_t nodeId;
        char coord;
        double value;

        ss >> token; // $ns_
        if (token != "$ns_")
            continue;

        ss >> token; // at
        ss >> time;  // time value

        std::string nodeStr;
        std::getline(ss, nodeStr, '"'); // Skip to first quote
        std::getline(ss, nodeStr, '"'); // Get content between quotes

        std::stringstream nodeSs(nodeStr);
        nodeSs >> token; // $node_(
        
        // Extract node ID
        size_t start = nodeStr.find('(');
        size_t end = nodeStr.find(')');
        if (start != std::string::npos && end != std::string::npos)
        {
            nodeId = std::stoul(nodeStr.substr(start + 1, end - start - 1));
            
            // Always update time for the current line
            if (!currentPositions.count(nodeId))
            {
                currentPositions[nodeId].satId = nodeId;
                currentPositions[nodeId].satName = "SAT_" + std::to_string(nodeId);
                currentPositions[nodeId].timestep = 0;
                currentPositions[nodeId].isGeodetic = false;
            }
            // Update time for every line (important!)
            currentPositions[nodeId].time = time;
            
            // Extract coordinate and value
            size_t coordPos = nodeStr.find("set");
            if (coordPos != std::string::npos)
            {
                std::string coordStr = nodeStr.substr(coordPos + 4);
                std::stringstream coordSs(coordStr);
                coordSs >> coord; // X_, Y_, or Z_
                coordSs >> token; // _
                coordSs >> value;

                if (coord == 'X')
                    currentPositions[nodeId].position.x = value;
                else if (coord == 'Y')
                    currentPositions[nodeId].position.y = value;
                else if (coord == 'Z')
                {
                    currentPositions[nodeId].position.z = value;
                    // Z is the last coordinate, so save this position
                    NS_LOG_DEBUG("Parsed position for node " << nodeId << " at time " << time << "s: ("
                                                            << currentPositions[nodeId].position.x << ", "
                                                            << currentPositions[nodeId].position.y << ", "
                                                            << currentPositions[nodeId].position.z << ")");
                    m_satelliteData[nodeId].push_back(currentPositions[nodeId]);
                    m_satelliteNames[nodeId] = currentPositions[nodeId].satName;
                    currentPositions[nodeId].timestep++;
                }
            }
        }
    }

    file.close();

    // Optional: load satellite names from a sidecar CSV file
    // Expected format: sat_id,sat_name
    std::string namesFile = filename + ".names.csv";
    std::ifstream namesStream(namesFile);
    if (namesStream.is_open())
    {
        std::string namesLine;
        // Skip header if present
        if (std::getline(namesStream, namesLine))
        {
            auto header = ParseCsvLine(namesLine);
            if (header.size() >= 2 && header[0].find("sat_id") == std::string::npos)
            {
                auto fields = ParseCsvLine(namesLine);
                if (fields.size() >= 2)
                {
                    uint32_t satId = std::stoul(fields[0]);
                    m_satelliteNames[satId] = fields[1];
                }
            }
        }

        while (std::getline(namesStream, namesLine))
        {
            if (namesLine.empty())
            {
                continue;
            }
            auto fields = ParseCsvLine(namesLine);
            if (fields.size() < 2)
            {
                continue;
            }
            uint32_t satId = std::stoul(fields[0]);
            m_satelliteNames[satId] = fields[1];
        }
        namesStream.close();
        NS_LOG_INFO("Loaded satellite names from " << namesFile);
    }

    m_numSatellites = m_satelliteData.size();
    NS_LOG_INFO("Loaded " << m_numSatellites << " satellites from trace file " << filename);

    return m_numSatellites;
}

uint32_t
LeoSimLoader::LoadGroundDevicesFromCsv(const std::string& filename, bool clearExisting)
{
    return LoadGroundDevicesFromCsvInternal(filename, clearExisting, "");
}

uint32_t
LeoSimLoader::LoadServersFromCsv(const std::string& filename)
{
    return LoadGroundDevicesFromCsvInternal(filename, false, "SERVER");
}

uint32_t
LeoSimLoader::LoadUEsFromCsv(const std::string& filename)
{
    return LoadGroundDevicesFromCsvInternal(filename, false, "UE");
}

uint32_t
LeoSimLoader::LoadGroundDevicesFromCsvInternal(const std::string& filename,
                                                bool clearExisting,
                                                const std::string& defaultDeviceType)
{
    NS_LOG_FUNCTION(this << filename);

    std::ifstream file(filename);
    if (!file.is_open())
    {
        NS_LOG_ERROR("Failed to open file: " << filename);
        return 0;
    }

    std::string line;
    // Read header
    if (!std::getline(file, line))
    {
        NS_LOG_ERROR("Failed to read header from file: " << filename);
        return 0;
    }

    auto headerFields = ParseCsvLine(line);
    std::vector<std::string> headerNormalized;
    headerNormalized.reserve(headerFields.size());
    for (const auto& field : headerFields)
    {
        std::string normalized = field;
        std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        headerNormalized.push_back(normalized);
    }

    auto findIndex = [&](const std::vector<std::string>& names) -> int {
        for (size_t i = 0; i < headerNormalized.size(); ++i)
        {
            for (const auto& name : names)
            {
                if (headerNormalized[i] == name)
                {
                    return static_cast<int>(i);
                }
            }
        }
        return -1;
    };

    int deviceIdIndex = findIndex({"device_id", "deviceid", "id"});
    int deviceNameIndex = findIndex({"device_name", "devicename", "name"});
    int deviceTypeIndex = findIndex({"device_type", "devicetype", "type"});
    int latitudeIndex = findIndex({"latitude_deg", "latitude", "lat"});
    int longitudeIndex = findIndex({"longitude_deg", "longitude", "lon", "lng"});
    int altitudeIndex = findIndex({"altitude_m", "altitude", "alt"});
    int xIndex = findIndex({"x_m", "x"});
    int yIndex = findIndex({"y_m", "y"});
    int zIndex = findIndex({"z_m", "z"});

    bool isGeodetic = (latitudeIndex >= 0 && longitudeIndex >= 0);

    if (clearExisting)
    {
        m_groundDevices.clear();
    }

    while (std::getline(file, line))
    {
        if (line.empty())
            continue;

        auto fields = ParseCsvLine(line);
        if (fields.empty())
            continue;

        GroundDevice device;
        device.deviceType = defaultDeviceType;

        size_t idIndex = (deviceIdIndex >= 0) ? static_cast<size_t>(deviceIdIndex) : 0;
        size_t nameIndex = (deviceNameIndex >= 0) ? static_cast<size_t>(deviceNameIndex) : 1;

        if (fields.size() <= idIndex)
            continue;

        device.deviceId = std::stoul(fields[idIndex]);
        if (fields.size() > nameIndex)
        {
            device.deviceName = fields[nameIndex];
        }

        if (deviceTypeIndex >= 0 && fields.size() > static_cast<size_t>(deviceTypeIndex))
        {
            const auto& typeValue = fields[deviceTypeIndex];
            if (!typeValue.empty())
            {
                device.deviceType = typeValue;
            }
        }

        if (isGeodetic)
        {
            size_t latIndex = (latitudeIndex >= 0) ? static_cast<size_t>(latitudeIndex) : 2;
            size_t lonIndex = (longitudeIndex >= 0) ? static_cast<size_t>(longitudeIndex) : 3;
            size_t altIndex = (altitudeIndex >= 0) ? static_cast<size_t>(altitudeIndex) : 4;
            size_t maxIndex = std::max(latIndex, std::max(lonIndex, altIndex));

            if (fields.size() <= maxIndex)
                continue;

            device.isGeodetic = true;

            double lat = std::stod(fields[latIndex]);
            double lon = std::stod(fields[lonIndex]);
            double alt = std::stod(fields[altIndex]);

            // Convert to Cartesian
            device.position = GeodeticToCartesian(lat, lon, alt);
        }
        else
        {
            size_t xPosIndex = (xIndex >= 0) ? static_cast<size_t>(xIndex) : 2;
            size_t yPosIndex = (yIndex >= 0) ? static_cast<size_t>(yIndex) : 3;
            size_t zPosIndex = (zIndex >= 0) ? static_cast<size_t>(zIndex) : 4;
            size_t maxIndex = std::max(xPosIndex, std::max(yPosIndex, zPosIndex));

            if (fields.size() <= maxIndex)
                continue;

            device.position.x = std::stod(fields[xPosIndex]);
            device.position.y = std::stod(fields[yPosIndex]);
            device.position.z = std::stod(fields[zPosIndex]);
            device.isGeodetic = false;
        }

        m_groundDevices[device.deviceId] = device;
    }

    file.close();

    m_numGroundDevices = m_groundDevices.size();
    NS_LOG_INFO("Loaded " << m_numGroundDevices << " ground devices from " << filename);

    return m_numGroundDevices;
}

uint32_t
LeoSimLoader::GetNumSatellites() const
{
    return m_numSatellites;
}

uint32_t
LeoSimLoader::GetNumGroundDevices() const
{
    return m_numGroundDevices;
}

std::string
LeoSimLoader::GetSatelliteName(uint32_t satId) const
{
    auto it = m_satelliteNames.find(satId);
    if (it != m_satelliteNames.end())
    {
        return it->second;
    }
    return "UNKNOWN";
}

std::string
LeoSimLoader::GetGroundDeviceName(uint32_t deviceId) const
{
    auto it = m_groundDevices.find(deviceId);
    if (it != m_groundDevices.end())
    {
        return it->second.deviceName;
    }
    return "UNKNOWN";
}

std::string
LeoSimLoader::GetGroundDeviceType(uint32_t deviceId) const
{
    auto it = m_groundDevices.find(deviceId);
    if (it != m_groundDevices.end())
    {
        return it->second.deviceType;
    }
    return "";
}

std::vector<uint32_t>
LeoSimLoader::GetGroundDeviceIdsByType(const std::string& deviceType) const
{
    std::vector<uint32_t> deviceIds;

    for (const auto& entry : m_groundDevices)
    {
        if (entry.second.deviceType == deviceType)
        {
            deviceIds.push_back(entry.first);
        }
    }

    return deviceIds;
}

bool
LeoSimLoader::ApplySatelliteMobility(NodeContainer& nodes)
{
    NS_LOG_FUNCTION(this);

    if (nodes.GetN() != m_numSatellites)
    {
        NS_LOG_WARN("Number of nodes (" << nodes.GetN() << ") does not match number of satellites ("
                                         << m_numSatellites << ")");
    }

    for (uint32_t i = 0; i < std::min(nodes.GetN(), m_numSatellites); ++i)
    {
        Ptr<Node> node = nodes.Get(i);

        std::string satName = GetSatelliteName(i);
        if (satName.empty() || satName == "UNKNOWN")
        {
            satName = "SAT_" + std::to_string(i);
        }

        if (Names::FindName(node).empty())
        {
            std::string uniqueName = satName;
            if (Names::Find<Object>(uniqueName))
            {
                uniqueName = satName + "_" + std::to_string(i);
            }
            Names::Add(uniqueName, node);
        }
        
        // Install mobility model if not already present
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        if (!mobility)
        {
            Ptr<ConstantPositionMobilityModel> constMobility =
                CreateObject<ConstantPositionMobilityModel>();
            node->AggregateObject(constMobility);
            mobility = constMobility;
        }

        // Set initial position
        if (m_satelliteData.count(i) && !m_satelliteData[i].empty())
        {
            mobility->SetPosition(m_satelliteData[i][0].position);
            NS_LOG_INFO("Set initial position for satellite " << i << " (" << GetSatelliteName(i)
                                                               << "): " << m_satelliteData[i][0].position);
        }

        // Schedule position updates
        ScheduleSatellitePositions(node, i);
    }

    return true;
}

bool
LeoSimLoader::ApplyGroundDevicePositions(NodeContainer& nodes)
{
    NS_LOG_FUNCTION(this);

    if (nodes.GetN() != m_numGroundDevices)
    {
        NS_LOG_WARN("Number of nodes (" << nodes.GetN()
                                         << ") does not match number of ground devices ("
                                         << m_numGroundDevices << ")");
    }

    uint32_t deviceId = 0;
    for (auto it = m_groundDevices.begin(); it != m_groundDevices.end(); ++it)
    {
        if (deviceId >= nodes.GetN())
            break;

        Ptr<Node> node = nodes.Get(deviceId);
        
        // Install mobility model if not already present
        Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
        if (!mobility)
        {
            Ptr<ConstantPositionMobilityModel> constMobility =
                CreateObject<ConstantPositionMobilityModel>();
            node->AggregateObject(constMobility);
            mobility = constMobility;
        }

        // Set position
        mobility->SetPosition(it->second.position);
        NS_LOG_INFO("Set position for ground device " << deviceId << " (" << it->second.deviceName
                                                       << "): " << it->second.position);

        deviceId++;
    }

    return true;
}

std::vector<SatellitePosition>
LeoSimLoader::GetSatellitePositions(uint32_t satId) const
{
    auto it = m_satelliteData.find(satId);
    if (it != m_satelliteData.end())
    {
        return it->second;
    }
    return std::vector<SatellitePosition>();
}

Vector
LeoSimLoader::GetGroundDevicePosition(uint32_t deviceId) const
{
    auto it = m_groundDevices.find(deviceId);
    if (it != m_groundDevices.end())
    {
        return it->second.position;
    }
    return Vector(0, 0, 0);
}

void
LeoSimLoader::ScheduleSatellitePositions(Ptr<Node> node, uint32_t satId)
{
    NS_LOG_FUNCTION(this << node << satId);

    auto it = m_satelliteData.find(satId);
    if (it == m_satelliteData.end())
    {
        NS_LOG_WARN("No position data for satellite " << satId);
        return;
    }

    // Schedule all position updates
    for (const auto& pos : it->second)
    {
        Simulator::Schedule(Seconds(pos.time),
                            &LeoSimLoader::UpdateNodePosition,
                            node,
                            pos.position);
    }

    NS_LOG_INFO("Scheduled " << it->second.size() << " position updates for satellite " << satId);
}

void
LeoSimLoader::UpdateNodePosition(Ptr<Node> node, Vector position)
{
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel>();
    if (mobility)
    {
        mobility->SetPosition(position);
    }
}

Vector
LeoSimLoader::GeodeticToCartesian(double latitude, double longitude, double altitude)
{
    // Convert degrees to radians
    double lat = latitude * M_PI / 180.0;
    double lon = longitude * M_PI / 180.0;

    // Calculate radius of curvature in prime vertical
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));

    // Calculate Cartesian coordinates
    double x = (N + altitude) * std::cos(lat) * std::cos(lon);
    double y = (N + altitude) * std::cos(lat) * std::sin(lon);
    double z = (N * (1.0 - WGS84_E2) + altitude) * std::sin(lat);

    return Vector(x, y, z);
}

Vector
LeoSimLoader::CartesianToGeodetic(const Vector& position)
{
    double x = position.x;
    double y = position.y;
    double z = position.z;

    // Calculate longitude
    double lon = std::atan2(y, x);

    // Calculate latitude using iterative method
    double p = std::sqrt(x * x + y * y);
    double lat = std::atan2(z, p * (1.0 - WGS84_E2));
    
    double lat_prev;
    double N;
    int iterations = 0;
    const int MAX_ITERATIONS = 10;
    const double CONVERGENCE = 1e-12;

    do
    {
        lat_prev = lat;
        N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat) * std::sin(lat));
        lat = std::atan2(z + WGS84_E2 * N * std::sin(lat), p);
        iterations++;
    } while (std::abs(lat - lat_prev) > CONVERGENCE && iterations < MAX_ITERATIONS);

    // Calculate altitude
    double alt = p / std::cos(lat) - N;

    // Convert to degrees
    lat = lat * 180.0 / M_PI;
    lon = lon * 180.0 / M_PI;

    return Vector(lat, lon, alt);
}

} // namespace ns3
