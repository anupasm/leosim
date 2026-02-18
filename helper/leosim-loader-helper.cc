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

#include "leosim-loader-helper.h"

#include "ns3/leosim-loader.h"
#include "ns3/log.h"
#include "ns3/names.h"
#include "ns3/node.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimLoaderHelper");

LeoSimLoaderHelper::LeoSimLoaderHelper()
    : m_verbose(false)
{
    m_loader = CreateObject<LeoSimLoader>();
}

LeoSimLoaderHelper::~LeoSimLoaderHelper()
{
}

bool
LeoSimLoaderHelper::LoadSatellitesFromCsv(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);

    uint32_t numLoaded = m_loader->LoadSatellitesFromCsv(filename);
    
    if (m_verbose)
    {
        std::cout << "Loaded " << numLoaded << " satellites from " << filename << std::endl;
        for (uint32_t i = 0; i < numLoaded; ++i)
        {
            std::cout << "  Satellite " << i << ": " << m_loader->GetSatelliteName(i) << std::endl;
        }
    }

    return numLoaded > 0;
}

bool
LeoSimLoaderHelper::LoadSatellitesFromTrace(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);

    uint32_t numLoaded = m_loader->LoadSatellitesFromTrace(filename);
    
    if (m_verbose)
    {
        std::cout << "Loaded " << numLoaded << " satellites from trace " << filename << std::endl;
        for (uint32_t i = 0; i < numLoaded; ++i)
        {
            std::cout << "  Satellite " << i << ": " << m_loader->GetSatelliteName(i) << std::endl;
        }
    }

    return numLoaded > 0;
}

bool
LeoSimLoaderHelper::LoadGroundDevicesFromCsv(const std::string& filename)
{
    NS_LOG_FUNCTION(this << filename);

    uint32_t numLoaded = m_loader->LoadGroundDevicesFromCsv(filename);
    
    if (m_verbose)
    {
        std::cout << "Loaded " << numLoaded << " ground devices from " << filename << std::endl;
        for (uint32_t i = 0; i < numLoaded; ++i)
        {
            std::cout << "  Ground Device " << i << ": " << m_loader->GetGroundDeviceName(i)
                      << std::endl;
        }
    }

    return numLoaded > 0;
}

NodeContainer
LeoSimLoaderHelper::CreateSatelliteNodes()
{
    NS_LOG_FUNCTION(this);

    uint32_t numSatellites = m_loader->GetNumSatellites();
    NodeContainer nodes;
    nodes.Create(numSatellites);

    // Apply mobility to nodes
    m_loader->ApplySatelliteMobility(nodes);

    // Assign names to nodes
    for (uint32_t i = 0; i < numSatellites; ++i)
    {
        std::string name = m_loader->GetSatelliteName(i);
        if (Names::FindName(nodes.Get(i)).empty())
        {
            Names::Add(name, nodes.Get(i));
        }
        
        if (m_verbose)
        {
            std::cout << "Created satellite node " << i << ": " << name << std::endl;
        }
    }

    return nodes;
}

NodeContainer
LeoSimLoaderHelper::CreateGroundDeviceNodes()
{
    NS_LOG_FUNCTION(this);

    uint32_t numDevices = m_loader->GetNumGroundDevices();
    NodeContainer nodes;
    nodes.Create(numDevices);

    // Apply positions to nodes
    m_loader->ApplyGroundDevicePositions(nodes);

    // Assign names to nodes
    for (uint32_t i = 0; i < numDevices; ++i)
    {
        std::string name = m_loader->GetGroundDeviceName(i);
        if (Names::FindName(nodes.Get(i)).empty())
        {
            Names::Add(name, nodes.Get(i));
        }
        
        if (m_verbose)
        {
            std::cout << "Created ground device node " << i << ": " << name << std::endl;
        }
    }

    return nodes;
}

bool
LeoSimLoaderHelper::ApplySatelliteMobility(NodeContainer& nodes)
{
    NS_LOG_FUNCTION(this);
    return m_loader->ApplySatelliteMobility(nodes);
}

bool
LeoSimLoaderHelper::ApplyGroundDevicePositions(NodeContainer& nodes)
{
    NS_LOG_FUNCTION(this);
    return m_loader->ApplyGroundDevicePositions(nodes);
}

uint32_t
LeoSimLoaderHelper::GetNumSatellites() const
{
    return m_loader->GetNumSatellites();
}

uint32_t
LeoSimLoaderHelper::GetNumGroundDevices() const
{
    return m_loader->GetNumGroundDevices();
}

std::string
LeoSimLoaderHelper::GetSatelliteName(uint32_t satId) const
{
    return m_loader->GetSatelliteName(satId);
}

std::string
LeoSimLoaderHelper::GetGroundDeviceName(uint32_t deviceId) const
{
    return m_loader->GetGroundDeviceName(deviceId);
}

Ptr<LeoSimLoader>
LeoSimLoaderHelper::GetLoader() const
{
    return m_loader;
}

void
LeoSimLoaderHelper::SetVerbose(bool verbose)
{
    m_verbose = verbose;
}

} // namespace ns3
