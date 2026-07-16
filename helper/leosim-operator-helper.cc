/*
 * Copyright (c) 2026 Anupa De Silva
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

#include "leosim-operator-helper.h"

#include "ns3/log.h"
#include "ns3/leosim-mobility-model.h"

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimOperatorHelper");

LeoSimOperatorHelper::LeoSimOperatorHelper()
{
    m_model = CreateObject<LeoSimOperatorModel>();
}

void
LeoSimOperatorHelper::SetLoader(Ptr<LeoSimLoader> loader)
{
    m_loader = loader;
}

void
LeoSimOperatorHelper::RegisterSatellites(const NodeContainer& satNodes)
{
    NS_LOG_FUNCTION(this << satNodes.GetN());

    for (uint32_t i = 0; i < satNodes.GetN(); ++i)
    {
        Ptr<Node> node = satNodes.Get(i);
        if (!node)
        {
            continue;
        }

        LeoSimOperatorId opId = "default";
        if (m_loader)
        {
            uint32_t satId = i;
            Ptr<LeoSimMobilityModel> mobility = node->GetObject<LeoSimMobilityModel>();
            if (mobility)
            {
                satId = mobility->GetNodeId();
            }
            opId = m_loader->GetSatelliteOperator(satId);
        }

        m_model->RegisterNode(node->GetId(), opId, LEOSIM_ROLE_SATELLITE);
    }
}

void
LeoSimOperatorHelper::RegisterGroundDevices(const NodeContainer& ueNodes,
                                            const NodeContainer& serverNodes)
{
    NS_LOG_FUNCTION(this << ueNodes.GetN() << serverNodes.GetN());

    for (uint32_t i = 0; i < ueNodes.GetN(); ++i)
    {
        Ptr<Node> node = ueNodes.Get(i);
        if (!node)
        {
            continue;
        }

        LeoSimOperatorId opId = "default";
        if (m_loader)
        {
            uint32_t deviceId = i;
            Ptr<LeoSimMobilityModel> mobility = node->GetObject<LeoSimMobilityModel>();
            if (mobility)
            {
                deviceId = mobility->GetNodeId();
            }
            opId = m_loader->GetGroundDeviceOperator(deviceId);
        }

        m_model->RegisterNode(node->GetId(), opId, LEOSIM_ROLE_UE);
    }

    for (uint32_t i = 0; i < serverNodes.GetN(); ++i)
    {
        Ptr<Node> node = serverNodes.Get(i);
        if (!node)
        {
            continue;
        }

        LeoSimOperatorId opId = "default";
        if (m_loader)
        {
            uint32_t deviceId = i;
            Ptr<LeoSimMobilityModel> mobility = node->GetObject<LeoSimMobilityModel>();
            if (mobility)
            {
                deviceId = mobility->GetNodeId();
            }
            opId = m_loader->GetGroundDeviceOperator(deviceId);
        }

        m_model->RegisterNode(node->GetId(), opId, LEOSIM_ROLE_SERVER);
    }
}

void
LeoSimOperatorHelper::SetNodeOperator(Ptr<Node> node, const LeoSimOperatorId& opId)
{
    if (!node)
    {
        return;
    }

    LeoSimNodeRole role = m_model->GetRole(node->GetId());
    m_model->RegisterNode(node->GetId(), opId, role);
}

void
LeoSimOperatorHelper::LoadSharingMatrix(const std::string& csvFile)
{
    m_model->LoadSharingMatrixFromCsv(csvFile);
}

void
LeoSimOperatorHelper::SetUniformCrossOperatorAlpha(double alphaDl,
                                                    double alphaUl,
                                                    double alphaIsl)
{
    std::vector<LeoSimOperatorId> operators = m_model->GetAllOperators();

    for (size_t i = 0; i < operators.size(); ++i)
    {
        for (size_t j = i + 1; j < operators.size(); ++j)
        {
            m_model->SetAlpha(operators[i], operators[j], alphaDl, alphaUl, alphaIsl);
        }
    }
}

Ptr<LeoSimOperatorModel>
LeoSimOperatorHelper::Build()
{
    if (m_verbose)
    {
        m_model->PrintSummary();
    }
    return m_model;
}

void
LeoSimOperatorHelper::SetVerbose(bool v)
{
    m_verbose = v;
    m_model->SetVerbose(v);
}

} // namespace ns3
