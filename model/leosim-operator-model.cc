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

#include "leosim-operator-model.h"

#include "ns3/fatal-error.h"
#include "ns3/log.h"

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>

namespace ns3
{

NS_LOG_COMPONENT_DEFINE("LeoSimOperatorModel");
NS_OBJECT_ENSURE_REGISTERED(LeoSimOperatorModel);

TypeId
LeoSimOperatorModel::GetTypeId()
{
    static TypeId tid = TypeId("ns3::LeoSimOperatorModel")
                            .SetParent<Object>()
                            .SetGroupName("LeoSim")
                            .AddConstructor<LeoSimOperatorModel>();
    return tid;
}

void
LeoSimOperatorModel::RegisterNode(uint32_t nodeId,
                                  const LeoSimOperatorId& opId,
                                  LeoSimNodeRole role,
                                  const std::string& name)
{
    m_nodeInfo[nodeId] = {nodeId, opId, role, name};

    if (m_verbose)
    {
        NS_LOG_INFO("Registered node " << nodeId << " as operator " << opId);
    }
}

void
LeoSimOperatorModel::RegisterNodes(const NodeContainer& nodes,
                                   const LeoSimOperatorId& opId,
                                   LeoSimNodeRole role)
{
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        Ptr<Node> node = nodes.Get(i);
        if (!node)
        {
            continue;
        }
        RegisterNode(node->GetId(), opId, role);
    }
}

LeoSimOperatorId
LeoSimOperatorModel::GetOperatorId(uint32_t nodeId) const
{
    auto it = m_nodeInfo.find(nodeId);
    if (it == m_nodeInfo.end())
    {
        NS_LOG_WARN("Operator info not found for node " << nodeId);
        return "unknown";
    }

    return it->second.operatorId;
}

LeoSimNodeRole
LeoSimOperatorModel::GetRole(uint32_t nodeId) const
{
    auto it = m_nodeInfo.find(nodeId);
    if (it == m_nodeInfo.end())
    {
        NS_LOG_WARN("Role info not found for node " << nodeId << "; defaulting to UE role");
        return LEOSIM_ROLE_UE;
    }

    return it->second.role;
}

bool
LeoSimOperatorModel::IsSameOperator(uint32_t nodeA, uint32_t nodeB) const
{
    return GetOperatorId(nodeA) == GetOperatorId(nodeB);
}

std::vector<uint32_t>
LeoSimOperatorModel::GetNodesByOperator(const LeoSimOperatorId& opId) const
{
    std::vector<uint32_t> nodes;
    for (const auto& kv : m_nodeInfo)
    {
        if (kv.second.operatorId == opId)
        {
            nodes.push_back(kv.first);
        }
    }
    return nodes;
}

std::vector<LeoSimOperatorId>
LeoSimOperatorModel::GetAllOperators() const
{
    std::set<LeoSimOperatorId> uniqueOperators;
    for (const auto& kv : m_nodeInfo)
    {
        uniqueOperators.insert(kv.second.operatorId);
    }

    return std::vector<LeoSimOperatorId>(uniqueOperators.begin(), uniqueOperators.end());
}

void
LeoSimOperatorModel::SetAlpha(const LeoSimOperatorId& opA,
                              const LeoSimOperatorId& opB,
                              double alphaDl,
                              double alphaUl,
                              double alphaIsl)
{
    NS_ASSERT_MSG(alphaDl >= 0.0 && alphaDl <= 1.0 && alphaUl >= 0.0 && alphaUl <= 1.0 && alphaIsl >= 0.0 && alphaIsl <= 1.0,
                  "Alpha values must be in [0,1]");

    if (opA == opB)
    {
        NS_ASSERT_MSG(alphaDl == 1.0 && alphaUl == 1.0 && alphaIsl == 1.0,
                      "Intra-operator alpha must be 1.0");
    }

    const std::string key = MakeKey(opA, opB);
    m_sharingMatrix[key] = {opA, opB, alphaDl, alphaUl, alphaIsl};
}

void
LeoSimOperatorModel::SetAlphaSymmetric(const LeoSimOperatorId& opA,
                                       const LeoSimOperatorId& opB,
                                       double alpha)
{
    SetAlpha(opA, opB, alpha, alpha, alpha);
}

double
LeoSimOperatorModel::GetAlpha(uint32_t nodeA, uint32_t nodeB, LeoSimLinkDirection dir) const
{
    const LeoSimOperatorId opA = GetOperatorId(nodeA);
    const LeoSimOperatorId opB = GetOperatorId(nodeB);
    return GetAlphaByOperator(opA, opB, dir);
}

double
LeoSimOperatorModel::GetAlphaByOperator(const LeoSimOperatorId& opA,
                                        const LeoSimOperatorId& opB,
                                        LeoSimLinkDirection dir) const
{
    if (opA == opB)
    {
        return 1.0;
    }

    auto it = m_sharingMatrix.find(MakeKey(opA, opB));
    if (it == m_sharingMatrix.end())
    {
        return 1.0;
    }

    switch (dir)
    {
        case LEOSIM_DIR_DOWNLINK:
            return it->second.alphaDl;
        case LEOSIM_DIR_UPLINK:
            return it->second.alphaUl;
        case LEOSIM_DIR_ISL:
            return it->second.alphaIsl;
        default:
            return it->second.alphaDl;
    }
}

void
LeoSimOperatorModel::LoadSharingMatrixFromCsv(const std::string& csvFile)
{
    std::ifstream file(csvFile);
    if (!file.is_open())
    {
        NS_FATAL_ERROR("Failed to open sharing matrix CSV: " << csvFile);
    }

    std::string line;
    uint32_t lineNo = 0;
    while (std::getline(file, line))
    {
        ++lineNo;

        const auto firstContent = line.find_first_not_of(" \t\r");
        if (firstContent == std::string::npos || line[firstContent] == '#')
        {
            continue;
        }

        if (line.compare(firstContent, std::string("OperatorA,OperatorB,AlphaDL,AlphaUL,AlphaISL").size(),
                         "OperatorA,OperatorB,AlphaDL,AlphaUL,AlphaISL") == 0)
        {
            continue;
        }

        try
        {
            std::stringstream ss(line);
            std::string opA;
            std::string opB;
            std::string alphaDlStr;
            std::string alphaUlStr;
            std::string alphaIslStr;

            if (!std::getline(ss, opA, ',') || !std::getline(ss, opB, ',') ||
                !std::getline(ss, alphaDlStr, ',') || !std::getline(ss, alphaUlStr, ',') ||
                !std::getline(ss, alphaIslStr, ','))
            {
                NS_FATAL_ERROR("Invalid sharing matrix row format at line " << lineNo << " in " << csvFile);
            }

            const double alphaDl = std::stod(alphaDlStr);
            const double alphaUl = std::stod(alphaUlStr);
            const double alphaIsl = std::stod(alphaIslStr);
            SetAlpha(opA, opB, alphaDl, alphaUl, alphaIsl);
        }
        catch (const std::exception& e)
        {
            NS_FATAL_ERROR("Failed to parse sharing matrix CSV " << csvFile << " at line " << lineNo << ": " << e.what());
        }
    }
}

void
LeoSimOperatorModel::SaveSharingMatrixToCsv(const std::string& csvFile) const
{
    std::ofstream file(csvFile);
    if (!file.is_open())
    {
        NS_FATAL_ERROR("Failed to open sharing matrix CSV for writing: " << csvFile);
    }

    file << "OperatorA,OperatorB,AlphaDL,AlphaUL,AlphaISL\n";
    for (const auto& kv : m_sharingMatrix)
    {
        const LeoSimSharingEntry& e = kv.second;
        file << e.operatorA << ',' << e.operatorB << ',' << e.alphaDl << ',' << e.alphaUl << ',' << e.alphaIsl << '\n';
    }
}

uint64_t
LeoSimOperatorModel::GetEffectiveDataRateBps(uint32_t nodeA,
                                              uint32_t nodeB,
                                              uint64_t baseRateBps,
                                              LeoSimLinkDirection dir) const
{
    const double alpha = GetAlpha(nodeA, nodeB, dir);
    return static_cast<uint64_t>(std::max(1.0, alpha * static_cast<double>(baseRateBps)));
}

double
LeoSimOperatorModel::GetRoutingCostMultiplier(uint32_t nodeA,
                                               uint32_t nodeB,
                                               LeoSimLinkDirection dir) const
{
    const double alpha = GetAlpha(nodeA, nodeB, dir);
    if (alpha <= 0.0)
    {
        return 1e6;
    }
    return 1.0 / alpha;
}

void
LeoSimOperatorModel::SetVerbose(bool v)
{
    m_verbose = v;
}

void
LeoSimOperatorModel::PrintSummary() const
{
    const auto operators = GetAllOperators();

    NS_LOG_INFO("Registered nodes: " << m_nodeInfo.size());
    NS_LOG_INFO("Operators: " << operators.size());
    NS_LOG_INFO("Sharing agreements: " << m_sharingMatrix.size());

    for (const auto& kv : m_sharingMatrix)
    {
        const LeoSimSharingEntry& e = kv.second;
        NS_LOG_INFO("  " << e.operatorA << " <-> " << e.operatorB << ": DL=" << e.alphaDl
                           << " UL=" << e.alphaUl << " ISL=" << e.alphaIsl);
    }
}

std::string
LeoSimOperatorModel::MakeKey(const LeoSimOperatorId& a, const LeoSimOperatorId& b) const
{
    return (a < b) ? (a + "|" + b) : (b + "|" + a);
}

} // namespace ns3
