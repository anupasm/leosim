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

#ifndef LEOSIM_OPERATOR_MODEL_H
#define LEOSIM_OPERATOR_MODEL_H

#include "ns3/object.h"
#include "ns3/node-container.h"

#include <map>
#include <string>
#include <vector>

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Operator identity tag (e.g. "OpA", "Starlink", "OneWeb").
 */
using LeoSimOperatorId = std::string;

/**
 * \ingroup leosim
 * \brief Role classification for LeoSim nodes.
 */
enum LeoSimNodeRole
{
    LEOSIM_ROLE_SATELLITE, //!< Satellite node
    LEOSIM_ROLE_UE,        //!< User equipment node
    LEOSIM_ROLE_SERVER,    //!< Ground server node
    LEOSIM_ROLE_GATEWAY    //!< Gateway node
};

/**
 * \ingroup leosim
 * \brief Direction classification for links between nodes.
 */
enum LeoSimLinkDirection
{
    LEOSIM_DIR_DOWNLINK, //!< Satellite to UE/server
    LEOSIM_DIR_UPLINK,   //!< UE/server to satellite
    LEOSIM_DIR_ISL       //!< Satellite to satellite (ISL)
};

/**
 * \ingroup leosim
 * \brief Sharing agreement entry between two operators.
 *
 * Each alpha field must be in the range [0,1].
 */
struct LeoSimSharingEntry
{
    LeoSimOperatorId operatorA;
    LeoSimOperatorId operatorB;
    double alphaDl;
    double alphaUl;
    double alphaIsl;
};

/**
 * \ingroup leosim
 * \brief Operator metadata associated with a node.
 */
struct LeoSimNodeOperatorInfo
{
    uint32_t nodeId;
    LeoSimOperatorId operatorId;
    LeoSimNodeRole role;
    std::string name;
};

/**
 * \ingroup leosim
 * \brief Operator registry and symmetric inter-operator sharing policy model.
 *
 * This model stores operator identity metadata for each registered node and
 * maintains a sharing matrix indexed by operator pairs. The sharing matrix is
 * symmetric and keyed using a lexicographically sorted pair representation
 * ("OpA|OpB"), allowing consistent lookup regardless of argument order.
 */
class LeoSimOperatorModel : public Object
{
    public:
        /**
         * \brief Get the type ID.
         * \return the object TypeId
         */
        static TypeId GetTypeId();

        /**
         * \brief Register one node with operator identity and node role.
         * \param nodeId Node identifier
         * \param opId Operator identifier
         * \param role Node role
         * \param name Optional display name
         */
        void RegisterNode(uint32_t nodeId,
                                            const LeoSimOperatorId& opId,
                                            LeoSimNodeRole role,
                                            const std::string& name = "");

        /**
         * \brief Register all nodes in a container with the same operator and role.
         * \param nodes Node container
         * \param opId Operator identifier
         * \param role Node role
         */
        void RegisterNodes(const NodeContainer& nodes, const LeoSimOperatorId& opId, LeoSimNodeRole role);

        /**
         * \brief Get the operator ID for a node.
         * \param nodeId Node identifier
         * \return Operator identifier for the node
         */
        LeoSimOperatorId GetOperatorId(uint32_t nodeId) const;

        /**
         * \brief Get the role for a node.
         * \param nodeId Node identifier
         * \return Registered role for the node
         */
        LeoSimNodeRole GetRole(uint32_t nodeId) const;

        /**
         * \brief Check whether two nodes belong to the same operator.
         * \param nodeA First node identifier
         * \param nodeB Second node identifier
         * \return true if both nodes have the same operator ID
         */
        bool IsSameOperator(uint32_t nodeA, uint32_t nodeB) const;

        /**
         * \brief Get all node IDs registered for a specific operator.
         * \param opId Operator identifier
         * \return Vector of matching node IDs
         */
        std::vector<uint32_t> GetNodesByOperator(const LeoSimOperatorId& opId) const;

        /**
         * \brief Get the list of all distinct registered operators.
         * \return Vector of unique operator IDs
         */
        std::vector<LeoSimOperatorId> GetAllOperators() const;

        /**
         * \brief Set directional sharing factors between two operators.
         * \param opA First operator
         * \param opB Second operator
         * \param alphaDl Downlink sharing factor
         * \param alphaUl Uplink sharing factor
         * \param alphaIsl ISL sharing factor
         */
        void SetAlpha(const LeoSimOperatorId& opA,
                                    const LeoSimOperatorId& opB,
                                    double alphaDl,
                                    double alphaUl,
                                    double alphaIsl);

        /**
         * \brief Set a single symmetric sharing factor for all directions.
         * \param opA First operator
         * \param opB Second operator
         * \param alpha Sharing factor used for downlink, uplink, and ISL
         */
        void SetAlphaSymmetric(const LeoSimOperatorId& opA, const LeoSimOperatorId& opB, double alpha);

        /**
         * \brief Get sharing factor between two nodes for a given link direction.
         * \param nodeA First node identifier
         * \param nodeB Second node identifier
         * \param dir Link direction
         * \return Sharing factor in [0,1]
         */
        double GetAlpha(uint32_t nodeA, uint32_t nodeB, LeoSimLinkDirection dir) const;

        /**
         * \brief Get sharing factor between two operators for a given direction.
         * \param opA First operator
         * \param opB Second operator
         * \param dir Link direction
         * \return Sharing factor in [0,1]
         */
        double GetAlphaByOperator(const LeoSimOperatorId& opA,
                                                            const LeoSimOperatorId& opB,
                                                            LeoSimLinkDirection dir) const;

        /**
         * \brief Load sharing matrix entries from a CSV file.
         * \param csvFile CSV file path
         */
        void LoadSharingMatrixFromCsv(const std::string& csvFile);

        /**
         * \brief Save the current sharing matrix entries to a CSV file.
         * \param csvFile CSV file path
         */
        void SaveSharingMatrixToCsv(const std::string& csvFile) const;

        /**
         * \brief Compute effective data rate by applying sharing factor.
         * \param nodeA First node identifier
         * \param nodeB Second node identifier
         * \param baseRateBps Base data rate in bps
         * \param dir Link direction
         * \return Effective data rate in bps
         */
        uint64_t GetEffectiveDataRateBps(uint32_t nodeA,
                                                                         uint32_t nodeB,
                                                                         uint64_t baseRateBps,
                                                                         LeoSimLinkDirection dir) const;

        /**
         * \brief Compute routing cost multiplier from sharing factor.
         * \param nodeA First node identifier
         * \param nodeB Second node identifier
         * \param dir Link direction
         * \return Cost multiplier
         */
        double GetRoutingCostMultiplier(uint32_t nodeA, uint32_t nodeB, LeoSimLinkDirection dir) const;

        /**
         * \brief Enable or disable verbose logging.
         * \param v Verbose flag
         */
        void SetVerbose(bool v);

        /**
         * \brief Print operator and sharing matrix summary.
         */
        void PrintSummary() const;

    private:
        std::map<uint32_t, LeoSimNodeOperatorInfo> m_nodeInfo;
        std::map<std::string, LeoSimSharingEntry> m_sharingMatrix;
        bool m_verbose = false;

        std::string MakeKey(const LeoSimOperatorId& a, const LeoSimOperatorId& b) const;
};

} // namespace ns3

#endif // LEOSIM_OPERATOR_MODEL_H
