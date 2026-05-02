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

#ifndef LEOSIM_OPERATOR_HELPER_H
#define LEOSIM_OPERATOR_HELPER_H

#include "ns3/leosim-loader.h"
#include "ns3/leosim-operator-model.h"
#include "ns3/node-container.h"
#include "ns3/ptr.h"

namespace ns3
{

/**
 * \ingroup leosim
 * \brief Helper for building LeoSim operator assignments and sharing policy.
 *
 * This helper registers node/operator mappings into LeoSimOperatorModel and
 * provides convenience methods to load and configure cross-operator sharing.
 */
class LeoSimOperatorHelper
{
  public:
    /**
     * \brief Constructor.
     */
    LeoSimOperatorHelper();

    /**
     * \brief Set the LeoSim loader used for operator lookup.
     * \param loader Loader object
     */
    void SetLoader(Ptr<LeoSimLoader> loader);

    /**
     * \brief Register satellite nodes and their operators.
     *
     * Each satellite index i in satNodes is mapped using
     * loader->GetSatelliteOperator(i) and registered with
     * LEOSIM_ROLE_SATELLITE.
     *
     * \param satNodes Satellite node container
     */
    void RegisterSatellites(const NodeContainer& satNodes);

    /**
     * \brief Register UE and server nodes and their operators.
     *
     * UEs are registered with LEOSIM_ROLE_UE and servers with
     * LEOSIM_ROLE_SERVER using loader->GetGroundDeviceOperator(i).
     *
     * \param ueNodes UE node container
     * \param serverNodes Server node container
     */
    void RegisterGroundDevices(const NodeContainer& ueNodes,
                               const NodeContainer& serverNodes);

    /**
     * \brief Override operator assignment for one node.
     * \param node Node to update
     * \param opId Operator identifier
     */
    void SetNodeOperator(Ptr<Node> node, const LeoSimOperatorId& opId);

    /**
     * \brief Load sharing matrix from CSV into the operator model.
     * \param csvFile Sharing matrix CSV file
     */
    void LoadSharingMatrix(const std::string& csvFile);

    /**
     * \brief Set uniform cross-operator alpha values for all operator pairs.
     * \param alphaDl Downlink alpha
     * \param alphaUl Uplink alpha
     * \param alphaIsl ISL alpha
     */
    void SetUniformCrossOperatorAlpha(double alphaDl, double alphaUl, double alphaIsl);

    /**
     * \brief Finalize and return the operator model.
     *
     * If verbose is enabled, prints a summary before returning.
     *
     * \return Built operator model
     */
    Ptr<LeoSimOperatorModel> Build();

    /**
     * \brief Enable/disable verbose logging.
     * \param v Verbose flag
     */
    void SetVerbose(bool v);

  private:
    Ptr<LeoSimLoader> m_loader;
    Ptr<LeoSimOperatorModel> m_model;
    bool m_verbose = false;
};

} // namespace ns3

#endif /* LEOSIM_OPERATOR_HELPER_H */
