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

#ifndef LEOSIM_VISUALIZATION_HELPER_H
#define LEOSIM_VISUALIZATION_HELPER_H

#include "leosim-loader-helper.h"

#include "ns3/leosim-channel-model.h"
#include "ns3/leosim-beam-manager.h"
#include "ns3/node-container.h"

#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace ns3
{

class Packet;

/**
 * \ingroup leosim
 * \brief Helper for logging node positions and channel connections to file for visualization
 *
 * This helper schedules position logging callbacks for satellite, server,
 * and UE nodes at regular intervals during simulation. It also logs active
 * channel connections between nodes. Output is written to CSV files suitable
 * for visualization with the provided Python script.
 */
class LeoSimVisualizationHelper
{
  public:
    LeoSimVisualizationHelper();
    ~LeoSimVisualizationHelper();

    /**
     * \brief Set the output file for position data
     * \param outputFile Path to output CSV file
     */
    void SetOutputFile(std::string outputFile);

    /**
     * \brief Set the output file for channel/link data
     * \param linkFile Path to output link CSV file
     */
    void SetLinkFile(std::string linkFile);

    /**
     * \brief Set the output file for packet data
     * \param packetFile Path to packet CSV file
     */
    void SetPacketFile(std::string packetFile);

    /**
     * \brief Enable/disable packet logging
     * \param enable True to enable packet logging
     */
    void EnablePacketLogging(bool enable);

    /**
     * \brief Set the loader helper reference
     * \param loaderHelper Reference to the LeoSimLoaderHelper
     */
    void SetLoaderHelper(LeoSimLoaderHelper& loaderHelper);

    /**
     * \brief Set the channel model to track connections
     * \param channelModel Pointer to the LeoSimChannelModel
     */
    void SetChannelModel(Ptr<LeoSimChannelModel> channelModel);

    /**
     * \brief Set the ISL channel model to track inter-satellite connections
     * \param islChannelModel Pointer to the ISL LeoSimChannelModel
     */
    void SetIslChannelModel(Ptr<LeoSimChannelModel> islChannelModel);

    /**
     * \brief Schedule position logging for all nodes
     * \param satellites Container of satellite nodes
     * \param servers Container of server nodes
     * \param ues Container of UE nodes
     * \param interval Logging interval in seconds
     * \param simTime Total simulation time in seconds
     */
    void SchedulePositionLogging(NodeContainer satellites,
                                 NodeContainer servers,
                                 NodeContainer ues,
                                 double interval,
                                 double simTime);

    /**
     * \brief Install packet logging hooks (after devices are created)
     * \param satellites Container of satellite nodes for tracing
     * \param servers Container of server nodes for tracing
     * \param ues Container of UE nodes for tracing
     */
    void InstallPacketLogging(NodeContainer satellites, NodeContainer servers, NodeContainer ues);

    /**
     * \brief Initialize output files with CSV headers
     */
    void Initialize();

    /**
     * \brief Close output files
     */
    void Finalize();

    /**
     * \brief Set the output file for beam state data
     * \param filename Path to beam CSV file
     */
    void SetBeamFile(const std::string& filename);

    /**
     * \brief Set the output file for handover event data
     * \param filename Path to handover CSV file
     */
    void SetHandoverFile(const std::string& filename);

    /**
     * \brief Set the output file for CHO configuration data
     * \param filename Path to CHO CSV file
     */
    void SetChoFile(const std::string& filename);

    /**
     * \brief Initialize beam state and handover logging with header rows
     */
    void InitBeamLogging();

    /**
     * \brief Log current beam state for a UE
     * \param ueId UE node ID
     * \param rec Beam record with RSRP, SNR, elevation, etc.
     * \param topsisScore TOPSIS ranking score
     */
    void LogBeamState(uint32_t ueId, const LeoSimBeamRecord& rec, double topsisScore);

    /**
     * \brief Log handover event to file
     * \param evt Complete handover event with timing and metrics
     */
    void LogHandoverEvent(const LeoSimHandoverEvent& evt);

    /**
     * \brief Log CHO configuration candidates for a UE
     * \param ueId UE node ID
     * \param servingSatId Current serving satellite ID
     * \param candidates Vector of TOPSIS-ranked candidate beams
     */
    void LogChoConfig(uint32_t ueId, 
                      uint32_t servingSatId,
                      const std::vector<LeoSimTopsisCandidate>& candidates);

    /**
     * \brief Finalize beam logging by closing CSV files
     */
    void FinalizeBeamLogging();

  private:
    /**
     * \brief Log a single node position
     * \param node Node to log
     * \param nodeType Type of node (SATELLITE, SERVER, UE)
     * \param nodeId ID of the node
     * \param nodeName Name of the node
     */
    void LogNodePosition(Ptr<Node> node,
                        std::string nodeType,
                        uint32_t nodeId,
                        std::string nodeName);

    /**
     * \brief Log active channel connections
     */
    void LogChannelConnections();

    /**
     * \brief Log active ISL connections
     */
    void LogIslConnections();

    /**
     * \brief Log active ground connections
     */
    void LogGroundConnections();

    void OnPhyTx(std::string context, Ptr<const Packet> packet);
    void OnPhyRx(std::string context, Ptr<const Packet> packet, double snrDb, double dopplerHz);
    void OnPhyRxBasic(std::string context, Ptr<const Packet> packet);
    void OnPhyRxDrop(std::string context, Ptr<const Packet> packet);

    Ptr<Node> GetDevicePeerNode(int nodeId, int deviceId) const;
    std::string GetLinkType(Ptr<Node> node, Ptr<Node> peerNode) const;
    bool IsSatelliteNode(Ptr<Node> node) const;

    bool ParseContextIds(const std::string& context, int& nodeId, int& deviceId) const;

    std::string m_outputFile;
    std::string m_linkFile;
    std::string m_packetFile;
    std::string m_beamFile;
    std::string m_handoverFile;
    std::string m_choFile;
    std::ofstream m_posFile;
    std::ofstream m_linkFileStream;
    std::ofstream m_packetFileStream;
    std::ofstream m_beamFileStream;
    std::ofstream m_handoverFileStream;
    std::ofstream m_choFileStream;
    LeoSimLoaderHelper* m_loaderHelper;
    Ptr<LeoSimChannelModel> m_channelModel;
    Ptr<LeoSimChannelModel> m_islChannelModel;
    NodeContainer m_satellites;
    NodeContainer m_servers;
    NodeContainer m_ues;
    bool m_enablePacketLogging;
    bool m_packetLoggingInstalled;
};

} // namespace ns3

#endif /* LEOSIM_VISUALIZATION_HELPER_H */
