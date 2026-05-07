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
#include "ns3/leosim-operator-model.h"
#include "ns3/leosim-weather-model.h"
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
     * \brief Enable/disable beam + handover logging
     *
     * When enabled, this helper can log per-UE serving beam state changes,
     * CHO candidate configuration, and handover events.
     */
    void EnableBeamLogging(bool enable);

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
     * \brief Attach a beam manager to receive beam/handover callbacks
     *
     * Call this after creating the beam manager. If beam logging is enabled
     * and files are initialized, callbacks are installed immediately.
     */
    void SetBeamManager(Ptr<LeoSimBeamManager> beamManager);

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
     * \brief Set the output file for operator assignment data
     * \param filename Path to operator CSV file
     */
    void SetOperatorFile(const std::string& filename);

    /**
     * \brief Set the output file for sharing-state data
     * \param filename Path to sharing CSV file
     */
    void SetSharingFile(const std::string& filename);

    /**
     * \brief Write operator assignments for all nodes to CSV
     * \param model Operator model used to resolve operator ID and role
     * \param allNodes Container of all nodes to export
     */
    void InitOperatorLogging(Ptr<LeoSimOperatorModel> model, const NodeContainer& allNodes);

    /**
     * \brief Append active cross-operator sharing state to CSV
     * \param model Operator model used for sharing queries
     * \param channelModel Ground channel model
     * \param islChannelModel ISL channel model
     * \param simTime Simulation time for the snapshot row
     */
    void LogSharingState(Ptr<LeoSimOperatorModel> model,
               Ptr<LeoSimChannelModel> channelModel,
               Ptr<LeoSimChannelModel> islChannelModel,
               double simTime);

    /**
     * \brief Initialize beam state and handover logging with header rows
     */
    void InitBeamLogging();

    /**
     * \brief Set the maximum UEs per beam for utilization calculation
     * \param maxUes Maximum number of UEs per beam (default: 20)
     */
    void SetMaxUesPerBeam(uint32_t maxUes);

    /**
     * \brief Log current beam state for a UE
     * \param ueId UE node ID
     * \param rec Beam record with RSRP, SINR, elevation, etc.
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

    /**
     * \brief Set the output file for per-node weather state data
     * \param filename Path to weather CSV file
     */
    void SetWeatherFile(const std::string& filename);

    /**
     * \brief Set the output file for per-link attenuation breakdown data
     * \param filename Path to attenuation CSV file
     */
    void SetAttenuationFile(const std::string& filename);

    /**
     * \brief Write CSV headers for weather and attenuation log files
     *
     * Must be called once before the simulation starts (before Simulator::Run()).
     * Opens both files in truncate mode and writes their respective header rows.
     */
    void InitWeatherLogging();

    /**
     * \brief Append one row per ground node to leosim_weather.csv
     * \param model Weather model used to query state and parameters
     * \param groundNodes Container of all ground nodes to log
     * \param simTime Current simulation time (seconds) for the timestamp column
     */
    void LogWeatherState(Ptr<LeoSimWeatherModel> model,
                         const NodeContainer& groundNodes,
                         double simTime);

    /**
     * \brief Append one row per active UE-satellite link to leosim_attenuation.csv
     * \param channelModel Ground channel model used to iterate active links
     * \param ueNodes Container of UE nodes
     * \param satNodes Container of satellite nodes (unused; reserved for future use)
     * \param simTime Current simulation time (seconds) for the timestamp column
     */
    void LogAttenuationState(Ptr<LeoSimChannelModel> channelModel,
                              const NodeContainer& ueNodes,
                              const NodeContainer& satNodes,
                              double simTime);

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

    void LogServingBeamSnapshot();

    void OnPhyTx(std::string context, Ptr<const Packet> packet);
    void OnPhyRx(std::string context, Ptr<const Packet> packet, double snrDb, double dopplerHz);
    void OnPhyRxBasic(std::string context, Ptr<const Packet> packet);
    void OnPhyRxDrop(std::string context, Ptr<const Packet> packet);

    Ptr<Node> GetDevicePeerNode(int nodeId, int deviceId) const;
    std::string GetLinkType(Ptr<Node> node, Ptr<Node> peerNode) const;
    bool IsSatelliteNode(Ptr<Node> node) const;

    bool ParseContextIds(const std::string& context, int& nodeId, int& deviceId) const;

    void InstallBeamManagerCallbacks();
    void OnBeamState(uint32_t ueId, LeoSimBeamRecord rec, double topsisScore);
    void OnHandoverEvent(LeoSimHandoverEvent evt);
    void OnChoConfig(uint32_t ueId,
             uint32_t servingSatId,
             std::vector<LeoSimTopsisCandidate> candidates);

    std::string m_outputFile;
    std::string m_linkFile;
    std::string m_packetFile;
    std::string m_beamFile;
    std::string m_handoverFile;
    std::string m_choFile;
    std::string m_operatorFile;
    std::string m_sharingFile;
    std::string m_weatherFile;
    std::string m_attenuationFile;
    std::ofstream m_posFile;
    std::ofstream m_linkFileStream;
    std::ofstream m_packetFileStream;
    std::ofstream m_beamFileStream;
    std::ofstream m_handoverFileStream;
    std::ofstream m_choFileStream;
    LeoSimLoaderHelper* m_loaderHelper;
    Ptr<LeoSimChannelModel> m_channelModel;
    Ptr<LeoSimChannelModel> m_islChannelModel;
    Ptr<LeoSimBeamManager> m_beamManager;
    NodeContainer m_satellites;
    NodeContainer m_servers;
    NodeContainer m_ues;
    bool m_enablePacketLogging;
    bool m_packetLoggingInstalled;
    bool m_enableBeamLogging;
    bool m_beamLoggingInitialized;
    bool m_beamCallbacksInstalled;
    uint32_t m_maxUesPerBeam;          //!< Maximum UEs per beam for utilization calculation
};

} // namespace ns3

#endif /* LEOSIM_VISUALIZATION_HELPER_H */
