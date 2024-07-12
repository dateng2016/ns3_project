/*
 * This example program allows one to run ns-3 DSDV, AODV, or OLSR under
 * a typical random waypoint mobility model.
 *
 * By default, the simulation runs for 200 simulated seconds, of which
 * the first 50 are used for start-up time.  The number of nodes is 50.
 * Nodes move according to RandomWaypointMobilityModel with a speed of
 * 20 m/s and no pause time within a 300x1500 m region.  The WiFi is
 * in ad hoc mode with a 2 Mb/s rate (802.11b) and a Friis loss model.
 * The transmit power is set to 7.5 dBm.
 *
 * It is possible to change the mobility and density of the network by
 * directly modifying the speed and the number of nodes.  It is also
 * possible to change the characteristics of the network by changing
 * the transmit power (as power increases, the impact of mobility
 * decreases and the effective density increases).
 *
 * By default, OLSR is used, but specifying a value of 2 for the protocol
 * will cause AODV to be used, and specifying a value of 3 will cause
 * DSDV to be used.
 *
 * By default, there are 10 source/sink data pairs sending UDP data
 * at an application rate of 2.048 Kb/s each.    This is typically done
 * at a rate of 4 64-byte packets per second.  Application data is
 * started at a random time between 50 and 51 seconds and continues
 * to the end of the simulation.
 *
 * The program outputs a few items:
 * - packet receptions are notified to stdout such as:
 *   <timestamp> <node-id> received one packet from <src-address>
 * - each second, the data reception statistics are tabulated and output
 *   to a comma-separated value (csv) file
 * - some tracing and flow monitor configuration that used to work is
 *   left commented inline in the program
 */

#include "ns3/aodv-module.h"
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/dsdv-module.h"
#include "ns3/dsr-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/olsr-module.h"
#include "ns3/yans-wifi-helper.h"
#include <fstream>
#include <iostream>

// -----------------------------

#include "ns3/ns2-mobility-helper.h"
#include "ns2-node-utility.h"
#include <regex>
#include "custom-data-tag.h"
#include "ns3/log.h"
#include "ns3/simulator.h"

#define RED_CODE "\033[91m"
#define GREEN_CODE "\033[32m"
#define END_CODE "\033[0m"

// -----------------------------

using namespace ns3;
using namespace dsr;
using namespace std;


string FileNamePrefix = "scratch/simulation-record/";

// -----------------------------

NS_LOG_COMPONENT_DEFINE("manet-routing-compare");

Ns2NodeUtility::Ns2NodeUtility (std::string name)
{
    m_file_name = name;
    m_input_file.open(m_file_name);

    std::string line;

    std::vector <uint32_t> node_ids;

    while (std::getline (m_input_file, line))
    {
        std::smatch sm;
        std::regex r("\\$ns_ at (\\d*.\\d) \"\\$node_\\((\\d*)\\)");
        if (std::regex_search(line, sm, r))
        {
            //std::cout << "MATCH!" << std::endl;

            uint32_t id = std::stoi(sm[2]);
            double new_latest = std::stof(sm[1]);

            if ( std::find (node_ids.begin(), node_ids.end(), id) != node_ids.end())
            {
                double entry_time = std::get<0> (m_node_times [id]);
                m_node_times [id] = std::make_pair(entry_time, new_latest);
            } 
            else
            {
                m_node_times [id] = std::make_pair(new_latest, new_latest);
            }
            node_ids.push_back(id);
        }
        else
        {
              //std::cout << "No Match!" << std::endl;
        }
    }
}

uint32_t
Ns2NodeUtility::GetNNodes ()
{
    return m_node_times.size();
}

double
Ns2NodeUtility::GetEntryTimeForNode (uint32_t nodeId)
{
    return std::get<0>(m_node_times [nodeId]);
}

double
Ns2NodeUtility::GetExitTimeForNode (uint32_t nodeId)
{
    return std::get<1>(m_node_times [nodeId]);
}

double
Ns2NodeUtility::GetSimulationTime()
{
    double time = 0;

    for (uint32_t i=0 ; i<m_node_times.size(); i++)
        time = std::max ( time,  std::get<1>(m_node_times[i]) );

    return time;
}


void
Ns2NodeUtility::PrintInformation()
{
    uint32_t s = m_node_times.size();
    for (uint32_t i=0 ; i<s; i++)
    {
        std::cout << "Node " << i << " started " << std::get<0>(m_node_times[i]) << " and ended " << std::get<1>(m_node_times[i]) << std::endl;
    }
}

void
CourseChange (std::string context, Ptr<const MobilityModel> model)
{
    Vector position = model->GetPosition ();
    NS_LOG_UNCOND (Simulator::Now() << ": " << context <<
                  " x = " << position.x << ", y = " << position.y);
}

// -----------------------------

/**
 * Routing experiment class.
 *
 * It handles the creation and run of an experiment.
 */
class RoutingExperiment
{
  public:
    RoutingExperiment();
    /**
     * Run the experiment.
     */
    void Run();

    /**
     * Handles the command-line parameters.
     * \param argc The argument count.
     * \param argv The argument vector.
     */
    void CommandSetup(int argc, char** argv);

  private:
    /**
     * Setup the receiving socket in a Sink Node.
     * \param addr The address of the node.
     * \param node The node pointer.
     * \return the socket.
     */
    Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);
    /**
     * Receive a packet.
     * \param socket The receiving socket.
     */
    void ReceivePacket(Ptr<Socket> socket);
    /**
     * Compute the throughput.
     */
    void CheckThroughput();

    uint32_t port{9};            //!< Receiving port number.
    uint32_t bytesTotal{0};      //!< Total received bytes.
    uint32_t packetsReceived{0}; //!< Total received packets.

    std::string m_CSVfileName{"manet-routing.output.csv"};                      //!< CSV filename.
    int m_nSinks{10};                                                           //!< Number of sink nodes.
    std::string m_protocolName{"AODV"};                                         //!< Protocol name.
    double m_txp{7.5};                                                          //!< Tx power.
    bool m_traceMobility{false};                                                //!< Enable mobility tracing.
    bool m_flowMonitor{true};                                                   //!< Enable FlowMonitor.
    std::string location{"GTLow"};                                              //!< mobility file path.
};

RoutingExperiment::RoutingExperiment()
{
}

static inline std::string
PrintReceivedPacket(Ptr<Socket> socket, Ptr<Packet> packet, Address senderAddress)
{
    std::ostringstream oss;

    oss << Simulator::Now().GetSeconds() << " " << socket->GetNode()->GetId();

    if (InetSocketAddress::IsMatchingType(senderAddress))
    {
        InetSocketAddress addr = InetSocketAddress::ConvertFrom(senderAddress);
        oss << " received one packet from " << addr.GetIpv4();
    }
    else
    {
        oss << " received one packet!";
    }
    return oss.str();
}

void
RoutingExperiment::ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    Address senderAddress;
    while ((packet = socket->RecvFrom(senderAddress)))
    {
        bytesTotal += packet->GetSize();
        packetsReceived += 1;
        NS_LOG_UNCOND(PrintReceivedPacket(socket, packet, senderAddress));
    }
}

void
RoutingExperiment::CheckThroughput()
{
    double kbs = (bytesTotal * 8.0) / 1000;
    // std::cout << GREEN_CODE << "bytesTotal: " << bytesTotal << " bytes" << END_CODE << std::endl;
    bytesTotal = 0;

    std::ofstream out(location+m_protocolName+".csv", std::ios::app);

    out << (Simulator::Now()).GetSeconds() << "," << kbs << "," << packetsReceived << ","
        << m_nSinks << "," << m_protocolName << "," << m_txp << "" << std::endl;

    out.close();
    packetsReceived = 0;
    Simulator::Schedule(Seconds(1.0), &RoutingExperiment::CheckThroughput, this);
    // std::cout << GREEN_CODE << "Throughput: " << kbs << " Kbps" << END_CODE << std::endl;
}

Ptr<Socket>
RoutingExperiment::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node)
{
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> sink = Socket::CreateSocket(node, tid);
    InetSocketAddress local = InetSocketAddress(addr, port);
    sink->Bind(local);
    sink->SetRecvCallback(MakeCallback(&RoutingExperiment::ReceivePacket, this));

    return sink;
}

void
RoutingExperiment::CommandSetup(int argc, char** argv)
{
    CommandLine cmd(__FILE__);
    cmd.AddValue("CSVfileName", "The name of the CSV output file name", m_CSVfileName);
    cmd.AddValue("traceMobility", "Enable mobility tracing", m_traceMobility);
    cmd.AddValue("protocol", "Routing protocol (OLSR, AODV, DSDV, DSR)", m_protocolName);
    cmd.AddValue("flowMonitor", "enable FlowMonitor", m_flowMonitor);
    cmd.AddValue("nsinks", "the number of sink nodes", m_nSinks);
    cmd.AddValue("location", "location", location);
    cmd.Parse(argc, argv);

    std::vector<std::string> allowedProtocols{"OLSR", "AODV", "DSDV", "DSR"};

    if (std::find(std::begin(allowedProtocols), std::end(allowedProtocols), m_protocolName) ==
        std::end(allowedProtocols))
    {
        NS_FATAL_ERROR("No such protocol:" << m_protocolName);
    }
}

int
main(int argc, char* argv[])
{
    RoutingExperiment experiment;
    experiment.CommandSetup(argc, argv);
    experiment.Run();

    return 0;
}

    // std::string mobility_file = "scratch/simulation-record/FreTexas.tcl"; // 198 nodes 1137 seconds
    // std::string mobility_file = "scratch/simulation-record/GTHigh.tcl"; // 265 nodes 2812 seconds
    // std::string mobility_file = "scratch/simulation-record/GTLow.tcl"; // 121 nodes 1417 seconds
    // std::string mobility_file = "scratch/simulation-record/I285Atlanta.tcl"; // 1545 nodes 1180 seconds
    // std::string mobility_file = "scratch/simulation-record/TimesSquareHigh.tcl"; // 445 nodes 802 seconds
    // std::string mobility_file = "scratch/simulation-record/trace_short.tcl";  // 71 nodes 

void
RoutingExperiment::Run()
{
    Packet::EnablePrinting();

    // blank out the last output file and write the column headers
    // std::ofstream out(m_CSVfileName);
    // out << "SimulationSecond,"
    //     << "ReceiveRate,"
    //     << "PacketsReceived,"
    //     << "NumberOfSinks,"
    //     << "RoutingProtocol,"
    //     << "TransmissionPower" << std::endl;
    // out.close();

    std::string rate("2048bps");
    std::string phyMode("DsssRate11Mbps");
    std::string tr_name("manet-routing-compare");

    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("64"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue(rate));

    // Set Non-unicastMode rate to unicast mode
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    NS_LOG_UNCOND("About to read mobility trace file.\n");

    //A tool I created so that we only start the applications within nodes when they actually enter the simulation.
    Ns2NodeUtility ns2_utility (FileNamePrefix+location+".tcl");

    uint32_t nnodes = ns2_utility.GetNNodes();
    NS_LOG_UNCOND("There are " << nnodes << " nodes in the simulation.\n");

    double sim_time = ns2_utility.GetSimulationTime();
    
    // double sim_time_end = sim_time * 0.6; // We only care about the time that resembles the desired scenario
    
    NS_LOG_UNCOND("The total simulation time is " << sim_time << " seconds.\n");

    NodeContainer adhocNodes;
    adhocNodes.Create(nnodes);

    // setting up wifi phy and channel using helpers
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211b);

    YansWifiPhyHelper wifiPhy;

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    WifiMacHelper wifiMac;
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer adhocDevices = wifi.Install(wifiPhy, wifiMac, adhocNodes);

    //Using the bulit-in ns-2 mobility helper
    Ns2MobilityHelper sumo_trace (FileNamePrefix + location +".tcl");
    sumo_trace.Install(); //install ns-2 mobility in all nodes

    AodvHelper aodv;
    OlsrHelper olsr;
    DsdvHelper dsdv;
    DsrHelper dsr;
    DsrMainHelper dsrMain;
    Ipv4ListRoutingHelper list;
    InternetStackHelper internet;

    if (m_protocolName == "OLSR")
    {
        list.Add(olsr, 100);
        internet.SetRoutingHelper(list);
        internet.Install(adhocNodes);
    }
    else if (m_protocolName == "AODV")
    {
        list.Add(aodv, 100);
        internet.SetRoutingHelper(list);
        internet.Install(adhocNodes);
    }
    else if (m_protocolName == "DSDV")
    {
        list.Add(dsdv, 100);
        internet.SetRoutingHelper(list);
        internet.Install(adhocNodes);
    }
    else if (m_protocolName == "DSR")
    {
        internet.Install(adhocNodes);
        dsrMain.Install(dsr, adhocNodes);
        if (m_flowMonitor)
        {
            NS_FATAL_ERROR("Error: FlowMonitor does not work with DSR. Terminating.");
        }
    }
    else
    {
        NS_FATAL_ERROR("No such protocol:" << m_protocolName);
    }

    NS_LOG_INFO("assigning ip address");

    // Assign IP addresses
    Ipv4AddressHelper address;
    if (nnodes < 255) // up to 254 usable IP addresses
    {
        address.SetBase("10.1.1.0", "255.255.255.0");
    }
    else if (nnodes > 255 && nnodes < 512)
    {
        address.SetBase("10.1.0.0", "255.255.254.0"); // up to 510 usable IP addresses
    }
    else if (nnodes > 512 && nnodes < 1024)
    {
        address.SetBase("10.1.0.0", "255.255.252.0"); // up to 1022 usable IP addresses
    }
    else if (nnodes > 1024 && nnodes < 2048)
    {
        address.SetBase("10.1.0.0", "255.255.248.0"); // up to 2046 usable IP addresses
    }
    
    Ipv4InterfaceContainer adhocInterfaces = address.Assign(adhocDevices);

    // OnOffHelper onoff1("ns3::UdpSocketFactory", Address());
    // onoff1.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.1]"));
    // onoff1.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.9]"));

    // for (int i = 0; i < m_nSinks; i++)
    // {
    //     Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(i), adhocNodes.Get(i));

    //     AddressValue remoteAddress(InetSocketAddress(adhocInterfaces.GetAddress(i), port));
    //     onoff1.SetAttribute("Remote", remoteAddress);

    //     Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();
    //     ApplicationContainer temp = onoff1.Install(adhocNodes.Get(i + m_nSinks));
    //     temp.Start(Seconds(var->GetValue(sim_time * 0.5, sim_time * 0.5 + 3)));
    //     temp.Stop(Seconds(sim_time_end));
    // }

    // Set up applications to send packets from each node to the previous node, starting from node 2
    for (uint32_t i = 1; i < nnodes; ++i) {
        uint32_t senderIndex = i;
        uint32_t receiverIndex = i - 1;

        Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(receiverIndex), adhocNodes.Get(receiverIndex));
        Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();

        OnOffHelper onoff("ns3::UdpSocketFactory", Address(InetSocketAddress(adhocInterfaces.GetAddress(receiverIndex), port)));
        onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
        onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));
        ApplicationContainer temp = onoff.Install(adhocNodes.Get(senderIndex));
        temp.Start(Seconds(var->GetValue(sim_time *4.0 / 5,sim_time *4 /5 + 3.0)));
        temp.Stop(Seconds(sim_time));
    }

    // std::stringstream ss;
    // ss << nnodes;
    // std::string nodes = ss.str();

    // std::stringstream ss2;
    // ss2 << nodeSpeed;
    // std::string sNodeSpeed = ss2.str();

    // std::stringstream ss3;
    // ss3 << nodePause;
    // std::string sNodePause = ss3.str();

    // std::stringstream ss4;
    // ss4 << rate;
    // std::string sRate = ss4.str();

    // NS_LOG_INFO("Configure Tracing.");
    // tr_name = tr_name + "_" + m_protocolName +"_" + nodes + "nodes_" + sNodeSpeed + "speed_" +
    // sNodePause + "pause_" + sRate + "rate";

    // AsciiTraceHelper ascii;
    // Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream(tr_name + ".tr");
    // wifiPhy.EnableAsciiAll(osw);
    // AsciiTraceHelper ascii;
    // MobilityHelper::EnableAsciiAll(ascii.CreateFileStream(tr_name + ".mob"));

    FlowMonitorHelper flowmonHelper;
    Ptr<FlowMonitor> flowmon;
    if (m_flowMonitor)
    {
        flowmon = flowmonHelper.InstallAll();
    }

    NS_LOG_INFO("Run Simulation.");

    // CheckThroughput();

    Simulator::Stop(Seconds(sim_time));
    Simulator::Run();

    if (m_flowMonitor)
    {
        flowmon->SerializeToXmlFile("scratch/results/" + location + m_protocolName + ".flowmon", false, false);
    }

    Simulator::Destroy();
}


