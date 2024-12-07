#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/mobility-module.h"
#include "ns3/aodv-helper.h"
#include "ns3/olsr-helper.h"
#include "ns3/aomdv-helper.h"
#include "ns3/ipv4-static-routing-helper.h"
#include "ns3/ipv4-list-routing-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/seq-ts-header.h"
#include "ns3/flow-monitor-module.h"
#include <fstream>

using namespace ns3;

uint32_t currentSequenceNumber;
int64_t senttime;
double bytesTotal;
int64_t packetsReceived;
int64_t rcv;
int64_t sqhd;
int64_t delay;
double xx;
double throughput;
double remainingEnergy = 0.0;
uint32_t numPackets = 1000;
uint32_t seed = 0;
double Emin = 1000.0;

NS_LOG_COMPONENT_DEFINE("BlackHole");

// Generate a random number in the range given inclusive
int randomInRange(int start = 0, int end = 1000)
{
    int range = end - start + 1;
    int result = rand() % range + start;
    return result;
}

void ReceivePacket(Ptr<Socket> socket)
{
    // NS_LOG_UNCOND ("Received One packet!");
    Ptr<Packet> packet;
    while (packet = socket->Recv())
    {
        rcv = Simulator::Now().GetNanoSeconds(); // time at the receive
        SeqTsHeader seqTsx;
        packet->RemoveHeader(seqTsx);
        sqhd = seqTsx.GetTs().GetNanoSeconds();
        currentSequenceNumber = seqTsx.GetSeq();
        bytesTotal += packet->GetSize();
        packetsReceived += 1;
        // NS_LOG_UNCOND("For the received packet, Seq No = " << currentSequenceNumber << " ,timestamp on the packet = " << sqhd);//Just to check seq number and Tx time
        delay = delay + (rcv - sqhd); // delay calculation
    }
    xx = rcv - senttime;
    throughput = (bytesTotal * 8 * 1000000000) / (1024 * xx);
}

static void GenerateTraffic(Ptr<Socket> socket, uint32_t pktSize, uint32_t pktCount, Time pktInterval)
{
    if (pktCount > 0)
    {
        SeqTsHeader seqTs;
        seqTs.SetSeq(pktCount);
        Ptr<Packet> p = Create<Packet>(pktSize - (8 + 4)); // 8+4 : the size of the seqTs header
        p->AddHeader(seqTs);
        socket->Send(p);
        // NS_LOG_UNCOND ("Sending "<< pktCount  << " packet! \n");   //pktCount is the sequence number
        if (pktCount == numPackets)
        {
            senttime = Simulator::Now().GetNanoSeconds();
        }
        Simulator::Schedule(pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1,
                            pktInterval);
    }
    else
    {
        socket->Close();
    }
}

void EnergyCheck(NodeContainer c)
{
    for (NodeContainer::Iterator i = c.Begin(); i < c.End(); i++)
    {
        double e = ((*i)->GetObject<EnergySourceContainer>()->Get(0)->GetRemainingEnergy());
        Emin = std::min(Emin, e);
        remainingEnergy += e;
    }
    // NS_LOG_UNCOND("Total energy remaining " << Esum);
}

void PrintEnergyOfNode(NodeContainer c)
{
    for (NodeContainer::Iterator i = c.Begin(); i < c.End(); i++)
    {
        double e = ((*i)->GetObject<EnergySourceContainer>()->Get(0)->GetRemainingEnergy());
        NS_LOG_UNCOND("Energy of node " << *i << " = " << e);
    }
    return;
}

int main(int argc, char *argv[])
{
    std::string phyMode("DsssRate1Mbps");
    double distance = 10;       // m
    uint32_t packetSize = 1000; // bytes
    // uint32_t numPackets = 1000; now defined as global variable for throughput calculations
    uint32_t numNodes = 25; // by default, 5x5

    // sourceNode and sinkNode are set after the commandline arguments are set
    uint32_t sourceNode = 10;
    uint32_t sinkNode = 14;
    double interval = 0.1; // seconds
    bool verbose = false;
    bool tracing = false;

    std::string protocolName;

    CommandLine cmd;
    cmd.AddValue("phyMode", "Wifi Phy mode", phyMode);
    cmd.AddValue("distance", "distance (m)", distance);
    cmd.AddValue("packetSize", "size of application packet sent", packetSize);
    cmd.AddValue("numPackets", "number of packets generated", numPackets);
    cmd.AddValue("interval", "interval (seconds) between packets", interval);
    cmd.AddValue("verbose", "turn on all WifiNetDevice log components", verbose);
    cmd.AddValue("tracing", "turn on ascii and pcap tracing", tracing);
    cmd.AddValue("numNodes", "number of nodes", numNodes);
    cmd.AddValue("sinkNode", "Receiver node number", sinkNode);
    cmd.AddValue("sourceNode", "Sender node number", sourceNode);
    cmd.AddValue("seed", "Seed Value", seed);
    cmd.Parse(argc, argv);

    srand(seed); // setting the seed for rand() function in randomInRange function

    // to set random sourceNode and sinkNode
    // sinkNode = randomInRange(0,numNodes-1);
    // sourceNode = sinkNode;
    // while(sourceNode==sinkNode){
    //     sourceNode = randomInRange(0,numNodes-1);
    // }

    // Convert to time object
    Time interPacketInterval = Seconds(interval);

    // Fix non-unicast data rate to be the same as that of unicast
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    NodeContainer c;
    NodeContainer not_malicious;
    NodeContainer malicious;
    c.Create(numNodes);

    for (uint32_t i = 0; i < numNodes; i++)
    {
        if (i == 88)
        {
            malicious.Add(c.Get(i));
        }
        else
        {
            not_malicious.Add(c.Get(i));
        }
    }

    // The below set of helpers will help us to put together the wifi NICs we want
    WifiHelper wifi;
    if (verbose)
    {
        wifi.EnableLogComponents(); // Turn on all Wifi logging
    }

    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    // set it to zero; otherwise, gain will be added
    wifiPhy.Set("RxGain", DoubleValue(-40));
    // wifiPhy.Set ("RxGain", DoubleValue (-45));
    // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    // Add an upper mac and disable rate control
    WifiMacHelper wifiMac;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", "DataMode", StringValue(phyMode),
                                 "ControlMode", StringValue(phyMode));
    // Set it to adhoc mode
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, c);

    // Mobility
    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                  "MinX", DoubleValue(0.0),
                                  "MinY", DoubleValue(0.0),
                                  "DeltaX", DoubleValue(distance),
                                  "DeltaY", DoubleValue(distance),
                                  "GridWidth", UintegerValue(5),
                                  "LayoutType", StringValue("RowFirst"));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    // mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
    //                                "X", StringValue ("ns3::UniformRandomVariable[Min=0|Max=400]"),
    //                                "Y", StringValue ("ns3::UniformRandomVariable[Min=0|Max=400]"));
    // mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

    // mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
    //                           "Mode", StringValue ("Time"),
    //                           "Time", StringValue ("2s"),
    //                           "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
    //                           "Bounds", StringValue ("0|200|0|200"));

    // ObjectFactory pos;
    // pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    // pos.Set ("X", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
    // pos.Set ("Y", StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=500.0]"));
    // Ptr<PositionAllocator> posAlloc = pos.Create ()->GetObject<PositionAllocator> ();
    // mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
    //                             // "Speed", StringValue ("ns3::UniformRandomVariable[Min=0|Max=20]"),
    //                             "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=5.0]"),
    //                             "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"),
    //                             "PositionAllocator", PointerValue (posAlloc));
    // mobility.SetPositionAllocator (posAlloc);

    mobility.Install(c);

    /** Energy Model **/
    /***************************************************************************/
    /* energy source */
    // Setting different initial energy
    uint32_t minInitialEnergy = numPackets / 5;
    uint32_t maxInitialEnergy = numPackets / 2;
    BasicEnergySourceHelper basicSourceHelper;
    std::map<Ptr<Node>, int> NodeInitialEnergyMap;
    for (NetDeviceContainer ::Iterator it = devices.Begin(); it != devices.End(); it++)
    {
        Ptr<Node> currentNode = (*it)->GetNode();
        int NodeEnergy = randomInRange(minInitialEnergy, maxInitialEnergy);
        NodeInitialEnergyMap.insert(std::make_pair(currentNode, NodeEnergy));
        basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(NodeEnergy));
        EnergySourceContainer sources = basicSourceHelper.Install(currentNode);
        WifiRadioEnergyModelHelper radioEnergyHelper;
        // configure radio energy model
        radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174));
        // install device model
        DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(*it, sources);
    }

    // PrintEnergyOfNode (c);
    // NS_LOG_UNCOND("Printing NodeInitialEnergyMap");
    // double totalInitialEnergy = 0.0;
    // for (auto pp : NodeInitialEnergyMap)
    // {
    //     //NS_LOG_UNCOND("energy of node "<< pp.first << " = " << pp.second);
    //     totalInitialEnergy += pp.second;
    // }

    AomdvHelper aomdv;
    AomdvHelper malicious_aomdv;
    Ipv4ListRoutingHelper list;
    Ipv4ListRoutingHelper list1;

    list.Add(aomdv, 10);
    protocolName = "AOMDV";

    InternetStackHelper internet;
    internet.SetRoutingHelper(list); // has effect on the next Install ()
    internet.Install(not_malicious);

    malicious_aomdv.Set(
        "IsMalicious",
        BooleanValue(true));
    list1.Add(malicious_aomdv, 10);
    internet.SetRoutingHelper(list1);
    internet.Install(malicious);

    Ipv4AddressHelper ipv4;
    NS_LOG_INFO("Assign IP Addresses.");
    ipv4.SetBase("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer i = ipv4.Assign(devices);

    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket(c.Get(sinkNode), tid);
    InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), 80);
    recvSink->Bind(local);
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    Ptr<Socket> source = Socket::CreateSocket(c.Get(sourceNode), tid);
    InetSocketAddress remote = InetSocketAddress(i.GetAddress(sinkNode, 0), 80);
    source->Connect(remote);

    // if (tracing == true)
    // {
    //     AsciiTraceHelper ascii;
    //     wifiPhy.EnableAsciiAll (ascii.CreateFileStream ("fanet-compare.tr"));
    //     // wifiPhy.EnablePcap ("fanet-compare", devices);
    //     // Trace routing tables
    //     Ptr<OutputStreamWrapper> routingStream =
    //         Create<OutputStreamWrapper> ("fanet-compare.routes", std::ios::out);
    //     if (protocol == 1)
    //     {
    //         aodv.PrintRoutingTableAllEvery (Seconds (1), routingStream);
    //     }
    //     else if (protocol == 2)
    //     {
    //         aomdv.PrintRoutingTableAllEvery (Seconds (1), routingStream);
    //     }
    //     // else
    //     // {
    //     //     olsr.PrintRoutingTableAllEvery (Seconds (1), routingStream);
    //     // }
    //     Ptr<OutputStreamWrapper> neighborStream =
    //         Create<OutputStreamWrapper> ("fanet-compare.neighbors", std::ios::out);
    //     if (protocol == 1)
    //     {
    //         aodv.PrintNeighborCacheAllEvery (Seconds (1), neighborStream);
    //     }
    //     else if (protocol == 2)
    //     {
    //         aomdv.PrintNeighborCacheAllEvery (Seconds (1), neighborStream);
    //     }
    //     // else
    //     // {
    //     //     olsr.PrintNeighborCacheAllEvery (Seconds (1), neighborStream);
    //     // }

    //     // To do-- enable an IP-level trace that shows forwarding events only
    // }

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    double stopTime = (double)numPackets / 5.0;
    Simulator::Schedule(Seconds(5.0), &GenerateTraffic, source, packetSize, numPackets,
                        interPacketInterval);
    Simulator::Schedule(Seconds(stopTime - 20), &EnergyCheck, c);
    // Simulator::Schedule(Seconds (5.0) , &EnergyCheck, c);
    //  Output what we are doing
    AnimationInterface anim("blackhole.xml");
    anim.SetMaxPktsPerTraceFile(99999999999999);
    Simulator::Stop(Seconds(stopTime));
    Simulator::Run();
    Simulator::Destroy();

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin();
         i != stats.end(); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(i->first);
        if ((t.sourceAddress == "10.1.1.11" && t.destinationAddress == "10.1.1.15"))
        {
            std::cout << "Flow " << i->first << " (" << t.sourceAddress << " -> "
                      << t.destinationAddress << ")\n";
            std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
            std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
            std::cout << "  Throughput: "
                      << i->second.rxBytes * 8.0 /
                             (i->second.timeLastRxPacket.GetSeconds() -
                              i->second.timeFirstTxPacket.GetSeconds()) /
                             1024 / 1024
                      << " Mbps\n";
        }
    }

    NS_LOG_UNCOND("\n\n\nSimulation Report");
    NS_LOG_UNCOND("Seed = " << seed);
    NS_LOG_UNCOND("Protocol = " << protocolName);
    NS_LOG_UNCOND("Source = " << sourceNode << "\nSink = " << sinkNode);
    NS_LOG_UNCOND("Total Nodes = " << numNodes);
    NS_LOG_UNCOND("NumPackets = " << numPackets);
    NS_LOG_UNCOND("Packet Loss = " << numPackets - packetsReceived);
    NS_LOG_UNCOND("Total delay =  " << delay / 1000000 << "ms ");
    NS_LOG_UNCOND("Average Delay = " << ((double)delay / (double)packetsReceived) / (double)1000000.0 << "ms");
    NS_LOG_UNCOND("Throughput = " << throughput << "kbps");

    std::fstream fout;
    fout.open("compare.csv", std::ios::out | std::ios::app);
    fout << sourceNode << ", "
         << sinkNode << ", "
         << numNodes << ", "
         << numPackets << ", "
         << numPackets - packetsReceived << ", "
         << delay / 1000000 << ", "
         << ((double)delay / (double)packetsReceived) / (double)1000000.0 << ", "
         << throughput
         << "\n";

    return 0;
}
