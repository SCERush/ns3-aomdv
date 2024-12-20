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

uint32_t numPackets = 1000;
uint32_t currentSequenceNumber;
int64_t sent;
int64_t receive;
int64_t packetsReceived;
int64_t sqhd;
int64_t delay;
double xx;
double bytesTotal;
double throughput;

NS_LOG_COMPONENT_DEFINE("Base");

// Generate a random number in the range given inclusive
int randomInRange (int start = 0, int end = 1000)
{
    int range = end - start + 1;
    int result = rand () % range + start;
    return result;
}

void ReceivePacket(Ptr<Socket> socket)
{
    Ptr<Packet> packet;
    while (packet = socket->Recv())
    {
        receive = Simulator::Now().GetNanoSeconds(); // time at the receive
        SeqTsHeader seqTsx;
        packet->RemoveHeader(seqTsx);
        sqhd = seqTsx.GetTs().GetNanoSeconds();
        currentSequenceNumber = seqTsx.GetSeq();
        bytesTotal += packet->GetSize();
        packetsReceived += 1;
        delay = delay + (receive - sqhd); // delay calculation
    }
    xx = receive - sent;
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
        if (pktCount == numPackets)
        {
            sent = Simulator::Now().GetNanoSeconds();
        }
        Simulator::Schedule(pktInterval, &GenerateTraffic, socket, pktSize, pktCount - 1, pktInterval);
    }
    else
    {
        socket->Close();
    }
}

int main(int argc, char *argv[])
{
    uint32_t packetSize = 1000;
    uint32_t numNodes = 50;
    uint32_t sourceNode = 10;
    uint32_t sinkNode = 14;
    double interval = 0.1;
    bool verbose = false;

    std::string phyMode("DsssRate1Mbps");
    Time interPacketInterval = Seconds(interval);
    Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue(phyMode));

    NodeContainer c;
    NodeContainer not_malicious;
    NodeContainer malicious;
    c.Create(numNodes);

    for (uint32_t i = 0; i < numNodes; i++)
    {
        // if (i == 5 || i == 7 || i == 20 || i == 23 || i == 27 || i == 45 || i == 37 || i == 42 || i == 19 || i == 34)
        if (i == 5 || i == 7 || i == 9)
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
    wifiPhy.Set("RxGain", DoubleValue(-20));
    wifiPhy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);

    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    wifiPhy.SetChannel(wifiChannel.Create());

    // Add an upper mac and disable rate control
    WifiMacHelper wifiMac;
    wifi.SetStandard(WIFI_PHY_STANDARD_80211b);
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager", 
                                 "DataMode", StringValue(phyMode),
                                 "ControlMode", StringValue(phyMode)
                                );
    // Set it to adhoc mode
    wifiMac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, c);

    // Mobility
    MobilityHelper mobility;
    ObjectFactory pos;
    pos.SetTypeId("ns3::RandomRectanglePositionAllocator");
    pos.Set("X", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
    pos.Set("Y", StringValue("ns3::UniformRandomVariable[Min=0.0|Max=1000.0]"));
    Ptr<PositionAllocator> posAlloc = pos.Create()->GetObject<PositionAllocator>();
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              // "Speed", StringValue ("ns3::UniformRandomVariable[Min=0|Max=20]"),
                              "Speed", StringValue("ns3::ConstantRandomVariable[Constant=5.0]"),
                              "Pause", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"),
                              "PositionAllocator", PointerValue(posAlloc));
    mobility.SetPositionAllocator(posAlloc);
    mobility.Install(c);

    /** Energy Model **/
    uint32_t minInitialEnergy = numPackets / 5;
    uint32_t maxInitialEnergy = numPackets / 2;
    BasicEnergySourceHelper basicSourceHelper;
    std::map<Ptr<Node>, int> NodeInitialEnergyMap;
    for (NetDeviceContainer ::Iterator it = devices.Begin (); it != devices.End (); it++)
    {
        Ptr<Node> currentNode = (*it)->GetNode ();
        int NodeEnergy = randomInRange (minInitialEnergy, maxInitialEnergy);
        NodeInitialEnergyMap.insert (std::make_pair (currentNode, NodeEnergy));
        basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (NodeEnergy));
        EnergySourceContainer sources = basicSourceHelper.Install (currentNode);
        WifiRadioEnergyModelHelper radioEnergyHelper;
        // configure radio energy model
        radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.0174));
        // install device model
        DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install (*it, sources);
    }

    AomdvHelper aomdv;
    AomdvHelper malicious_aomdv;
    Ipv4ListRoutingHelper list;
    Ipv4ListRoutingHelper list1;

    list.Add(aomdv, 10);
    InternetStackHelper internet;
    internet.SetRoutingHelper(list);
    internet.Install(not_malicious);

    malicious_aomdv.Set("IsMalicious", BooleanValue(true));
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

    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    double stopTime = (double)numPackets / 5.0;
    Simulator::Schedule(Seconds(5.0), &GenerateTraffic, source, packetSize, numPackets,
                        interPacketInterval);

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

    NS_LOG_UNCOND("\n\nSimulation Report");
    NS_LOG_UNCOND("Source = " << sourceNode << "\nSink = " << sinkNode);
    NS_LOG_UNCOND("Total Nodes = " << numNodes);
    NS_LOG_UNCOND("NumPackets = " << numPackets);
    NS_LOG_UNCOND("Packet Loss = " << numPackets - packetsReceived);
    NS_LOG_UNCOND("Total delay =  " << delay / 1000000 << "ms ");
    NS_LOG_UNCOND("Average Delay = " << ((double)delay / (double)packetsReceived) / (double)1000000.0 << "ms");
    NS_LOG_UNCOND("Throughput = " << throughput << "kbps");

    std::fstream fout;
    fout.open("blackhole1.csv", std::ios::out | std::ios::app);
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
