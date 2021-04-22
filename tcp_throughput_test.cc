 /* Tcp Throughput Test for investigating HE performance with respect to 
 	TCP send and receiver buffer size. 
    Copyright (C) 2020  Berat Sert <beratsert41@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

 *Program reports station-wise WiFi metrics. The user can specify the
 *application data rate, the number of stations in the network, tcp socket 
 *sizes and the variant of TCP i.e. congestion control algorithm to use.
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-sink.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/tcp-westwood.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/int64x64-128.h"

NS_LOG_COMPONENT_DEFINE ("wifi-tcp-throughput");

using namespace ns3;

Ptr<PacketSink> sink;                         /* Pointer to the packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */

void
CalculateThroughput ()
{
  Time now = Simulator::Now ();                                         /* Return the simulator's virtual time. */
  double cur = (sink->GetTotalRx () - lastTotalRx) * (double) 8 / 1e5;     /* Convert Application RX Packets to MBits. */
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;
  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
}

main (int argc, char *argv[])
{
  uint32_t payloadSize = 1472;                       /* Transport layer payload size in bytes. */
  std::string dataRate = "10Mbps";                  /* Application layer datarate. */
  std::string tcpVariant = "TcpNewReno";             /* TCP variant type. */               
  double simulationTime = 10;                        /* Simulation time in seconds. */
  bool pcapTracing = false;                          /* PCAP Tracing is enabled or not. */
  bool sack = true;
  bool useExtendedBlockAck = false;
  int channelWidth = 20;
  int nSta = 6;
  int nAp = 1;
  std::string prefix_file_name = "trial_finalv3";
  double TxPacketsSum = 0;
  double  TxBytesSum = 0;
  double TxOfferedSum = 0;
  double RxPacketsSum = 0;
  double RxBytesSum = 0;
  int64x64_t delaySumCalc = 0;
  int RcvBufSize = 17;
  int SndBufSize = 17;

  /* Command line argument parser setup. */
  CommandLine cmd;
  cmd.AddValue ("payloadSize", "Payload size in bytes", payloadSize);
  cmd.AddValue ("dataRate", "Application data rate", dataRate);
  cmd.AddValue ("tcpVariant", "Transport protocol to use: TcpNewReno, "
                "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                "TcpBic, TcpYeah, TcpIllinois, TcpWestwood, TcpWestwoodPlus, TcpLedbat ", tcpVariant);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("pcap", "Enable/disable PCAP Tracing", pcapTracing);
  cmd.AddValue ("nSta", "Number of STA in the network", nSta);
  cmd.AddValue ("RcvBufSize", "Specify the size for tcp socket receiver buffer", RcvBufSize);
  cmd.AddValue ("SndBufSize", "Specify the size for tcp socket sender buffer");
  cmd.Parse (argc, argv);

  tcpVariant = std::string ("ns3::") + tcpVariant;
  // Select TCP variant
  if (tcpVariant.compare ("ns3::TcpWestwoodPlus") == 0)
    {
      // TcpWestwoodPlus is not an actual TypeId name; we need TcpWestwood here
      Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TcpWestwood::GetTypeId ()));
      // the default protocol type in ns3::TcpWestwood is WESTWOOD
      Config::SetDefault ("ns3::TcpWestwood::ProtocolType", EnumValue (TcpWestwood::WESTWOODPLUS));
    }
  else
    {
      TypeId tcpTid;
      NS_ABORT_MSG_UNLESS (TypeId::LookupByNameFailSafe (tcpVariant, &tcpTid), "TypeId " << tcpVariant << " not found");
      Config::SetDefault ("ns3::TcpL4Protocol::SocketType", TypeIdValue (TypeId::LookupByName (tcpVariant)));
    }

  /* Configure TCP Options */
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));
  Config::SetDefault ("ns3::TcpSocket::RcvBufSize", UintegerValue (1 << RcvBufSize));
  Config::SetDefault ("ns3::TcpSocket::SndBufSize", UintegerValue (1 << SndBufSize));
  Config::SetDefault ("ns3::TcpSocketBase::Sack", BooleanValue (sack));

  WifiMacHelper wifiMac;

  /* Set up Legacy Channel */
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9)); // legacy channel operates on 2.4 Ghz
 
  // Creating the Stations and Ap nodes
  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (nSta);
  NodeContainer wifiApNode;
  wifiApNode.Create (nAp);

  // Adding physical loss model to channel
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  channel.AddPropagationLoss ("ns3::RangePropagationLossModel"); 

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.SetChannel (channel.Create ());

  // Declaring the wifi standard and MCS Level
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211ax_5GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("HeMcs8"), "ControlMode", StringValue ("HeMcs0"));
  
  WifiMacHelper mac;

  Ssid ssid = Ssid ("simple-mpdu-aggregation");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid));

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (phy, mac, wifiStaNodes);

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid),
               "EnableBeaconJitter", BooleanValue (false));

  NetDeviceContainer apDevice;
  apDevice = wifi.Install (phy, mac, wifiApNode);

  //Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/BE_MaxAmpduSize", UintegerValue (maxAmpduSize));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/ChannelWidth", UintegerValue (channelWidth));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/GuardInterval", TimeValue (NanoSeconds (800)));
  Config::Set ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MpduBufferSize", UintegerValue (useExtendedBlockAck ? 256 : 64));
  
  // Setting mobility model
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();


  
  //positionAlloc->Add (Vector (0.0, 0.0, 0.0)); // add position for the stations and the AP
  

  mobility.SetPositionAllocator (positionAlloc);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mobility.Install (wifiApNode);
  mobility.Install (wifiStaNodes);

  InternetStackHelper stack;
  stack.Install (wifiApNode);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer StaInterface;
  StaInterface = address.Assign (staDevices);
  Ipv4InterfaceContainer ApInterface;
  ApInterface = address.Assign (apDevice);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  /* Install TCP Receiver on the access point */
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9));
  ApplicationContainer sinkApp = sinkHelper.Install (wifiApNode);
  sink = StaticCast<PacketSink> (sinkApp.Get (0));

  /* Install TCP/UDP Transmitter on the station */
  OnOffHelper server ("ns3::TcpSocketFactory", (InetSocketAddress (ApInterface.GetAddress (0), 9)));
  server.SetAttribute ("PacketSize", UintegerValue (payloadSize));
  server.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  server.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  server.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
  ApplicationContainer serverApp = server.Install (wifiStaNodes);

  /* Start Applications */
  sinkApp.Start (Seconds (0.0));
  serverApp.Start (Seconds (1.0));
  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

  /* Start Simulation */
  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();

  double averageThroughput = ((sink->GetTotalRx () * 8) / (1e6 * simulationTime));

  Simulator::Destroy ();

  
  flowmon.SerializeToXmlFile (prefix_file_name + ".flowmonitor", true, true);
  
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
     {
       // first 2 FlowIds are for ECHO apps, we don't want to display them
       //
       // Duration for throughput measurement is 9.0 seconds, since
       //   StartTime of the OnOffApplication is at about "second 1"
       // and
       //   Simulator::Stops at "second 10".
     	
       if (i->first < (nSta + nAp))
         {
         	Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
         	TxPacketsSum = TxPacketsSum + i->second.txPackets;
  			TxBytesSum = TxBytesSum + i->second.txBytes;
  			TxOfferedSum = TxOfferedSum + i->second.txBytes * 8.0 / 9.0 / 1000 / 1000  ;
  			RxPacketsSum = RxPacketsSum + i->second.rxPackets;
  			RxBytesSum = RxBytesSum + i->second.rxBytes ;
  			delaySumCalc = delaySumCalc + i->second.delaySum;
           
           std::cout << "Flow " << i->first - 2 << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
           std::cout << "  Tx Packets: " << i->second.txPackets << "\n";
           std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
           std::cout << "  TxOffered:  " << i->second.txBytes * 8.0 / 9.0 / 1000 / 1000  << " Mbps\n";
           std::cout << "  Rx Packets: " << i->second.rxPackets << "\n";
           std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
           std::cout << "  delaySum: " << i->second.delaySum / i->second.rxPackets << "\n";
           std::cout << "  Throughput: " << i->second.rxBytes * 8.0 / 10.0 / 1000 / 1000  << " Mbps\n";
         }
     }
  
  std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;

           std::cout << "  Avg. Tx Packets: " << TxPacketsSum/nSta << "\n";
           std::cout << "  Avg. Tx Bytes:   " << TxBytesSum/nSta << "\n";
           std::cout << "  Avg. TxOffered:  " << TxOfferedSum/nSta  << " Mbps\n";
           std::cout << "  Avg. Rx Packets: " << RxPacketsSum/nSta << "\n";
           std::cout << "  Avg. Rx Bytes:   " << RxBytesSum/nSta << "\n";
           std::cout << "  Avg. delaySum: " << delaySumCalc/TxPacketsSum << "\n";
  return 0;
}
