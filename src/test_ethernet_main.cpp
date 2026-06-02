#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>
#include "VCFEthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"
#include "hytech_msgs.pb.h"

#include <array>
#include <cstring>

#include "ProtobufMsgInterface.h"
#include "etl/optional.h"

using namespace qindesign::network;

// EthernetUDP socket; 

// void init_ethernet_device()
// {
//     Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
//     socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
//     recv_socket.begin(5555);
// }

// void test_send()
// {
//     hytech_msgs_VCFData_s msg = VCFEthernetInterface::make_vcf_data_msg(ADCInterfaceInstance::instance(), DashboardInterfaceInstance::instance(), PedalsSystemInstance::instance());
//     if (handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size, hytech_msgs_VCFData_s>(EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().VCRData_port, &socket, msg, &hytech_msgs_VCFData_s_msg)) {
//         Serial.println("Sent");
//     } else {
//         Serial.println("Failed");
//     }
// }

// void test_receive()
// {
//     //handle_ethernet_socket_receive
//     //curr_millis, socket(recv), msg desc, sizeof buffer, ref to empty protoc struct
//     //return optional struct
// }

// void setup()
// {
//     init_ethernet_device();
// }

// void loop()
// {
//     etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&socket, &hytech_msgs_VCRData_s_msg);
//     if (protoc_struct)
//     {
//         Serial.printf("message RR: %d\n", (*protoc_struct).rear_loadcell_data.RR_loadcell_analog);
//         Serial.printf("message RL: %d\n", (*protoc_struct).rear_loadcell_data.RL_loadcell_analog);
        
//     } 


//     //test_send();
//     //Serial.println("loopin");
// }

// Static setup
static const IPAddress MY_IP     (192, 168, 1,  10);
static const IPAddress PEER_IP   (192, 168, 1,  11);
static const IPAddress SUBNET    (255, 255, 255,  0);
static const IPAddress GATEWAY   (192, 168, 1,   1);
static constexpr uint16_t PORT   = 5010;
static constexpr uint32_t SEND_INTERVAL_MS = 500;

// ── Globals ───────────────────────────────────────────────────────────────────
EthernetUDP udp;
uint32_t    lastSend  = 0;
uint32_t    sendCount = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Ethernet.begin(MY_IP, SUBNET, GATEWAY);

  // Wait for link
  Serial.print("Waiting for link...");
  while (!Ethernet.linkState()) {
    Serial.print('.');
    delay(250);
  }
  Serial.println(" linked!");
  Serial.print("IP: ");
  Serial.println(Ethernet.localIP());

  // Bind UDP socket
  if (!udp.begin(PORT)) {
    Serial.println("ERROR: failed to bind UDP socket!");
    while (true) {}
  }
  Serial.print("Listening on UDP port ");
  Serial.println(PORT);
  Serial.println("Ready.\n");
}

void loop() {
  // ── Receive ──────────────────────────────────────────────────────────────
  int pktSize = udp.parsePacket();
  if (pktSize > 0) {
    char buf[256];
    int  len = udp.read(buf, sizeof(buf) - 1);
    if (len < 0) len = 0;
    buf[len] = '\0';

    Serial.print("[RX] from ");
    Serial.print(udp.remoteIP());
    Serial.print("  \"");
    Serial.print(buf);
    Serial.println("\"");
  }

  // ── Send every SEND_INTERVAL_MS ──────────────────────────────────────────
  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    sendCount++;

    char msg[64];
    snprintf(msg, sizeof(msg), "Hello from VCF, packet %lu", sendCount);

    udp.beginPacket(PEER_IP, PORT);
    udp.write((const uint8_t*)msg, strlen(msg));
    bool ok = udp.endPacket();

    Serial.print("[TX] -> ");
    Serial.print(PEER_IP);
    Serial.print("  \"");
    Serial.print(msg);
    Serial.print("\"  ");
    Serial.println(ok ? "OK" : "FAILED");
  }
}