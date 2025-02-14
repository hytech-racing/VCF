#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>
#include "VCREthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "hytech_msgs.pb.h"


#include <array>
#include <cstring>

#include "ProtobufMsgInterface.h"

using namespace qindesign::network;
EthernetUDP socket; 
//EthernetUDP recv_socket; 


const IPAddress default_VCR_ip(192, 168, 1, 30); //(for now) sender
const IPAddress receive_ip(192, 168, 1, 31); // receiver
const IPAddress default_dns(192, 168, 1, 1);
const IPAddress default_gateway(192, 168, 1, 1);
const IPAddress car_subnet(255, 255, 255, 0);
uint16_t port1 = 4444;
uint16_t port2 = 5555;
//hytech_msgs_VCRData_s msg = hytech_msgs_VCRData_s_init_zero;
VCRData_s vcr_state;
//hytech_msgs_VCRData_s msg = {};

// uint8_t default_MCU_MAC_address[6] = 
//     {0x04, 0xe9, 0xe5, 0x10, 0x1f, 0x22};

void init_ethernet_device()
{
    Ethernet.begin(default_VCR_ip, default_dns, default_gateway, car_subnet);
    socket.begin(4444);
    //recv_socket.begin(5555);
}

void test_send()
{
    hytech_msgs_VCRData_s msg = VCREthernetInterface::make_vcr_data_msg(vcr_state);
    if (handle_ethernet_socket_send_pb<hytech_msgs_VCRData_s, hytech_msgs_VCRData_s_size>(receive_ip, port1, &socket, msg, &hytech_msgs_VCRData_s_msg)) {
        Serial.println("Sent");
    } else {
        Serial.println("Failed");
    }

}

void test_receive()
{
    //handle_ethernet_socket_receive
    //curr_millis, socket(recv), msg desc, sizeof buffer, ref to empty protoc struct
    //return optional struct
}

void setup()
{
    init_ethernet_device();
}

void loop()
{
    test_send();
    //Serial.println("loopin");
}