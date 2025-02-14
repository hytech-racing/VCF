#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>
#include "VCFEthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "hytech_msgs.pb.h"


#include <array>
#include <cstring>

#include "ProtobufMsgInterface.h"
#include "etl/optional.h"

using namespace qindesign::network;
EthernetUDP socket; 
//EthernetUDP recv_socket; 


const IPAddress default_VCF_ip(192, 168, 1, 30); //(for now) sender
const IPAddress receive_ip(192, 168, 1, 31); // receiver
const IPAddress default_dns(192, 168, 1, 1);
const IPAddress default_gateway(192, 168, 1, 1);
const IPAddress car_subnet(255, 255, 255, 0);
uint16_t port1 = 4444;
uint16_t port2 = 5555;
//hytech_msgs_VCRData_s msg = hytech_msgs_VCRData_s_init_zero;
VCFData_s vcf_state;
//hytech_msgs_VCRData_s msg = {};

// uint8_t default_MCU_MAC_address[6] = 
//     {0x04, 0xe9, 0xe5, 0x10, 0x1f, 0x22};

void init_ethernet_device()
{
    Ethernet.begin(default_VCF_ip, default_dns, default_gateway, car_subnet);
    socket.begin(4444);
    //recv_socket.begin(5555);
}

void test_send()
{
    hytech_msgs_VCFData_s msg = VCFEthernetInterface::make_vcf_data_msg(vcf_state);
    if (handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s, hytech_msgs_VCFData_s_size>(receive_ip, port1, &socket, msg, &hytech_msgs_VCFData_s_msg)) {
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
    etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&socket, &hytech_msgs_VCRData_s_msg);
    if (protoc_struct)
    {
        Serial.printf("message RR: %d\n", (*protoc_struct).rear_loadcell_data.RR_loadcell_analog);
        Serial.printf("message RL: %d\n", (*protoc_struct).rear_loadcell_data.RL_loadcell_analog);
        
    } 


    //test_send();
    //Serial.println("loopin");
}