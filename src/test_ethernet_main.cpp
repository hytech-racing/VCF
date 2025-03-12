#include <Arduino.h>
#include <cstdint>
#include <QNEthernet.h>
#include "VCFEthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"
#include "hytech_msgs.pb.h"
#include "VCF_Globals.h"


#include <array>
#include <cstring>

#include "ProtobufMsgInterface.h"
#include "etl/optional.h"

using namespace qindesign::network;
EthernetUDP vcr_data_socket; 
EthernetUDP vcf_data_socket;

VCFData_s vcf_data = {};

void setup()
{
    Serial.begin(115200);
    EthernetIPDefsInstance::create();
    uint8_t mac[6];
    qindesign::network::Ethernet.macAddress(mac);
    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);

    vcr_data_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    vcf_data_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
}

void loop()
{
    if (millis() % 1000 < 500) {
        etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&vcr_data_socket, &hytech_msgs_VCRData_s_msg);
        if (protoc_struct) {
            VCFEthernetInterface::receive_pb_msg_vcr(protoc_struct.value(), vcf_data, millis());
            
            Serial.println("Received bytes: (protoc struct value)");
            char *string_ptr = (char*) &(protoc_struct.value());
            uint32_t kk = sizeof(protoc_struct.value());
            while(kk--)
                Serial.printf("%02X ", *string_ptr++);
            Serial.println("");
        } else {
            Serial.printf("Did not receive VCR message!\n");
        }

        vcf_data.interface_data.front_loadcell_data.FL_loadcell_analog += 1;
        hytech_msgs_VCFData_s send_protoc_struct = VCFEthernetInterface::make_vcf_data_msg(vcf_data);

        Serial.println("Send bytes: (protoc struct value)");
        char *string_ptr = (char*) &(send_protoc_struct);
        uint32_t kk = sizeof(send_protoc_struct);
        while(kk--)
            Serial.printf("%02X ", *string_ptr++);
        Serial.println("");

        handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size>(EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().VCFData_port,
            &vcf_data_socket, send_protoc_struct, hytech_msgs_VCFData_s_fields);

        delay(500);
    }
}