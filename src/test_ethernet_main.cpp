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

qindesign::network::EthernetUDP vcf_data_socket;
qindesign::network::EthernetUDP vcr_data_socket;

VCFData_s vcf_data = {};

void setup()
{
    Serial.begin(115200); //NOLINT (Serial baudrate)
    EthernetIPDefsInstance::create();
    uint8_t mac[6]; //NOLINT (Mac addr always 6 bytes)
    qindesign::network::Ethernet.macAddress(&mac[0]);
    qindesign::network::Ethernet.begin(mac, EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);

    vcr_data_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    vcf_data_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
}

void loop()
{
    if (millis() % 1000 < 500) { //NOLINT
        etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&vcr_data_socket, &hytech_msgs_VCRData_s_msg);
        if (protoc_struct) {
            VCFEthernetInterface::receive_pb_msg_vcr(protoc_struct.value(), vcf_data, millis());
            
            Serial.println("Received bytes: (protoc struct value)");
            char *string_ptr = (char*) &(protoc_struct.value());
            uint32_t kk = sizeof(protoc_struct.value());
            while(kk--)
                Serial.printf("%02X ", *string_ptr++); //NOLINT
            Serial.println("");

            Serial.printf("vcf_data.interface_data.pedal_sensor_data.accel_1 = %d\n", vcf_data.interface_data.pedal_sensor_data.accel_1);
        } else {
            Serial.printf("Did not receive VCR message!\n");
        }

        vcf_data.system_data.pedals_system_data.accel_percent += 0.01f;
        vcf_data.system_data.pedals_system_data.brake_percent += 0.02f;
        vcf_data.system_data.pedals_system_data.regen_percent += 0.03f;
        hytech_msgs_VCFData_s send_protoc_struct = VCFEthernetInterface::make_vcf_data_msg(vcf_data);

        Serial.println("Send bytes: (protoc struct value)");
        char *string_ptr = (char*) &(send_protoc_struct);
        uint32_t kk = sizeof(send_protoc_struct);
        while(kk--)
            Serial.printf("%02X ", *string_ptr++);
        Serial.println("");

        handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size>(EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().VCFData_port,
            &vcf_data_socket, send_protoc_struct, hytech_msgs_VCFData_s_fields);



        // vcf_data_socket.beginPacket(EthernetIPDefsInstance::instance().vcr_ip, EthernetIPDefsInstance::instance().VCFData_port);
        // uint8_t buffer[hytech_msgs_VCFData_s_size];
        // buffer[4] = 6;
        // buffer[5] = 7;
        // buffer[6] = 0;
        // vcf_data_socket.write(buffer, hytech_msgs_VCFData_s_size);
        // vcf_data_socket.endPacket();

        // Serial.println("Sent bytes (write buffer contents)");
        // *string_ptr = (char*) &(buffer);
        // kk = sizeof(buffer);
        // while(kk--)
        //     Serial.printf("%02X ", *string_ptr++);
        // Serial.println("\n");
        
        

        delay(500);
    }
}