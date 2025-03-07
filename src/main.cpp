#ifdef ARDUINO
#include <Arduino.h>
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

/* From HT_SCHED libdep */
// #include "ht_sched.hpp"

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"
#include "VCFEthernetInterface.h"
#include "ProtobufMsgInterface.h"

/* Scheduler setup */
// HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

using namespace qindesign::network;
EthernetUDP vcf_data_socket;
EthernetUDP vcr_data_socket;

void setup()
{
    Serial.begin(115200);
    EthernetIPDefsInstance::create();
    uint8_t mac[6];
    Ethernet.macAddress(mac);
    Ethernet.begin(mac, EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    init_handle_receive_vcr_ethernet_data();
    init_handle_send_vcf_ethernet_data();
}

void loop()
{
    run_handle_send_vcf_ethernet_data();
    run_handle_receive_vcr_ethernet_data();
    delay(1000);
}