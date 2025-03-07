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
    delay(1000);
    Serial.println("Bald1");
    EthernetIPDefsInstance::create();
    Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    Serial.println("Bald2");
    init_handle_receive_vcr_ethernet_data();
    Serial.println("Bald3");
    init_handle_send_vcf_ethernet_data();
    Serial.println("Bald4");
}

void loop()
{
    run_handle_send_vcf_ethernet_data();
    run_handle_receive_vcr_ethernet_data();
    Serial.println("Bald");
    delay(1000);
}