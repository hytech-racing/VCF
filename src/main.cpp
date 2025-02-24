#ifdef ARDUINO
#include <Arduino.h>
#endif



/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"
#include "VCFEthernetInterface.h"

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

void setup() {
    std::function<unsigned long()> func = micros;
    scheduler.setTimingFunction(func);

    scheduler.schedule(read_adc1_task);
    scheduler.schedule(read_adc2_task);
    scheduler.schedule(buzzer_control_task);
    scheduler.schedule(send_vcf_data_task);
    scheduler.schedule(recv_vcr_data_task);

    

    qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    protobuf_send_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);
    protobuf_recv_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);
    
}

void loop() {
    scheduler.run();
    Serial.println("looping");
}