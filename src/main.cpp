#ifdef ARDUINO
#include <Arduino.h>
#endif



/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

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
    scheduler.setTimingFunction(micros);

    scheduler.schedule(read_adc1_task);
    scheduler.schedule(read_adc2_task);
    scheduler.schedule(buzzer_control_task);
    scheduler.schedule(send_vcf_data_task);
    scheduler.schedule(recv_vcr_data_task);

    qindesign::network::Ethernet.begin(default_VCF_ip, default_dns, default_gateway, car_subnet);
    protobuf_send_socket.begin(VCF_SEND_PORT);
    protobuf_recv_socket.begin(VCF_RECV_PORT);
    
}

void loop() {
    scheduler.run();
}