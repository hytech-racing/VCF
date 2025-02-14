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



/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;

const IPAddress debug_ip(192, 168, 1, 31); // Computer receive IP
const IPAddress default_VCF_ip(192, 168, 1, 30);
const IPAddress default_dns(192, 168, 1, 1); 
const IPAddress default_gateway(192, 168, 1, 1); 
const IPAddress car_subnet(255, 255, 255, 30); 

uint16_t port = 7777;

void setup() {
    scheduler.setTimingFunction(micros);

    scheduler.schedule(read_adc1_task);
    scheduler.schedule(read_adc2_task);
    scheduler.schedule(buzzer_control_task);
    scheduler.schedule(send_vcf_data_task);

    qindesign::network::Ethernet.begin(default_VCF_ip, default_dns, default_gateway, car_subnet);
    protobuf_send_socket.begin(port);
}

void loop() {
    scheduler.run();
}