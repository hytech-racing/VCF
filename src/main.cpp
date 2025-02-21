#ifdef ARDUINO
#include <Arduino.h>
#endif



/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"

#define _TASK_MICRO_RES // NOLINT
#include <TScheduler.hpp>

/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"



/* Scheduler setup */
TsScheduler task_scheduler;



/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;



void setup() {
    scheduler.setTimingFunction(micros);

    scheduler.schedule(read_adc1_task);
    scheduler.schedule(read_adc2_task);
    scheduler.schedule(read_gpio_task);
    scheduler.schedule(buzzer_control_task);
}

void loop() {
    scheduler.run();
}