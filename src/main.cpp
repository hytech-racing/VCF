#ifdef ARDUINO
#include <Arduino.h>
#endif



/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"


/* From Arduino Libraries */
#include "QNEthernet.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"



/* Scheduler setup */




/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;



void setup() {
    scheduler.setTimingFunction(micros);

    scheduler.enable(read_adc1_task);
    scheduler.enable(read_adc2_task);
    scheduler.enable(read_gpio_task);
    scheduler.enable(buzzer_control_task);
}

void loop() {
    scheduler.run();
}