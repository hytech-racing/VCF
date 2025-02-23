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

// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use 
constexpr unsigned long TASK_INTERVAL_PERIOD = 1000; // in us

/* Task declarations */
// from https://github.com/arkhipenko/TaskScheduler/wiki/API-Task#task note that we will use 
TsTask read_adc_task(TASK_INTERVAL_PERIOD, TASK_FOREVER, &run_read_adc1_task, &task_scheduler, false, &init_adc_task);// 1000us is 1kHz //NOLINT
TsTask read_gpio_task(TASK_INTERVAL_PERIOD, TASK_FOREVER, &run_read_gpio_task, &task_scheduler, false, &init_read_gpio_task); 
TsTask buzzer_control_task(TASK_INTERVAL_PERIOD, TASK_FOREVER, &run_buzzer_control_task, &task_scheduler, false, &init_buzzer_control_task);


/* Ethernet message sockets */ // TODO: Move this into its own interface
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;



void setup() {
    read_adc_task.enable();
    read_gpio_task.enable();
    buzzer_control_task.enable();
}

void loop() {
    task_scheduler.execute();
}