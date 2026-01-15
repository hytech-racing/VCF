#ifdef ARDUINO
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"
#include "ht_task.hpp"

#include <Arduino.h>
#include <EEPROM.h>

/* From Arduino Libraries */
#include "hytech.h"
#include "QNEthernet.h"

/* Local includes */
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include "VCF_Tasks.h"
#include "PedalsSystem.h"
#include "DashboardInterface.h"
#include "VCFEthernetInterface.h"
#include "WatchdogSystem.h"
#include "EEPROMUtilities.h"



/* CAN Interface stuff */
#include "VCFCANInterfaceImpl.h"
#include "CANInterface.h"
// #include "VCFEthernetInterface.h"
#include "ht_sched.hpp"
#include "ht_task.hpp"

#include "hytech.h"

// Tasks
HT_TASK::Task async_main(HT_TASK::DUMMY_FUNCTION, &async_tasks::handle_async_main, VCFTaskConstants::MAIN_TASK_PRIORITY, VCFTaskConstants::MAIN_TASK_PERIOD);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, &handle_CAN_send, VCFTaskConstants::CAN_SEND_PRIORITY, VCFTaskConstants::CAN_SEND_PERIOD);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, &send_dash_data, VCFTaskConstants::DASH_SEND_PRIORITY, VCFTaskConstants::DASH_SEND_PERIOD);
HT_TASK::Task pedals_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_pedals_data, VCFTaskConstants::PEDALS_PRIORITY, VCFTaskConstants::PEDALS_SEND_PERIOD);
HT_TASK::Task adc1_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc1_task, VCFTaskConstants::LOADCELL_SAMPLE_PRIORITY, VCFTaskConstants::LOADCELL_SAMPLE_PERIOD);
HT_TASK::Task pedals_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc2_task, VCFTaskConstants::PEDALS_PRIORITY, VCFTaskConstants::PEDALS_SAMPLE_PERIOD);
HT_TASK::Task buzzer_control_task(&init_buzzer_control_task, &run_buzzer_control_task, VCFTaskConstants::BUZZER_PRIORITY, VCFTaskConstants::BUZZER_WRITE_PERIOD);
HT_TASK::Task read_dash_GPIOs_task(HT_TASK::DUMMY_FUNCTION, &run_dash_GPIOs_task, VCFTaskConstants::DASH_SAMPLE_PRIORITY, VCFTaskConstants::DASH_SAMPLE_PERIOD);
HT_TASK::Task read_ioexpander_task(&create_ioexpander, &read_ioexpander, VCFTaskConstants::DASH_SAMPLE_PRIORITY, VCFTaskConstants::DASH_SAMPLE_PERIOD);
HT_TASK::Task read_digital_steering_sensor(HT_TASK::DUMMY_FUNCTION, &run_read_digital_steering_sensor, VCFTaskConstants::DEBUG_PRIORITY, VCFTaskConstants::DEBUG_PERIOD);
HT_TASK::Task neopixels_task(&init_neopixels_task, &run_update_neopixels_task, VCFTaskConstants::NEOPIXEL_UPDATE_PRIORITY, VCFTaskConstants::NEOPIXEL_UPDATE_PERIOD);
HT_TASK::Task ethernet_send_task(init_handle_send_vcf_ethernet_data, run_handle_send_vcf_ethernet_data, VCFTaskConstants::ETHERNET_SEND_PRIORITY, VCFTaskConstants::ETHERNET_SEND_PERIOD);
HT_TASK::Task steering_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_steering_data, VCFTaskConstants::STEERING_SEND_PRIORITY, VCFTaskConstants::STEERING_SEND_PERIOD);
HT_TASK::Task front_suspension_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_front_suspension_data, VCFTaskConstants::LOADCELL_SEND_PRIORITY, VCFTaskConstants::LOADCELL_SEND_PERIOD);

HT_TASK::Task kick_watchdog_task(&init_kick_watchdog, &run_kick_watchdog, VCFTaskConstants::WATCHDOG_PRIORITY, VCFTaskConstants::WATCHDOG_KICK_PERIOD); 
HT_TASK::Task pedals_calibration_task(HT_TASK::DUMMY_FUNCTION, &update_pedals_calibration_task, VCFTaskConstants::PEDALS_RECALIBRATION_PRIORITY, VCFTaskConstants::PEDALS_RECALIBRATION_PERIOD); 

HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, &debug_print, VCFTaskConstants::DEBUG_PRIORITY, VCFTaskConstants::DEBUG_PERIOD);

void setup() {
    setup_all_interfaces(); //must be first (if we ever have a setup systems)

    // Setup scheduler
    HT_SCHED::Scheduler::getInstance().setTimingFunction(micros);

    // Schedule Tasks
    HT_SCHED::Scheduler::getInstance().schedule(kick_watchdog_task);
    HT_SCHED::Scheduler::getInstance().schedule(async_main); 
    HT_SCHED::Scheduler::getInstance().schedule(CAN_send);
    HT_SCHED::Scheduler::getInstance().schedule(dash_CAN_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(buzzer_control_task);
    HT_SCHED::Scheduler::getInstance().schedule(pedals_message_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(adc1_sample);
    HT_SCHED::Scheduler::getInstance().schedule(pedals_sample);
    HT_SCHED::Scheduler::getInstance().schedule(read_dash_GPIOs_task);
    HT_SCHED::Scheduler::getInstance().schedule(read_ioexpander_task);
    HT_SCHED::Scheduler::getInstance().schedule(neopixels_task);
    HT_SCHED::Scheduler::getInstance().schedule(steering_message_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(front_suspension_message_enqueue);
    HT_SCHED::Scheduler::getInstance().schedule(debug_state_print_task);
    HT_SCHED::Scheduler::getInstance().schedule(pedals_calibration_task);
    HT_SCHED::Scheduler::getInstance().schedule(ethernet_send_task);
    HT_SCHED::Scheduler::getInstance().schedule(read_digital_steering_sensor);
}

void loop() {
    HT_SCHED::Scheduler::getInstance().run();
}