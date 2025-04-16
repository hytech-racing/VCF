/**
 * This file includes all of the Task definitions required for the Task
 * Scheduler. See the Task Scheduler GitHub
 * (https://github.com/hytech-racing/HT_SCHED) and the relevant BookStack page
 * (https://wiki.hytechracing.org/books/ht09-design/page/ht-task-scheduler) for
 * usage directions.
 *
 * Generally, defining a task takes three steps:
 * 1) Define the "init" function. Name this function init_<taskname>. This init
 * funciton's inputs MUST be the same for all init functions, taking in
 * sysMicros and a taskInfo reference. 2) Define the "run" function. Name this
 * function run_<taskname>. Similar to the init function, this function's inputs
 * MUST be sysMicros and taskInfo. 3) Define the function itself. This requires
 * using the HT_TASK::Task constructor and passing in your init function, your
 * run function, a priority level, and a loop interval (in micros). 4) Add the
 * function to your scheduler.
 *
 */

/* -------------------------------------------------- */
/*                   TASK PRIORITIES                  */
/* -------------------------------------------------- */
/*
 * (1-10) - Car-critical functions (watchdog, shutdown circuit)
 * (11-100) - Performance-critical functions (reading pedals, interfacing w/
 * inverters, etc) (100+) - Telemetry, logging functions (could be idle
 * functions, too)
 */

#ifndef VCF_TASKS
#define VCF_TASKS

#include "SharedFirmwareTypes.h"
#include "VCFCANInterfaceImpl.h"
#include "VCFEthernetInterface.h"
#include "ht_sched.hpp"
#include "ht_task.hpp"
#include "BuzzerController.h"
#include "IOExpanderUtils.h"
#include "NeopixelController.h"
#include "WatchdogSystem.h"

/**
 * The read_adc1 task will command adc1 to sample all eight channels, convert
 * the outputs, and store them in structs defined in shared_firmware_types. This
 * function relies on adc_1 being defined in VCFGlobals.h.
 */
HT_TASK::TaskResponse init_adc_task();
HT_TASK::TaskResponse run_read_adc1_task();

HT_TASK::TaskResponse run_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse update_pedals_calibration_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You
 * may edit this manually to add sensors.
 *
 * The read_adc2 task will command adc2 to sample all eight channels, convert
 * the outputs, and store them in a struct defined in shared_firmware_types.
 * This function relies on adc_2 being defined in VCFGlobals.h.
 */
HT_TASK::TaskResponse run_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * The buzzer_control task will control the buzzer control pin. This function
 * relies on the buzzer_control pin definition in VCF_Constants.h;
 */
HT_TASK::TaskResponse init_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_buzzer_control_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * The handle_send_VCF_ethernet_data task will send a protobuf message from VCF
 * to a destination port defined in EthernetAddressDefs. This function relies on
 * the VCF (sending) socket and vcf_data defined in VCFGlobals.h, and Ethernet
 * constants defined in EthernetAddressDefs.h.
 *
 */
HT_TASK::TaskResponse init_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_handle_send_vcf_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

/**
 * The handle_receive_VCR_ethernet_data task will receive a protobuf message
 * from VCR. This function relies on the VCF (receiving) socket and vcf_data
 * defined in VCFGlobals.h, and Ethernet constants defined in
 * EthernetAddressDefs.h.
 *
 */
HT_TASK::TaskResponse init_handle_receive_vcr_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);
HT_TASK::TaskResponse run_handle_receive_vcr_ethernet_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse send_dash_data(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo);

HT_TASK::TaskResponse enqueue_pedals_data(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info);

// this task attempts to send any data that is enqueued at 250hz. this will be the max rate that you can send over the CAN bus.
// you dont have to enqeue at this rate, but this allows us to have 2 layers of rate limiting on CAN sending
HT_TASK::TaskResponse handle_CAN_send(const unsigned long &sysMicros, const HT_TASK::TaskInfo &taskInfo); // NOLINT (capitalization of CAN)

HT_TASK::TaskResponse handle_CAN_receive(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalization of CAN)

HT_TASK::TaskResponse run_dash_GPIOs_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info); // NOLINT (capitalization of GPIOs)

HT_TASK::TaskResponse create_ioexpander(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
HT_TASK::TaskResponse read_ioexpander(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);

HT_TASK::TaskResponse init_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
HT_TASK::TaskResponse run_update_neopixels_task(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);

HT_TASK::TaskResponse enqueue_front_suspension_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse enqueue_steering_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

HT_TASK::TaskResponse init_kick_watchdog(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

namespace async_tasks {
    // the others in the VCF Tasks can just stay there, they dont need forward declarations.
    HT_TASK::TaskResponse handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info);
}
namespace setup_handlers {
    void setup_hardware();
}

#endif /* VCF_TASKS */
