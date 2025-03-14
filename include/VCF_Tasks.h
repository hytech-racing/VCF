/**
 * This file includes all of the Task definitions required for the Task Scheduler. See the Task
 * Scheduler GitHub (https://github.com/hytech-racing/HT_SCHED) and the relevant BookStack page
 * (https://wiki.hytechracing.org/books/ht09-design/page/ht-task-scheduler) for usage directions.
 * 
 * Generally, defining a task takes three steps:
 * 1) Define the "init" function. Name this function init_<taskname>. This init funciton's inputs
 *    MUST be the same for all init functions, taking in sysMicros and a taskInfo reference.
 * 2) Define the "run" function. Name this function run_<taskname>. Similar to the init function,
 *    this function's inputs MUST be sysMicros and taskInfo.
 * 3) Define the function itself. This requires using the HT_TASK::Task constructor and passing
 *    in your init function, your run function, a priority level, and a loop interval (in micros).
 * 4) Add the function to your scheduler.
 * 
 */

/* -------------------------------------------------- */
/*                   TASK PRIORITIES                  */
/* -------------------------------------------------- */
/*
 * (1-10) - Car-critical functions (watchdog, shutdown circuit)
 * (11-100) - Performance-critical functions (reading pedals, interfacing w/ inverters, etc)
 * (100+) - Telemetry, logging functions (could be idle functions, too)
 */

#ifndef VCF_TASKS
#define VCF_TASKS
 
 
#include "SharedFirmwareTypes.h"
#include "VCFCANInterfaceImpl.h"
#include "VCFEthernetInterface.h"
#include "ht_sched.hpp"
#include "ht_task.hpp"

/**
 * The read_adc1 task will command adc1 to sample all eight channels, convert the outputs, and
 * store them in structs defined in shared_firmware_types. This function relies on adc_1 being
 * defined in VCFGlobals.h.
 */
bool init_adc_task();
bool run_read_adc1_task();
 
/**
 * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
 * manually to add sensors.
 * 
 * The read_adc2 task will command adc2 to sample all eight channels, convert the outputs, and
 * store them in a struct defined in shared_firmware_types. This function relies on adc_2 being
 * defined in VCFGlobals.h.
 */
bool run_read_adc2_task();
 
 
/**
 * The buzzer_control task will control the buzzer control pin. This function relies on the 
 * buzzer_control pin definition in VCF_Constants.h;
 */
bool init_buzzer_control_task();
bool run_buzzer_control_task();
 
 
/**
 * The read_gpio task will read the GPIO pins and store the results in a struct defined in 
 * shared_firmware_types. This function relies on the GPIO pins being defined in VCFConstants.h.
 */
bool init_read_gpio_task();
bool run_read_gpio_task();


/**
 * The handle_send_VCF_ethernet_data task will send a protobuf message from VCF to a destination 
 * port defined in EthernetAddressDefs. This function relies on the VCF (sending) socket and vcf_data 
 * defined in VCFGlobals.h, and Ethernet constants defined in EthernetAddressDefs.h.
 * 
*/
bool init_handle_send_vcf_ethernet_data();
bool run_handle_send_vcf_ethernet_data();


 /**
 * The handle_receive_VCR_ethernet_data task will receive a protobuf message from VCR. This function 
 * relies on the VCF (receiving) socket and vcf_data defined in VCFGlobals.h, and Ethernet constants 
 * defined in EthernetAddressDefs.h.
 * 
 */
bool init_handle_receive_vcr_ethernet_data();
bool run_handle_receive_vcr_ethernet_data();

bool handle_CAN_send(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalization of CAN)

bool handle_CAN_receive(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo); // NOLINT (capitalization of CAN)

bool send_dash_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo);

 #endif /* VCF_TASKS */