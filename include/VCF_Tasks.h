/**
 * This file includes all of the Task definitions required for the TaskScheduler library.
 * TaskScheduler is an open-source library for cooperative multitasking on Arduino platforms.
 * See the TaskScheduler GitHub (https://github.com/arkhipenko/TaskScheduler) for full documentation.
 * 
 * Generally, defining a task takes three steps:
 * 1) Define the "init" function. Name this function init_<taskname>. This function should return
 *    a boolean value indicating successful initialization.
 * 2) Define the "run" function. Name this function run_<taskname>. This function should have a void
 *    return type and will be called at the specified interval.
 * 3) Define the task itself. This requires using the TsTask constructor and passing in:
 *    - Execution interval in microseconds
 *    - Number of iterations (typically TASK_FOREVER)
 *    - Pointer to the run function
 *    - Pointer to the task scheduler
 *    - Initial enabled state (typically false)
 *    - Optional pointer to the init function
 * 4) Enable the task in the setup() function with task_name.enable()
 * 
 * Example:
 * 
 * bool init_example_task() {
 *     // Setup code here
 *     return true;
 * }
 * 
 * void run_example_task() {
 *     // Task execution code here
 * }
 * 
 * TsTask example_task(1000, TASK_FOREVER, &run_example_task, &task_scheduler, false, &init_example_task);
 */

/* -------------------------------------------------- */
/*                   TASK INTERVALS                   */
/* -------------------------------------------------- */
/*
 * Critical functions: 1-10ms (100-1000Hz)
 * Performance monitoring: 10-100ms (10-100Hz)
 * Status updates, non-critical functions: 100ms+ (<10Hz)
 */

 #ifndef VCF_TASKS
 #define VCF_TASKS
 
 
 /* From shared_firmware_types library */
 #include "SharedFirmwareTypes.h"
 
 #define _TASK_MICRO_RES
 #include <TScheduler.hpp>
 
 extern TsScheduler task_scheduler;
 
 /**
  * The read_adc1 task will command adc1 to sample all eight channels, convert the outputs, and
  * store them in structs defined in shared_firmware_types. This function relies on adc_1 being
  * defined in VCFGlobals.h.
  */
 bool init_read_adc1_task();
 void run_read_adc1_task();
 extern TsTask read_adc1_task;
 
 
 
 /**
  * NOTE: These channels are UNUSED BY DEFAULT and exist ONLY FOR TESTING. You may edit this
  * manually to add sensors.
  * 
  * The read_adc2 task will command adc2 to sample all eight channels, convert the outputs, and
  * store them in a struct defined in shared_firmware_types. This function relies on adc_2 being
  * defined in VCFGlobals.h.
  */
 bool init_read_adc2_task();
 void run_read_adc2_task();
 extern TsTask read_adc2_task;
 
 /**
  * The buzzer_control task will control the buzzer control pin. This function relies on the 
  * buzzer_control pin definition in VCF_Constants.h;
  */
 bool init_buzzer_control_task();
 void run_buzzer_control_task();
 extern TsTask buzzer_control_task;
 
 /**
  * The read_gpio task will read the GPIO pins and store the results in a struct defined in 
  * shared_firmware_types. This function relies on the GPIO pins being defined in VCFConstants.h.
  */
 bool init_read_gpio_task();
 void run_read_gpio_task();
 extern TsTask read_gpio_task;
 
 
 
 
 #endif /* VCF_TASKS */