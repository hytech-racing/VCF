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

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> main_can;

// Tasks
HT_TASK::Task async_main(HT_TASK::DUMMY_FUNCTION, &async_tasks::handle_async_main, VCFConstants::MAIN_TASK_PRIORITY, VCFConstants::MAIN_TASK_PERIOD);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, &handle_CAN_send, VCFConstants::CAN_SEND_PRIORITY, VCFConstants::CAN_SEND_PERIOD);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, &send_dash_data, VCFConstants::DASH_SEND_PRIORITY, VCFConstants::DASH_SEND_PERIOD);
HT_TASK::Task pedals_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_pedals_data, VCFConstants::PEDALS_PRIORITY, VCFConstants::PEDALS_SEND_PERIOD);
HT_TASK::Task adc1_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc1_task, VCFConstants::LOADCELL_SAMPLE_PRIORITY, VCFConstants::LOADCELL_SAMPLE_PERIOD);
HT_TASK::Task pedals_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc2_task, VCFConstants::PEDALS_PRIORITY, VCFConstants::PEDALS_SAMPLE_PERIOD);
HT_TASK::Task buzzer_control_task(&init_buzzer_control_task, &run_buzzer_control_task, VCFConstants::BUZZER_PRIORITY, VCFConstants::BUZZER_WRITE_PERIOD);
HT_TASK::Task read_dash_GPIOs_task(HT_TASK::DUMMY_FUNCTION, &run_dash_GPIOs_task, VCFConstants::DASH_SAMPLE_PRIORITY, VCFConstants::DASH_SAMPLE_PERIOD);
HT_TASK::Task read_ioexpander_task(&create_ioexpander, &read_ioexpander, VCFConstants::DASH_SAMPLE_PRIORITY, VCFConstants::DASH_SAMPLE_PERIOD);
HT_TASK::Task neopixels_task(&init_neopixels_task, &run_update_neopixels_task, VCFConstants::NEOPIXEL_UPDATE_PRIORITY, VCFConstants::NEOPIXEL_UPDATE_PERIOD);
HT_TASK::Task ethernet_send_task(init_handle_send_vcf_ethernet_data, run_handle_send_vcf_ethernet_data, VCFConstants::ETHERNET_SEND_PRIORITY, VCFConstants::ETHERNET_SEND_PERIOD);
HT_TASK::Task steering_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_steering_data, VCFConstants::STEERING_SEND_PRIORITY, VCFConstants::STEERING_SEND_PERIOD);
HT_TASK::Task front_suspension_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_front_suspension_data, VCFConstants::LOADCELL_SEND_PRIORITY, VCFConstants::LOADCELL_SEND_PERIOD);

HT_TASK::Task kick_watchdog_task(&init_kick_watchdog, &run_kick_watchdog, VCFConstants::WATCHDOG_PRIORITY, VCFConstants::WATCHDOG_KICK_PERIOD); 
HT_TASK::Task pedals_calibration_task(HT_TASK::DUMMY_FUNCTION, &update_pedals_calibration_task, VCFConstants::PEDALS_RECALIBRATION_PRIORITY, VCFConstants::PEDALS_RECALIBRATION_PERIOD); 

HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, &debug_print, VCFConstants::DEBUG_PRIORITY, VCFConstants::DEBUG_PERIOD);

void setup() {

    SPI.begin();
    Serial.begin(115200); // NOLINT

    setup_handlers::setup_hardware(&main_can);
    
    const uint32_t CAN_baudrate = 1000000;
    handle_CAN_setup(main_can, CAN_baudrate, &VCFCANInterfaceImpl::on_main_can_recv);

    // Setup scheduler
    HT_SCHED::Scheduler::getInstance().setTimingFunction(micros);

    EthernetIPDefsInstance::create();
    uint8_t mac[6]; // NOLINT (mac addresses are always 6 bytes)
    qindesign::network::Ethernet.macAddress(&mac[0]);
    qindesign::network::Ethernet.begin(mac, EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);

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
}

void loop() {
    HT_SCHED::Scheduler::getInstance().run();
}