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
HT_TASK::Task async_main(HT_TASK::DUMMY_FUNCTION, &async_tasks::handle_async_main, MAIN_TASK_PRIORITY, MAIN_TASK_PERIOD);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, &handle_CAN_send, CAN_SEND_PRIORITY, CAN_SEND_PERIOD);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, &send_dash_data, DASH_SEND_PRIORITY, DASH_SEND_PERIOD);
HT_TASK::Task pedals_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_pedals_data, PEDALS_PRIORITY, PEDALS_SEND_PERIOD);
HT_TASK::Task adc1_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc1_task, LOADCELL_SAMPLE_PRIORITY, LOADCELL_SAMPLE_PERIOD);
HT_TASK::Task pedals_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc2_task, PEDALS_PRIORITY, PEDALS_SAMPLE_PERIOD);
HT_TASK::Task buzzer_control_task(&init_buzzer_control_task, &run_buzzer_control_task, BUZZER_PRIORITY, BUZZER_WRITE_PERIOD);
HT_TASK::Task read_dash_GPIOs_task(HT_TASK::DUMMY_FUNCTION, &run_dash_GPIOs_task, DASH_SAMPLE_PRIORITY, DASH_SAMPLE_PERIOD);
HT_TASK::Task read_ioexpander_task(&create_ioexpander, &read_ioexpander, DASH_SAMPLE_PRIORITY, DASH_SAMPLE_PERIOD);
HT_TASK::Task neopixels_task(&init_neopixels_task, &run_update_neopixels_task, NEOPIXEL_UPDATE_PRIORITY, NEOPIXEL_UPDATE_PERIOD);
HT_TASK::Task ethernet_send_task(init_handle_send_vcf_ethernet_data, run_handle_send_vcf_ethernet_data, ETHERNET_SEND_PRIORITY, ETHERNET_SEND_PERIOD);
HT_TASK::Task steering_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_steering_data, STEERING_SEND_PRIORITY, STEERING_SEND_PERIOD);
HT_TASK::Task front_suspension_message_enqueue(HT_TASK::DUMMY_FUNCTION, &enqueue_front_suspension_data, LOADCELL_SEND_PRIORITY, LOADCELL_SEND_PERIOD);

HT_TASK::Task kick_watchdog_task(&init_kick_watchdog, &run_kick_watchdog, WATCHDOG_PRIORITY, WATCHDOG_KICK_PERIOD); 
HT_TASK::Task pedals_calibration_task(HT_TASK::DUMMY_FUNCTION, &update_pedals_calibration_task, PEDALS_RECALIBRATION_PRIORITY, PEDALS_RECALIBRATION_PERIOD); 


HT_TASK::TaskResponse debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Serial.println("accel1 raw accel2 raw");
    // Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_1);
    // Serial.print("   ");
    // Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_2);
    // Serial.println();
    // Serial.println("brake1 raw brake2 raw");
    // Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_1);
    // Serial.print("   ");
    // Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_2);
    // Serial.println();
    // Serial.println("accel brake percents");
    // Serial.print(VCFData_sInstance::instance().system_data.pedals_system_data.accel_percent);
    // Serial.print("   ");
    // Serial.print(VCFData_sInstance::instance().system_data.pedals_system_data.brake_percent);
    // Serial.println();
    // Serial.println("implaus");
    // Serial.println(VCFData_sInstance::instance().system_data.pedals_system_data.implausibility_has_exceeded_max_duration);

    // Serial.println("accel 1 min/max");
    // Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_1);
    // Serial.print("   ");
    // Serial.print(PedalsSystemInstance::instance().get_accel_params().max_pedal_1);
    // Serial.println();
    // Serial.println("accel 2 min/max");
    // Serial.print(PedalsSystemInstance::instance().get_accel_params().min_pedal_2);
    // Serial.print("   ");
    // Serial.print(PedalsSystemInstance::instance().get_accel_params().max_pedal_2);
    // Serial.println();
    // Serial.println("brake 1 min/max");
    // Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_1);
    // Serial.print("   ");
    // Serial.print(PedalsSystemInstance::instance().get_brake_params().max_pedal_1);
    // Serial.println();
    // Serial.println("brake 2 min/max");
    // Serial.print(PedalsSystemInstance::instance().get_brake_params().min_pedal_2);
    // Serial.print("   ");
    // Serial.print(PedalsSystemInstance::instance().get_brake_params().max_pedal_2);
    // Serial.println();
    // Serial.println();
    
    // Serial.print("Load Cell FR:  ");
    // Serial.println(VCFData_sInstance::instance().interface_data.front_loadcell_data.FR_loadcell_analog);
    // Serial.print("Load Cell FL:  ");
    // Serial.println(VCFData_sInstance::instance().interface_data.front_loadcell_data.FL_loadcell_analog);
    // Serial.print("Suspot FR:  ");
    // Serial.println(VCFData_sInstance::instance().interface_data.front_suspot_data.FR_sus_pot_analog);
    // Serial.print("Suspot FL:  ");
    // Serial.println(VCFData_sInstance::instance().interface_data.front_suspot_data.FL_sus_pot_analog);
    

    // Serial.print("Dim button: ");
    // Serial.println(VCFData_sInstance::instance().interface_data.dash_input_state.dim_btn_is_pressed);
    // Serial.print("preset button: ");
    // Serial.println(VCFData_sInstance::instance().interface_data.dash_input_state.preset_btn_is_pressed);
    // Serial.print("mc reset button: ");
    // Serial.println(VCFData_sInstance::instance().interface_data.dash_input_state.mc_reset_btn_is_pressed);
    // Serial.print("mode button: ");
    // Serial.println(VCFData_sInstance::instance().interface_data.dash_input_state.mode_btn_is_pressed);
    // Serial.print("start button: ");
    // Serial.println(VCFData_sInstance::instance().interface_data.dash_input_state.start_btn_is_pressed);
    // Serial.print("data button: ");
    // Serial.println(VCFData_sInstance::instance().interface_data.dash_input_state.data_btn_is_pressed);

    // Serial.println("jkkjhhkjkjh");

    return HT_TASK::TaskResponse::YIELD;
}


HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, debug_print, DEBUG_PRIORITY, DEBUG_PERIOD);

void setup() {

    SPI.begin();
    Serial.begin(115200); // NOLINT

    // Initialize all singletons
    ADCInterfaceInstance::create(
        ADCPinout_s {
            ADC1_CS,
            ADC2_CS
        },
        ADCChannels_s {
            STEERING_1_CHANNEL,
            STEERING_2_CHANNEL,
            FR_LOADCELL_CHANNEL,
            FL_LOADCELL_CHANNEL,
            FR_SUS_POT_CHANNEL,
            FL_SUS_POT_CHANNEL,
            ACCEL_1_CHANNEL,
            ACCEL_2_CHANNEL,
            BRAKE_1_CHANNEL,
            BRAKE_2_CHANNEL
        },
        ADCScales_s { 
            STEERING_1_SCALE, 
            STEERING_2_SCALE, 
            FR_LOADCELL_SCALE,
            FL_LOADCELL_SCALE,
            FR_SUS_POT_SCALE,
            FL_SUS_POT_SCALE, 
            ACCEL_1_SCALE, 
            ACCEL_2_SCALE, 
            BRAKE_1_SCALE, 
            BRAKE_2_SCALE
        }, 
        ADCOffsets_s {
            STEERING_1_OFFSET,
            STEERING_2_OFFSET,
            FR_LOADCELL_OFFSET,
            FL_LOADCELL_OFFSET,
            FR_SUS_POT_OFFSET,
            FL_SUS_POT_OFFSET,
            ACCEL_1_OFFSET,
            ACCEL_2_OFFSET,
            BRAKE_1_OFFSET,
            BRAKE_2_OFFSET
        }
    );

    EthernetIPDefsInstance::create();
    VCRData_sInstance::create();
    VCFData_sInstance::create();

    // Create pedals singleton
    PedalsParams accel_params = {
        .min_pedal_1 = EEPROMUtilities::read_eeprom_32bit(ACCEL_1_MIN_ADDR),
        .min_pedal_2 = EEPROMUtilities::read_eeprom_32bit(ACCEL_2_MIN_ADDR),
        .max_pedal_1 = EEPROMUtilities::read_eeprom_32bit(ACCEL_1_MAX_ADDR),
        .max_pedal_2 = 
        EEPROMUtilities::read_eeprom_32bit(ACCEL_2_MAX_ADDR),
        .activation_percentage = 0.10, // NOLINT
        .min_sensor_pedal_1 = 90, // NOLINT
        .min_sensor_pedal_2 = 90, // NOLINT
        .max_sensor_pedal_1 = 4000, // NOLINT
        .max_sensor_pedal_2 = 4000, // NOLINT
        .deadzone_margin = .03, // NOLINT
        .implausibility_margin = IMPLAUSIBILITY_PERCENT,
        .mechanical_activation_percentage = 0.05 // NOLINT
    };
    
    PedalsParams brake_params = {
        .min_pedal_1 = EEPROMUtilities::read_eeprom_32bit(BRAKE_1_MIN_ADDR),
        .min_pedal_2 = EEPROMUtilities::read_eeprom_32bit(BRAKE_2_MIN_ADDR),
        .max_pedal_1 = EEPROMUtilities::read_eeprom_32bit(BRAKE_1_MAX_ADDR),
        .max_pedal_2 = EEPROMUtilities::read_eeprom_32bit(BRAKE_2_MAX_ADDR),
        .activation_percentage = 0.50, // NOLINT
        .min_sensor_pedal_1 = 90, // NOLINT
        .min_sensor_pedal_2 = 90, // NOLINT
        .max_sensor_pedal_1 = 4000, // NOLINT
        .max_sensor_pedal_2 = 4000, // NOLINT
        .deadzone_margin = .04, //NOLINT
        .implausibility_margin = IMPLAUSIBILITY_PERCENT,
        .mechanical_activation_percentage = 0.5 //NOLINT
    };

    PedalsSystemInstance::create(accel_params, brake_params); //pass in the two different params
    
    // Create dashboard singleton
    DashboardGPIOs_s dashboard_gpios = {
        .DIM_BUTTON = BTN_DIM_READ,
        .PRESET_BUTTON = BTN_PRESET_READ,
        .MC_CYCLE_BUTTON = BTN_MC_CYCLE_READ,
        .MODE_BUTTON = BTN_MODE_READ,
        .START_BUTTON = BTN_START_READ,
        .DATA_BUTTON = BTN_DATA_READ,
        .LEFT_SHIFTER_BUTTON = LEFT_SHIFTER,
        .RIGHT_SHIFTER_BUTTON = RIGHT_SHIFTER,
    };

    DashboardInterfaceInstance::create(dashboard_gpios); //NOLINT (linter things dashboard_gpios is not initialized)
    ACUInterfaceInstance::create();
    VCRInterfaceInstance::create();
    // Create can singletons
    VCFCANInterfaceImpl::CANInterfacesInstance::create(DashboardInterfaceInstance::instance(), ACUInterfaceInstance::instance(), VCRInterfaceInstance::instance()); 
    auto main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv, &main_can); // NOLINT (Not sure why it's uninitialized. I think it is.)

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