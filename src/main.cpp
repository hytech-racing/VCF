#ifdef ARDUINO
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"
#include "ht_task.hpp"

#include <Arduino.h>

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


/* CAN Interface stuff */
#include "VCFCANInterfaceImpl.h"
#include "CANInterface.h"
// #include "VCFEthernetInterface.h"
#include "ht_sched.hpp"
#include "ht_task.hpp"

#include "hytech.h"

/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> main_can;

const PedalsParams accel_params = {
    .min_pedal_1 = 1831,
    .min_pedal_2 = 1735,
    .max_pedal_1 = 2926,
    .max_pedal_2 = 650,
    .activation_percentage = 0.05,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .03,
    .implausibility_margin = IMPLAUSIBILITY_PERCENT,
    .mechanical_activation_percentage = 0.05
};

const PedalsParams brake_params = {
    .min_pedal_1 = 1203,
    .min_pedal_2 = 2324,
    .max_pedal_1 = 1642,
    .max_pedal_2 = 1855,
    .activation_percentage = 0.06,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .04,
    .implausibility_margin = IMPLAUSIBILITY_PERCENT,
    .mechanical_activation_percentage = 0.65
};

// Tasks
// Send Periods
constexpr unsigned long dash_send_period = 40000;             // 4 000 us = 250 Hz
constexpr unsigned long dash_receive_period = 4000;
HT_TASK::Task async_main(HT_TASK::DUMMY_FUNCTION, &async_tasks::handle_async_main, MAIN_TASK_PRIORITY);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, &handle_CAN_send, CAN_SEND_PRIORITY, CAN_SEND_PERIOD);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, &send_dash_data, DASH_SEND_PRIORITY, DASH_SEND_PERIOD);
HT_TASK::Task pedals_message_enqueue(HT_TASK::DUMMY_FUNCTION, &send_pedals_data, PEDALS_PRIORITY, PEDALS_SAMPLE_PERIOD);
HT_TASK::Task pedals_sample(HT_TASK::DUMMY_FUNCTION, &run_read_adc2_task, PEDALS_PRIORITY, PEDALS_SEND_PERIOD);
HT_TASK::Task dash_CAN_receive(HT_TASK::DUMMY_FUNCTION, &receive_dash_inputs, PEDALS_PRIORITY, dash_receive_period);
HT_TASK::Task buzzer_control_task(&init_buzzer_control_task, &run_buzzer_control_task, BUZZER_PRIORITY, BUZZER_WRITE_PERIOD);

bool debug_print(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    Serial.println("accel1 raw accel2 raw");
    Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_1);
    Serial.print("   ");
    Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_2);
    Serial.println();
    Serial.println("brake1 raw brake2 raw");
    Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_1);
    Serial.print("   ");
    Serial.print(VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_2);
    Serial.println();
    Serial.println("accel brake percents");
    Serial.print(VCFData_sInstance::instance().system_data.pedals_system_data.accel_percent);
    Serial.print("   ");
    Serial.print(VCFData_sInstance::instance().system_data.pedals_system_data.brake_percent);
    Serial.println();
    Serial.println("implaus");
    Serial.print(VCFData_sInstance::instance().system_data.pedals_system_data.implausibility_has_exceeded_max_duration);
    return true;
}


HT_TASK::Task debug_state_print_task(HT_TASK::DUMMY_FUNCTION, debug_print, DEBUG_PRIORITY, DEBUG_PERIOD);

void setup() {
    SPI.begin();
    Serial.begin(115200); // NOLINT

    EthernetIPDefsInstance::create();
    
    VCRData_sInstance::create();
    VCFData_sInstance::create();
    PedalsSystemInstance::create(accel_params, brake_params); //pass in the two different params


    float adc_1_scales[channels_within_mcp_adc], adc_1_offsets[channels_within_mcp_adc], adc_2_scales[channels_within_mcp_adc], adc_2_offsets[channels_within_mcp_adc];
    adc_1_scales[STEERING_1_CHANNEL] = STEERING_1_SCALE;
    adc_1_offsets[STEERING_1_CHANNEL] = STEERING_1_OFFSET;
    adc_1_scales[STEERING_2_CHANNEL] = STEERING_2_SCALE;
    adc_1_offsets[STEERING_2_CHANNEL] = STEERING_2_OFFSET;
    adc_1_scales[FR_SUS_POT_CHANNEL] = FR_SUS_POT_SCALE;
    adc_1_offsets[FR_SUS_POT_CHANNEL] = FR_SUS_POT_OFFSET;
    adc_1_scales[FL_SUS_POT_CHANNEL] = FL_SUS_POT_SCALE;
    adc_1_offsets[FL_SUS_POT_CHANNEL] = FL_SUS_POT_OFFSET;
    adc_1_scales[FR_LOADCELL_CHANNEL] = FR_LOADCELL_SCALE;
    adc_1_offsets[FR_LOADCELL_CHANNEL] = FR_LOADCELL_OFFSET;
    adc_1_scales[FL_LOADCELL_CHANNEL] = FL_LOADCELL_SCALE;
    adc_1_offsets[FL_LOADCELL_CHANNEL] = FL_LOADCELL_OFFSET;

    adc_2_scales[ACCEL_1_CHANNEL] = ACCEL_1_SCALE;
    adc_2_offsets[ACCEL_1_CHANNEL] = ACCEL_1_OFFSET;
    adc_2_scales[ACCEL_2_CHANNEL] = ACCEL_2_SCALE;
    adc_2_offsets[ACCEL_2_CHANNEL] = ACCEL_2_OFFSET;
    adc_2_scales[BRAKE_1_CHANNEL] = BRAKE_1_SCALE;
    adc_2_offsets[BRAKE_1_CHANNEL] = BRAKE_1_OFFSET;
    adc_2_scales[BRAKE_2_CHANNEL] = BRAKE_2_SCALE;
    adc_2_offsets[BRAKE_2_CHANNEL] = BRAKE_2_OFFSET;

    ADCsOnVCFInstance::create(adc_1_scales, adc_1_offsets, adc_2_scales, adc_2_offsets);
    // Setup scheduler
    
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
        .DIAL_SDA = I2C_SDA,
        .DIAL_SCL = I2C_SCL
    };

    // Create can singletons
    DashboardInterfaceInstance::create(dashboard_gpios);
    VCFCANInterfaceImpl::CANInterfacesInstance::create(DashboardInterfaceInstance::instance()); 
    auto main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv, &main_can); // NOLINT (Not sure why it's uninitialized. I think it is.)
    

    // hardware setup
    
    Serial.begin(115200); // NOLINT (common baud rate)

    // qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    VCFCANInterfaceObjects can_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    const uint32_t CAN_baudrate = 500000;
    handle_CAN_setup(main_can, CAN_baudrate, &VCFCANInterfaceImpl::on_main_can_recv);
    
    // qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    // Setup scheduler
    scheduler.setTimingFunction(micros);

    // Schedule Tasks
    scheduler.schedule(async_main); 
    scheduler.schedule(CAN_send);
    
    scheduler.schedule(dash_CAN_enqueue);
    // scheduler.schedule(dash_CAN_receive);
    
    scheduler.schedule(pedals_message_enqueue);
    scheduler.schedule(pedals_sample);
    // scheduler.schedule(debug_state_print_task);

}

void loop() {
    scheduler.run();
}