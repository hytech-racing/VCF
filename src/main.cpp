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
    .min_pedal_1 = 1790,
    .min_pedal_2 = 1690,
    .max_pedal_1 = 2830,
    .max_pedal_2 = 670,
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
    .min_pedal_1 = 1180,
    .min_pedal_2 = 2500,
    .max_pedal_1 = 1660,
    .max_pedal_2 = 1770,
    .activation_percentage = 0.05,
    .min_sensor_pedal_1 = 90,
    .min_sensor_pedal_2 = 90,
    .max_sensor_pedal_1 = 4000,
    .max_sensor_pedal_2 = 4000,
    .deadzone_margin = .03,
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
HT_TASK::Task dash_CAN_receive(HT_TASK::DUMMY_FUNCTION, &receive_dash_inputs, 5, dash_receive_period);

    



void setup() {
    Serial.begin(115200);


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
    VCFCANInterfaceImpl::CANInterfacesInstance::create(DashboardInterfaceInstance::instance()); 
    auto main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv, &main_can); // NOLINT (Not sure why it's uninitialized. I think it is.)
    

    // hardware setup
    
    Serial.begin(115200); // NOLINT (common baud rate)

    qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    VCFCANInterfaceObjects can_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    const uint32_t CAN_baudrate = 500000;
    handle_CAN_setup(main_can, CAN_baudrate, &VCFCANInterfaceImpl::on_main_can_recv);
    
    qindesign::network::Ethernet.begin(EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);
    // Setup scheduler
    scheduler.setTimingFunction(micros);

    // Schedule Tasks
    scheduler.schedule(async_main); 
    scheduler.schedule(CAN_send);
    
    scheduler.schedule(dash_CAN_enqueue);
    scheduler.schedule(dash_CAN_receive);
    
    scheduler.schedule(pedals_message_enqueue);
    scheduler.schedule(pedals_sample);
    

    
}

void loop() {
    scheduler.run();
    // DashboardInterface inst = DashboardInterfaceInstance::instance(); 
    // DashInputState_s dash_outputs = iSerial.println("sending");nst.get_dashboard_outputs(); 
    // Serial.println(dash_outputs.start_btn_is_pressed);
    // handle_CAN_receive(); 
    // handle_CAN_send();
    // send_dash_data(); 
    // // CAN_message_t msg;
    // // msg.id = 3;
    // // VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().MAIN_CAN.write(msg);
    // // Serial.println("pluh");
}