#ifdef ARDUINO
#endif


/* From shared_firmware_types libdep */
#include "SharedFirmwareTypes.h"
#include "EthernetAddressDefs.h"

/* From HT_SCHED libdep */
#include "ht_sched.hpp"
#include "ht_task.hpp"

/* From HT_CAN libdep */
#include "hytech.h"

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
#include "VCFCANInterfaceImpl.h"


/* Scheduler setup */
HT_SCHED::Scheduler& scheduler = HT_SCHED::Scheduler::getInstance();

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
HT_TASK::Task CAN_receive(HT_TASK::DUMMY_FUNCTION, handle_CAN_receive, CAN_RECV_PRIORITY);
HT_TASK::Task CAN_send(HT_TASK::DUMMY_FUNCTION, handle_CAN_send, CAN_SEND_PRIORITY);
HT_TASK::Task dash_CAN_enqueue(HT_TASK::DUMMY_FUNCTION, enqueue_dash_data, DASH_SEND_PRIORITY, DASH_SEND_PERIOD);
HT_TASK::Task pedals_update_task(HT_TASK::DUMMY_FUNCTION, update_pedals_system, PEDALS_UPDATE_PRIORITY);
HT_TASK::Task read_adc1_task(HT_TASK::DUMMY_FUNCTION, run_read_adc1_task, ADC1_TASK_PRIORITY, ADC1_SAMPLE_PERIOD);
HT_TASK::Task read_adc2_task(HT_TASK::DUMMY_FUNCTION, run_read_adc2_task, ADC2_TASK_PRIORITY, ADC2_SAMPLE_PERIOD);
HT_TASK::Task buzzer_control_task(init_buzzer_control_task, run_buzzer_control_task, BUZZER_UPDATE_PRIORITY, BUZZER_UPDATE_PERIOD);
HT_TASK::Task send_vcf_data_ethernet_task(init_handle_send_vcf_ethernet_data, run_handle_send_vcf_ethernet_data, ETHERNET_SEND_PRIORITY, ETHERNET_SEND_PERIOD);
HT_TASK::Task recv_vcr_data_ethernet_task(init_handle_receive_vcr_ethernet_data, run_handle_receive_vcr_ethernet_data, ETHERNET_RECV_PRIORITY, ETHERNET_RECV_PERIOD);

void setup() {
    Serial.begin(115200); // NOLINT (common baud rate)

    const uint32_t CAN_baudrate = 500000;

    // Setup scheduler
    scheduler.setTimingFunction(micros);

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
    DashboardInterfaceInstance::create(dashboard_gpios); // NOLINT (idk why it's saying this is uninitialized. It definitely is.)

    // Create PedalsSystem singleton
    PedalsSystemInstance::create(accel_params, brake_params); //pass in the two different params

    // Initialize ADCs
    init_adc_bundle();

    // Create can singletons
    VCFCANInterfaceImpl::CANInterfacesInstance::create(DashboardInterfaceInstance::instance()); 
    
    etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)> main_can_recv;
    main_can_recv = etl::delegate<void(CANInterfaces &, const CAN_message_t &, unsigned long)>::create<VCFCANInterfaceImpl::vcf_recv_switch>();
    VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::create(main_can_recv); // NOLINT (Not sure why it's uninitialized. I think it is.)

    VCFCANInterfaceObjects can_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();

    // Setup CAN
    handle_CAN_setup(can_interface_objects.MAIN_CAN, CAN_baudrate, VCFCANInterfaceImpl::on_main_can_recv);

    EthernetIPDefsInstance::create();
    uint8_t mac[6]; // NOLINT (mac addresses are always 6 bytes)
    qindesign::network::Ethernet.macAddress(&mac[0]);
    qindesign::network::Ethernet.begin(mac, EthernetIPDefsInstance::instance().vcf_ip, EthernetIPDefsInstance::instance().default_dns, EthernetIPDefsInstance::instance().default_gateway, EthernetIPDefsInstance::instance().car_subnet);

    // Schedule Tasks
    scheduler.schedule(CAN_receive); 
    scheduler.schedule(CAN_send); 
    scheduler.schedule(dash_CAN_enqueue);
    scheduler.schedule(pedals_update_task);
    scheduler.schedule(read_adc1_task);
    scheduler.schedule(read_adc2_task);
    scheduler.schedule(send_vcf_data_ethernet_task);
    scheduler.schedule(recv_vcr_data_ethernet_task);

}

void loop() {
    scheduler.run();
}