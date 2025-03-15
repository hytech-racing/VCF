#include "VCF_Tasks.h"
#include "SharedFirmwareTypes.h"
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include <QNEthernet.h>
#include "ProtobufMsgInterface.h"
#include "ht_task.hpp"
#include "hytech.h"
#include "hytech_msgs.pb.h"
#include "VCFCANInterfaceImpl.h"
#include "CANInterface.h"
#include "SystemTimeInterface.h"
#include "PedalsSystem.h"

#include "Arduino.h"


bool init_adc_task()
{
    // Initilize one at a time to remove dependence on implicit ordering to match channels.
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

    return true;
}
bool run_read_adc1_task()
{
    // Samples all eight channels.
    ADCsOnVCFInstance::instance().adc_1.tick();

    VCFData_sInstance::instance().interface_data.steering_data.analog_steering_degrees = ADCsOnVCFInstance::instance().adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    VCFData_sInstance::instance().interface_data.front_loadcell_data.FL_loadcell_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.front_loadcell_data.FR_loadcell_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.front_suspot_data.FL_sus_pot_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    VCFData_sInstance::instance().interface_data.front_suspot_data.FR_sus_pot_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    return true;
}

bool run_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Samples all eight channels.
    ADCsOnVCFInstance::instance().adc_2.tick();
    // Serial.println("sampling");
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.accel_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    VCFData_sInstance::instance().interface_data.pedal_sensor_data.brake_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;

    return true;
}

bool init_read_gpio_task()
{
    // Setting digital/analog buttons D10-D6, A8 as inputs
    pinMode(BTN_DIM_READ, INPUT);
    pinMode(BTN_PRESET_READ, INPUT);
    pinMode(BTN_MC_CYCLE_READ, INPUT);
    pinMode(BTN_MODE_READ, INPUT);
    pinMode(BTN_START_READ, INPUT);
    pinMode(BTN_DATA_READ, INPUT);
    
    return true;
}
bool run_read_gpio_task()
{
    // Doing digital read on all digital inputs
    int dimButton = digitalRead(BTN_DIM_READ);
    int presetButton = digitalRead(BTN_PRESET_READ);
    int mcCycleButton = digitalRead(BTN_MC_CYCLE_READ);
    int modeButton = digitalRead(BTN_MODE_READ);
    int startButton = digitalRead(BTN_START_READ);
    int dataButton = digitalRead(BTN_DATA_READ);
    
    VCFData_sInstance::instance().interface_data.dash_input_state.dim_btn_is_pressed = dimButton;
    VCFData_sInstance::instance().interface_data.dash_input_state.preset_btn_is_pressed = presetButton;
    VCFData_sInstance::instance().interface_data.dash_input_state.mc_reset_btn_is_pressed = mcCycleButton;
    VCFData_sInstance::instance().interface_data.dash_input_state.mode_btn_is_pressed = modeButton;
    VCFData_sInstance::instance().interface_data.dash_input_state.start_btn_is_pressed = startButton;
    VCFData_sInstance::instance().interface_data.dash_input_state.data_btn_is_pressed = dataButton;

    return true;
}

bool init_buzzer_control_task()
{
    pinMode(BUZZER_CONTROL_PIN, OUTPUT);

    return true;
}
bool run_buzzer_control_task()
{
    digitalWrite(BUZZER_CONTROL_PIN, VCRData_sInstance::instance().system_data.buzzer_is_active);
    return true;
}

bool init_handle_send_vcf_ethernet_data() {
    VCF_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

    return true;
}

bool run_handle_send_vcf_ethernet_data() {
    hytech_msgs_VCFData_s msg = VCFEthernetInterface::make_vcf_data_msg(VCFData_sInstance::instance());
    if(handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size, hytech_msgs_VCFData_s>
            (EthernetIPDefsInstance::instance().vcr_ip, 
            EthernetIPDefsInstance::instance().VCRData_port, 
            &VCF_socket, 
            msg, 
            &hytech_msgs_VCFData_s_msg)) {
        return true;
    } else {
        return false;
    }
}

bool init_handle_receive_vcr_ethernet_data() {
    VCF_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

    return true;
}

bool run_handle_receive_vcr_ethernet_data() {
    etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&VCF_socket, &hytech_msgs_VCRData_s_msg);
    if (protoc_struct) {
        return true;
    } else {
        return false;
    }
}

bool send_dash_data(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{   
    CANInterfaces can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance(); 
    DashInputState_s dash_outputs = can_interfaces.dash_interface.get_dashboard_outputs();

    DASH_INPUT_t msg_out;

    msg_out.start_button = dash_outputs.start_btn_is_pressed;
    msg_out.preset_button = dash_outputs.preset_btn_is_pressed;
    msg_out.motor_controller_cycle_button = dash_outputs.mc_reset_btn_is_pressed;
    msg_out.mode_button = dash_outputs.mode_btn_is_pressed;
    msg_out.start_button = dash_outputs.start_btn_is_pressed;
    msg_out.data_button_is_pressed = dash_outputs.data_btn_is_pressed;
    msg_out.left_shifter_button = dash_outputs.left_paddle_is_pressed;
    msg_out.right_shifter_button = dash_outputs.right_paddle_is_pressed;    

    CAN_util::enqueue_msg(&msg_out, &Pack_DASH_INPUT_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    
    return true;
}

bool send_pedals_data(const unsigned long &sys_micros, const HT_TASK::TaskInfo& task_info)
{
    PEDALS_SYSTEM_DATA_t pedals_data = {};
    pedals_data.accel_implausible = VCFData_sInstance::instance().system_data.pedals_system_data.accel_is_implausible;
    pedals_data.brake_implausible = VCFData_sInstance::instance().system_data.pedals_system_data.brake_is_implausible;
    pedals_data.brake_accel_implausibility = VCFData_sInstance::instance().system_data.pedals_system_data.brake_and_accel_pressed_implausibility_high;

    pedals_data.accel_pedal_active = VCFData_sInstance::instance().system_data.pedals_system_data.accel_is_pressed;
    pedals_data.brake_pedal_active = VCFData_sInstance::instance().system_data.pedals_system_data.brake_is_pressed;
    pedals_data.mechanical_brake_active = VCFData_sInstance::instance().system_data.pedals_system_data.mech_brake_is_active;
    pedals_data.implaus_exceeded_max_duration = VCFData_sInstance::instance().system_data.pedals_system_data.implausibility_has_exceeded_max_duration;

    
    pedals_data.accel_pedal_ro = HYTECH_accel_pedal_ro_toS(VCFData_sInstance::instance().system_data.pedals_system_data.accel_percent);
    pedals_data.brake_pedal_ro = HYTECH_brake_pedal_ro_toS(VCFData_sInstance::instance().system_data.pedals_system_data.brake_percent);
    // Serial.println(pedals_data.brake_pedal_ro);
    // Serial.println(pedals_data.accel_pedal_ro);
    CAN_util::enqueue_msg(&pedals_data, &Pack_PEDALS_SYSTEM_DATA_hytech, VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance().main_can_tx_buffer);
    return true;
}

bool handle_CAN_send(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Serial.println("asdf");
    VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    VCFCANInterfaceImpl::send_all_CAN_msgs(vcf_interface_objects.main_can_tx_buffer, &vcf_interface_objects.MAIN_CAN);
    return true;
}

namespace async_tasks 
{
    // these are async tasks. we want these to run as fast as possible p much
    void handle_async_CAN_receive() //NOLINT caps for CAN
    {
        VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
        CANInterfaces& vcf_can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance(); 
        process_ring_buffer(vcf_interface_objects.main_can_rx_buffer, vcf_can_interfaces, sys_time::hal_millis(), vcf_interface_objects.can_recv_switch);
    }

    void handle_async_recvs()
    {
        // ethernet, etc...
        
        handle_async_CAN_receive();
    }
    bool handle_async_main(const unsigned long& sys_micros, const HT_TASK::TaskInfo& task_info)
    {
        handle_async_recvs();
        
        VCFData_sInstance::instance().system_data.pedals_system_data = PedalsSystemInstance::instance().evaluate_pedals(VCFData_sInstance::instance().interface_data.pedal_sensor_data, sys_time::hal_millis());
        // Serial.println(VCFData_sInstance::instance().system_data.pedals_system_data.accel_percent);
        return true;
    }
};
