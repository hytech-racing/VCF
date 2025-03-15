#include "VCF_Tasks.h"
#include "VCF_Globals.h"
#include "VCF_Constants.h"
#include <QNEthernet.h>
#include "ProtobufMsgInterface.h"
// #include "VCFEthernetInterface.h"
#include "hytech_msgs.pb.h"
#include "SystemTimeInterface.h"
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

    vcf_data.interface_data.steering_data.analog_steering_degrees = ADCsOnVCFInstance::instance().adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    vcf_data.interface_data.front_loadcell_data.FL_loadcell_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion;
    vcf_data.interface_data.front_loadcell_data.FR_loadcell_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion;
    vcf_data.interface_data.front_suspot_data.FL_sus_pot_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    vcf_data.interface_data.front_suspot_data.FR_sus_pot_analog = ADCsOnVCFInstance::instance().adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    return true;
}

bool run_read_adc2_task()
{
    // Samples all eight channels.
    ADCsOnVCFInstance::instance().adc_2.tick();

    vcf_data.interface_data.pedal_sensor_data.accel_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    vcf_data.interface_data.pedal_sensor_data.accel_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    vcf_data.interface_data.pedal_sensor_data.brake_1 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    vcf_data.interface_data.pedal_sensor_data.brake_2 = ADCsOnVCFInstance::instance().adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;

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
    
    vcf_data.interface_data.dash_input_state.dim_btn_is_pressed = dimButton;
    vcf_data.interface_data.dash_input_state.preset_btn_is_pressed = presetButton;
    vcf_data.interface_data.dash_input_state.mc_reset_btn_is_pressed = mcCycleButton;
    vcf_data.interface_data.dash_input_state.mode_btn_is_pressed = modeButton;
    vcf_data.interface_data.dash_input_state.start_btn_is_pressed = startButton;
    vcf_data.interface_data.dash_input_state.data_btn_is_pressed = dataButton;

    return true;
}

bool init_buzzer_control_task()
{
    pinMode(BUZZER_CONTROL_PIN, OUTPUT);

    return true;
}
bool run_buzzer_control_task()
{
    digitalWrite(BUZZER_CONTROL_PIN, vcf_data.system_data.buzzer_is_active);
    
    return true;
}

bool init_handle_send_vcf_ethernet_data() {
    VCF_socket.begin(EthernetIPDefsInstance::instance().VCFData_port);

    return true;
}
bool run_handle_send_vcf_ethernet_data() {
    hytech_msgs_VCFData_s msg = VCFEthernetInterface::make_vcf_data_msg(vcf_data);
    if(handle_ethernet_socket_send_pb<hytech_msgs_VCFData_s_size, hytech_msgs_VCFData_s>
            (EthernetIPDefsInstance::instance().vcr_ip, 
            EthernetIPDefsInstance::instance().VCFData_port, 
            &VCF_socket, 
            msg, 
            &hytech_msgs_VCFData_s_msg)) {
        return true;
    } else {
        return false;
    }
}

bool init_handle_receive_vcr_ethernet_data() {
    VCR_socket.begin(EthernetIPDefsInstance::instance().VCRData_port);

    return true;
}

bool run_handle_receive_vcr_ethernet_data() {
    etl::optional<hytech_msgs_VCRData_s> protoc_struct = handle_ethernet_socket_receive<hytech_msgs_VCRData_s_size, hytech_msgs_VCRData_s>(&VCR_socket, &hytech_msgs_VCRData_s_msg);
    if (protoc_struct) {
        VCFEthernetInterface::receive_pb_msg_vcr(protoc_struct.value(), vcf_data, millis());
        return true;
    } else {
        return false;
    }
}


bool handle_CAN_send(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    VCFCANInterfaceImpl::send_all_CAN_msgs(vcf_interface_objects.main_can_tx_buffer, &vcf_interface_objects.MAIN_CAN);
    return true;
}

bool handle_CAN_receive(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    VCFCANInterfaceObjects& vcf_interface_objects = VCFCANInterfaceImpl::VCFCANInterfaceObjectsInstance::instance();
    CANInterfaces& vcf_can_interfaces = VCFCANInterfaceImpl::CANInterfacesInstance::instance(); 
    process_ring_buffer(vcf_interface_objects.main_can_rx_buffer, vcf_can_interfaces, sys_time::hal_millis(), vcf_interface_objects.can_recv_switch);
    return true;
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
