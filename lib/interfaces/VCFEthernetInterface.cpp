#include "VCFEthernetInterface.h"
#include "SharedFirmwareTypes.h"

hytech_msgs_VCFData_s VCFEthernetInterface::make_vcf_data_msg(VCFData_s &shared_state)
{
	hytech_msgs_VCFData_s out;

    //has_value
    out.has_dash_input_state = true;
    out.has_front_loadcell_data = true;
    out.has_front_suspot_data = true;
    out.has_pedals_system_data = true;
    out.has_steering_data = true;
    out.has_vcf_ethernet_link_data = true;

    // Load cells
    out.front_loadcell_data.FL_loadcell_analog = shared_state.interface_data.front_loadcell_data.FL_loadcell_analog;
    out.front_loadcell_data.FR_loadcell_analog = shared_state.interface_data.front_loadcell_data.FR_loadcell_analog;

    // Sus pots
    out.front_suspot_data.FL_sus_pot_analog = shared_state.interface_data.front_suspot_data.FL_sus_pot_analog;
    out.front_suspot_data.FL_sus_pot_analog = shared_state.interface_data.front_suspot_data.FL_sus_pot_analog;

    // Steering
    out.steering_data.analog_steering_degrees = shared_state.interface_data.steering_data.analog_steering_degrees;
    out.steering_data.digital_steering_analog = shared_state.interface_data.steering_data.digital_steering_analog;
    
    // Dash
    out.dash_input_state.dim_btn_is_pressed = shared_state.interface_data.dash_input_state.dim_btn_is_pressed;
    out.dash_input_state.preset_btn_is_pressed = shared_state.interface_data.dash_input_state.preset_btn_is_pressed;
    out.dash_input_state.mc_reset_btn_is_pressed = shared_state.interface_data.dash_input_state.mc_reset_btn_is_pressed;
    out.dash_input_state.mode_btn_is_pressed = shared_state.interface_data.dash_input_state.mode_btn_is_pressed;
    out.dash_input_state.start_btn_is_pressed = shared_state.interface_data.dash_input_state.start_btn_is_pressed;
    out.dash_input_state.data_btn_is_pressed = shared_state.interface_data.dash_input_state.data_btn_is_pressed;
    out.dash_input_state.left_paddle_is_pressed = shared_state.interface_data.dash_input_state.left_paddle_is_pressed;
    out.dash_input_state.right_paddle_is_pressed = shared_state.interface_data.dash_input_state.right_paddle_is_pressed;
    out.dash_input_state.dial_state = (hytech_msgs_ControllerMode_e) (shared_state.interface_data.dash_input_state.dial_state);

    // Ethernet link data
    out.vcf_ethernet_link_data.vcr_link = shared_state.interface_data.vcf_ethernet_link_data.vcr_link;
    out.vcf_ethernet_link_data.teensy_link = shared_state.interface_data.vcf_ethernet_link_data.teensy_link;
    out.vcf_ethernet_link_data.dash_link = shared_state.interface_data.vcf_ethernet_link_data.dash_link;

    // Pedals system
    out.pedals_system_data.accel_is_implausible = shared_state.system_data.pedals_system_data.accel_is_implausible;
    out.pedals_system_data.brake_is_implausible = shared_state.system_data.pedals_system_data.brake_is_implausible;
    out.pedals_system_data.brake_is_pressed = shared_state.system_data.pedals_system_data.brake_is_pressed;
    out.pedals_system_data.accel_is_pressed = shared_state.system_data.pedals_system_data.accel_is_pressed;
    out.pedals_system_data.mech_brake_is_active = shared_state.system_data.pedals_system_data.mech_brake_is_active;
    out.pedals_system_data.brake_and_accel_pressed_implausibility_high = shared_state.system_data.pedals_system_data.brake_and_accel_pressed_implausibility_high;
    out.pedals_system_data.implausibility_has_exceeded_max_duration = shared_state.system_data.pedals_system_data.implausibility_has_exceeded_max_duration;
    out.pedals_system_data.accel_percent = shared_state.system_data.pedals_system_data.accel_percent;
    out.pedals_system_data.brake_percent = shared_state.system_data.pedals_system_data.brake_percent;
    out.pedals_system_data.regen_percent = shared_state.system_data.pedals_system_data.regen_percent;

    return out;
}

void VCFEthernetInterface::receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, VCFData_s &shared_state, unsigned long curr_millis){
    shared_state.system_data.buzzer_is_active = msg_in.buzzer_is_active;
}
