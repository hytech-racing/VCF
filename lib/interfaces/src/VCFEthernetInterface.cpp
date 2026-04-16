#include "VCFEthernetInterface.h"
#include "SharedFirmwareTypes.h"
#include "ht_can_version.h"
#include "hytech_msgs_version.h"
#include "device_fw_version.h"

hytech_msgs_VCFData_s VCFEthernetInterface::make_vcf_data_msg(ADCInterface &ADCInterfaceInstance, DashboardInterface &dashInstance, PedalsSystem &pedalsInstance, SteeringSystem &steeringInstance)
{
	hytech_msgs_VCFData_s out;

    // has_value
    out.has_dash_input_state = true;
    out.has_front_loadcell_data = true;
    out.has_front_suspot_data = true;
    out.has_pedals_system_data = true;
    out.has_steering_data = true;
    out.has_vcf_ethernet_link_data = true;
    out.has_vcf_shutdown_data = true;

    // Load cells
    out.front_loadcell_data.FL_loadcell_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_filtered_FL_load_cell());
    out.front_loadcell_data.FR_loadcell_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_filtered_FR_load_cell());

    // Sus pots
    out.front_suspot_data.FL_sus_pot_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_filtered_FL_sus_pot());
    out.front_suspot_data.FR_sus_pot_analog = static_cast<uint32_t>(ADCInterfaceInstance.get_filtered_FR_sus_pot());

    // Steering
    out.steering_data.analog_steering_degrees = ADCInterfaceInstance.get_steering_degrees_cw().conversion;
    out.steering_data.digital_steering_analog = ADCInterfaceInstance.get_steering_degrees_ccw().conversion;
    
    //SteeringSystem
    out.steering_system_data.analog_raw = steeringInstance.get_steering_system_data().analog_raw;
    out.steering_system_data.digital_raw = steeringInstance.get_steering_system_data().digital_raw;
    out.steering_system_data.analog_steering_angle = steeringInstance.get_steering_system_data().analog_steering_angle;
    out.steering_system_data.digital_steering_angle = steeringInstance.get_steering_system_data().digital_steering_angle;
    out.steering_system_data.output_steering_angle = steeringInstance.get_steering_system_data().output_steering_angle;
    out.steering_system_data.analog_steering_velocity_deg_s = steeringInstance.get_steering_system_data().analog_steering_velocity_deg_s;
    out.steering_system_data.digital_steering_velocity_deg_s = steeringInstance.get_steering_system_data().digital_steering_velocity_deg_s;
    out.steering_system_data.digital_oor_implausibility = steeringInstance.get_steering_system_data().digital_oor_implausibility;
    out.steering_system_data.analog_oor_implausibility = steeringInstance.get_steering_system_data().analog_oor_implausibility;
    out.steering_system_data.sensor_disagreement_implausibility = steeringInstance.get_steering_system_data().sensor_disagreement_implausibility;
    out.steering_system_data.dtheta_exceeded_analog = steeringInstance.get_steering_system_data().dtheta_exceeded_analog;
    out.steering_system_data.dtheta_exceeded_digital = steeringInstance.get_steering_system_data().dtheta_exceeded_digital;
    out.steering_system_data.both_sensors_fail = steeringInstance.get_steering_system_data().both_sensors_fail;
    out.steering_system_data.interface_sensor_error = steeringInstance.get_steering_system_data().interface_sensor_error;

    //TODO: MODIFY ETH STRUCT
    // Dash
    out.dash_input_state.dim_btn_is_pressed = dashInstance.get_dashboard_outputs().brightness_ctrl_btn_is_pressed;
    out.dash_input_state.preset_btn_is_pressed = dashInstance.get_dashboard_outputs().preset_btn_is_pressed;
    out.dash_input_state.mc_reset_btn_is_pressed = dashInstance.get_dashboard_outputs().mc_reset_btn_is_pressed;
    out.dash_input_state.mode_btn_is_pressed = 0;
    out.dash_input_state.start_btn_is_pressed = dashInstance.get_dashboard_outputs().start_btn_is_pressed;
    out.dash_input_state.data_btn_is_pressed = dashInstance.get_dashboard_outputs().data_btn_is_pressed;
    out.dash_input_state.left_paddle_is_pressed = 0;
    out.dash_input_state.right_paddle_is_pressed = dashInstance.get_dashboard_outputs().BUTTON_2;
    out.dash_input_state.dial_state = (hytech_msgs_ControllerMode_e) (dashInstance.get_dashboard_outputs().dial_state);

    // Ethernet link data
    /*** Ethernet link data values initialized to 1 in VCFDataInstance_s ***/
    out.vcf_ethernet_link_data.vcr_link = 1;
    out.vcf_ethernet_link_data.teensy_link = 1;
    out.vcf_ethernet_link_data.dash_link = 1;

    // Pedals system
    out.pedals_system_data.accel_is_implausible = pedalsInstance.get_pedals_system_data().accel_is_implausible;
    out.pedals_system_data.brake_is_implausible = pedalsInstance.get_pedals_system_data().brake_is_implausible;
    out.pedals_system_data.brake_is_pressed = pedalsInstance.get_pedals_system_data().brake_is_pressed;
    out.pedals_system_data.accel_is_pressed = pedalsInstance.get_pedals_system_data().accel_is_pressed;
    out.pedals_system_data.mech_brake_is_active = pedalsInstance.get_pedals_system_data().mech_brake_is_active;
    out.pedals_system_data.brake_and_accel_pressed_implausibility_high = pedalsInstance.get_pedals_system_data().brake_and_accel_pressed_implausibility_high;
    out.pedals_system_data.implausibility_has_exceeded_max_duration = pedalsInstance.get_pedals_system_data().implausibility_has_exceeded_max_duration;
    out.pedals_system_data.accel_percent = pedalsInstance.get_pedals_system_data().accel_percent;
    out.pedals_system_data.brake_percent = pedalsInstance.get_pedals_system_data().brake_percent;
    out.pedals_system_data.regen_percent = pedalsInstance.get_pedals_system_data().regen_percent;

    // Shutdown Senses
    out.vcf_shutdown_data.d_inertia_switch_out_read = ADCInterfaceInstance.shdn_d().conversion;
    out.vcf_shutdown_data.d_inertia_switch = ADCInterfaceInstance.shdn_d().conversion > SHDN_HIGH_THRESHOLD ? true : false;
    out.vcf_shutdown_data.h_driver_brb_out_read = ADCInterfaceInstance.shdn_h().conversion;
    out.vcf_shutdown_data.h_driver_brb = ADCInterfaceInstance.shdn_h().conversion > SHDN_HIGH_THRESHOLD ? true : false;

    /* Firmware Version */
    out.has_firmware_version_info = true;
    out.firmware_version_info.project_is_dirty = device_status_t::project_is_dirty;
    out.firmware_version_info.project_on_main_or_master = device_status_t::project_on_main_or_master;
    std::array<char, ver_hash_len> ver_hash = convert_version_to_char_arr(device_status_t::firmware_version);
    std::copy(ver_hash.begin(), ver_hash.end(), std::begin(out.firmware_version_info.git_hash));
    out.has_msg_versions = true;
    out.msg_versions.ht_can_version = HT_CAN_LIB_VERSION;

    // working with bytes in nanopb
    std::string_view version_view(version);
    const size_t version_len = [&]() -> size_t {
        return std::min(version_view.size(), sizeof(out.msg_versions.ht_proto_version.bytes));
    }();
    out.msg_versions.ht_proto_version.size = version_len;
    std::copy(version_view.begin(), version_view.begin() + version_len, std::begin(out.msg_versions.ht_proto_version.bytes));

    return out;
}

void VCFEthernetInterface::receive_pb_msg_vcr(const hytech_msgs_VCRData_s &msg_in, VCFData_s &shared_state, unsigned long curr_millis) {
    shared_state.system_data.buzzer_is_active = msg_in.buzzer_is_active;
}
