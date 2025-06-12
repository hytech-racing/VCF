#include "VCRInterface.h"

void VCRInterface::receive_dash_control_data(const CAN_message_t &can_msg)
{
    DASHBOARD_BUZZER_CONTROL_t unpacked_msg;
    Unpack_DASHBOARD_BUZZER_CONTROL_hytech(&unpacked_msg, can_msg.buf, can_msg.len);            

    if (unpacked_msg.dash_buzzer_flag) {
        BuzzerController::getInstance().activate(millis());
    }

    _is_in_pedals_calibration_state = unpacked_msg.in_pedal_calibration_state;

    if (unpacked_msg.torque_limit_enum_value < ((int) TorqueLimit_e::TCMUX_NUM_TORQUE_LIMITS)) // check for validity
    {
        _torque_limit = (TorqueLimit_e) unpacked_msg.torque_limit_enum_value;
    }
}

void VCRInterface::receive_vehicle_state_data(const CAN_message_t &can_msg)
{
    VEHICLE_STATE_t unpacked_msg;
    Unpack_VEHICLE_STATE_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    _vehicle_state_value = unpacked_msg.drivetrain_state;
}

void VCRInterface::receive_software_status(const CAN_message_t &can_msg)
{
    DRIVEBRAIN_STATE_DATA_t unpacked_msg;
    Unpack_DRIVEBRAIN_STATE_DATA_hytech(&unpacked_msg, can_msg.buf, can_msg.len);
    _is_db_in_ctrl = unpacked_msg.drivebrain_in_control;
}