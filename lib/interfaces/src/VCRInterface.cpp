#include "VCRInterface.h"

void VCRInterface::receive_dash_control_data(const CAN_message_t &can_msg)
{
    DASHBOARD_BUZZER_CONTROL_t unpacked_msg;
    Unpack_DASHBOARD_BUZZER_CONTROL_hytech(&unpacked_msg, can_msg.buf, can_msg.len);    //NOLINT         

    if (unpacked_msg.dash_buzzer_flag) {
        BuzzerController::getInstance().activate(millis());
    }

    _is_in_pedals_calibration_state = unpacked_msg.in_pedal_calibration_state;

    if (unpacked_msg.torque_limit_enum_value < ((int) TorqueLimit_e::TCMUX_NUM_TORQUE_LIMITS)) // check for validity
    {
        _torque_limit = (TorqueLimit_e) unpacked_msg.torque_limit_enum_value;
    }
}

void VCRInterface::receive_car_states_data(const CAN_message_t &can_msg)
{
    CAR_STATES_t unpacked_msg;
    Unpack_CAR_STATES_hytech(&unpacked_msg, can_msg.buf, can_msg.len); //NOLINT
    _vehicle_state_value = static_cast<VehicleState_e>(unpacked_msg.vehicle_state);
    _drivetrain_state_value = static_cast<DrivetrainState_e>(unpacked_msg.drivetrain_state);
    _is_db_in_ctrl = unpacked_msg.drivebrain_in_control;
}