#ifndef VCR_INTERFACE_H
#define VCR_INTERFACE_H

#include "SharedFirmwareTypes.h"
#include "Arduino.h"
#include "BuzzerController.h"
#include "etl/singleton.h"
#include "hytech.h"
#include "FlexCAN_T4.h"

class VCRInterface 
{
    public:

        bool is_in_pedals_calibration_state() {return _is_in_pedals_calibration_state;}
        TorqueLimit_e get_torque_limit_mode() {return _torque_limit;}

        void receive_dash_control_data(const CAN_message_t &can_msg)
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

        void disable_calibration_state() {_is_in_pedals_calibration_state = false;}
    
    private: 

        bool _is_in_pedals_calibration_state = false;
        TorqueLimit_e _torque_limit = TorqueLimit_e::TCMUX_LOW_TORQUE;

};



using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif /* VCR_INTERFACE_H */