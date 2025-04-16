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

        void receive_dash_control_data(const CAN_message_t &can_msg)
        {
            DASHBOARD_BUZZER_CONTROL_t unpacked_msg;
            Unpack_DASHBOARD_BUZZER_CONTROL_hytech(&unpacked_msg, can_msg.buf, can_msg.len);            

            if (unpacked_msg.dash_buzzer_flag) {
                BuzzerController::getInstance().activate(millis());
            }

            _is_in_pedals_calibration_state = unpacked_msg.in_pedal_calibration_state;
        }
    
    private: 

        bool _is_in_pedals_calibration_state = false;

};



using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif /* VCR_INTERFACE_H */