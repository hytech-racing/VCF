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

        void receive_dash_control_data(const CAN_message_t &can_msg);

        void disable_calibration_state() {_is_in_pedals_calibration_state = false;}

        void receive_car_states_data(const CAN_message_t &can_msg);

        VehicleState_e get_vehicle_state() {return _vehicle_state_value;}
        bool get_db_in_ctrl() {return _is_db_in_ctrl;}
    
    private: 

        bool _is_in_pedals_calibration_state = false;
        VehicleState_e _vehicle_state_value;
        DrivetrainState_e _drivetrain_state_value;
        bool _is_db_in_ctrl;
        TorqueLimit_e _torque_limit = TorqueLimit_e::TCMUX_LOW_TORQUE;

};



using VCRInterfaceInstance = etl::singleton<VCRInterface>;

#endif /* VCR_INTERFACE_H */