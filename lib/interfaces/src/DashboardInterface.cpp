#include "DashboardInterface.h"
#include "SharedFirmwareTypes.h"
#include "CANInterface.h"
#include "VCFCANInterfaceImpl.h"

/* Button reads */
DashInputState_s DashboardInterface::get_dashboard_outputs() 
{
    // _dashboard_outputs.dim_btn_is_pressed = !digitalRead(_dashboard_gpios.DIM_BUTTON);
    _dashboard_outputs.dim_btn_is_pressed = 0;

    _dashboard_outputs.preset_btn_is_pressed= !digitalRead(_dashboard_gpios.PRESET_BUTTON);
    _dashboard_outputs.mc_reset_btn_is_pressed = !digitalRead(_dashboard_gpios.MC_CYCLE_BUTTON);
    // _dashboard_outputs.mode_btn_is_pressed = !digitalRead(_dashboard_gpios.MODE_BUTTON);
    _dashboard_outputs.dim_btn_is_pressed = 0;
    _dashboard_outputs.start_btn_is_pressed = !digitalRead(_dashboard_gpios.START_BUTTON);
    _dashboard_outputs.data_btn_is_pressed = !digitalRead(_dashboard_gpios.DATA_BUTTON);
    _dashboard_outputs.left_paddle_is_pressed = !digitalRead(_dashboard_gpios.LEFT_SHIFTER_BUTTON);
    _dashboard_outputs.right_paddle_is_pressed = !digitalRead(_dashboard_gpios.RIGHT_SHIFTER_BUTTON); 
    
    return _dashboard_outputs;
}

void DashboardInterface::receive_ACU_OK(const CAN_message_t &can_msg) 
{
    ACU_OK_t unpacked_msg;
    Unpack_ACU_OK_hytech(&unpacked_msg, can_msg.buf, can_msg.len); // NOLINT (implicitly decay pointer)

    constexpr uint32_t acu_ok_init_timeout_ms = 2000;
    
    bms_ok = unpacked_msg.bms_ok;
    imd_ok = unpacked_msg.imd_ok;
}

void DashboardInterface::set_dial_state(ControllerMode_e mode) {
    _dashboard_outputs.dial_state = mode;
}