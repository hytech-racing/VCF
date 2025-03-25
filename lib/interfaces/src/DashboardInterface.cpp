#include "DashboardInterface.h"

/* Button reads */
DashInputState_s DashboardInterface::get_dashboard_outputs() 
{
    _dashboard_outputs.dim_btn_is_pressed = !digitalRead(_dashboard_gpios.DIM_BUTTON);
    _dashboard_outputs.preset_btn_is_pressed= !digitalRead(_dashboard_gpios.PRESET_BUTTON);
    _dashboard_outputs.mc_reset_btn_is_pressed = !digitalRead(_dashboard_gpios.MC_CYCLE_BUTTON);
    _dashboard_outputs.mode_btn_is_pressed = !digitalRead(_dashboard_gpios.MODE_BUTTON);
    _dashboard_outputs.start_btn_is_pressed = !digitalRead(_dashboard_gpios.START_BUTTON);
    _dashboard_outputs.data_btn_is_pressed = !digitalRead(_dashboard_gpios.DATA_BUTTON);
    _dashboard_outputs.left_paddle_is_pressed = !digitalRead(_dashboard_gpios.LEFT_SHIFTER_BUTTON);
    _dashboard_outputs.right_paddle_is_pressed = !digitalRead(_dashboard_gpios.RIGHT_SHIFTER_BUTTON); 

    return _dashboard_outputs;
}

