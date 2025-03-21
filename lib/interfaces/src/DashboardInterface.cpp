#include "DashboardInterface.h"
#include "SharedFirmwareTypes.h"

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
    // _dashboard_outputs.buzzer_complete = check_buzzer_complete(); 

    return _dashboard_outputs;
}

void DashboardInterface::start_buzzer()
{
    if (!_buzzer_active)
    {
        _buzzer_last_activated = sys_time::hal_millis(); 
        digitalWrite(_dashboard_gpios.BUZZER, HIGH);
        _buzzer_active = true;
    }
}

bool DashboardInterface::check_buzzer_complete() 
{
    constexpr unsigned long buzzer_duration = 2000;
    return sys_time::hal_millis() - _buzzer_last_activated > buzzer_duration; 
}
   
void DashboardInterface::end_buzzer()
{
    digitalWrite(_dashboard_gpios.BUZZER, LOW); 
}

void DashboardInterface::update_dash_state(DashInputState_s input) 
{
    // if (input.start_buzzer)
    // {
    //     start_buzzer(); 
    // }
    
    // if (input.stop_buzzer)
    // {
    //     end_buzzer(); 
    // }
}
