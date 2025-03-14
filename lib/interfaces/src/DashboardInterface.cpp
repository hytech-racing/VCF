#include "DashboardInterface.h"


/* Button reads */
DashboardOutputs_s DashboardInterface::get_dashboard_outputs() 
{
    _dashboard_outputs.DIM_BUTTON = !digitalRead(_dashboard_gpios.DIM_BUTTON);
    _dashboard_outputs.PRESET_BUTTON = !digitalRead(_dashboard_gpios.PRESET_BUTTON);
    _dashboard_outputs.MC_CYCLE_BUTTON = !digitalRead(_dashboard_gpios.MC_CYCLE_BUTTON);
    _dashboard_outputs.MODE_BUTTON = !digitalRead(_dashboard_gpios.MODE_BUTTON);
    _dashboard_outputs.START_BUTTON = !digitalRead(_dashboard_gpios.START_BUTTON);
    _dashboard_outputs.DATA_BUTTON = !digitalRead(_dashboard_gpios.DATA_BUTTON);
    _dashboard_outputs.LEFT_SHIFTER_BUTTON = !digitalRead(_dashboard_gpios.LEFT_SHIFTER_BUTTON);
    _dashboard_outputs.RIGHT_SHIFTER_BUTTON = !digitalRead(_dashboard_gpios.RIGHT_SHIFTER_BUTTON); 

    return _dashboard_outputs;
}


