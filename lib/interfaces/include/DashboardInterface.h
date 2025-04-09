#ifndef DASHBOARD_INTERFACE_H
#define DASHBOARD_INTERFACE_H

#include "hytech_msgs.pb.h"
#include "SharedFirmwareTypes.h"
#include "Arduino.h"
#include "etl/singleton.h"
#include "hytech.h"
#include <Wire.h>
#include "SystemTimeInterface.h"


// Struct representing dashboard gpios
struct DashboardGPIOs_s {

    // GPIO
    uint8_t DIM_BUTTON; 
    uint8_t PRESET_BUTTON; 
    uint8_t MC_CYCLE_BUTTON; 
    uint8_t MODE_BUTTON; 
    uint8_t START_BUTTON; 
    uint8_t DATA_BUTTON; 
    uint8_t LEFT_SHIFTER_BUTTON; 
    uint8_t RIGHT_SHIFTER_BUTTON; 

};

class DashboardInterface 
{
    public: 
        DashboardInterface() = delete; 

        DashboardInterface(DashboardGPIOs_s gpios) : _dashboard_gpios(gpios) 
        {
            pinMode(_dashboard_gpios.START_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.PRESET_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.MC_CYCLE_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.MODE_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.DIM_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.DATA_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.LEFT_SHIFTER_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.RIGHT_SHIFTER_BUTTON, INPUT_PULLUP); 
        }

        // Reading gpios 
        DashInputState_s get_dashboard_outputs();
    
    private: 

        DashboardGPIOs_s _dashboard_gpios;
        DashInputState_s _dashboard_outputs;

};

using DashboardInterfaceInstance = etl::singleton<DashboardInterface>;

#endif /* DASHBOARD_INTERFACE_H */