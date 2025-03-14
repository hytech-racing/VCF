#ifndef DASHBOARD_INTERFACE_H
#define DASHBOARD_INTERFACE_H

#include "hytech_msgs.pb.h"
#include "SharedFirmwareTypes.h"
#include "Arduino.h"
#include "etl/singleton.h"

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

    // I2C
    uint8_t DIAL_SDA; 
    uint8_t DIAL_SCL; 

};

// Struct representing button outputs
struct DashboardOutputs_s {

    // GPIO
    bool DIM_BUTTON;    
    bool PRESET_BUTTON; 
    bool MC_CYCLE_BUTTON; 
    bool MODE_BUTTON; 
    bool START_BUTTON; 
    bool DATA_BUTTON; 
    bool LEFT_SHIFTER_BUTTON; 
    bool RIGHT_SHIFTER_BUTTON; 
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
            pinMode(_dashboard_gpios.START_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.DATA_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.LEFT_SHIFTER_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.RIGHT_SHIFTER_BUTTON, INPUT_PULLUP); 
        }

        // Receing gpios 
        DashboardOutputs_s get_dashboard_outputs();


    private: 
        DashboardGPIOs_s _dashboard_gpios; 
        DashboardOutputs_s _dashboard_outputs;
};

using DashboardInterfaceInstance = etl::singleton<DashboardInterface>;

#endif /* DASHBOARD_INTERFACE_H */