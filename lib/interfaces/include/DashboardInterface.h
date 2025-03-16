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

    // I2C
    uint8_t DIAL_SDA; 
    uint8_t DIAL_SCL; 

    // Buzzer gpio
    uint8_t BUZZER; 

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
            pinMode(_dashboard_gpios.BUZZER, OUTPUT);
        }

        // Reading gpios 
        DashOutputState_s get_dashboard_outputs();

        // Writing gpios
        void start_buzzer(); 

        bool check_buzzer_complete(); 

        void end_buzzer(); 

        // Method to tick dash state (DashInput is just a reflection of the CAN message to dash
        // that gets sent over CAN)
        void update_dash_state(DashInputState_s state);

    
    private: 
        DashboardGPIOs_s _dashboard_gpios; 
        DashOutputState_s _dashboard_outputs;

        unsigned long _buzzer_last_activated = 0; 
        bool _buzzer_active = false;
};

using DashboardInterfaceInstance = etl::singleton<DashboardInterface>;

#endif /* DASHBOARD_INTERFACE_H */