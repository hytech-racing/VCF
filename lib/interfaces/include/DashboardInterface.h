#ifndef DASHBOARD_INTERFACE_H
#define DASHBOARD_INTERFACE_H

#include "Arduino.h"
#include "etl/singleton.h"
#include "MCP23017.h"
#include <Wire.h>
#include "FlexCAN_T4.h"

#include "hytech_msgs.pb.h"
#include "SharedFirmwareTypes.h"
#include "hytech.h"

#include "SystemTimeInterface.h"


// Struct representing dashboard gpios
struct DashboardGPIOs_s {

    // GPIO
    uint8_t BRIGHTNESS_CONTROL_PIN;
    uint8_t PRESET_BUTTON;
    uint8_t MC_CYCLE_BUTTON;
    uint8_t START_BUTTON;
    uint8_t DATA_BUTTON;
    uint8_t BUTTON_2;
};

class DashboardInterface 
{
    public: 
        DashboardInterface() = delete; 

        DashboardInterface(DashboardGPIOs_s gpios, 
                uint8_t io_expander_addr, TwoWire &i2c_bus)
            : _dashboard_gpios(gpios), _io_expander(MCP23017(io_expander_addr, i2c_bus))
        {
            pinMode(_dashboard_gpios.START_BUTTON, INPUT_PULLUP);
            pinMode(_dashboard_gpios.PRESET_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.MC_CYCLE_BUTTON, INPUT_PULLUP);
            pinMode(_dashboard_gpios.BRIGHTNESS_CONTROL_PIN, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.DATA_BUTTON, INPUT_PULLUP); 
            pinMode(_dashboard_gpios.BUTTON_2, INPUT_PULLUP);

            _dash_created_millis = sys_time::hal_millis();

            i2c_bus.begin();
            _initIOExpander();
        }

        // Reading gpios 
        DashInputState_s get_dashboard_outputs();

        // Stores outputs
        DashInputState_s get_dashboard_stored_state();
        
        /**
         * Syncs stored outputs with last read outputs.
         * Used to store previous state of buttons to determine if they are clicked or not.
         * In other words, to find the falling edge.
         */
        void sync_dashboard_stored_state();

        // Receiving
        void receive_ACU_OK(const CAN_message_t &can_msg);

        bool bms_ok = true;
        bool imd_ok = true;

        void set_dial_state(ControllerMode_e mode);

        void read_ioexpander();
    
    private: 

        DashboardGPIOs_s _dashboard_gpios;
        DashInputState_s _dashboard_outputs;
        DashInputState_s _dashboard_stored_state;

        MCP23017 _io_expander;

        unsigned long _dash_created_millis;

        inline void _initIOExpander() {
            _io_expander.init();

            _io_expander.portMode(MCP23017Port::A, 0b00000000); // 0b0000 0000 = 0
            _io_expander.portMode(MCP23017Port::B, 0b01111111); // 0b0111 1111 = 127

            _io_expander.writeRegister(MCP23017Register::GPPU_B, 0xFF); // Internal pull-ups
            _io_expander.writeRegister(MCP23017Register::IPOL_B, 0xFF); // Polarity (inverted)
        }
};

using DashboardInterfaceInstance = etl::singleton<DashboardInterface>;

#endif /* DASHBOARD_INTERFACE_H */