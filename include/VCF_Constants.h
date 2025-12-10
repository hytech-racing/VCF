#ifndef VCF_CONSTANTS
#define VCF_CONSTANTS

#include <stdint.h>

// hardware connections constants
namespace VCFInterfaceConstants { 
    constexpr int ADC0_CS = 10; // MCP3208. ADC1 in VCF schematic. Used for steering, sus pots, and load cells.
    constexpr int ADC1_CS = 38; // MCP3208. ADC2 in VCF schematic. Used for pedal position sensors.

    //Teensy 4.1 GPIO pins  
    // constexpr int BTN_DIM_READ = 28;
    // constexpr int BTN_PRESET_READ = 31;
    constexpr int BTN_MC_CYCLE_READ = 31; // MUST NOT BE MAPPED ANYMORE. USED TO BE 27.
    // constexpr int BTN_MODE_READ = 27; // USED TO BE 26.
    constexpr int BTN_START_READ = 29;
    constexpr int BTN_DATA_READ = 30;
    constexpr int BUZZER_CONTROL_PIN = 32;
    constexpr int RIGHT_SHIFTER = 26;
    constexpr int LEFT_SHIFTER = 27;

    constexpr int NEOPIXEL_CONTROL_PIN = 33;
    constexpr int NEOPIXEL_COUNT = 12; // 12 neopixeles on dashboard
    //ADC Pins
    /* Channels on ADC_1 */
    constexpr int FR_LOADCELL_CHANNEL     = 0;
    constexpr int FL_LOADCELL_CHANNEL     = 1;
    constexpr int FL_SUS_POT_CHANNEL      = 2;
    constexpr int FR_SUS_POT_CHANNEL      = 3;
    constexpr int STEERING_2_CHANNEL      = 4;
    constexpr int STEERING_1_CHANNEL      = 5;
    // constexpr int UNUSED_CHANNEL       = 6;
    // constexpr int UNUSED_CHANNEL       = 7;

    /* Channels on ADC_2 */
    // constexpr int UNUSED_CHANNEL       = 0;
    // constexpr int UNUSED_CHANNEL       = 1;
    constexpr int ACCEL_1_CHANNEL         = 2;
    constexpr int ACCEL_2_CHANNEL         = 3;
    constexpr int BRAKE_1_CHANNEL         = 4;
    constexpr int BRAKE_2_CHANNEL         = 5;
    // constexpr int UNUSED_CHANNEL       = 6;
    // constexpr int UNUSED_CHANNEL       = 7;

    // watchdog pins
    constexpr int WATCHDOG_PIN = 36;
    constexpr int SOFTWARE_OK_PIN = 37; // Watchdog's !MR pin

        //ADC configs
    /* Scaling and offset */
    constexpr float STEERING_1_SCALE = 1; // TODO: Figure out what these mean
    constexpr float STEERING_1_OFFSET = 0;
    constexpr float STEERING_2_SCALE = 1; // TODO: Figure out if steering 2 = steering 1
    constexpr float STEERING_2_OFFSET = 0;
    // Scale for steering sensor = 0.02197265 . Offset has to be mechanically determined

    // constexpr float FR_LOADCELL_SCALE = 0.81; //Values are from the old MCU rev15 // TODO: Calibrate load cells
    // constexpr float FR_LOADCELL_OFFSET = 36.8;
    // constexpr float FL_LOADCELL_SCALE = 0.63;
    // constexpr float FL_LOADCELL_OFFSET = -3.9;


    constexpr float FR_LOADCELL_SCALE =  1.0; //Values 
    constexpr float FR_LOADCELL_OFFSET = 0.0;
    constexpr float FL_LOADCELL_SCALE =  1.0; //Values 
    constexpr float FL_LOADCELL_OFFSET = 0.0;

    constexpr float FR_SUS_POT_SCALE = 1;
    constexpr float FR_SUS_POT_OFFSET = 0;
    constexpr float FL_SUS_POT_SCALE = 1;
    constexpr float FL_SUS_POT_OFFSET = 0;

    constexpr float ACCEL_1_SCALE = 1; // TODO: Figure out what these should be
    constexpr float ACCEL_1_OFFSET = 0;
    constexpr float ACCEL_2_SCALE = 1;
    constexpr float ACCEL_2_OFFSET = 0;
    constexpr float BRAKE_1_SCALE = 1;
    constexpr float BRAKE_1_OFFSET = 0;
    constexpr float BRAKE_2_SCALE = 1;
    constexpr float BRAKE_2_OFFSET = 0;

    // EEPROM addresses for min and max calibration values
    constexpr uint32_t ACCEL_1_MIN_ADDR = 0;
    constexpr uint32_t ACCEL_2_MIN_ADDR = 4;
    constexpr uint32_t ACCEL_1_MAX_ADDR = 8;
    constexpr uint32_t ACCEL_2_MAX_ADDR = 12;
    constexpr float ACCEL_ACTIVATION_PERCENTAGE = 0.10f;
    constexpr uint32_t ACCEL_MIN_SENSOR_PEDAL_1 = 90;
    constexpr uint32_t ACCEL_MIN_SENSOR_PEDAL_2 = 90;
    constexpr uint32_t ACCEL_MAX_SENSOR_PEDAL_1 = 4000;
    constexpr uint32_t ACCEL_MAX_SENSOR_PEDAL_2 = 4000;
    constexpr float ACCEL_DEADZONE_MARGIN = 0.03f;
    constexpr float ACCEL_MECHANICAL_ACTIVATION_PERCENTAGE = 0.05f;

    constexpr uint32_t BRAKE_1_MIN_ADDR = 16;
    constexpr uint32_t BRAKE_2_MIN_ADDR = 20;
    constexpr uint32_t BRAKE_1_MAX_ADDR = 24;
    constexpr uint32_t BRAKE_2_MAX_ADDR = 28;
    constexpr float BRAKE_ACTIVATION_PERCENTAGE = 0.50f;
    constexpr uint32_t BRAKE_MIN_SENSOR_PEDAL_1 = 90;
    constexpr uint32_t BRAKE_MIN_SENSOR_PEDAL_2 = 90;
    constexpr uint32_t BRAKE_MAX_SENSOR_PEDAL_1 = 4000;
    constexpr uint32_t BRAKE_MAX_SENSOR_PEDAL_2 = 4000;
    constexpr float BRAKE_DEADZONE_MARGIN = 0.04f;
    constexpr float BRAKE_MECHANICAL_ACTIVATION_PERCENTAGE = 0.5f;

    constexpr unsigned long WATCHDOG_KICK_INTERVAL_MS = 10UL;
}

// calibration and processing constants
namespace VCFSystemConstants { 
    constexpr float LBS_TO_NEWTONS = 4.4482216153;
}

// software configuration constants
namespace VCFTaskConstants {
    const size_t SERIAL_BAUDRATE = 115200;

    // task priorities and periods
    constexpr unsigned long MAIN_TASK_PRIORITY = 5;
    constexpr unsigned long MAIN_TASK_PERIOD = 100;               // 100 us = 10 kHz

    constexpr unsigned long CAN_SEND_PRIORITY = 10;
    constexpr unsigned long CAN_SEND_PERIOD = 2000;               // 2 000 us = 500 Hz

    constexpr unsigned long PEDALS_SEND_PERIOD = 4000;            // 4 000 us = 250 Hz
    constexpr unsigned long PEDALS_SAMPLE_PERIOD = 500;           // 500 us = 2 kHz
    constexpr unsigned long PEDALS_PRIORITY = 5;

    constexpr unsigned long BUZZER_WRITE_PERIOD = 100000;         // 100 000 us = 10 Hz
    constexpr unsigned long BUZZER_PRIORITY = 20;

    constexpr unsigned long DASH_SAMPLE_PERIOD = 100000;          // 100 000 us = 10 Hz
    constexpr unsigned long DASH_SAMPLE_PRIORITY = 21;

    constexpr unsigned long DASH_SEND_PERIOD = 100000;            // 100 000 us = 10 Hz
    constexpr unsigned long DASH_SEND_PRIORITY = 7;

    constexpr unsigned long DEBUG_PRIORITY = 100;
    constexpr unsigned long DEBUG_PERIOD = 10000;                 // 10 000 us = 2 Hz

    constexpr unsigned long NEOPIXEL_UPDATE_PRIORITY = 90;
    constexpr unsigned long NEOPIXEL_UPDATE_PERIOD = 100000;      // 100 000 us = 10 Hz

    constexpr unsigned long STEERING_SEND_PERIOD = 4000;          // 4 000 us = 250 Hz
    constexpr unsigned long STEERING_SEND_PRIORITY = 25;

    constexpr unsigned long LOADCELL_SAMPLE_PERIOD = 250;         // 250 us = 4 kHz
    constexpr unsigned long LOADCELL_SAMPLE_PRIORITY = 24;

    constexpr unsigned long ETHERNET_SEND_PERIOD = 100000;        // 100 000 us = 10Hz
    constexpr unsigned long ETHERNET_SEND_PRIORITY = 20;

    constexpr unsigned long LOADCELL_SEND_PERIOD = 4000;          // 4 000 us = 250 Hz
    constexpr unsigned long LOADCELL_SEND_PRIORITY = 25;


    constexpr unsigned long PEDALS_RECALIBRATION_PRIORITY = 150;
    constexpr unsigned long PEDALS_RECALIBRATION_PERIOD = 100000; // 100 000 us = 10 Hz
    
    // IIR filter alphas
    constexpr float LOADCELL_IIR_FILTER_ALPHA = 0.01f;

    constexpr unsigned long WATCHDOG_PRIORITY = 1;
    constexpr unsigned long WATCHDOG_KICK_PERIOD = 1000;          // 1 000 us = 1000 Hz

}

#endif /* VCF_CONSTANTS */