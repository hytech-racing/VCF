#ifndef VCF_CONSTANTS
#define VCF_CONSTANTS

#include <stdint.h>

// hardware connections constants
namespace VCFInterfaceConstants {
    /* Teensy 4.1 GPIO pins */
    
    /* Not on Schematic
    // constexpr int BTN_DIM_READ = 28;
    // constexpr int BTN_PRESET_READ = 31;
    // constexpr int BTN_MODE_READ = 27; // USED TO BE 26.
    */

    constexpr int BTN_MC_CYCLE_READ = 31; // DB/MC_RESET on schematic
    constexpr int BTN_START_READ = 29; // RTD on schematic
    constexpr int BTN_DATA_READ = 30; // DATA_MARK on schematic
    constexpr int BUZZER_CONTROL_PIN = 32;
    constexpr int BRIGHTNESS_CONTROL_PIN = 26; //BUTTON_1 on schematic
    constexpr int BUTTON_2 = 27; // BUTTON_2 on schematic
    constexpr int BTN_PRESET_READ = 28; // Pedals recal button (brightness control on schematic)
    
    constexpr int NEOPIXEL_CONTROL_PIN = 33;
    constexpr int NEOPIXEL_COUNT = 12; // 12 neopixeles on dashboard
    
    // watchdog pins
    constexpr int WATCHDOG_PIN = 36;
    constexpr int SOFTWARE_OK_PIN = 37; // Watchdog's !MR pin
    
    // watchdog kick interval
    constexpr unsigned long WATCHDOG_KICK_INTERVAL_MS = 10UL; // 10 ms = 100 Hz
    
    //ADC Pins
    constexpr int ADC0_CS = 10; // MCP3208. ADC0 in VCF schematic. Used for steering, sus pots, and load cells.
    constexpr int ADC1_CS = 38; // MCP3208. ADC1 in VCF schematic. Used for pedal position sensors.

    /* Channels on ADC_0 */
    // constexpr int UNUSED_CHANNEL         = 0;
    constexpr int PEDAL_REF_2V5_CHANNEL     = 1;    // 2.5V reference used for improved pedal calibration
    constexpr int STEERING_1_CHANNEL        = 2;    // Analog steering sensor 1
    constexpr int STEERING_2_CHANNEL        = 3;    // Analog steering sensor 2
    constexpr int ACCEL_1_CHANNEL           = 4;    // Accel pedal sensor 1
    constexpr int ACCEL_2_CHANNEL           = 5;    // Accel pedal sensor 2
    constexpr int BRAKE_1_CHANNEL           = 6;    // Brake pedal sensor 1
    constexpr int BRAKE_2_CHANNEL           = 7;    // Brake pedal sensor 2

    /* Channels on ADC_1 */
    constexpr int SHDN_H_CHANNEL                = 0;    // SHDN_H sense
    constexpr int SHDN_D_CHANNEL                = 1;    // SHDN_D sense
    constexpr int FL_LOADCELL_CHANNEL           = 2;    // Front left load cell
    constexpr int FR_LOADCELL_CHANNEL           = 3;    // Front right load cell
    constexpr int FR_SUS_POT_CHANNEL            = 4;    // Front right suspension potentiometer
    constexpr int FL_SUS_POT_CHANNEL            = 5;    // Front left suspension potentiometer
    constexpr int BRAKE_PRESSURE_FRONT_CHANNEL  = 6;    // Front brake pressure sensor
    constexpr int BRAKE_PRESSURE_REAR_CHANNEL   = 7;    // Rear brake pressure sensor

    //ADC configs
    /* Scaling and offset */
    constexpr float PEDAL_REF_2V5_SCALE = 1.0;
    constexpr float PEDAL_REF_2V5_OFFSET = 0;

    constexpr float STEERING_1_SCALE = 1.0; // TODO: Figure out what these mean
    constexpr float STEERING_1_OFFSET = 0;
    constexpr float STEERING_2_SCALE = 1.0; // TODO: Figure out if steering 2 = steering 1
    constexpr float STEERING_2_OFFSET = 0;
    // Scale for steering sensor = 0.02197265 . Offset has to be mechanically determined

    constexpr float ACCEL_1_SCALE = 1.0; // TODO: Figure out what these should be
    constexpr float ACCEL_1_OFFSET = 0;
    constexpr float ACCEL_2_SCALE = 1.0;
    constexpr float ACCEL_2_OFFSET = 0;

    constexpr float BRAKE_1_SCALE = 1.0;
    constexpr float BRAKE_1_OFFSET = 0;
    constexpr float BRAKE_2_SCALE = 1.0;
    constexpr float BRAKE_2_OFFSET = 0;

    constexpr float SHDN_H_SCALE = 0.00697841165926; // Conversion from ADC read to shutdown voltage based on schematic - maps reading of 3439 to 24V
    constexpr float SHDN_H_OFFSET = 0;
    constexpr float SHDN_D_SCALE = 0.00697841165926; // Conversion from ADC read to shutdown voltage based on schematic - maps reading of 3439 to 24V
    constexpr float SHDN_D_OFFSET = 0;

    // constexpr float FL_LOADCELL_SCALE = 0.63;
    // constexpr float FL_LOADCELL_OFFSET = -3.9;
    // constexpr float FR_LOADCELL_SCALE = 0.81; //Values are from the old MCU rev15 // TODO: Calibrate load cells
    // constexpr float FR_LOADCELL_OFFSET = 36.8;

    constexpr float FL_LOADCELL_SCALE =  1.0; //Values 
    constexpr float FL_LOADCELL_OFFSET = 0.0;
    constexpr float FR_LOADCELL_SCALE =  1.0; //Values 
    constexpr float FR_LOADCELL_OFFSET = 0.0;

    constexpr float FR_SUS_POT_SCALE = 1.0;
    constexpr float FR_SUS_POT_OFFSET = 0;
    constexpr float FL_SUS_POT_SCALE = 1.0;
    constexpr float FL_SUS_POT_OFFSET = 0;

    constexpr float BRAKE_PRESSURE_FRONT_SCALE = 1.0;
    constexpr float BRAKE_PRESSURE_FRONT_OFFSET = 0;
    constexpr float BRAKE_PRESSURE_REAR_SCALE = 1.0;
    constexpr float BRAKE_PRESSURE_REAR_OFFSET = 0;

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
}

// calibration and processing constants
namespace VCFSystemConstants { 
    constexpr float LBS_TO_NEWTONS = 4.4482216153;

    // Steering System Constants
    constexpr uint32_t MIN_STEERING_SIGNAL_ANALOG_ADDR = 56; //Raw ADC value from analog sensor at minimum (left) steering angle (calibration) TODO: test and find real values for min&max
    constexpr uint32_t MAX_STEERING_SIGNAL_ANALOG_ADDR = 60; //Raw ADC value from analog sensor at maximum (right) steering angle
    constexpr uint32_t MIN_STEERING_SIGNAL_DIGITAL_ADDR = 32; //Raw ADC value from digital sensor at minimum (left) steering angle
    constexpr uint32_t MAX_STEERING_SIGNAL_DIGITAL_ADDR = 36; //Raw ADC value from digital sensor at maximum (right) steering angle

    constexpr int32_t ANALOG_MIN_WITH_MARGINS_ADDR = 40;
    constexpr int32_t ANALOG_MAX_WITH_MARGINS_ADDR = 44;
    constexpr int32_t DIGITAL_MIN_WITH_MARGINS_ADDR = 48;
    constexpr int32_t DIGITAL_MAX_WITH_MARGINS_ADDR = 52;


    // implausibility values
    constexpr float ANALOG_TOL = 0.005f; //+- 0.5% error (analog sensor tolerance according to datasheet)
    constexpr float DIGITAL_TOL_DEG = 0.2f; // +- 0.2 degrees error
   
    // rate of angle change
    constexpr float MAX_DTHETA_THRESHOLD = 5.0f; //maximum change in angle since last reading to consider the reading valid

    // degrees per bit
    constexpr float DEG_PER_COUNT_DIGITAL = 360.0f / 16384.0f;
    constexpr float DEG_PER_COUNT_ANALOG = 360.0f / 4096.0f;
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
    constexpr unsigned long STEERING_SAMPLE_PERIOD = 1000;         // 2000 us = 500 Hz
    constexpr unsigned long STEERING_SAMPLE_PRIORITY = 10;

    constexpr unsigned long LOADCELL_SAMPLE_PERIOD = 250;         // 250 us = 4 kHz
    constexpr unsigned long LOADCELL_SAMPLE_PRIORITY = 24;

    constexpr unsigned long ETHERNET_SEND_PERIOD = 100000;        // 100 000 us = 10Hz
    constexpr unsigned long ETHERNET_SEND_PRIORITY = 20;

    constexpr unsigned long LOADCELL_SEND_PERIOD = 4000;          // 4 000 us = 250 Hz
    constexpr unsigned long LOADCELL_SEND_PRIORITY = 25;

    constexpr unsigned long PEDALS_RECALIBRATION_PRIORITY = 150;
    constexpr unsigned long PEDALS_RECALIBRATION_PERIOD = 100000; // 100 000 us = 10 Hz
    
    constexpr unsigned long STEERING_RECALIBRATION_PRIORITY = 150; // TODO: Determine real values for these
    constexpr unsigned long STEERING_RECALIBRATION_PERIOD = 100000;

    // IIR filter alphas
    constexpr float LOADCELL_IIR_FILTER_ALPHA = 0.01f;

    constexpr unsigned long WATCHDOG_PRIORITY = 1;
    constexpr unsigned long WATCHDOG_KICK_PERIOD = 10000;          // 10 000 us = 100 Hz
}

#endif /* VCF_CONSTANTS */
