#ifndef VCF_CONSTANTS
#define VCF_CONSTANTS


/* -------------------------------------------------- */
/*                 Teensy 4.1 GPIO pins               */
/* -------------------------------------------------- */


// Digital Pins on Teensy
constexpr int ADC1_CS = 17; // MCP3208. ADC1 in VCF schematic. Used for steering, sus pots, and load cells.
constexpr int ADC2_CS = 16; // MCP3208. ADC2 in VCF schematic. Used for pedal position sensors.

constexpr int BTN_DIM_READ = 29;
constexpr int BTN_PRESET_READ = 31;
constexpr int BTN_MC_CYCLE_READ = 27;
constexpr int BTN_MODE_READ = 26;
constexpr int BTN_START_READ = 30;
constexpr int BTN_DATA_READ = 28;
constexpr int BUZZER_CONTROL_PIN = 1;
constexpr int RIGHT_SHIFTER = 9;
constexpr int LEFT_SHIFTER = 10;

constexpr int NEOPIXEL_CONTROL_PIN = 0;
constexpr int NEOPIXEL_COUNT = 12; // 12 neopixeles on dashboard

/* -------------------------------------------------- */
/*                 ADC pins and configs               */
/* -------------------------------------------------- */

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

/* Scaling and offset */
constexpr float STEERING_1_SCALE = 0.02197265f; // TODO: Figure out what these mean
constexpr float STEERING_1_OFFSET = 0;
constexpr float STEERING_2_SCALE = 0.02197265f; // TODO: Figure out if steering 2 = steering 1
constexpr float STEERING_2_OFFSET = 0;
// Scale for steering sensor = 0.02197265 . Offset has to be mechanically determined

constexpr float FR_LOADCELL_SCALE = 0.1149f; //Values are from the old MCU rev15 // TODO: Calibrate load cells
constexpr float FR_LOADCELL_OFFSET = 13.526f / FR_LOADCELL_SCALE;
constexpr float FL_LOADCELL_SCALE = 0.118f;
constexpr float FL_LOADCELL_OFFSET = 25.721f / FL_LOADCELL_SCALE;
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



/* -------------------------------------------------- */
/*           Task Periods and Priorities              */
/* -------------------------------------------------- */
constexpr unsigned long MAIN_TASK_PRIORITY = 5;

constexpr unsigned long CAN_SEND_PRIORITY = 10;
constexpr unsigned long CAN_SEND_PERIOD = 2000;

constexpr unsigned long PEDALS_SEND_PERIOD = 3000;        // 3 000 us = 333 Hz
constexpr unsigned long PEDALS_SAMPLE_PERIOD = 500;       // 500 us = 2 kHz
constexpr unsigned long PEDALS_PRIORITY = 5;

constexpr unsigned long BUZZER_WRITE_PERIOD = 100000;     // 100 000 us = 10 Hz
constexpr unsigned long BUZZER_PRIORITY = 20;

constexpr unsigned long DASH_SAMPLE_PERIOD = 100000;     // 100 000 us = 10 Hz
constexpr unsigned long DASH_SAMPLE_PRIORITY = 21;

constexpr unsigned long DASH_SEND_PERIOD = 100000;        // 100,000 us = 10 Hz
constexpr unsigned long DASH_SEND_PRIORITY = 7;

constexpr unsigned long DEBUG_PRIORITY = 100;
constexpr unsigned long DEBUG_PERIOD = 10000;             // 500,000 us = 2 Hz

constexpr unsigned long NEOPIXEL_UPDATE_PRIORITY = 90;
constexpr unsigned long NEOPIXEL_UPDATE_PERIOD = 100000;             // 100,000 us = 10 Hz

constexpr unsigned long STEERING_SEND_PERIOD = 100000;        // 100,000 us = 10 Hz
constexpr unsigned long STEERING_SEND_PRIORITY = 25;

constexpr unsigned long LOADCELL_SEND_PERIOD = 100000;        // 100,000 us = 10 Hz
constexpr unsigned long LOADCELL_SEND_PRIORITY = 25;

constexpr unsigned long WATCHDOG_PRIORITY = 1;
constexpr unsigned long WATCHDOG_KICK_PERIOD = 1000; // 100 us = 10000 Hz


constexpr int WATCHDOG_PIN = 33;
constexpr int SOFTWARE_OK_PIN = 34; // Watchdog's !RESET pin
constexpr unsigned long WATCHDOG_KICK_INTERVAL_MS = 10000UL;

#endif /* VCF_CONSTANTS */