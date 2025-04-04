#ifndef VCF_CONSTANTS
#define VCF_CONSTANTS


/* -------------------------------------------------- */
/*                 Teensy 4.1 GPIO pins               */
/* -------------------------------------------------- */





// Digital Pins on Teensy
constexpr int ADC1_CS = 33; // MCP3208. ADC1 in VCF schematic. Used for steering, sus pots, and load cells.
constexpr int ADC2_CS = 34; // MCP3208. ADC2 in VCF schematic. Used for pedal position sensors.

constexpr int BTN_DIM_READ = 10;
constexpr int BTN_PRESET_READ = 9;
constexpr int BTN_MC_CYCLE_READ = 8;
constexpr int BTN_MODE_READ = 7;
constexpr int BTN_START_READ = 6;
constexpr int BTN_DATA_READ = 22;
constexpr int BUZZER_CONTROL_PIN = 3;
constexpr int RIGHT_SHIFTER = 26;
constexpr int LEFT_SHIFTER = 27;

constexpr int I2C_SCL = 4;
constexpr int I2C_SDA = 5;



/* -------------------------------------------------- */
/*                 ADC pins and configs               */
/* -------------------------------------------------- */

/* Channels on ADC_1 */
const int STEERING_1_CHANNEL      = 0;
const int STEERING_2_CHANNEL      = 1;
const int FR_SUS_POT_CHANNEL      = 2;
const int FR_LOADCELL_CHANNEL     = 3;
const int FL_SUS_POT_CHANNEL      = 4;
const int FL_LOADCELL_CHANNEL     = 5;
// const int UNUSED_CHANNEL       = 6;
// const int UNUSED_CHANNEL       = 7;

/* Channels on ADC_2 */
// const int UNUSED_CHANNEL       = 0;
// const int UNUSED_CHANNEL       = 1;
const int ACCEL_1_CHANNEL         = 2;
const int ACCEL_2_CHANNEL         = 3;
const int BRAKE_1_CHANNEL         = 4;
const int BRAKE_2_CHANNEL         = 5;
// const int UNUSED_CHANNEL       = 6;
// const int UNUSED_CHANNEL       = 7;

/* Scaling and offset */
const float STEERING_1_SCALE = 0.02197265f; // TODO: Figure out what these mean
const float STEERING_1_OFFSET = 0;
const float STEERING_2_SCALE = 0.02197265f; // TODO: Figure out if steering 2 = steering 1
const float STEERING_2_OFFSET = 0;
// Scale for steering sensor = 0.02197265 . Offset has to be mechanically determined

const float FR_LOADCELL_SCALE = 0.1149f; //Values are from the old MCU rev15 // TODO: Calibrate load cells
const float FR_LOADCELL_OFFSET = 13.526f / FR_LOADCELL_SCALE;
const float FL_LOADCELL_SCALE = 0.118f;
const float FL_LOADCELL_OFFSET = 25.721f / FL_LOADCELL_SCALE;
const float FR_SUS_POT_SCALE = 1;
const float FR_SUS_POT_OFFSET = 0;
const float FL_SUS_POT_SCALE = 1;
const float FL_SUS_POT_OFFSET = 0;

const float ACCEL_1_SCALE = 1; // TODO: Figure out what these should be
const float ACCEL_1_OFFSET = 0;
const float ACCEL_2_SCALE = 1;
const float ACCEL_2_OFFSET = 0;
const float BRAKE_1_SCALE = 1;
const float BRAKE_1_OFFSET = 0;
const float BRAKE_2_SCALE = 1;
const float BRAKE_2_OFFSET = 0;



/* -------------------------------------------------- */
/*           Task Periods and Priorities              */
/* -------------------------------------------------- */
constexpr int MAIN_TASK_PRIORITY = 5;
constexpr int PEDALS_PRIORITY = 5;
constexpr int DASH_SEND_PRIORITY = 7;

constexpr int DEBUG_PRIORITY = 100;

constexpr unsigned long DASH_SEND_PERIOD = 100000;        // 100,000 us = 10 Hz
constexpr unsigned long DEBUG_PERIOD = 50000;             // 50,000 us = 20 Hz

constexpr unsigned long CAN_SEND_PRIORITY = 10;
constexpr unsigned long CAN_SEND_PERIOD = 2000;

constexpr unsigned long PEDALS_SEND_PERIOD = 3000;        // 3 000 us = 333 Hz
constexpr unsigned long PEDALS_SAMPLE_PERIOD = 500;

constexpr unsigned long BUZZER_WRITE_PERIOD = 100000;     // 100 000 us = 10 Hz
constexpr unsigned long BUZZER_PRIORITY = 20;


#endif /* VCF_CONSTANTS */