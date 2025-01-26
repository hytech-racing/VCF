#ifndef VCF_CONSTANTS
#define VCF_CONSTANTS


/* -------------------------------------------------- */
/*                 Teensy 4.1 GPIO pins               */
/* -------------------------------------------------- */

// Analog Pins on Teensy
const int DASH_LINK = 41;
const int TEENSY_LINK = 40;
const int VCR_LINK = 39;
const int MAG_3_LINK = 38;

const int LEFT_SHIFTER = 27;
const int RIGHT_SHIFTER = 26;
const int POE_1_LINK = 25;
const int POE_2_LINK = 24;

const int BRAKE_MISSING_OUT = 23;
const int BTN_DATA_READ = 22;
const int ETH_SDA = 20;
const int ETH_SCL = 19;
const int SHDN_B_OUT = 18;
const int SHDN_A_OUT = 17;
const int SHDN_C_OUT = 16;
const int STEERING_TX = 15;
const int STEERING_RX = 14;


// Digital Pins on Teensy
const int ADC1_CS = 33; // MCP3208. ADC1 in VCF schematic. Used for steering, sus pots, and load cells.
const int ADC2_CS = 34; // MCP3208. ADC2 in VCF schematic. Used for pedal position sensors.

const int TELEM_CAN_TX = 29;
const int TELEM_CAN_RX = 28;

const int SPI_CLK = 13;
const int SPI_MISO = 12;
const int SPI_MOSI = 11;

const int BTN_DIM_READ = 10;
const int BTN_PRESET_READ = 9;
const int BTN_MC_CYCLE_READ = 8;
const int BTN_MODE_READ = 7;
const int BTN_START_READ = 6;
const int DIAL_EXPANDER_SDA = 5;
const int DIAL_EXPANDER_SCL = 4;
const int BUZZER_CTRL = 3;
const int DASH_NEOPIXEL = 2;
const int DASH_TX = 1;
const int DASH_RX = 0;





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
const float STEERING_1_SCALE = 1; // TODO: Figure out what these mean
const float STEERING_1_OFFSET = 0;
const float STEERING_2_SCALE = 1;
const float STEERING_2_OFFSET = 0;

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
/*                  Pedal calibration                 */
/* -------------------------------------------------- */
// I don't know what needs to go here. //TODO: Figure out what additional constants we need for pedals system


#endif /* VCF_CONSTANTS */