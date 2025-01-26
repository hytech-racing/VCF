#include "VCF_Tasks.h"
#include "VCF_Globals.h"
#include "VCF_Constants.h"


bool init_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_1.setChannelScaleAndOffset(STEERING_1_CHANNEL, STEERING_1_SCALE, STEERING_1_OFFSET);
    adc_1.setChannelScaleAndOffset(STEERING_2_CHANNEL, STEERING_2_SCALE, STEERING_2_OFFSET);
    adc_1.setChannelScaleAndOffset(FR_SUS_POT_CHANNEL, FR_SUS_POT_SCALE, FR_SUS_POT_OFFSET);
    adc_1.setChannelScaleAndOffset(FR_LOADCELL_CHANNEL, FR_LOADCELL_SCALE, FR_LOADCELL_OFFSET);
    adc_1.setChannelScaleAndOffset(FL_SUS_POT_CHANNEL, FL_SUS_POT_SCALE, FL_SUS_POT_OFFSET);
    adc_1.setChannelScaleAndOffset(FL_LOADCELL_CHANNEL, FL_LOADCELL_SCALE, FL_LOADCELL_OFFSET);
    return true;
}
bool run_read_adc1_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_1.sample(); // Samples all eight channels.
    adc_1.convert(); // Converts all eight channels.

    interface_data.steering_unfiltered.analog_steering_unfiltered_degrees = adc_1.data.conversions[STEERING_1_CHANNEL].conversion; // Only using steering 1 for now
    interface_data.front_loadcells_unfiltered.FL_loadcell_unfiltered_pounds = adc_1.data.conversions[FL_LOADCELL_CHANNEL].conversion;
    interface_data.front_loadcells_unfiltered.FR_loadcell_unfiltered_pounds = adc_1.data.conversions[FR_LOADCELL_CHANNEL].conversion;
    interface_data.front_suspots_unfiltered.FL_sus_pot_unfiltered_analog = adc_1.data.conversions[FL_SUS_POT_CHANNEL].raw; // Just use raw for suspots
    interface_data.front_suspots_unfiltered.FR_sus_pot_unfiltered_analog = adc_1.data.conversions[FR_SUS_POT_CHANNEL].raw; // Just use raw for suspots

    return true;
}
HT_TASK::Task read_adc1_task = HT_TASK::Task(init_read_adc1_task, run_read_adc1_task, 10, 1000UL); // 1000us is 1kHz //NOLINT



bool init_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_2.setChannelScaleAndOffset(ACCEL_1_CHANNEL, ACCEL_1_SCALE, ACCEL_1_OFFSET);
    adc_2.setChannelScaleAndOffset(ACCEL_2_CHANNEL, ACCEL_2_SCALE, ACCEL_2_OFFSET);
    adc_2.setChannelScaleAndOffset(BRAKE_1_CHANNEL, BRAKE_1_SCALE, BRAKE_1_OFFSET);
    adc_2.setChannelScaleAndOffset(BRAKE_2_CHANNEL, BRAKE_2_SCALE, BRAKE_2_OFFSET);

    return true;
}
bool run_read_adc2_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    adc_2.sample(); // Samples all eight channels.
    adc_2.convert(); // Converts all eight channels.

    interface_data.pedals_unfiltered.accel1_unfiltered_percent = adc_2.data.conversions[ACCEL_1_CHANNEL].conversion;
    interface_data.pedals_unfiltered.accel2_unfiltered_percent = adc_2.data.conversions[ACCEL_2_CHANNEL].conversion;
    interface_data.pedals_unfiltered.brake1_unfiltered_percent = adc_2.data.conversions[BRAKE_1_CHANNEL].conversion;
    interface_data.pedals_unfiltered.brake1_unfiltered_percent = adc_2.data.conversions[BRAKE_2_CHANNEL].conversion;

    return true;
}

bool init_read_gpio_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Setting digital/analog buttons D10-D6, A8 as inputs
    pinMode(BTN_DIM_READ, INPUT);
    pinMode(BTN_PRESET_READ, INPUT);
    pinMode(BTN_MC_CYCLE_READ, INPUT);
    pinMode(BTN_MODE_READ, INPUT);
    pinMode(BTN_START_READ, INPUT);
    pinMode(BTN_DATA_READ, INPUT); // this is an analog pin, so technically no need to use pinMode

    // Setting digital controls D5-D0 as inputs
    pinMode(DIAL_EXPANDER_SDA, INPUT);
    pinMode(DIAL_EXPANDER_SCL, INPUT);
    pinMode(BUZZER_CTRL, INPUT);
    pinMode(DASH_NEOPIXEL, INPUT);
    pinMode(DASH_TX, INPUT);
    pinMode(DASH_RX, INPUT);

    // Setting digital SPI controls D13-D11 as inputs
    pinMode(SPI_CLK, INPUT);
    pinMode(SPI_MISO, INPUT);
    pinMode(SPI_MOSI, INPUT);

    // Setting ADC chip select D33 and D34 as inputs
    pinMode(ADC1_CS, INPUT);
    pinMode(ADC2_CS, INPUT);

    // Setting telemetry CAN controls D29 and D28 as inputs
    pinMode(TELEM_CAN_TX, INPUT);
    pinMode(TELEM_CAN_RX, INPUT);
    
    return true;
}
bool run_read_gpio_task(const unsigned long& sysMicros, const HT_TASK::TaskInfo& taskInfo)
{
    // Doing digital read on all digital inputs
    
    // Doing analog read on all analog inputs
    
    return true;
}

HT_TASK::Task read_adc2_task = HT_TASK::Task(init_read_adc2_task, run_read_adc2_task, 10, 1000UL); // 1000us is 1kHz //NOLINT
