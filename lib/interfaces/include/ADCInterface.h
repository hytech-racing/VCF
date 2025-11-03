#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H

/// @brief This file defines ADC interface - which should replace the handling of ADC code in the VCF repo. The actual implementation is currently in VCF_Tasks and we want to move it here.

#include <Arduino.h>
#include "SharedFirmwareTypes.h"
#include <MCP_ADC.h>
#include <VCF_Constants.h>
#include "etl/singleton.h"

/* -------------------------------------------------- */
/*                 ADC pins and configs               */
/* -------------------------------------------------- */
/* Channels Per ADC*/
constexpr unsigned int MCP_ADC_CHANNELS = 8;

/* Filter Alpha*/
constexpr unsigned int IIR_FILTER_ALPHA = 0.01f;

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
constexpr float STEERING_1_SCALE = 1; // TODO: Figure out what these mean
constexpr float STEERING_1_OFFSET = 0;
constexpr float STEERING_2_SCALE = 1; // TODO: Figure out if steering 2 = steering 1
constexpr float STEERING_2_OFFSET = 0;
// Scale for steering sensor = 0.02197265 . Offset has to be mechanically determined

// constexpr float FR_LOADCELL_SCALE = 0.81; //Values are from the old MCU rev15 // TODO: Calibrate load cells
// constexpr float FR_LOADCELL_OFFSET = 36.8;
// constexpr float FL_LOADCELL_SCALE = 0.63;
// constexpr float FL_LOADCELL_OFFSET = -3.9;

constexpr float LBS_TO_NEWTONS = 4.4482216153;

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


class ADCInterface
{
    public:
        ADCInterface(
                const float (&adc_1_scales)[MCP_ADC_CHANNELS],
                const float (&adc_1_offsets)[MCP_ADC_CHANNELS],
                const float (&adc_2_scales)[MCP_ADC_CHANNELS],
                const float (&adc_2_offsets)[MCP_ADC_CHANNELS]
            ) :
        adc1(ADC1_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_1_scales, adc_1_offsets), 
        adc2(ADC2_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_2_scales, adc_2_offsets)
        {};

        struct ADC1InterfaceData_s {
            float analog_steering_degrees;
            float FL_load_cell;
            float FR_load_cell;
            float FL_sus_pot;
            float FR_sus_pot;
        };

        struct ADC2InterfaceData_s {
            float acceleration_1;
            float acceleration_2;
            float brake_1;
            float brake_2;
        };
        
        using ADC1Data = etl::singleton<ADCInterface::ADC1InterfaceData_s>;
        using ADC2Data = etl::singleton<ADCInterface::ADC2InterfaceData_s>;

        ADC1InterfaceData_s read_adc1();
        ADC2InterfaceData_s read_adc2();

    private:
        
        MCP_ADC<MCP_ADC_CHANNELS> adc1;
        MCP_ADC<MCP_ADC_CHANNELS> adc2;
        static float iir_filter(float alpha, float prev_value, float new_value);
};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif /* ADC_INTERFACE_H */