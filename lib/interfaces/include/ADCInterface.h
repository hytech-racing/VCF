#ifndef ADC_INTERFACE_H
#define ADC_INTERFACE_H

/// @brief This file defines ADC interface. The interface instantiates the ADC modules and provides getter functions to access each sensor's values.

#include <MCP_ADC.h>
#include "etl/singleton.h"

namespace adc_default_parameters {
  constexpr const unsigned int channels_within_mcp_adc = 8;
}
struct ADCPinout_s {
    int adc0_spi_cs_pin;
    int adc1_spi_cs_pin;
};

struct ADCChannels_s {
    /* ADC 0 */
    int steering_cw_channel;
    int steering_ccw_channel;
    int fr_loadcell_channel;
    int fl_loadcell_channel;
    int fr_suspot_channel;
    int fl_suspot_channel;

    /* ADC 1 */
    int accel_1_channel;
    int accel_2_channel;
    int brake_1_channel;
    int brake_2_channel;
};

struct ADCScales_s {
    /* ADC 0 */
    float steering_cw_scale;
    float steering_ccw_scale;
    float fr_loadcell_scale;
    float fl_loadcell_scale;
    float fr_suspot_scale;
    float fl_suspot_scale;

    /* ADC 1 */
    float accel_1_scale;
    float accel_2_scale;
    float brake_1_scale;
    float brake_2_scale;
};

struct ADCOffsets_s {
    /* ADC 0 */
    float steering_cw_offset;
    float steering_ccw_offset;
    float fr_loadcell_offset;
    float fl_loadcell_offset;
    float fr_suspot_offset;
    float fl_suspot_offset;

    /* ADC 1 */
    float accel_1_offset;
    float accel_2_offset;
    float brake_1_offset;
    float brake_2_offset;
};

struct ADCInterfaceParams_s {
    ADCPinout_s pinouts;
    ADCChannels_s channels;
    ADCScales_s scales;
    ADCOffsets_s offsets;
};

class ADCInterface
{
    /* ------ Public Functions ------ */
    public:
        ADCInterface(ADCPinout_s pinouts, ADCChannels_s channels, ADCScales_s scales, ADCOffsets_s offsets) :
            _adc_parameters {
                .pinouts = pinouts,
                .channels = channels,
                .scales = scales,
                .offsets = offsets
            },
            _adc0 (
                _adc_parameters.pinouts.adc0_spi_cs_pin,
                MCP_ADC_DEFAULT_SPI_SDI,
                MCP_ADC_DEFAULT_SPI_SDO,
                MCP_ADC_DEFAULT_SPI_CLK,
                MCP_ADC_DEFAULT_SPI_SPEED,
                adc0_scales().data(),
                adc0_offsets().data()
            ), 
            _adc1 (
                _adc_parameters.pinouts.adc1_spi_cs_pin,
                MCP_ADC_DEFAULT_SPI_SDI,
                MCP_ADC_DEFAULT_SPI_SDO,
                MCP_ADC_DEFAULT_SPI_CLK,
                MCP_ADC_DEFAULT_SPI_SPEED,
                adc1_scales().data(),
                adc1_offsets().data()
            )
        {};

        void adc0_tick();
        void adc1_tick();

        static float iir_filter(float alpha, float prev_value, float new_value);

        /* ------ ADC 0 ------ */
        
        /**
         * @return Analog Steering Degrees [Steering 1]
         */
        AnalogConversion_s steering_degrees_cw();
        
        /**
         * @return Analog Steering Degrees [Steering 2]
         */
        AnalogConversion_s steering_degrees_ccw();
        
        /**
         * @return Front Left Cell
         */
        AnalogConversion_s FL_load_cell();
        
        /**
         * @return Front Right Load Cell
         */
        AnalogConversion_s FR_load_cell();
        
        /**
         * @return Front Left Suspension Potentiometer Reading 
         */
        AnalogConversion_s FL_sus_pot();
        
        /**
         * @return Front Right Suspension Potentiometer Reading
         */
        AnalogConversion_s FR_sus_pot();

        /* ------ ADC 1 ------ */

        /**
         * @return Acceleration Pedal 1
         */
        AnalogConversion_s acceleration_1();

        /**
         * @return Acceleration Pedal 2
         */
        AnalogConversion_s acceleration_2();
        
        /**
         * @return Brake Pedal 1
         */
        AnalogConversion_s brake_1();
        
        /**
         * @return Brake Pedal 2
         */
        AnalogConversion_s brake_2();
        
    /* ------ Private Functions ------ */
    private:
        std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_scales();
        std::array<float, adc_default_parameters::channels_within_mcp_adc> adc0_offsets();
        std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_scales();
        std::array<float, adc_default_parameters::channels_within_mcp_adc> adc1_offsets();

    /* ------ Private Data Members ------ */
    private:
        ADCInterfaceParams_s _adc_parameters = {};
        // MCP3208. ADC0 in VCF schematic. Used for steering, load cells, and sus pots.
        MCP_ADC<adc_default_parameters::channels_within_mcp_adc> _adc0;
        // MCP3208. ADC1 in VCF schematic. Used for pedal position sensors.
        MCP_ADC<adc_default_parameters::channels_within_mcp_adc> _adc1;

};

using ADCInterfaceInstance = etl::singleton<ADCInterface>;

#endif /* ADC_INTERFACE_H */