// #include "MCP_ADC.h"
#include "VCF_Constants.h"
// #include "etl/singleton.h"
// #include "VCF_Tasks.h"
#include "ADCInterface.h"

#include "Logger.h"

// constexpr unsigned int channels_within_mcp_adc = 8;
// using ADC1Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
// using ADC2Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
void setup()
{
    SPI.begin();
    ADCInterfaceInstance::create(
        ADCPinout_s {
            ADC1_CS,
            ADC2_CS
        },
        ADCChannels_s {
            STEERING_1_CHANNEL,
            STEERING_2_CHANNEL,
            FR_LOADCELL_CHANNEL,
            FL_LOADCELL_CHANNEL,
            FR_SUS_POT_CHANNEL,
            FL_SUS_POT_CHANNEL,
            ACCEL_1_CHANNEL,
            ACCEL_2_CHANNEL,
            BRAKE_1_CHANNEL,
            BRAKE_2_CHANNEL
        },
        ADCScales_s { 
            STEERING_1_SCALE, 
            STEERING_2_SCALE, 
            FR_LOADCELL_SCALE,
            FL_LOADCELL_SCALE,
            FR_SUS_POT_SCALE,
            FL_SUS_POT_SCALE, 
            ACCEL_1_SCALE, 
            ACCEL_2_SCALE, 
            BRAKE_1_SCALE, 
            BRAKE_2_SCALE
        }, 
        ADCOffsets_s {
            STEERING_1_OFFSET,
            STEERING_2_OFFSET,
            FR_LOADCELL_OFFSET,
            FL_LOADCELL_OFFSET,
            FR_SUS_POT_OFFSET,
            FL_SUS_POT_OFFSET,
            ACCEL_1_OFFSET,
            ACCEL_2_OFFSET,
            BRAKE_1_OFFSET,
            BRAKE_2_OFFSET
        }
    );

    // float scales[channels_within_mcp_adc] = {1, 1, 1, 1, 1, 1, 1, 1};
    // float offsets[channels_within_mcp_adc] = {0, 0, 0, 0, 0, 0, 0, 0};
    // ADC2Instance::create(ADC2_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);
    // ADC1Instance::create(ADC1_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);
}

void loop()
{
    // ADC1Instance::instance().tick();

    ADCInterfaceInstance::instance().adc1_tick();
    hal_printf("ADC1 Front Right Load Cell Raw: %d\n", ADCInterfaceInstance::instance().FR_load_cell().raw);
    hal_printf("ADC1 Front Left Load Cell Raw:  %d\n", ADCInterfaceInstance::instance().FL_load_cell().raw);
    hal_printf("ADC1 Front Right Sus Pot Raw:   %d\n", ADCInterfaceInstance::instance().FR_sus_pot().raw);
    hal_printf("ADC1 Front Left Sus Pot Raw:    %d\n", ADCInterfaceInstance::instance().FL_sus_pot().raw);
    hal_printf("ADC1 Steering Degrees CCW Raw:  %d\n", ADCInterfaceInstance::instance().steering_degrees_ccw().raw);
    hal_printf("ADC1 Steering Degrees CW Raw:   %d\n", ADCInterfaceInstance::instance().steering_degrees_cw().raw);

    hal_printf("\n\n");
    // ADC2Instance::instance().tick();
    ADCInterfaceInstance::instance().adc2_tick();
    hal_printf("ADC2 Acceleration 1 Raw: %d\n", ADCInterfaceInstance::instance().acceleration_1().raw);
    hal_printf("ADC2 Acceleration 2 Raw: %d\n", ADCInterfaceInstance::instance().acceleration_2().raw);
    hal_printf("ADC2 Brake 1 Raw:        %d\n", ADCInterfaceInstance::instance().brake_1().raw);
    hal_printf("ADC2 Brake 2 Raw:        %d\n", ADCInterfaceInstance::instance().brake_2().raw);

    hal_printf("\n\n");

    

}