#include "MCP_ADC.h"
#include "VCF_Constants.h"
#include "etl/singleton.h"

#include "Logger.h"

constexpr unsigned int channels_within_mcp_adc = 8;
using ADC1Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
// using ADC2Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
void setup()
{
    SPI.begin();
    float scales[channels_within_mcp_adc] = {1, 1, 1, 1, 1, 1, 1, 1};
    float offsets[channels_within_mcp_adc] = {0, 0, 0, 0, 0, 0, 0, 0};
    // ADC2Instance::create(ADC2_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);
    ADC1Instance::create(ADC1_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, scales, offsets);
}

void loop()
{
    ADC1Instance::instance().tick();
    hal_printf("ADC1 reading 0 %d\n", ADC1Instance::instance().data.conversions[0].raw);
    hal_printf("ADC1 reading 1 %d\n", ADC1Instance::instance().data.conversions[1].raw);
    hal_printf("ADC1 reading 2 %d\n", ADC1Instance::instance().data.conversions[2].raw);
    hal_printf("ADC1 reading 3 %d\n", ADC1Instance::instance().data.conversions[3].raw);
    hal_printf("ADC1 reading 4 %d\n", ADC1Instance::instance().data.conversions[4].raw);
    hal_printf("ADC1 reading 5 %d\n", ADC1Instance::instance().data.conversions[5].raw);

    hal_printf("\n\n");
    // ADC2Instance::instance().tick();
    // hal_printf("ADC2 reading 0 %d\n", ADC2Instance::instance().data.conversions[0].raw);
    // hal_printf("ADC2 reading 1 %d\n", ADC2Instance::instance().data.conversions[1].raw);
    // hal_printf("ADC2 reading 2 %d\n", ADC2Instance::instance().data.conversions[2].raw);
    // hal_printf("ADC2 reading 3 %d\n", ADC2Instance::instance().data.conversions[3].raw);
    // hal_printf("ADC2 reading 4 %d\n", ADC2Instance::instance().data.conversions[4].raw);
    // hal_printf("ADC2 reading 5 %d\n", ADC2Instance::instance().data.conversions[5].raw);

    // hal_printf("\n\n");

    

}