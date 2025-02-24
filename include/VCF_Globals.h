#ifndef VCF_GLOBALS
#define VCF_GLOBALS

/* C++ library includes */
#include <array>

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-interfaces */
#include "EthernetAddressDefs.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCF_Constants.h"

/* From Embedded Template Library */
#include <etl/singleton.h>

/* Interface and system data structs */
extern VCFData_s vcf_data; // NOLINT
extern VCFInterfaceData_s interface_data;
extern VCFSystemData_s system_data;
extern VCRSystemData_s vcr_system_data;

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;

/**
 * Bundle of the two ADCs that exist on VCF. This allows us to use a singleton class that has two (and only two!) instances of MCP_ADC.
 */
struct ADCsOnVCF_s
{
    ADCsOnVCF_s(const float (&adc_1_scales)[channels_within_mcp_adc],
                const float (&adc_1_offsets)[channels_within_mcp_adc],
                const float (&adc_2_scales)[channels_within_mcp_adc],
                const float (&adc_2_offsets)[channels_within_mcp_adc]) :
        adc_1(ADC1_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_1_scales, adc_1_offsets),
        adc_2(ADC2_CS, MCP_ADC_DEFAULT_SPI_SDI, MCP_ADC_DEFAULT_SPI_SDO, MCP_ADC_DEFAULT_SPI_CLK, MCP_ADC_DEFAULT_SPI_SPEED, adc_2_scales, adc_2_offsets)
    {}
    
    // MCP3208. ADC1 in VCF schematic. Used for steering, load cells, and sus pots.
    MCP_ADC<channels_within_mcp_adc> adc_1;

    // MCP3208. ADC2 in VCF schematic. Used for pedal position sensors.
    MCP_ADC<channels_within_mcp_adc> adc_2;

};

using ADCsOnVCFInstance = etl::singleton<ADCsOnVCF_s>;

#endif /* VCF_GLOBALS */