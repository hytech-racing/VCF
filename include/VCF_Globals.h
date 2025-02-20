#ifndef VCF_GLOBALS
#define VCF_GLOBALS

#include "etl/singleton.h"

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCF_Constants.h"

/* Interface and system data structs */
extern VCFInterfaceData_s interface_data; // NOLINT
extern VCFSystemData_s system_data; // NOLINT
extern VCRSystemData_s vcr_system_data; // NOLINT

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;
using ADC1Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
using ADC2Instance = etl::singleton<MCP_ADC<channels_within_mcp_adc>>;
// extern MCP_ADC<channels_within_mcp_adc> adc_1; // MCP3208. ADC1 in VCF schematic. Used for steering, load cells, and sus pots.
// extern MCP_ADC<channels_within_mcp_adc> adc_2; // MCP3208. ADC2 in VCF schematic. Used for pedal position sensors.


#endif /* VCF_GLOBALS */