#include "VCF_Globals.h"
#include "VCF_Constants.h"

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Interface and system data structs */
VCFData_s vcf_data = {};
VCRData_s vcr_data = {};

qindesign::network::EthernetUDP VCF_socket;
qindesign::network::EthernetUDP VCR_socket;

// /* ADC setup */
// MCP_ADC<channels_within_mcp_adc> adc_1 = MCP_ADC<channels_within_mcp_adc>(ADC1_CS);
// MCP_ADC<channels_within_mcp_adc> adc_2 = MCP_ADC<channels_within_mcp_adc>(ADC2_CS);