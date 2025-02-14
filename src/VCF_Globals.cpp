#include "VCF_Globals.h"
#include "VCF_Constants.h"

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Interface and system data structs */
VCFData_s vcf_data = {};

/* ADC setup */
MCP_ADC<channels_within_mcp_adc> adc_1 = MCP_ADC<channels_within_mcp_adc>(ADC1_CS);
MCP_ADC<channels_within_mcp_adc> adc_2 = MCP_ADC<channels_within_mcp_adc>(ADC2_CS);

uint16_t VCF_SEND_PORT = 7777;
uint16_t VCF_RECV_PORT = 8888;

/* Ethernet socket instantiation */
qindesign::network::EthernetUDP protobuf_send_socket;
qindesign::network::EthernetUDP protobuf_recv_socket;

/* Ethernet Constants */
const IPAddress debug_ip(192, 168, 1, 31); // Computer receive IP
const IPAddress default_VCF_ip(192, 168, 1, 30);
const IPAddress default_dns(192, 168, 1, 1); 
const IPAddress default_gateway(192, 168, 1, 1); 
const IPAddress car_subnet(255, 255, 255, 0); 