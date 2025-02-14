#ifndef VCF_GLOBALS
#define VCF_GLOBALS

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCF_Constants.h"

#include "QNEthernet.h"

/* Interface and system data structs */
extern VCFData_s vcf_data; // NOLINT

/* ADC setup */
constexpr unsigned int channels_within_mcp_adc = 8;
extern MCP_ADC<channels_within_mcp_adc> adc_1; // MCP3208. ADC1 in VCF schematic. Used for steering, load cells, and sus pots.
extern MCP_ADC<channels_within_mcp_adc> adc_2; // MCP3208. ADC2 in VCF schematic. Used for pedal position sensors.

/* Ethernet Constants */
extern const IPAddress debug_ip;
extern const IPAddress default_VCF_ip;
extern const IPAddress default_dns;
extern const IPAddress default_gateway;
extern const IPAddress car_subnet;

extern uint16_t VCF_SEND_PORT;
extern uint16_t VCF_RECV_PORT;

/* Ethernet message sockets */ // TODO: Move this into its own interface
extern qindesign::network::EthernetUDP protobuf_send_socket;
extern qindesign::network::EthernetUDP protobuf_recv_socket;

#endif /* VCF_GLOBALS */