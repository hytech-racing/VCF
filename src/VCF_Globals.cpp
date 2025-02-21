#include "VCF_Globals.h"
#include "VCF_Constants.h"

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-types */
#include "SharedFirmwareTypes.h"

/* Interface and system data structs */
VCFData_s vcf_data = {};

uint16_t VCF_PORT = EthernetIPDefsInstance::instance().VCFData_port;

/* Ethernet Constants */
const IPAddress debug_ip(192, 168, 1, 31); // Computer receive IP
const IPAddress default_VCF_ip(192, 168, 1, 30);
const IPAddress default_dns(192, 168, 1, 1); 
const IPAddress default_gateway(192, 168, 1, 1); 
const IPAddress car_subnet(255, 255, 255, 0); 