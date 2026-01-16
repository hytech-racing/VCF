#ifndef VCF_GLOBALS
#define VCF_GLOBALS

/* C++ library includes */
#include <array>

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"
#include "ORBIS_BR10.h"

/* From shared-firmware-interfaces */
#include "EthernetAddressDefs.h"

/* From shared-firmware-types */
#include "PedalsSystem.h"
#include "SharedFirmwareTypes.h"

/* Local includes */
#include "VCF_Constants.h"

/* From Embedded Template Library */
#include <etl/singleton.h>

/* Ethernet includes */
#include "QNEthernet.h"

/* From MCP23017 Library */
#include "MCP23017.h"


/* Interface and system data structs */
using VCFData_sInstance = etl::singleton<VCFData_s>;
using VCRData_sInstance = etl::singleton<VCRData_s>;

/* IOExpander setup */
using IOExpanderInstance = etl::singleton<MCP23017>;

/* Digital Steering Setup */
using OrbisBRInstance = etl::singleton<OrbisBR10>;

/* Ethernet sockets */
extern qindesign::network::EthernetUDP VCF_socket;
extern qindesign::network::EthernetUDP VCR_socket;

#endif /* VCF_GLOBALS */