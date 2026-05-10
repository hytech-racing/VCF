#ifndef VCF_GLOBALS
#define VCF_GLOBALS

/* C++ library includes */
#include <array>

/* From shared-firmware-interfaces */
#include "MCP_ADC.h"

/* From shared-firmware-interfaces */
#include "EthernetAddressDefs.h"
#include "Orbis_BR.h"

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

/* IOExpander setup */
using IOExpanderInstance = etl::singleton<MCP23017>;

#endif /* VCF_GLOBALS */