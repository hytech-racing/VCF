#include "MCP23017.h"
#include "IOExpanderUtils.h"

/*
 * Retrieves the bit from the data frame.
 * Port A = 0, and is the lower byte of data. Port B = 1, and is the higher byte of data.
 */
bool IOExpanderUtils::getBit(uint16_t data, MCP23017Port port, uint8_t bit)
{
    return (data >> ((uint16_t) port * 8 + bit)) & 1;
}