#ifndef IO_EXPANDER_UTILS_H
#define IO_EXPANDER_UTILS_H

/* Standard int library */
#include <stdint.h>
#include <stdbool.h>

#include "MCP23017.h"

namespace IOExpanderUtils
{
    /**
    IOExpander's read() only reads.
    getBit() only get specified bit from previously read dataframe and does not read()
    @param data data from which to get the specified bit
    @param port port from which to get the bit from
    @param bit  bit number of port to get bit from
    */
    bool getBit(uint16_t data, MCP23017Port port, uint8_t bit);
}

#endif