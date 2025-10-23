#ifndef IO_EXPANDER_UTILS_H
#define IO_EXPANDER_UTILS_H

/* Standard int library */
#include <stdint.h>
#include <stdbool.h>

namespace IOExpanderUtils
{
    /*
    IOExpander's read() only reads.
    getBit() only get specified bit from previously read dataframe and does not read()
    @param port 0=A; 1=B
    */
    bool getBit(uint16_t data, bool port, int bit);
}

#endif