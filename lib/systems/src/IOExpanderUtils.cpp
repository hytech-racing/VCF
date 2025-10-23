#include "IOExpanderUtils.h"

bool IOExpanderUtils::getBit(uint16_t data, bool port, int bit)
{
    if(!port){ //0=A
        return (data>>bit)&1;
    }
    return (data>>(8+bit))&1; // NOLINT (B port is in upper 8 bits, while A port is in lower 8 bits)
}