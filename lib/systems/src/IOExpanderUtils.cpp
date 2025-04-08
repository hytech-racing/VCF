#include "IOExpanderUtils.h"

bool IOExpanderUtils::getBit(uint16_t data, bool port, int bit)
{
    if(!port){ //0=A
        return (data>>bit)&1;
    }
    return (data>>(8+bit))&1;
}