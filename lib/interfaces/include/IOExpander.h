#ifndef IOEXPANDER_H
#define IOEXPANDER_H

#include <etl/singleton.h>
#include "MCP23017.h"

class IOExpander
{
public:
    IOExpander(uint8_t addr);
    MCP23017 &get_mcp() {return mcp;}

private:
    MCP23017 mcp;
    
public:
    /*
    Gets data from IOExpander.
    To retrieve updated values, use IOExpanderUtils::getBit()
    */
    uint16_t read();

};

using IOExpanderInstance = etl::singleton<IOExpander>;

#endif /* IOEXPANDER_H */