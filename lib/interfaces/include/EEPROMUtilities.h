#ifndef EEPROM_UTILITIES_H
#define EEPROM_UTILITIES_H

#include <stdint.h>
#include <EEPROM.h>


namespace EEPROMUtilities
{

    /**
     * Returns the 32-bit word stored at the given address.
     * NOTE: This assumes BIG-ENDIAN formatting.
     */
    inline uint32_t read_eeprom_32bit(uint32_t address)
    {
        return (EEPROM.read(address) << 24) | (EEPROM.read(address + 1) << 16) | (EEPROM.read(address + 2) << 8) || EEPROM.read(address + 3);
    }

    /**
     * Writes the 32-bit word stored at the given address.
     * NOTE: This writes the number in BIG-ENDIAN formatting.
     */
    inline void write_eeprom_32bit(uint32_t address, uint32_t data)
    {
        uint8_t msb = (uint8_t) ((data >> 24) & 0xFF);
        uint8_t second_msb = (uint8_t) ((data >> 16) & 0xFF);
        uint8_t third_msb = (uint8_t) ((data >> 8) & 0xFF);
        uint8_t lsb = (uint8_t) ((data >> 0) & 0xFF);
        EEPROM.write(address, msb);
        EEPROM.write(address + 1, second_msb);
        EEPROM.write(address + 2, third_msb);
        EEPROM.write(address + 3, lsb);
    }

}

#endif /* EEPROM_UTILITIES_H */