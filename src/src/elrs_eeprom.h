#ifndef ELRS_EEPROM_H
#define ELRS_EEPROM_H

#define RESERVED_EEPROM_SIZE 3

#include <Wire.h>
#include "targets.h"
#ifdef PLATFORM_STM32
#include <extEEPROM.h>
#else
#include <EEPROM.h>
#endif

class ELRS_EEPROM
{
public:
    void Begin();
    uint8_t ReadByte(const uint16_t address);
    void WriteByte(const uint16_t address, const uint8_t value);

    void ReadUniqueID(uint8_t* uid);
    void WriteUniqueID(uint8_t* uid);
};

#endif
