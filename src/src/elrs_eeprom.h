#ifndef ELRS_EEPROM_H
#define ELRS_EEPROM_H

#include <Wire.h>
#include "targets.h"
#ifdef PLATFORM_STM32
#include <extEEPROM.h>
#else
#include <EEPROM.h>
#endif

typedef enum
{
    TXBASEMAX3,
    TXBASEMAX4,
    TXBASEMAX5,
    EXAMPLE_VAR_ONE,
    EXAMPLE_VAR_TWO
} eeprom_properties_e;

void EepromSetup();
uint8_t readEepromVar(eeprom_properties_e enumval);
void writeEepromVar(eeprom_properties_e enumval, uint8_t eepromVar);

#endif