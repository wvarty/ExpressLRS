#include "elrs_eeprom.h"

#ifdef PLATFORM_STM32
extEEPROM EEPROM(kbits_2, 1, 1);
#endif

void EepromSetup()
{
#ifdef PLATFORM_STM32
    Wire.setSDA(GPIO_PIN_EEPROM_SDA); // set is needed or it wont work :/
    Wire.setSCL(GPIO_PIN_EEPROM_SCK);
    Wire.begin();
    EEPROM.begin();
#else
    EEPROM.begin(3);
#endif
};

uint8_t readEepromVar(eeprom_properties_e enumval)
{
    switch (enumval)
    {
    case TXBASEMAX3:
        return EEPROM.read(0);
        break;
    case TXBASEMAX4:
        return EEPROM.read(1);
        break;
    case TXBASEMAX5:
        return EEPROM.read(2);
        break;
    case EXAMPLE_VAR_ONE:
        return EEPROM.read(3);
        break;
    case EXAMPLE_VAR_TWO:
        return EEPROM.read(4);
        break;
    default:
        return 0;
    }
}

void writeEepromVar(eeprom_properties_e enumval, uint8_t eepromVar)
{
    switch (enumval)
    {
    case TXBASEMAX3:
        EEPROM.write(0, eepromVar);
        break;
    case TXBASEMAX4:
        EEPROM.write(1, eepromVar);
        break;
    case TXBASEMAX5:
        EEPROM.write(2, eepromVar);
        break;
    case EXAMPLE_VAR_ONE:
        EEPROM.write(3, eepromVar);
        break;
    case EXAMPLE_VAR_TWO:
        EEPROM.write(4, eepromVar);
        break;
    default:
        break;
    }
}