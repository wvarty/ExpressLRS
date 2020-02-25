#include "elrs_eeprom.h"

#ifdef PLATFORM_STM32
extEEPROM EEPROM(kbits_2, 1, 1);
#endif

void
ELRS_EEPROM::Begin()
{
#ifdef PLATFORM_STM32
    Wire.setSDA(GPIO_PIN_EEPROM_SDA); // set is needed or it wont work :/
    Wire.setSCL(GPIO_PIN_EEPROM_SCK);
    Wire.begin();
    EEPROM.begin();
#else
    EEPROM.begin(RESERVED_EEPROM_SIZE);
#endif
}

uint8_t
ELRS_EEPROM::ReadByte(const uint16_t address)
{
    return EEPROM.read(address);
}

void
ELRS_EEPROM::WriteByte(const uint16_t address, const uint8_t value)
{
    EEPROM.write(address, value);
}

void
ELRS_EEPROM::ReadUniqueID(uint8_t* uid)
{
    uint8_t readAddress = 0;

    uid[3] = ReadByte(readAddress);
    readAddress += sizeof(uint8_t);
    uid[4] = ReadByte(readAddress);
    readAddress += sizeof(uint8_t);
    uid[5] = ReadByte(readAddress);
}

void
ELRS_EEPROM::WriteUniqueID(uint8_t* uid)
{
    uint8_t writeAddress = 0;

    WriteByte(writeAddress, uid[3]);
    writeAddress += sizeof(uint8_t);
    WriteByte(writeAddress, uid[4]);
    writeAddress += sizeof(uint8_t);
    WriteByte(writeAddress, uid[5]);
}
