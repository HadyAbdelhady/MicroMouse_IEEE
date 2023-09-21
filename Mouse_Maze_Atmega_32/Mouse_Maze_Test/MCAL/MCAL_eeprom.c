#include "MCAL_eeprom.h"

/******************** Create Functions ********************/
void EEPROM_write(uint16_t address, uint8_t data)
{
    // Wait for completion of previous write
    while (EEPROM->EECR & (1 << EEPROM_EEWE))
        ;

    // Set up address and data registers
    EEPROM->EEARL = address;
    EEPROM->EEDR = data;

    // Write logical one to EEMWE
    EEPROM->EECR |= (1 << EEPROM_EEMWE);

    // Start eeprom write by setting EEWE
    EEPROM->EECR |= (1 << EEPROM_EEWE);
}

void EEPROM_erase(uint16_t start_byte, uint16_t end_byte)
{
    for (; start_byte < end_byte; start_byte++)
    {
        EEPROM_write(start_byte, 0xFF); // Write 0xFF to erase each byte
    }
}

void EEPROM_write_nbyte(uint8_t *str, uint16_t start_byte, uint16_t nbyte)
{
    for (uint16_t i = 0; i < nbyte; i++)
    {
        EEPROM_write(start_byte + i, *str); /* Write each byte to EEPROM */
        str++;                              // Select Next Byte
    }
}

uint8_t EEPROM_read(uint8_t address)
{
    // Wait for completion of previous write
    while (EEPROM->EECR & (1 << EEPROM_EEWE))
        ;

    // Set up address register
    EEPROM->EEARL = address;

    // Start eeprom read by setting EERE
    EEPROM->EECR |= (1 << EEPROM_EERE);

    // Return data from data register
    return EEPROM->EEDR;
}

void EEPROM_read_nbyte(uint8_t *str, uint16_t start_byte, uint16_t nbyte)
{
    for (uint16_t i = 0; i < nbyte; i++)
    {
        *str = EEPROM_read(start_byte + i); /* Read each byte from EEPROM */
        str++;                              // Select Next Byte
    }
}
void EEPROM_write_float(uint16_t address, float value)
{
    int16_t scaledValue = (int16_t)(value * FLOAT_SCALE_FACTOR);

    if (scaledValue >= -32768 && scaledValue <= 32767)
    {
        // Store in one byte
        EEPROM_write(address, (int8_t)scaledValue);
    }
    else
    {
        // Store in two bytes
        for (int i = 0; i < 2; i++)
        {
            EEPROM_write(address + i, (int8_t)(scaledValue >> (i * 8)));
        }
    }
}

float EEPROM_read_float(uint16_t address)
{
    int16_t scaledValue = 0;

    if (EEPROM_read(address + 1) != -1) // Check if second byte is readable
    {
        // Two-byte value
        for (int i = 0; i < 2; i++)
        {
            int8_t byteValue = EEPROM_read(address + i);
            scaledValue |= ((int16_t)byteValue << (i * 8));
        }
    }
    else
    {
        // One-byte value
        scaledValue = EEPROM_read(address);
    }

    return ((float)scaledValue) / FLOAT_SCALE_FACTOR;
}
