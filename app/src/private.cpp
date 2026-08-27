#include "private.hpp"

const uint32_t BaudValue[] = {
    110, 300, 600, 1200, 2400, 4800, 9600,
    19200, 38400, 57600, 115200, 128000, 230400, 256000,
    460800, 500000, 512000, 600000, 750000, 921600,
    1000000, 1500000, 2000000
};

const uint32_t BaudValue_Size = sizeof(BaudValue) / sizeof(BaudValue[0]);

void Save_BaudValue(uint32_t baud, uint8_t interface)
{
    uint32_t addr = 0;
    switch (interface) {
    case 0: addr = EEPROM_ADDR_BAUD_VAL0; break;
    case 1: addr = EEPROM_ADDR_BAUD_VAL1; break;
    case 2: addr = EEPROM_ADDR_BAUD_VAL2; break;
    default: return;
    }
    uint8_t *p = (uint8_t *)&baud;
    for (int i = 0; i < 4; i++) ee_Write_Byte(addr + i, p[i]);
}

uint32_t Load_BaudValue(uint8_t interface)
{
    uint32_t addr = 0;
    switch (interface) {
    case 0: addr = EEPROM_ADDR_BAUD_VAL0; break;
    case 1: addr = EEPROM_ADDR_BAUD_VAL1; break;
    case 2: addr = EEPROM_ADDR_BAUD_VAL2; break;
    default: return 115200;
    }
    uint32_t baud = 0;
    uint8_t *p = (uint8_t *)&baud;
    for (int i = 0; i < 4; i++) p[i] = ee_Read_Byte(addr + i);

    for (size_t i = 0; i < BaudValue_Size; i++)
        if (baud == BaudValue[i]) return baud;
    return 115200;
}

void config_baudrate(uint32_t baud, uint8_t interface)
{
    Save_BaudValue(baud, interface);
}
