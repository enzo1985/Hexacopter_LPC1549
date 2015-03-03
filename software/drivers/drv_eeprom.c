#include "drv_eeprom.h"
#include "drv_iap.h"

/* Write data to EEPROM */
uint8_t EEPROM_Write(uint32_t address, uint8_t *data, uint32_t byteswrt)
{
    uint32_t command[5], result[4];

    command[0] = IAP_EEPROM_WRITE;
    command[1] = address;
    command[2] = (uint32_t) data;
    command[3] = byteswrt;
    command[4] = SystemCoreClock / 1000;
    iap_entry(command, result);

    return result[0];
}

/* Read data from EEPROM */
uint8_t EEPROM_Read(uint32_t address, uint8_t *data, uint32_t bytesrd)
{
    uint32_t command[5], result[4];

    command[0] = IAP_EEPROM_READ;
    command[1] = address;
    command[2] = (uint32_t) data;
    command[3] = bytesrd;
    command[4] = SystemCoreClock / 1000;
    iap_entry(command, result);

    return result[0];
}
