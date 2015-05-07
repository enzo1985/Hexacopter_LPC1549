#include "drv_eeprom.h"
#include "drv_iap.h"
#include "rtthread.h"

/* Write data to EEPROM */
uint8_t EEPROM_Write(uint32_t address, uint8_t *data, uint32_t len)
{
    uint32_t command[5], result[4];

    command[0] = IAP_EEPROM_WRITE;
    command[1] = address;
    command[2] = (uint32_t) data;
    command[3] = len;
    command[4] = SystemCoreClock / 1000;
    rt_enter_critical();
    iap_entry(command, result);
    rt_exit_critical();

    return result[0];
}

/* Read data from EEPROM */
uint8_t EEPROM_Read(uint32_t address, uint8_t *data, uint32_t len)
{
    uint32_t command[5], result[4];

    command[0] = IAP_EEPROM_READ;
    command[1] = address;
    command[2] = (uint32_t) data;
    command[3] = len;
    command[4] = SystemCoreClock / 1000;
    rt_enter_critical();
    iap_entry(command, result);
    rt_exit_critical();

    return result[0];
}
