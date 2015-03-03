#ifndef __DRV_EEPROM_H_
#define __DRV_EEPROM_H_
#include <lpc15xx.h>
#ifdef __cplusplus
extern "C" {
#endif



    /**
     * @brief   Write data to EEPROM
     * @param   address     : EEPROM address to be written to
     * @param   ptr         : Pointer to buffer to write from
     * @param   len : Number of bytes to write to EEPROM
     * @return  An IAP response definition from drv_iap.h
     */
    uint8_t EEPROM_Write(uint32_t address, uint8_t *data, uint32_t len);

    /**
     * @brief   Read data from EEPROM
     * @param   address : EEPROM address to be read from
     * @param   data        : Pointer to buffer to read to
     * @param   len : Number of bytes to read from EEPROM
     * @return  An IAP response definition from drv_iap.h
     */
    uint8_t EEPROM_Read(uint32_t address, uint8_t *data, uint32_t len);



#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_H_ */
