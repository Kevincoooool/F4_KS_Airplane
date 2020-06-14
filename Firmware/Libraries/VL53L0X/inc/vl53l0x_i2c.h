#ifndef _VL53L0X_I2C_PLATFORM_H_
#define _VL53L0X_I2C_PLATFORM_H_

#include "Drv_i2c2_soft.h"

#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

#define    BYTES_PER_WORD        2
#define    BYTES_PER_DWORD       4
                                    
int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t  *pdata, int32_t count);
int32_t VL53L0X_read_multi(uint8_t address,  uint8_t index, uint8_t  *pdata, int32_t count);
int32_t VL53L0X_write_byte(uint8_t address,  uint8_t index, uint8_t   data);
int32_t VL53L0X_write_word(uint8_t address,  uint8_t index, uint16_t  data);
int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t  data);
int32_t VL53L0X_read_byte(uint8_t address,  uint8_t index, uint8_t  *pdata);
int32_t VL53L0X_read_word(uint8_t address,  uint8_t index, uint16_t *pdata);
int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata);

#endif //_VL53L0X_I2C_PLATFORM_H_

