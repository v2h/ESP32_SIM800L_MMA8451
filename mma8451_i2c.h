#ifndef MMA8451_I2C_H
#define MMA8451_I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern esp_err_t MMA8451_I2C_init(uint8_t const scl, uint8_t const sda);
extern esp_err_t MMA8451_I2C_readReg8(uint8_t const i2cAddress, uint8_t regAddr, uint8_t *buffer, uint8_t regCount);
extern esp_err_t MMA8451_I2C_writeReg8(uint8_t const i2cAddress, uint8_t regAddr, uint8_t *data, uint8_t regCount);

#endif // MMA8451_I2C_H
#ifdef __cplusplus
}
#endif
