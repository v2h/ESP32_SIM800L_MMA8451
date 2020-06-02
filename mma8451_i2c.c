#include <esp_err.h>
#include <assert.h>
#include <driver/i2c.h>
#include <driver/gpio.h>
#include "mma8451_i2c.h"

// I2C common protocol defines
#define ACK_CHECK_EN  0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0              /*!< I2C master will not check ack from slave */

esp_err_t MMA8451_I2C_init(uint8_t const scl, uint8_t const sda) {
    if (scl > GPIO_NUM_MAX || sda > GPIO_NUM_MAX) return ESP_FAIL;
    i2c_config_t i2cConfig = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = sda,
        .scl_io_num       = scl,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(I2C_NUM_0, &i2cConfig);
    esp_err_t ret = i2c_driver_install(I2C_NUM_0, i2cConfig.mode, 0, 0, 0);
    return ret;
}

esp_err_t MMA8451_I2C_readReg8(uint8_t const i2cAddress, uint8_t regAddr, uint8_t *buffer, uint8_t regCount) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2cAddress << 1, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_start(cmd); // Repeated start
    i2c_master_write_byte(cmd, i2cAddress << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
    if (regCount > 1) {
        i2c_master_read(cmd, buffer, regCount - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + regCount - 1, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t MMA8451_I2C_writeReg8(uint8_t const i2cAddress, uint8_t regAddr, uint8_t *data, uint8_t regCount) {
    if (0 == regCount) {
        return ESP_FAIL;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2cAddress << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, regAddr, ACK_CHECK_EN);
    i2c_master_write(cmd, data, regCount, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}
