#include "mma8451.h"
#include "mma8451_registers.h"
#include "mma8451_i2c.h"

// !TODO: HPF_OUT

static esp_err_t MMA8451_setBits_(MMA8451_t * const mma, uint8_t regAddr, uint8_t mask, uint8_t value);

esp_err_t MMA8451_init(MMA8451_t * const mma, uint8_t const scl, uint8_t const sda, uint8_t i2cAddress) {
    esp_err_t ret = MMA8451_I2C_init(scl, sda);
    if (ESP_OK == ret) {
        mma->scl = scl;
        mma->sda = sda;
        mma->i2cAddress = i2cAddress;
        mma->params.overSamplingMode        = MMA8451_OVERSAMPLING_NORMAL;
        mma->params.hpfCutoff               = MMA8451_HPF_CUTOFF_16Hz_DEFAULT;
        mma->params.odr                     = MMA8451_DATA_RATE_DEFAULT;
        mma->params.transientDebouncCount   = 0;
        mma->params.transientThresholdCount = 0;
    }
    return ret;
}

esp_err_t MMA8451_standby(MMA8451_t * const mma) {
    uint8_t regVal;
    esp_err_t ret;
    ret = MMA8451_I2C_readReg8(mma->i2cAddress, MMA8451_REG_CTRL_REG1, &regVal, 1);
    if (ESP_OK != ret) return ret;
    regVal &= ~ACTIVE_MASK; // same as reg._ACTIVE_BIT = 0 with reg of type Reg8_t
    ret = MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_CTRL_REG1, &regVal, 1);
    return ret;
}

esp_err_t MMA8451_active(MMA8451_t * const mma) {
    uint8_t regVal;
    esp_err_t ret;
    ret = MMA8451_I2C_readReg8(mma->i2cAddress, MMA8451_REG_CTRL_REG1, &regVal, 1);
    if (ESP_OK != ret) return ret;
    regVal |= ACTIVE_MASK;
    ret = MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_CTRL_REG1, &regVal, 1);
    return ret;
}

esp_err_t MMA8451_softwareReset(MMA8451_t * const mma) {
    uint8_t regVal = ST_MASK;
    return MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_CTRL_REG2, &regVal, 1);
}

esp_err_t MMA8451_readData(MMA8451_t * const mma, MMA8451_Data_t * data) {
    // x_MSB - x_LSB - y_MSB - y_LSB - z_MSB - z_LSB
    // 14-bit data format, 2 LSBs are left out, thus shift right by 2
    esp_err_t ret;
    uint8_t buffer[6];
    ret = MMA8451_I2C_readReg8(mma->i2cAddress, MMA8451_REG_OUT_X_MSB, buffer, sizeof(*data));
    data->xi = (int16_t)(buffer[0] << 8 | buffer[1]);
    data->yi = (int16_t)(buffer[2] << 8 | buffer[2]);
    data->zi = (int16_t)(buffer[4] << 8 | buffer[5]);
    data->xi >>= 2;
    data->yi >>= 2;
    data->zi >>= 2;
    return ret;
}

esp_err_t MMA8451_setOverSamplingMode(MMA8451_t * const mma, MMA8451_Oversampling_t mode) {
    esp_err_t err = MMA8451_setBits_(mma, MMA8451_REG_CTRL_REG2, MODS_MASK, mode);
    if (ESP_OK == err)
        mma->params.overSamplingMode = mode;
    return err;
}

esp_err_t MMA8451_setLowNoiseMode(MMA8451_t * const mma, MMA8451_LowNoise_t mode) {
    esp_err_t err = MMA8451_setBits_(mma, MMA8451_REG_CTRL_REG1, LNOISE_MASK, mode);
    if (ESP_OK == err)
        mma->params.lowNoiseMode = mode;
    return err;
}

// Caution: range is limited to max. +/- 4G when LNOISE = 1
esp_err_t MMA8451_setRange(MMA8451_t * const mma, MMA8451_Range_t range) {
    if (MMA8451_RANGE_8G == range && MMA8451_LOWNOISE_ON == mma->params.lowNoiseMode)
        range = MMA8451_RANGE_4G; 
    esp_err_t err = MMA8451_setBits_(mma, MMA8451_REG_XYZ_DATA_CFG, FS_MASK, range);
    if (ESP_OK == err)
        mma->params.range = range;
    return err;
}

// Default is 800 Hz
esp_err_t MMA8451_setOutputDataRate(MMA8451_t * const mma, MMA8451_OutputDataRate_t odr) {
    esp_err_t err =  MMA8451_setBits_(mma, MMA8451_REG_CTRL_REG1, DR_MASK, odr);
    if (ESP_OK == err)
        mma->params.odr = odr;
    return err;
}

// mask = 00, 01, 10, 11; see table 23 in the datasheet
esp_err_t MMA8451_setHpfCutOff(MMA8451_t * const mma, MM8451_HpfCutoff_t cutoff) {
    if (cutoff > SEL_MASK) return ESP_FAIL;
    esp_err_t err = MMA8451_setBits_(mma, MMA8451_REG_HP_FILTER_CUTOFF, SEL_MASK, cutoff);
    if (ESP_OK == err)
        mma->params.hpfCutoff = cutoff;
    return err;
}

// This clears the transient interrupt output (from the sensor's side)
esp_err_t MMA8451_readTransientSource(MMA8451_t * const mma, uint8_t *buffer) {
    return MMA8451_I2C_readReg8(mma->i2cAddress, MMA8451_REG_TRANSIENT_SRC, buffer, 1);
}

esp_err_t MMA8451_setTransientThresholdCounts(MMA8451_t * const mma, uint8_t threshold) {
    if (threshold > TRANS_THS_MAX_COUNT) threshold = TRANS_THS_MAX_COUNT;
    esp_err_t err = MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_TRANSIENT_THS, &threshold, 1);
    if (ESP_OK == err)
        mma->params.transientThresholdCount = threshold;
    return err;
}

esp_err_t MMA8451_setTransientThreshold_mG(MMA8451_t * const mma, uint16_t threshold_mG) {
    if (threshold_mG > TRANS_THS_MAX_mG) threshold_mG = TRANS_THS_MAX_mG;
    uint8_t thresholdCount = (uint8_t)threshold_mG / TRANS_THS_mG_per_COUNT;
    esp_err_t err = MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_TRANSIENT_THS, &thresholdCount, 1);
    if (ESP_OK == err)
        mma->params.transientThresholdCount = thresholdCount;
    return err;
}

// 
esp_err_t MMA8451_setTransientDebounceCounter(MMA8451_t * const mma, uint8_t value) {
    esp_err_t err = MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_TRANSIENT_COUNT, &value, 1);
    if (ESP_OK == err)
        mma->params.transientDebouncCount = value;
    return err;
}

// When the high-pass filter is bypassed, the function behaves similar to the motion detection..
// ..so DONT BYPASS IT!
esp_err_t MMA8451_enableTransientDetection(MMA8451_t * const mma, bool byPassHPF) {
    uint8_t regValue = (byPassHPF) ? 
                        (ELE_MASK | ZTEFE_MASK | YTEFE_MASK | XTEFE_MASK | HPF_BYP_MASK) :
                        (ELE_MASK | ZTEFE_MASK | YTEFE_MASK | XTEFE_MASK);
    return MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_TRANSIENT_CFG, &regValue, 1);
}

// Interrupts are routed to INT2 pin by default (0)
// Interrupt polarity is active-low by default (0)
esp_err_t MMA8451_enableTransientInterrupt(MMA8451_t * const mma, bool activeHigh, bool useInt1) {
    esp_err_t ret;
    uint8_t regVal;
    // Set polarity
    if (activeHigh) {
        ret = MMA8451_setBits_(mma, MMA8451_REG_CTRL_REG3, IPOL_MASK, IPOL_MASK);
        if (ESP_OK != ret) return ret;
    }

    // Enable transient interrupt
    ret = MMA8451_setBits_(mma, MMA8451_REG_CTRL_REG4, INT_EN_TRANS_MASK, INT_EN_TRANS_MASK);
    if (ESP_OK != ret) return ret;
    // Map interrupt to output pin
    ret = MMA8451_I2C_readReg8(mma->i2cAddress, MMA8451_REG_CTRL_REG5, &regVal, 1);
    if (ESP_OK != ret) return ret;
    regVal = (useInt1) ? (regVal | INT_CFG_TRANS_MASK) : (regVal & (~INT_EN_TRANS_MASK));
    ret = MMA8451_I2C_writeReg8(mma->i2cAddress, MMA8451_REG_CTRL_REG5, &regVal, 1);
    return ret;
}

static esp_err_t MMA8451_setBits_(MMA8451_t * const mma, uint8_t regAddr, uint8_t mask, uint8_t value) {
    esp_err_t ret;
    uint8_t regVal;
    ret = MMA8451_I2C_readReg8(mma->i2cAddress, regAddr, &regVal, 1);
    if (ESP_OK != ret) return ret;
    regVal &= ~mask;
    regVal |= value;
    ret = MMA8451_I2C_writeReg8(mma->i2cAddress, regAddr, &regVal, 1);
    return ret;
}
