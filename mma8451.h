#ifndef MMA8451_H
#define MMA8451_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h> // Temp
#include <esp_err.h>
#include <stdbool.h>

#define MMA8451_DEFAULT_ADDRESS_A0_LOW (0x1C)
#define MMA8451_DEFAULT_ADDRESS_A0_HIGH (0x1D)

#define TRANS_THS_MAX_COUNT    127
#define TRANS_THS_MAX_mG       8000
#define TRANS_THS_mG_per_COUNT 63
#define SCALE_FACTOR_2G_MODE   4096   // (counts per G)
#define SCALE_FACTOR_4G_MODE   2048   // (counts per G)
#define SCALE_FACTOR_8G_MODE   1024   // (counts per G)

typedef union {
    int16_t v[3];
    struct {
        int16_t xi;
        int16_t yi;
        int16_t zi;
    };
} MMA8451_Data_t;

typedef enum {
    MMA8451_RANGE_DEFAULT = 0,
    MMA8451_RANGE_2G = 0,
    MMA8451_RANGE_4G = 1,
    MMA8451_RANGE_8G = 2,
    MMA8451_RANGE_MAX
} MMA8451_Range_t;

// default is 800Hz
typedef enum {
    MMA8451_DATA_RATE_DEFAULT = 0,
    MMA8451_DATA_RATE_800Hz   = 0,
    MMA8451_DATA_RATE_400Hz   = (1 << 3),
    MMA8451_DATA_RATE_200Hz   = (2 << 3),
    MMA8451_DATA_RATE_100Hz   = (3 << 3),
    MMA8451_DATA_RATE_50Hz    = (4 << 3),
    MMA8451_DATA_RATE_12p5Hz  = (5 << 3),
    MMA8451_DATA_RATE_6p25Hz  = (6 << 3),
    MMA8451_DATA_RATE_1p56Hz  = (7 << 3)
} MMA8451_OutputDataRate_t;

typedef enum {
    MMA8451_HPF_CUTOFF_16Hz_DEFAULT   = 0,
    MMA8451_HPF_CUTOFF_16Hz_ODR_800Hz = 0,
    MMA8451_HPF_CUTOFF_8Hz_ODR_800Hz  = 1,
    MMA8451_HPF_CUTOFF_4Hz_ODR_800Hz  = 2,
    MMA8451_HPF_CUTOFF_2Hz_ODR_800Hz  = 3
} MM8451_HpfCutoff_t;

typedef enum {
    MMA8451_OVERSAMPLING_NORMAL = 0,
    MMA8451_OVERSAMPLING_LOWNOISE_LOWPOWER = 1,
    MMA8451_OVERSAMPLING_HIGHRES = 2,
    MMA8451_OVERSAMPLING_LOWPOWER = 3
} MMA8451_Oversampling_t;

typedef enum {
    MMA8451_LOWNOISE_OFF = 0,
    MMA8451_LOWNOISE_ON  = (1 << 2)
} MMA8451_LowNoise_t;

typedef struct {
    MMA8451_Range_t          range;
    MMA8451_OutputDataRate_t odr;
    MM8451_HpfCutoff_t       hpfCutoff;
    uint8_t                  transientThresholdCount;
    uint8_t                  transientDebouncCount;
    MMA8451_Oversampling_t   overSamplingMode;
    MMA8451_LowNoise_t       lowNoiseMode;
} MMA8451_Params_t;

typedef struct {
    uint8_t scl;
    uint8_t sda;
    uint8_t i2cAddress;
    MMA8451_Params_t params;
} MMA8451_t;


extern esp_err_t MMA8451_init(MMA8451_t * const mma, uint8_t const scl, uint8_t const sda, uint8_t i2cAddress);
extern esp_err_t MMA8451_standby(MMA8451_t * const mma);
extern esp_err_t MMA8451_active(MMA8451_t * const mma);
extern esp_err_t MMA8451_softwareReset(MMA8451_t * const mma);
extern esp_err_t MMA8451_readData(MMA8451_t * const mma, MMA8451_Data_t * data);
extern esp_err_t MMA8451_setOverSamplingMode(MMA8451_t * const mma, MMA8451_Oversampling_t mode);
extern esp_err_t MMA8451_setLowNoiseMode(MMA8451_t * const mma, MMA8451_LowNoise_t mode);
extern esp_err_t MMA8451_setRange(MMA8451_t * const mma, MMA8451_Range_t range);
extern esp_err_t MMA8451_setOutputDataRate(MMA8451_t * const mma, MMA8451_OutputDataRate_t odr);
extern esp_err_t MMA8451_setHpfCutOff(MMA8451_t * const mma, MM8451_HpfCutoff_t cutoff);
extern esp_err_t MMA8451_readTransientSource(MMA8451_t * const mma, uint8_t *buffer);
extern esp_err_t MMA8451_setTransientThresholdCounts(MMA8451_t * const mma, uint8_t threshold);
extern esp_err_t MMA8451_setTransientThreshold_mG(MMA8451_t * const mma, uint16_t threshold_mG);
extern esp_err_t MMA8451_setTransientDebounceCounter(MMA8451_t * const mma, uint8_t value);
extern esp_err_t MMA8451_enableTransientDetection(MMA8451_t * const mma, bool byPassHPF);
extern esp_err_t MMA8451_enableTransientInterrupt(MMA8451_t * const mma, bool activeHigh, bool useInt1);

#endif // MMA8451_H
#ifdef __cplusplus
}
#endif
