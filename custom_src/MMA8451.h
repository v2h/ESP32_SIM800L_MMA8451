// Forked from the Adafruit MMA8451 library

#ifndef _MMA8451_H_
#define _MMA8451_H_

#include <Arduino.h>
#include <Wire.h>

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
#define MMA8451_DEFAULT_ADDRESS (0x1D) // if A is GND, its 0x1C
/*=========================================================================*/

#define MMA8451_REG_OUT_X_MSB     0x01
#define MMA8451_REG_SYSMOD        0x0B
#define MMA8451_REG_WHOAMI        0x0D
#define MMA8451_REG_XYZ_DATA_CFG  0x0E
#define MMA8451_REG_PL_STATUS     0x10
#define MMA8451_REG_PL_CFG        0x11
#define MMA8451_REG_CTRL_REG1     0x2A
#define MMA8451_REG_CTRL_REG2     0x2B
#define MMA8451_REG_CTRL_REG3     0x2C
#define MMA8451_REG_CTRL_REG4     0x2D
#define MMA8451_REG_CTRL_REG5     0x2E
#define MMA8451_REG_FF_MT_CFG     0x15
#define MMA8451_REG_FF_MT_SRC     0x16
#define MMA8451_REG_FF_MT_THS     0x17
#define MMA8451_REG_FF_MT_COUNT   0x18
#define MMA8451_REG_INT_SOURCE    0x0C

#define MMA8451_PL_PUF 0
#define MMA8451_PL_PUB 1
#define MMA8451_PL_PDF 2
#define MMA8451_PL_PDB 3
#define MMA8451_PL_LRF 4
#define MMA8451_PL_LRB 5
#define MMA8451_PL_LLF 6
#define MMA8451_PL_LLB 7

typedef enum
{
  MMA8451_RANGE_8_G = 0b10, // +/- 8g
  MMA8451_RANGE_4_G = 0b01, // +/- 4g
  MMA8451_RANGE_2_G = 0b00  // +/- 2g (default value)
} mma8451_range_t;

/* Used with register 0x2A (MMA8451_REG_CTRL_REG1) to set bandwidth */
typedef enum
{
  MMA8451_DATARATE_800_HZ = 0b000,  //  800Hz
  MMA8451_DATARATE_400_HZ = 0b001,  //  400Hz
  MMA8451_DATARATE_200_HZ = 0b010,  //  200Hz
  MMA8451_DATARATE_100_HZ = 0b011,  //  100Hz
  MMA8451_DATARATE_50_HZ = 0b100,   //   50Hz
  MMA8451_DATARATE_12_5_HZ = 0b101, // 12.5Hz
  MMA8451_DATARATE_6_25HZ = 0b110,  // 6.25Hz
  MMA8451_DATARATE_1_56_HZ = 0b111, // 1.56Hz

  MMA8451_DATARATE_MASK = 0b111
} mma8451_dataRate_t;

typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;
} mma8451_data_t;

typedef struct {
  uint8_t i2cAddress;
  mma8451_data_t data;
  // add more properties if need be
} mma8451_t;

// **********************************************************
// **********************************************************

bool MMA8451_begin(mma8451_t *const sensor);
void MMA8451_reset(mma8451_t *const sensor);

void MMA8451_readData(mma8451_t *const sensor);

bool MMA8451_setRange(mma8451_t *const sensor, mma8451_dataRate_t range);
mma8451_range_t MMA8451_getRange(mma8451_t *const sensor);

bool MMA8451_setDataRate(mma8451_t *const sensor, mma8451_dataRate_t rate);
mma8451_dataRate_t MMA8451_getDataRate(mma8451_t *const sensor);

void MMA8451_setInterruptThreshold(mma8451_t *const sensor, uint16_t thresshold_mg);
void MMA8451_setInterruptDuration(mma8451_t *const sensor, uint16_t duration_ms);

void MMA8451_enableInterrupt(mma8451_t *const sensor, uint16_t thresshold_mg, uint16_t duration_ms, bool activeHigh);
void MMA8451_disableInterrupt(mma8451_t *const sensor);
void MMA8451_clearInterrupt(mma8451_t *const sensor);

void MMA8451_clearInterrupt(mma8451_t *const sensor);

uint8_t MMA8451_getInterruptSource(mma8451_t *const sensor);
uint8_t MMA8451_getMotionSource(mma8451_t *const sensor);

#endif // _MMA8451_H_