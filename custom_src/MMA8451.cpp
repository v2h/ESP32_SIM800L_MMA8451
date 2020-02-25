
// // Forked from the Adafruit MMA8451 library

#include "custom_src/MMA8451.h"

//********************************************************************
#define THRESHOLD_RESOLUTION_MG (63)
// *******************************************************************
static inline uint8_t MMA8451_i2cread(void) {
  return Wire.read();
}

static inline void MMA8451_i2cWrite(uint8_t x) {
  Wire.write(x);
}

static void MMA8451_writeReg8(mma8451_t * const sensor, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(sensor->i2cAddress);
  MMA8451_i2cWrite((uint8_t)reg);
  MMA8451_i2cWrite((uint8_t)(value));
  Wire.endTransmission();
}

/*static*/ uint8_t MMA8451_readReg8(mma8451_t * const sensor, uint8_t reg) {
    Wire.beginTransmission(sensor->i2cAddress);
    MMA8451_i2cWrite(reg);
    Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!
    Wire.requestFrom(sensor->i2cAddress, (uint8_t)1);

    if (! Wire.available()) return -1;
    return (MMA8451_i2cread());
} 
// *******************************************************************

void MMA8451_reset(mma8451_t *const sensor) {
  MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG2, 0x40);
}

bool MMA8451_begin(mma8451_t *const sensor) {
  Wire.begin();
  uint8_t deviceID = MMA8451_readReg8(sensor, MMA8451_REG_WHOAMI);
  if (deviceID != 0x1A) {
    return false;
  }
  sensor->_isActivated = false;
  MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG1, 0); // standby mode
  MMA8451_writeReg8(sensor, MMA8451_REG_XYZ_DATA_CFG, MMA8451_RANGE_4_G); //range
  MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG2, 0x02); // hi res
  return true;
}

void MMA8451_readData(mma8451_t *const sensor) {
  Wire.beginTransmission(sensor->i2cAddress);
  MMA8451_i2cWrite(MMA8451_REG_OUT_X_MSB);
  Wire.endTransmission(false); // MMA8451 + friends uses repeated start!!

  Wire.requestFrom(sensor->i2cAddress, (uint8_t)6);
  sensor->data.x = Wire.read(); 
  sensor->data.x <<= 8; sensor->data.x |= Wire.read(); sensor->data.x >>= 2;
  sensor->data.y = Wire.read(); 
  sensor->data.y <<= 8; sensor->data.y |= Wire.read(); sensor->data.y >>= 2;
  sensor->data.z = Wire.read(); 
  sensor->data.z <<= 8; sensor->data.z |= Wire.read(); sensor->data.z >>= 2;
}

void MMA8451_setInterruptThreshold(mma8451_t *const sensor, uint16_t thresshold_mg) {
  MMA8451_writeReg8(sensor, MMA8451_REG_FF_MT_THS, (uint8_t)(thresshold_mg/THRESHOLD_RESOLUTION_MG) & 0x7F);
  Serial.print("set MMA8451_REG_FF_MT_THS: "); Serial.println(MMA8451_readReg8(sensor, MMA8451_REG_FF_MT_THS));
}

void MMA8451_setInterruptDuration(mma8451_t *const sensor, uint16_t duration_ms) {
  MMA8451_writeReg8(sensor, MMA8451_REG_FF_MT_COUNT, (duration_ms/1.25));
  Serial.print("set MMA8451_REG_FF_MT_COUNT: "); Serial.println(MMA8451_readReg8(sensor, MMA8451_REG_FF_MT_COUNT));
}

void MMA8451_enableInterrupt(mma8451_t *const sensor, uint16_t thresshold_mg, uint16_t duration_ms, bool usePin2,  bool activeHigh) {
  if (sensor->_isActivated) {
    MMA8451_standby(sensor);
  }
  if (activeHigh) {
    MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG3, 0x02);
  }
  MMA8451_setInterruptThreshold(sensor, thresshold_mg);
  MMA8451_setInterruptDuration(sensor, duration_ms);
  MMA8451_writeReg8(sensor, MMA8451_REG_FF_MT_CFG, 0xD8); // Motion interrupt
  MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG4, 0x04); // Freefall/motion interrupt output
  if(!usePin2) {
    MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG5, 0x04); // INT PIN 1
  }
  else {
    MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG5, 0x00); // INT PIN 2
  }
  if (!sensor->_isActivated) {
    MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG1, 0x01 | 0x04); // active, 800Hz, low noise
  }
}

uint8_t MMA8451_getInterruptSource(mma8451_t *const sensor) {
  return MMA8451_readReg8(sensor, MMA8451_REG_INT_SOURCE);
}

uint8_t MMA8451_getMotionSource(mma8451_t *const sensor) {
  return MMA8451_readReg8(sensor, MMA8451_REG_FF_MT_SRC);
}

void MMA8451_activate(mma8451_t *const sensor) {
  MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG1, 0x01 | 0x04); // active, 800Hz, low noise
  sensor->_isActivated = true;
  Serial.println(F("Sensor activated"));
}

void MMA8451_standby(mma8451_t *const sensor) {
  MMA8451_writeReg8(sensor, MMA8451_REG_CTRL_REG1, 0); // Put sensor back in standby mode
  sensor->_isActivated = false;
  Serial.println(F("Sensor in standby"));
}
