/*
 * MMA845XQ test code
 * (C) 2012 Akafugu Corporation
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */

//#include <Wire.h>
#include "my_MMA845XQ.h"

volatile bool g_accelInterruptFlag = false;

// If SA0 on MMA845XQ is connected to GND (as on Akafuino L)
//MMA845XQ accel(0x1C);
// If SA0 on MMA845XQ is connected to VCC or floating (as on Adafruit)
MMA8451Q accel(0x1D);

void IRAM_ATTR accelerometerISR();

#define INT_PIN 15

void setup() {
  Serial.begin(115200);
  Wire.begin();

//  accel.init(RANGE range, RESOLUTION resolution, LOW_NOISE lo_noise, DATA_RATE data_rate, OVERSAMPLE_MODE os_mode, HPF_MODE hpf_mode);

  accel.SWreset(); // RESET THE REGS TO FACTORY DEFAULT FIRST!
//  accel.setCommonParameters(accel.RANGE_2G, accel.RES_MAX, accel.LN_ON, accel.DR_800, accel.OS_NORMAL, accel.HPF_OFF);
  accel.setCommonParameters(accel.RANGE_2G, accel.RES_MAX, accel.LN_OFF, accel.DR_100, accel.OS_NORMAL, accel.HPF_OFF);

  accel.setMotionDetection();
  accel.setMotionThresholdG(0.1, false);
  accel.setDebounceCounter(5);
  
  accel.setInterrupt(accel.INT_EN_FF_MT, accel.INT1, true); // Which event should raise an interrupt, and to which MMA8451Q pin is it routed (pins INT1 and INT2)

  uint8_t mo_src = accel.getMotionSource(); // Read to clear EA flag

  accel.dumpRegisters();

// Block here for testing ...
//  while(1);
  
  pinMode(INT_PIN, INPUT_PULLDOWN);
  attachInterrupt(INT_PIN, accelerometerISR, RISING);
}

void loop() {

static uint8_t mo_src;

/*
mo_src = accel.getMotionSource();
Serial.print("1 mo_src: ");
Serial.println(mo_src,HEX);

mo_src = accel.getMotionSource();
Serial.print("2 mo_src: ");
Serial.println(mo_src,HEX);

delay(1000);
*/

/*
  while (!g_accelInterruptFlag);  
  g_accelInterruptFlag = false;
  mo_src = accel.getMotionSource();
  Serial.print("Motion Detected: ");
  Serial.println(mo_src,HEX);
  attachInterrupt(INT_PIN, accelerometerISR, RISING);
*/
  
/*
  accel.update();
  Serial.print(accel._xi); Serial.print("\t");
  Serial.print(accel._yi); Serial.print("\t");
  Serial.print(accel._zi); Serial.print("\t");
  Serial.print(accel._xf); Serial.print("\t");
  Serial.print(accel._yf); Serial.print("\t");
  Serial.print(accel._zf); Serial.println();

  delay(1000);
*/

  accel.update(); // update x,y,z measurements in accel object
  
  uint8_t res = accel._read_register(accel.INT_SOURCE);
  accel._PrintHex8(&res, 1);  Serial.print("\t");
  
  res = accel._read_register(accel.FF_MT_SRC);
  accel._PrintHex8(&res, 1);  Serial.print("\t");

  // This shows how the reading of FF_MT_SRC resets the event flags in INT_SOURCE.
  res = accel._read_register(accel.INT_SOURCE);
  accel._PrintHex8(&res, 1);  Serial.print("\t");

  // print x,y,z raw accel values
  Serial.print(accel._xi); Serial.print("\t");
  Serial.print(accel._yi); Serial.print("\t");
  Serial.print(accel._zi); Serial.print("\t");
  Serial.print(accel._xf); Serial.print("\t");
  Serial.print(accel._yf); Serial.print("\t");
  Serial.print(accel._zf); Serial.println();

/*
  Serial.print(accel._read_register(accel.INT_SOURCE));
  Serial.print("\t");
  Serial.print(accel._read_register(accel.FF_MT_SRC));
  Serial.print("\t");
  Serial.println(accel._read_register(accel.FF_MT_SRC));
*/
  delay(100);
}


void IRAM_ATTR accelerometerISR() {
//  MMA8451_getMotionSource(&g_sensor);
  detachInterrupt(INT_PIN);
  g_accelInterruptFlag = true;
//  Serial.println(F("\Accel Interrupt"));
}
