/*
 * MMA845XQ library
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

#include "MMA845XQ_Vibe.h"

MMA8451Q::MMA8451Q(uint8_t addr)
{
  _addr = addr;
  _rad2deg = 180.0 / M_PI;
}


const char * const MMA8451Q::regNames[] = {
/*  0 0x00 */  "STATUS", "OUT_X_MSB", "OUT_X_LSB", "OUT_Y_MSB", "OUT_Y_LSB", "OUT_Z_MSB", "OUT_Z_LSB", "Reserved",
/*  8 0x08 */  "Reserved", "F_SETUP", "TRIG_CFG", "SYSMOD", "INT_SOURCE", "WHO_AM_I", "XYZ_DATA_CFG", "HP_FILTER_CUTOFF",
/* 16 0x10 */  "PL_STATUS", "PL_CFG", "PL_COUNT", "PL_BF_ZCOMP", "P_L_THS_REG", "FF_MT_CFG", "FF_MT_SRC", "FF_MT_THS",
/* 24 0x18 */  "FF_MT_COUNT", "Reserved", "Reserved", "Reserved", "Reserved", "TRANSIENT_CFG", "TRANSIENT_SCR", "TRANSIENT_THS",
/* 32 0x20 */  "TRANSIENT_COUNT", "PULSE_CFG", "PULSE_SRC", "PULSE_THSX", "PULSE_THSY", "PULSE_THSZ", "PULSE_TMLT", "PULSE_LTCY",
/* 40 0x28 */  "PULSE_WIND", "ASLP_COUNT", "CTRL_REG1", "CTRL_REG2", "CTRL_REG3", "CTRL_REG4", "CTRL_REG5", "OFF_X",
/* 48 0x29 */  "OFF_Y", "OFF_Z"
};


uint8_t MMA8451Q::SWreset()
{
  _write_register(RST, CTRL_REG2);
  delay(10);
}


uint8_t MMA8451Q::setCommonParameters(RANGE range, RESOLUTION resolution, LOW_NOISE lo_noise, DATA_RATE data_rate, OVERSAMPLE_MODE os_mode, HPF_MODE hpf_mode)
{
  _highres = (resolution == RES_MAX); //Important! It is used in update(), i.e. data reading and printing

  /*_scale = range; // Purpose unclear.
  _step_factor = (_highres ? 0.0039 : 0.0156); // Base value at 2g setting
  if( range == RANGE_4G )
    _step_factor *= 2;
  else if (range == RANGE_8G )
    _step_factor *= 4;*/
  switch (range) {
    case RANGE_2G:
      _step_factor = 1/SCALE_RANGE_2G;
      break;
    case RANGE_4G:
      _step_factor = 1/SCALE_RANGE_4G;
      break;
    case RANGE_8G:
      _step_factor = 1/SCALE_RANGE_8G;
      break;
  }

  _who_am_i = _read_register(WHO_AM_I); // Get Who Am I from the device.
  // return value for MMA8541Q is 0x1A

  _standby();

  uint8_t result = 0;
  uint8_t datain = 0;
  uint8_t dataout = 0;

  datain = _read_register(SYSMOD); // Make sure MMA845x is in Stand-By mode
  if ((datain & 0x03) != 0 ) {
      Serial.print("MMA845x not in STAND BY mode\n\f");
      Serial.print("MMA845x:init failed\n\r");
      result = 1;
      return result;
  }

  // CTRL_REG1
  datain = _read_register(CTRL_REG1);
  dataout = (datain & 0xC1) | resolution | lo_noise | data_rate; // 0b ‭0010 1010
  _write_register(dataout, CTRL_REG1);        // Set resolution, Low Noise mode, and data rate

  // CTRL_REG2
  datain = _read_register(CTRL_REG2);
  dataout = (datain & 0xFC) | os_mode;
  _write_register(dataout, CTRL_REG2);        // Set Oversample mode for Active State

  // XYZ_DATA_CFG
  datain = _read_register(XYZ_DATA_CFG); // Not used. register consists of range and hpf_mode only.
  dataout = range | hpf_mode;
  _write_register(dataout, XYZ_DATA_CFG);     //Set HPF mode and range

  //result |= MMA845x::readRegister(HP_FILTER_CUTOFF,1, datain);
  //result |= MMA845x::writeRegister(HP_FILTER_CUTOFF, dataout); //REG 0xF HPF settings

  if(result != 0) {
      Serial.print("MMA845x:setParameters failed\n\r");
  }

  _active();

  return result ;
}

uint8_t MMA8451Q::get_CTRL_REG1()
{
  return _read_register(CTRL_REG1);
}

//////////////////////////////////////////////////////////
uint8_t MMA8451Q::getPLStatus()
{
	return _read_register(PL_STATUS);
}

//////////////////////////////////////////////////////////
uint8_t MMA8451Q::getPulse()
{
	_write_register(PELE, PULSE_CFG);
	return (_read_register(PULSE_SRC) & PEA);
}

//////////////////////////////////////////////////////////
void MMA8451Q::update()
{
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t)0x00);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)_addr, (uint8_t)(_highres ? 7 : 4));

  if (Wire.available())
  {
    _stat = Wire.read();
    if(_highres)
    {
      _xi = (int16_t)((Wire.read() << 8) + Wire.read());
      _yi = (int16_t)((Wire.read() << 8) + Wire.read());
      _zi = (int16_t)((Wire.read() << 8) + Wire.read());
      _xi >>= 2;
      _yi >>= 2;
      _zi >>= 2;
      /*_xf = ((float) _xi / 16) * _step_factor;
      _yf = ((float) _yi / 16) * _step_factor;
      _zf = ((float) _zi / 16) * _step_factor;*/
    }
    else
    {
      _xi = (int8_t)Wire.read();
      _yi = (int8_t)Wire.read();
      _zi = (int8_t)Wire.read();
      /*_xf = _xi*_step_factor;
      _yf = _yi*_step_factor;
      _zf = _zi*_step_factor;*/
    }
  }
}

//////////////////////////////////////////////////////////
/*
There are four (4) registers associated with the Motion/Freefall embedded function.
1.Register 0x15 FF/MT Config - Motion/Freefall Configuration
2.Register 0x17 FF_MT_THS - Setting the Threshold
3.Register 0x18 FF_MT_COUNT - Setting the Debounce Counter
4.Register 0x16 FF_MT_SRC - Motion/Freefall Source Detection
Refer to Table 12 for the complete list of all registers that can be used with Motion/Freefall.
 */
bool MMA8451Q::setMotionDetection()
{
  _standby();
// 0x15, 0xD8 (11011000)
//  _write_register(XEFE | YEFE | ZEFE | OAE | FELE);

  // FF_MT_CFG: Config
  // Mode 4: Motion detection with ELE = 1, OAE = 1, ZEFE = 0
  // FELE: Latch Enable, EA stays set. To clear EA the FF_MT_SRC register must be read.
  // OAE: 1: Motion flag (X or Y or Z > threshold), 0: Freefall flag (X and Y and Z < threshold)
  // XEFE, YEFE, ZEFE: Event flag enbale on X,Y,Z
  uint8_t dataout = XEFE | YEFE | 0 | OAE | FELE;
  _write_register( dataout , FF_MT_CFG);

  _active();
}


//////////////////////////////////////////////////////////
/*
The threshold resolution is 0.063 g/LSB and the threshold register has a range of 0 to 127 counts. The maximum range is to 8 g.
Note that even when the full-scale value is set to 2 g or 4 g the motion detects up to 8 g. If the low-noise bit is set in register 0x2A
then the maximum threshold will be limited to 4 g regardless of the full-scale range.
 */
bool MMA8451Q::setMotionThresholdG(float g, bool dbcntm)
{

  _standby();

  uint8_t ths = 0;

  if (g > 8.0) g = 8.0;
  if (g < 0.0) g = 0.0;

  ths = (uint8_t) (g / 8.0 * 127);
  ths &= THS_MASK;

  if (dbcntm) _write_register((ths | DBCNTM), FF_MT_THS);
  else _write_register(ths, FF_MT_THS);

  _active();
}


//////////////////////////////////////////////////////////
/*
When the internal debounce counter reaches the FF_MT_COUNT value a freefall/motion event flag is set. The debounce counter
will never increase beyond the FF_MT_COUNT value. Time step used for the debounce sample count depends on the ODR
chosen and the oversampling mode as shown in Table 38.
 */
bool MMA8451Q::setMotionDebounceCounter(uint8_t n)
{
  _standby();
  _write_register(n, FF_MT_COUNT);
  _active();
}


//////////////////////////////////////////////////////////
uint8_t MMA8451Q::getMotionSource()
{
  return (_read_register(FF_MT_SRC));
}


//////////////////////////////////////////////////////////
/*
 * When HPF_BYP is set then the HPF is bypassed => NO DC REMOVAL! This would be the same as motion detection.
 */
bool MMA8451Q::setTransientDetection()
{
  _standby();
//  uint8_t dataout = 0 | XTEFE | YTEFE | 0 | TELE;
//  uint8_t dataout = HPF_BYP | XTEFE | YTEFE | ZTEFE | TELE;
  uint8_t dataout = 0 | XTEFE | YTEFE | ZTEFE | TELE;
  _write_register( dataout , TRANSIENT_CFG);

  _active();
}

/*
The highpass filter cutoff frequency can be set by the user
to four different frequencies which are dependent on the
Output Data Rate (ODR) and the oversampling mode.
A higher cutoff frequency ensures the DC data or slower moving data
will be filtered out, allowing only the higher frequencies to pass.
 */


//////////////////////////////////////////////////////////
uint8_t MMA8451Q::getTransientSource()
{
  return (_read_register(TRANSIENT_SRC));
}


//////////////////////////////////////////////////////////
/*
The threshold THS[6:0] is a 7-bit unsigned number, 0.063 g/LSB. The maximum threshold is 8 g. Even if the part is set to full
scale at 2 g or 4 g this function will still operate up to 8 g. If the low-noise bit is set in register 0x2A, the maximum threshold to be
reached is 4 g.
 */
bool MMA8451Q::setTransientThresholdG(float g, bool dbcntm)
{
  _standby();

  uint8_t ths = 0;

  if (g > 8.0) g = 8.0;
  if (g < 0.0) g = 0.0;

  ths = (uint8_t) (g / 8.0 * 127);
  ths &= THS_MASK;

  if (dbcntm) _write_register((ths | DBCNTM), TRANSIENT_THS);
  else _write_register(ths, TRANSIENT_THS);

  _active();
}

bool MMA8451Q::setTransientThresholdN(uint8_t g, bool dbcntm)
{

  _standby();

  //uint8_t ths = 0;

  if (g > 127) g = 127;
  if (g < 0) g = 0;

  g &= THS_MASK;

  if (dbcntm) _write_register((g | DBCNTM), TRANSIENT_THS);
  else _write_register(g, TRANSIENT_THS);

  _active();
}

//////////////////////////////////////////////////////////
/*
The TRANSIENT_COUNT sets the minimum number of debounce counts continuously matching the condition where the
unsigned value of high-pass filtered data is greater than the user specified value of TRANSIENT_THS.
*/
bool MMA8451Q::setTransientDebounceCounter(uint8_t n)
{
  _standby();
  _write_register(n, TRANSIENT_COUNT);
  _active();
}


//////////////////////////////////////////////////////////
bool MMA8451Q::setHPFilterCutOff(uint8_t n)
{
  if (n > 3) n = 3;
//  if (n < 0) n = 0;

  _standby();
  uint8_t reg = _read_register(HP_FILTER_CUTOFF);
  reg &= 0b11111100;
  reg |= (n & 0b00000011);
  _write_register(reg, HP_FILTER_CUTOFF);
  _active();
}




//////////////////////////////////////////////////////////
bool MMA8451Q::setInterrupt(INTERRUPT_CFG_EN_SOURCE type, INTERRUPT_PIN pin, bool on)
{
  _standby();

	uint8_t current_value = _read_register(CTRL_REG4);

	if(on) current_value |= type;
	else   current_value &= ~(type);

	_write_register(current_value, CTRL_REG4);

	uint8_t current_routing_value = _read_register(CTRL_REG5);

	if (pin == INT2) current_routing_value &= ~(type);    // clear bit, set bit to 0, routed to INT2
	else if (pin == INT1) current_routing_value |= type;  // set bit, routed to INT1

	_write_register(current_routing_value, CTRL_REG5);

  _active();
}

//////////////////////////////////////////////////////////
bool MMA8451Q::disableAllInterrupts()
{
	_write_register(0x2D, 0);
}

/*
//////////////////////////////////////////////////////////
uint8_t MMA8451Q::dumpRegisters()
{
  Wire.beginTransmission(_addr);
  Wire.write(0);
  Wire.endTransmission(false);

  Wire.requestFrom(_addr, (uint8_t) 0x32);

  while (Wire.available()) {
    static uint8_t i = 0;

    uint8_t reg = Wire.read();

    _PrintHex8(&i, 1);
    Serial.print(",");
    Serial.print(regNames[i]);
    Serial.print(",");
    Serial.print(reg);
    Serial.print(",");
    _PrintHex8(&reg, 1);
    for (uint8_t k = 0; k<8; k++) {
      Serial.print(",");
      if ((1<<(7-k) & reg) == 0) Serial.print(0);
      else Serial.print(1);
    }
    Serial.println();

    i++;
  }
  return 0;
}
*/


//////////////////////////////////////////////////////////
/*
 * Dump the registers in active mode.
 */
uint8_t MMA8451Q::dumpRegisters()
{
    uint8_t reg = 0;

    for (uint8_t i = 0; i<0x32; i++) {

      reg = _read_register(i);

      _PrintHex8(&i, 1);
      Serial.print(",");
      Serial.print(regNames[i]);
      Serial.print(",");
      Serial.print(reg);
      Serial.print(",");
      _PrintHex8(&reg, 1);
      for (uint8_t k = 0; k<8; k++) {
        Serial.print(",");
        if ((1<<(7-k) & reg) == 0) Serial.print(0);
        else Serial.print(1);
      }
      Serial.println();
  }
  return 0;
}




//end public methods



//begin private methods

//////////////////////////////////////////////////////////
uint8_t MMA8451Q::_read_register(uint8_t offset)
{
  Wire.beginTransmission(_addr);
  Wire.write(offset);
  Wire.endTransmission(false);

  Wire.requestFrom(_addr, (uint8_t)1);

  if (Wire.available()) return Wire.read();
  return 0;
}


//////////////////////////////////////////////////////////
void MMA8451Q::_write_register(uint8_t b, uint8_t offset)
{
  Wire.beginTransmission(_addr);
  Wire.write(offset);
  Wire.write(b);
  Wire.endTransmission();
}

//////////////////////////////////////////////////////////
void MMA8451Q::_standby()
{
  uint8_t reg1 = 0x00;
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t) CTRL_REG1);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t)_addr, (uint8_t)1);

  if (Wire.available()) reg1 = Wire.read();

  reg1 = reg1 & ~ACTIVE; // Clear active bit

  Wire.beginTransmission(_addr); // Reset
  Wire.write((uint8_t) CTRL_REG1);
  Wire.write(reg1);
  Wire.endTransmission();
}

//////////////////////////////////////////////////////////
void MMA8451Q::_active()
{
  uint8_t reg1 = 0x00;
  Wire.beginTransmission(_addr); // Set to status reg
  Wire.write((uint8_t) CTRL_REG1);
  Wire.endTransmission(false);

  Wire.requestFrom((uint8_t) _addr, (uint8_t)1);
  if (Wire.available()) reg1 = Wire.read();

  reg1 = reg1 | ACTIVE; // Set active bit

  Wire.beginTransmission(_addr);
  Wire.write((uint8_t) CTRL_REG1);
  Wire.write(reg1);
  Wire.endTransmission();
}

//////////////////////////////////////////////////////////
void MMA8451Q::_PrintHex8(uint8_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
     char tmp[16];
       for (int i=0; i<length; i++) {
         sprintf(tmp, "0x%.2X",data[i]);
         Serial.print(tmp);
         //Serial.print(" ");
       }
}

//////////////////////////////////////////////////////////
void MMA8451Q::_PrintHex16(uint16_t *data, uint8_t length) // prints 16-bit data in hex with leading zeroes
{
       char tmp[16];
       for (int i=0; i<length; i++)
       {
         sprintf(tmp, "0x%.4X",data[i]);
         Serial.print(tmp);
         //Serial.print(" ");
       }
}


//end private methods
