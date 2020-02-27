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

#ifndef MMA8451Q_H
#define MMA8451Q_H

#define MMA_8451Q_DEFAULT_ADDRESS 0x1C

#include "Arduino.h"
#include <Wire.h>


// CLASS
class MMA8451Q
{
  public:
    uint8_t _read_register(uint8_t offset);
    void    _write_register(uint8_t b, uint8_t offset);

    void _PrintHex8(uint8_t *data, uint8_t length);
    void _PrintHex16(uint16_t *data, uint8_t length);

    int16_t _xi, _yi, _zi;
    float _xf, _yf, _zf;
    uint8_t _who_am_i;

    /**
     *  @enum SA0
     *  @brief Possible terminations for the ADDR pin
     */
    enum SA0 {
        SA0_VSS = 0, /*!< SA0 connected to VSS */
        SA0_VDD      /*!< SA0 connected to VDD */
    };

    /**
     *  @enum WHO_AM_I_VAL
     *  @brief Device ID's that this class is compatible with
     */
    enum WHO_AM_I_VAL {
        MMA8451 = 0x1A, /*!< MMA8451 WHO_AM_I register content */
        MMA8452 = 0x2A, /*!< MMA8452 WHO_AM_I register content */
        MMA8453 = 0x3A, /*!< MMA8453 WHO_AM_I register content */
    };

    /**
     * @enum SYS_MODE
     * @brief operating mode of MMA845x
     */
    enum SYS_MODE {
        STANDBY = 0,
        AWAKE, SLEEP
    };

    /**
     * @enum STATUS
     * @brief flags for data overwrite and data ready
     */
    enum STATUS {
        XDR   = 0x01,
        YDR   = 0x02,
        ZDR   = 0x04,
        XYZDR = 0x08,
        XOW   = 0x10,
        YOW   = 0x20,
        ZOW   = 0x40,
        XYZOW = 0x80
    };

    /**
     * @enum RANGE
     * @brief values for measurement range positive and negative
     */
    enum RANGE {
        RANGE_2G = 0,  // The total range is +- 2G, resol. 1G / 4096 counts (0.25 mg)
        RANGE_4G = 1,  // The total range is +- 4G, resol. 1G / 2048 counts (0.49 mg)
        RANGE_8G = 2   // The total range is +- 8G, resol. 1G / 1024 counts (0.98 mg)
    };

    /**
     * @enum RANGE
     * @brief values for measurement range positive and negative
     */
    enum SCALE_RANGE {
        SCALE_RANGE_2G = 4096,  //
        SCALE_RANGE_4G = 2048,  //
        SCALE_RANGE_8G = 1024   //
    };

    /**
     * @enum RESOLUTION
     * @brief selections for resolution of data, 8 bit or maximum
     */
    enum RESOLUTION {
        RES_MAX  = 0,   /* Read back full resolution - normal mode*/
        RES_8BIT = 2   /* Read back 8 bit values only - fast mode*/
    };

    /**
     *  @enum LOW_NOISE
     *  @brief Low Noise mode Note: 4g max reading when on
     */
    enum LOW_NOISE {
        LN_OFF = 0x00, /* Low Noise mode off */
        LN_ON  = 0x04  /* Low Noise mode on, 4g max readings */
    };

    /**
     *  @enum HPF_MODE
     *  @brief High Pass Filter mode
     */
    enum HPF_MODE {
        HPF_OFF = 0x00, /* High Pass Filter mode off */
        HPF_ON  = 0x10  /* High Pass Filter mode on */
    };

    /**
     * @enum DATA_RATE
     * @brief values for normal output data rate in Hz
     */
    enum DATA_RATE {
        DR_800  = 0x00,
        DR_400  = 0x08,
        DR_200  = 0x10,
        DR_100  = 0x18,
        DR_50   = 0x20,
        DR_12_5 = 0x28,
        DR_6_25 = 0x30,
        DR_1_56 = 0x38
    };

    /**
     * @enum ASLP_DATA_RATE
     * @brief values for auto_sleep mode data rate in HZ
     */
    enum ASLP_DATA_RATE {
        ASLPDR_50   = 0x00,
        ALSPDR_12_5 = 0x40,
        ASLPDR_6_25 = 0x80,
        ASLPDR_1_56 = 0xB0
    };

    /**
     *  @enum OVERSAMPLE_MODE
     *  @brief sets the oversample mode, Normal, Low power and noise, High resolution, or low power
     */
    enum OVERSAMPLE_MODE {
        OS_NORMAL = 0,
        OS_LO_PN, OS_HI_RES, OS_LO_POW
    };

    /**
     *  @enum INTERRUPT_CFG_AND_ENABLE
     *  @brief used to {configure, enable, detect source of} interrupts for corresponding functions
     *  (INT_FIFO used only in MMA8451)
     */
    enum INTERRUPT_CFG_EN_SOURCE {
        INT_EN_DRDY = 0x01,
        // 0x20 missing,
        INT_EN_FF_MT = 0x04,
        INT_EN_PULSE = 0x08,
        INT_EN_LNDPRT = 0x10,
        INT_EN_TRANS = 0x20,
        INT_EN_FIFO = 0x40,
        INT_EN_ASLP = 0x80
    };

    /**
     *  @enum INTERRUPT_PIN
     *  @brief chooses interrupt pin used to signal interrupt condition
     */
    enum INTERRUPT_PIN {
        INT1 = 1,
        INT2 = 0,
        INT_NONE = 2
    };

    /**
     *  @enum REGISTER
     *  @brief The device register map
     */
    enum REGISTER {
        STATUS = 0x00,
        OUT_X_MSB, OUT_X_LSB, OUT_Y_MSB, OUT_Y_LSB, OUT_Z_MSB, OUT_Z_LSB, ND1, ND2,

        F_SETUP = 0x09, TRIG_CFG, // only available on the MMA8451 variant

        SYSMOD = 0x0B,
        INT_SOURCE, WHO_AM_I, XYZ_DATA_CFG, HP_FILTER_CUTOFF, PL_STATUS,
        PL_CFG, PL_COUNT, PL_BF_ZCOMP, P_L_THS_REG, FF_MT_CFG, FF_MT_SRC,
        FF_MT_THS, FF_MT_COUNT,

        TRANSIENT_CFG = 0x1D,
        TRANSIENT_SRC, TRANSIENT_THS, TRANSIENT_COUNT, PULSE_CFG, PULSE_SRC,
        PULSE_THSX, PULSE_THSY, PULSE_THSZ, PULSE_TMLT, PULSE_LTCY, PULSE_WIND,
        ASLP_COUNT, CTRL_REG1, CTRL_REG2, CTRL_REG3, CTRL_REG4, CTRL_REG5,
        OFF_X, OFF_Y, OFF_Z
    };

    enum CTRL_REG1_MASKS {
      ACTIVE     = 0x01,
      F_READ     = 0x02,
      LNOISE     = 0x04,
      DR0        = 0x08,
      DR1        = 0x10,
      DR2        = 0x20,
      ASLP_RATE0 = 0x40,
      ASLP_RATE1 = 0x80
    };

    enum CTRL_REG2_MASKS {
      MODS0    = 0x01, // Active mode power scheme selection
      MODS1    = 0x02,
      SLPE     = 0x04, // Auto-sleep enable
      SMODS0   = 0x08, // Sleep mode power scheme selection
      SMODS1   = 0x10,
      // unused 0x20,
      RST      = 0x40, // Software reset
      ST       = 0x80  // Self-test enable
    };

    // FF_MT_CFG freefall/motion configuration register
    enum FF_MT_CFG_MASKS {
      // 0x01 not used
      // 0x02 not used
      // 0x04 not used
      XEFE = 0x08,
      YEFE = 0x10,
      ZEFE = 0x20,
      OAE  = 0x40,
      FELE = 0x80 // ELE in the datasheet, Freefall Motion Event Latch Enable
    };

    // FF_MT_THS (freefall and motion) and TRANSIENT_THS threshold register. Regs behave similarly.
    enum THS_MASKS {
      DBCNTM   = 0x80,
      THS_MASK = 0x7F
    };

    // FF_MT_SRC freefall/motion source register, READONLY
    enum FF_MT_SRC_MASKS {
      XHP = 0x01,
      XHE = 0x02,
      YHP = 0x04,
      YHE = 0x08,
      ZHP = 0x10,
      ZHE = 0x20,
      // 0x40 not used
      FEA = 0x80 // EA in the datasheet, Freefall Motion Event Avtive Flag
    };


    // TRANSIENT_CFG transient configuration register
    enum TRANSIENT_CFG_MASKS {
      HPF_BYP = 0x01,
      XTEFE   = 0x02,
      YTEFE   = 0x04,
      ZTEFE   = 0x08,
      TELE    = 0x10 // ELE in the datasheet, Transient Event Latch Enable
      // 0x20 not used
      // 0x40 not used
      // 0x80 not used
    };

    // TRANSIENT_SRC transient source register, READONLY
    enum TRANSIENT_SRC_MASKS {
      X_Trans_Pol = 0x01,
      XTRANSE     = 0x02,
      Y_Trans_Pol = 0x04,
      YTRANSE     = 0x08,
      Z_Trans_Pol = 0x10,
      ZTRANSE     = 0x20,
      TEA         = 0x40 // EA in the datasheet, Transient Event Avtive Flag
    };

    // PULSE_CFG pulse configuration register
    enum PULSE_CFG_MASKS {
      XSPEFE = 0x01,
      XDPEFE = 0x02,
      YSPEFE = 0x04,
      YDPEFE = 0x08,
      ZSPEFE = 0x10,
      ZDPEFE = 0x20,
      PELE   = 0x40,  // ELE in the datasheet, Pulse Event Latch Enable
      DPA    = 0x80
    };

    // PULSE_SRC pulse source register
    enum PULSE_SRC_MASKS {
      PolX = 0x01,
      PolY = 0x02,
      PolZ = 0x04,
      DPE  = 0x08,
      AxX  = 0x10,
      AxY  = 0x20,
      AxZ  = 0x40,
      PEA   = 0x80  // EA in the datasheet, Pulse Event Active Flag
    };


    MMA8451Q(uint8_t addr = MMA_8451Q_DEFAULT_ADDRESS);

    uint8_t SWreset();
    uint8_t setCommonParameters(RANGE range, RESOLUTION resolution, LOW_NOISE lo_noise, DATA_RATE data_rate, OVERSAMPLE_MODE os_mode, HPF_MODE hpf_mode);

    //    uint8_t set_range(RANGE range); // not implemented, yet.
    //    void begin(bool highres = true, uint8_t scale = 2);
    void update();
    uint8_t getPLStatus();
    uint8_t getPulse();

    bool setMotionDetection();
    bool setMotionThresholdG(float g, bool dbcntm = false);
    uint8_t getMotionSource();
    bool setMotionDebounceCounter(uint8_t n = 10);


    bool setTransientDetection();
    uint8_t getTransientSource();
    bool setTransientThresholdG(float g, bool dbcntm = false);
	  bool setTransientThresholdN(uint8_t g, bool dbcntm = false);
    bool setTransientDebounceCounter(uint8_t n = 10);

    bool setHPFilterCutOff(uint8_t n);

    uint8_t dumpRegisters();
    uint8_t get_CTRL_REG1();

    // Interrupts
    bool setInterrupt(INTERRUPT_CFG_EN_SOURCE type, INTERRUPT_PIN pin, bool on);
    bool disableAllInterrupts();

    static const char * const regNames[];

  private:

//	  uint8_t _read_register(uint8_t offset);
//  	void    _write_register(uint8_t b, uint8_t offset);

//   void _PrintHex8(uint8_t *data, uint8_t length);
//   void _PrintHex16(uint16_t *data, uint8_t length);

    void _standby();
    void _active();

    uint8_t _addr;
    uint8_t _stat;
    uint8_t _scale;
    float _step_factor;
    bool _highres;

    float _rad2deg;
};
#endif 
