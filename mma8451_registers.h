#ifndef MMA8451_PRIVATE_H
#define MMA8451_PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef union
{
  uint8_t Byte;
  struct {
    uint8_t _0   :1;
    uint8_t _1   :1;
    uint8_t _2   :1;
    uint8_t _3   :1;
    uint8_t _4   :1;
    uint8_t _5   :1;
    uint8_t _6   :1;
    uint8_t _7   :1;
  } Bit;
} Reg8_t;

#define MMA8451_REG_STATUS           0x00
#define MMA8451_REG_OUT_X_MSB        0x01
#define MMA8451_REG_F_SETUP          0x09
#define MMA8451_REG_TRIG_CFG         0x0A
#define MMA8451_REG_SYSMOD           0x0B
#define MMA8451_REG_INT_SOURCE       0x0C
#define MMA8451_REG_WHOAMI           0x0D
#define MMA8451_REG_XYZ_DATA_CFG     0x0E
#define MMA8451_REG_HP_FILTER_CUTOFF 0x0F
#define MMA8451_REG_PL_STATUS        0x10
#define MMA8451_REG_PL_CFG           0x11
#define MMA8451_REG_PL_COUNT         0x12
#define MMA8451_REG_PL_BF_ZCOMP      0x13
#define MMA8451_REG_P_L_THS_REG      0x14
#define MMA8451_REG_FF_MT_CFG        0x15
#define MMA8451_REG_FF_MT_SRC        0x16
#define MMA8451_REG_FF_MT_THS        0x17
#define MMA8451_REG_FF_MT_COUNT      0x18
#define MMA8451_REG_TRANSIENT_CFG    0x1D
#define MMA8451_REG_TRANSIENT_SRC    0x1E
#define MMA8451_REG_TRANSIENT_THS    0x1F
#define MMA8451_REG_TRANSIENT_COUNT  0x20
#define MMA8451_REG_PULSE_CFG        0x21
#define MMA8451_REG_PULSE_SRC        0x22
#define MMA8451_REG_PULSE_THSX       0x23
#define MMA8451_REG_PULSE_THSY       0x24
#define MMA8451_REG_PULSE_THSZ       0x25
#define MMA8451_REG_PULSE_TMLT       0x26
#define MMA8451_REG_PULSE_LTCY       0x27
#define MMA8451_REG_PULSE_WIND       0x28
#define MMA8451_REG_ASLP_COUNT       0x29
#define MMA8451_REG_CTRL_REG1        0x2A
#define MMA8451_REG_CTRL_REG2        0x2B
#define MMA8451_REG_CTRL_REG3        0x2C
#define MMA8451_REG_CTRL_REG4        0x2D
#define MMA8451_REG_CTRL_REG5        0x2E
#define MMA8451_REG_OFF_X            0x2F
#define MMA8451_REG_OFF_Y            0x30
#define MMA8451_REG_OFF_Z            0x31

/*
**  STATUS Registers
*/
//
#define ZYXOW_BIT             Bit._7
#define ZOW_BIT               Bit._6
#define YOR_BIT               Bit._5
#define XOR_BIT               Bit._4
#define ZYXDR_BIT             Bit._3
#define ZDR_BIT               Bit._2
#define YDR_BIT               Bit._1
#define XDR_BIT               Bit._0
//
#define ZYXOW_MASK            0x80
#define ZOW_MASK              0x40
#define YOR_MASK              0x20
#define XOR_MASK              0x10
#define ZYXDR_MASK            0x08
#define ZDR_MASK              0x04
#define YDR_MASK              0x02
#define XDR_MASK              0x01

/*
**  WHO_AM_I Device ID Register
*/
#define MMA8451Q_ID           0x1A

/*
**  F_STATUS FIFO Status Register
*/
//
#define F_OVF_BIT             Bit._7
#define F_WMRK_FLAG_BIT       Bit._6
#define F_CNT5_BIT            Bit._5
#define F_CNT4_BIT            Bit._4
#define F_CNT3_BIT            Bit._3
#define F_CNT2_BIT            Bit._2
#define F_CNT1_BIT            Bit._1
#define F_CNT0_BIT            Bit._0
//
#define F_OVF_MASK            0x80
#define F_WMRK_FLAG_MASK      0x40
#define F_CNT5_MASK           0x20
#define F_CNT4_MASK           0x10
#define F_CNT3_MASK           0x08
#define F_CNT2_MASK           0x04
#define F_CNT1_MASK           0x02
#define F_CNT0_MASK           0x01
#define F_CNT_MASK            0x3F

/*
**  F_SETUP FIFO Setup Register
*/
//
#define F_MODE1_BIT           Bit._7
#define F_MODE0_BIT           Bit._6
#define F_WMRK5_BIT           Bit._5
#define F_WMRK4_BIT           Bit._4
#define F_WMRK3_BIT           Bit._3
#define F_WMRK2_BIT           Bit._2
#define F_WMRK1_BIT           Bit._1
#define F_WMRK0_BIT           Bit._0
//
#define F_MODE1_MASK          0x80
#define F_MODE0_MASK          0x40
#define F_WMRK5_MASK          0x20
#define F_WMRK4_MASK          0x10
#define F_WMRK3_MASK          0x08
#define F_WMRK2_MASK          0x04
#define F_WMRK1_MASK          0x02
#define F_WMRK0_MASK          0x01
#define F_MODE_MASK           0xC0
#define F_WMRK_MASK           0x3F


/*
** TRIG CFG Register
*/
//
#define TRIG_TRANS_BIT      Bit._5
#define TRIG_LNDPRT_BIT     Bit._4
#define TRIG_PULSE_BIT      Bit._3
#define TRIG_FF_MT_BIT      Bit._2
//
#define TRIG_TRANS_MASK     0x20
#define TRIG_LNDPRT_MASK    0x10
#define TRIG_PULSE_MASK     0x08
#define TRIG_FF_MT_MASK     0x04


/*
**  SYSMOD System Mode Register
*/
//
#define FGERR_BIT             Bit._7
#define FGT_4_BIT             Bit._6
#define FGT_3_BIT             Bit._5
#define FGT_2_BIT             Bit._4
#define FGT_1_BIT             Bit._3
#define FGT_0_BIT             Bit._2
#define SYSMOD1_BIT           Bit._1
#define SYSMOD0_BIT           Bit._0
//
#define FGERR_MASK            0x80
#define FGT_4MASK             0x40
#define FGT_3MASK             0x20
#define FGT_2MASK             0x10
#define FGT_1MASK             0x08
#define FGT_0MASK             0x04
#define FGT_MASK              0x7C
#define SYSMOD1_MASK          0x02
#define SYSMOD0_MASK          0x01
#define SYSMOD_MASK           0x03

/*
**  INT_SOURCE System Interrupt Status Register
*/
//
#define SRC_ASLP_BIT          Bit._7
#define SRC_FIFO_BIT          Bit._6
#define SRC_TRANS_BIT         Bit._5
#define SRC_LNDPRT_BIT        Bit._4
#define SRC_PULSE_BIT         Bit._3
#define SRC_FF_MT_BIT         Bit._2
#define SRC_DRDY_BIT          Bit._0
//
#define SRC_ASLP_MASK         0x80
#define SRC_FIFO_MASK         0x40
#define SRC_TRANS_MASK        0x20
#define SRC_LNDPRT_MASK       0x10
#define SRC_PULSE_MASK        0x08
#define SRC_FF_MT_MASK        0x04
#define SRC_DRDY_MASK         0x01

/*
**  XYZ_DATA_CFG Sensor Data Configuration Register
*/
//
#define HPF_OUT_BIT           Bit._4
#define FS1_BIT               Bit._1
#define FS0_BIT               Bit._0
//
#define HPF_OUT_MASK          0x10
#define FS1_MASK              0x02
#define FS0_MASK              0x01
#define FS_MASK               0x03

#define FULL_SCALE_8G         FS1_MASK
#define FULL_SCALE_4G         FS0_MASK
#define FULL_SCALE_2G         0x00

/*
**  HP_FILTER_CUTOFF High Pass Filter Register
*/
//
#define PULSE_HPF_BYP         Bit._5
#define PULSE_LPF_EN          Bit._4
#define SEL1_BIT              Bit._1
#define SEL0_BIT              Bit._0
//
#define PULSE_HPF_BYP_MASK    0x20
#define PULSE_LPF_EN_MASK     0x10
#define SEL1_MASK             0x02
#define SEL0_MASK             0x01
#define SEL_MASK              0x03

/*
**  PL_STATUS Portrait/Landscape Status Register
**  PL_PRE_STATUS Portrait/Landscape Previous Data Status Register
*/
//
#define NEWLP_BIT             Bit._7
#define LO_BIT                Bit._6
#define LAPO1_BIT             Bit._2
#define LAPO0_BIT             Bit._1
#define BAFRO_BIT             Bit._0
//
#define NEWLP_MASK            0x80
#define LO_MASK               0x40
#define LAPO1_MASK            0x04
#define LAPO0_MASK            0x02
#define LAPO_MASK             0x06
#define BAFRO_MASK            0x01


/*
**  PL_CFG Portrait/Landscape Configuration Register
*/
//
#define DBCNTM_BIT            Bit._7
#define PL_EN_BIT             Bit._6
//
#define DBCNTM_MASK           0x80
#define PL_EN_MASK            0x40

/*
**  PL_BF_ZCOMP Back/Front and Z Compensation Register
*/
//
#define BKFR1_BIT             Bit._7
#define BKFR0_BIT             Bit._6
#define ZLOCK2_BIT            Bit._2
#define ZLOCK1_BIT            Bit._1
#define ZLOCK0_BIT            Bit._0
//
#define BKFR1_MASK            0x80
#define BKFR0_MASK            0x40
#define ZLOCK2_MASK           0x04
#define ZLOCK1_MASK           0x02
#define ZLOCK0_MASK           0x01
#define BKFR_MASK             0xC0
#define ZLOCK_MASK            0x07

/*
**  PL_P_L_THS Portrait to Landscape Threshold Registers
*/
//
#define P_L_THS4_Bit          Bit._7
#define P_L_THS3_Bit          Bit._6
#define P_L_THS2_Bit          Bit._5
#define P_L_THS1_Bit          Bit._4
#define P_L_THS0_Bit          Bit._3
#define HYS2_Bit              Bit._2
#define HYS1_Bit              Bit._1
#define HYS0_Bit              Bit._0
//
#define P_L_THS4_MASK         0x80
#define P_L_THS3_MASK         0x40
#define P_L_THS2_MASK         0x20
#define P_L_THS1_MASK         0x10
#define P_L_THS0_MASK         0x08
#define P_L_THS_MASK          0xF8
#define HYS2_MASK             0x04
#define HYS1_MASK             0x02
#define HYS0_MASK             0x01
#define HYS_MASK              0x07

  
/*
**  FF_MT_CFG Freefall and Motion Configuration Registers
*/
//
#define ELE_BIT               Bit._7
#define OAE_BIT               Bit._6
#define ZEFE_BIT              Bit._5
#define YEFE_BIT              Bit._4
#define XEFE_BIT              Bit._3
//
#define ELE_MASK              0x80
#define OAE_MASK              0x40
#define ZEFE_MASK             0x20
#define YEFE_MASK             0x10
#define XEFE_MASK             0x08

/*
**  FF_MT_SRC Freefall and Motion Source Registers
*/
//
#define EA_BIT                Bit._7
#define ZHE_BIT               Bit._5
#define ZHP_BIT               Bit._4
#define YHE_BIT               Bit._3
#define YHP_BIT               Bit._2
#define XHE_BIT               Bit._1
#define XHP_BIT               Bit._0
//
#define EA_MASK               0x80
#define ZHE_MASK              0x20
#define ZHP_MASK              0x10
#define YHE_MASK              0x08
#define YHP_MASK              0x04
#define XHE_MASK              0x02
#define XHP_MASK              0x01

/*
**  FF_MT_THS Freefall and Motion Threshold Registers
**  TRANSIENT_THS Transient Threshold Register
*/
//
#define DBCNTM_BIT            Bit._7
#define THS6_BIT              Bit._6
#define THS5_BIT              Bit._5
#define THS4_BIT              Bit._4
#define THS3_BIT              Bit._3
#define THS2_BIT              Bit._2
#define THS1_BIT              Bit._1
#define THS0_BIT              Bit._0
//
#define DBCNTM_MASK           0x80
#define THS6_MASK             0x40
#define THS5_MASK             0x20
#define THS4_MASK             0x10
#define THS3_MASK             0x08
#define THS2_MASK             0x04
#define TXS1_MASK             0x02
#define THS0_MASK             0x01
#define THS_MASK              0x7F

/*
**  TRANSIENT_CFG Transient Configuration Register
*/
//
#define TELE_BIT              Bit._4
#define ZTEFE_BIT             Bit._3
#define YTEFE_BIT             Bit._2
#define XTEFE_BIT             Bit._1
#define HPF_BYP_BIT           Bit._0
//
#define TELE_MASK             0x10
#define ZTEFE_MASK            0x08
#define YTEFE_MASK            0x04
#define XTEFE_MASK            0x02
#define HPF_BYP_MASK          0x01

/*
**  TRANSIENT_SRC Transient Source Register
*/
//
#define TEA_BIT               Bit._6
#define ZTRANSE_BIT           Bit._5
#define ZTRANSPOL_BIT         Bit._4
#define YTRANSE_BIT           Bit._3
#define YTRANSPOL_BIT         Bit._2
#define XTRANSE_BIT           Bit._1
#define XTRANSPOL_BIT         Bit._0

//
#define TEA_MASK              0x40
#define ZTRANSE_MASK          0x20
#define ZTRANSEPOL_MASK       0x10
#define YTRANSE_MASK          0x08
#define YTRANSEPOL_MASK       0x04
#define XTRANSE_MASK          0x02
#define XTRANSEPOL_MASK       0x01

/*
**  PULSE_CFG Pulse Configuration Register
*/
#define PULSE_CFG_REG         0x21
//
#define DPA_BIT               Bit._7
#define PELE_BIT              Bit._6
#define ZDPEFE_BIT            Bit._5
#define ZSPEFE_BIT            Bit._4
#define YDPEFE_BIT            Bit._3
#define YSPEFE_BIT            Bit._2
#define XDPEFE_BIT            Bit._1
#define XSPEFE_BIT            Bit._0
//
#define DPA_MASK              0x80
#define PELE_MASK             0x40
#define ZDPEFE_MASK           0x20
#define ZSPEFE_MASK           0x10
#define YDPEFE_MASK           0x08
#define YSPEFE_MASK           0x04
#define XDPEFE_MASK           0x02
#define XSPEFE_MASK           0x01

/*
**  PULSE_SRC Pulse Source Register
*/
//
#define PEA_BIT               Bit._7
#define PAXZ_BIT              Bit._6
#define PAXY_BIT              Bit._5
#define PAXX_BIT              Bit._4
#define PDPE_BIT              Bit._3
#define POLZ_BIT              Bit._2
#define POLY_BIT              Bit._1
#define POLX_BIT              Bit._0
//
#define PEA_MASK              0x80
#define PAXZ_MASK             0x40
#define PAXY_MASK             0x20
#define PAXX_MASK             0x10
#define PDPE_MASK             0x08
#define POLZ_MASK             0x04
#define POLY_MASK             0x02
#define POLX_MASK             0x01

/*
**  PULSE_THS XYZ Pulse Threshold Registers
*/
//
#define PTHS_MASK             0x7F

/*
**  CTRL_REG1 System Control 1 Register
*/
//
#define ASLP_RATE1_BIT        Bit._7
#define ASLP_RATE0_BIT        Bit._6
#define DR2_BIT               Bit._5
#define DR1_BIT               Bit._4
#define DR0_BIT               Bit._3
#define LNOISE_BIT            Bit._2
#define FREAD_BIT             Bit._1
#define ACTIVE_BIT            Bit._0
//
#define ASLP_RATE1_MASK       0x80
#define ASLP_RATE0_MASK       0x40
#define DR2_MASK              0x20
#define DR1_MASK              0x10
#define DR0_MASK              0x08
#define LNOISE_MASK           0x04
#define FREAD_MASK            0x02
#define ACTIVE_MASK           0x01
#define ASLP_RATE_MASK        0xC0
#define DR_MASK               0x38
//                      
#define ASLP_RATE_20MS        0x00
#define ASLP_RATE_80MS        ASLP_RATE0_MASK
#define ASLP_RATE_160MS       ASLP_RATE1_MASK
#define ASLP_RATE_640MS       (ASLP_RATE1_MASK+ASLP_RATE0_MASK)
//
#define DATA_RATE_1250US      0x00
#define DATA_RATE_2500US      DR0_MASK
#define DATA_RATE_5MS         DR1_MASK
#define DATA_RATE_10MS        (DR1_MASK+DR0_MASK)
#define DATA_RATE_20MS        (DR2_MASK)
#define DATA_RATE_80MS        (DR2_MASK+DR0_MASK)
#define DATA_RATE_160MS       (DR2_MASK+DR1_MASK)
#define DATA_RATE_640MS       (DR2_MASK+DR1_MASK+DR0_MASK)

#define DATA_RATE_800Hz       DATA_RATE_1250US
#define DATA_RATE_400Hz       DATA_RATE_2500US
#define DATA_RATE_200Hz       DATA_RATE_5MS   
#define DATA_RATE_100Hz       DATA_RATE_10MS  
#define DATA_RATE_50Hz        DATA_RATE_20MS  
#define DATA_RATE_12p5Hz      DATA_RATE_80MS  
#define DATA_RATE_6p25Hz      DATA_RATE_160MS 
#define DATA_RATE_1p56Hz      DATA_RATE_640MS 

/*
**  CTRL_REG2 System Control 2 Register
*/
//
#define ST_BIT                Bit._7
#define RST_BIT               Bit._6
#define SMODS1_BIT            Bit._4
#define SMODS0_BIT            Bit._3
#define SLPE_BIT              Bit._2
#define MODS1_BIT             Bit._1
#define MODS0_BIT             Bit._0
//
#define ST_MASK               0x80
#define RST_MASK              0x40
#define SMODS1_MASK           0x10   // FL, correction
#define SMODS0_MASK           0x08   // FL, correction
#define SLPE_MASK             0x04
#define MODS1_MASK            0x02
#define MODS0_MASK            0x01
#define SMODS_MASK            0x18
#define MODS_MASK             0x03

/*
**  CTRL_REG3 Interrupt Control Register
*/
//
#define FIFO_GATE_BIT         Bit._7
#define WAKE_TRANS_BIT        Bit._6
#define WAKE_LNDPRT_BIT       Bit._5
#define WAKE_PULSE_BIT        Bit._4
#define WAKE_FF_MT_1_BIT      Bit._3
#define WAKE_FF_MT_2_BIT      Bit._2
#define IPOL_BIT              Bit._1
#define PP_OD_BIT             Bit._0
//
#define FIFO_GATE_MASK        0x80
#define WAKE_TRANS_MASK       0x40
#define WAKE_LNDPRT_MASK      0x20
#define WAKE_PULSE_MASK       0x10
#define WAKE_FF_MT_1_MASK     0x08
#define WAKE_FF_MT_2_MASK     0x04
#define IPOL_MASK             0x02
#define PP_OD_MASK            0x01

/*
**  CTRL_REG4 Interrupt Enable Register
*/
//
#define INT_EN_ASLP_BIT       Bit._7
#define INT_EN_FIFO_BIT       Bit._6
#define INT_EN_TRANS_BIT      Bit._5
#define INT_EN_LNDPRT_BIT     Bit._4
#define INT_EN_PULSE_BIT      Bit._3
#define INT_EN_FF_MT_BIT      Bit._2
#define INT_EN_DRDY_BIT       Bit._0
//
#define INT_EN_ASLP_MASK      0x80
#define INT_EN_FIFO_MASK      0x40
#define INT_EN_TRANS_MASK     0x20
#define INT_EN_LNDPRT_MASK    0x10
#define INT_EN_PULSE_MASK     0x08
#define INT_EN_FF_MT_1_MASK   0x04
#define INT_EN_DRDY_MASK      0x01

/*
**  CTRL_REG5 Interrupt Configuration Register
*/
//
#define INT_CFG_ASLP_BIT      Bit._7
#define INT_CFG_FIFO_BIT      Bit._6
#define INT_CFG_TRANS_BIT     Bit._5
#define INT_CFG_LNDPRT_BIT    Bit._4
#define INT_CFG_PULSE_BIT     Bit._3
#define INT_CFG_FF_MT_BIT     Bit._2
#define INT_CFG_DRDY_BIT      Bit._0
//
#define INT_CFG_ASLP_MASK     0x80
#define INT_CFG_FIFO_MASK     0x40
#define INT_CFG_TRANS_MASK    0x20
#define INT_CFG_LNDPRT_MASK   0x10
#define INT_CFG_PULSE_MASK    0x08
#define INT_CFG_FF_MT_MASK    0x04
#define INT_CFG_DRDY_MASK     0x01

#endif // MMA8451_PRIVATE_H
#ifdef __cplusplus
}
#endif
