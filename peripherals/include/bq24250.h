#ifndef __BQ24250_H__
#define __BQ24250_H__

#include <stdint.h>
#include <stdbool.h>

#define BQ24250_I2C_ADDRESS 0x6a

/*register #1*/
#define BQ24250_WD_MASK 0x40
#define BQ24250_WD_SHIFT 06

#define BQ24250_CHARGE_MASK 0x30
#define BQ24250_CHARGE_SHIFT 0x4

/*
#define BQ24250_CHARGE_READY 0
#define BQ24250_CHARGE_PROGRESS 1
#define BQ24250_CHARGE_DONE 2
#define BQ24250_CHARGE_FAULT 3
*/

#define BQ24250_FAULT_MASK 0x0f
#define BQ24250_FAULT_SHIFT 0x0

/*register #2*/
#define BQ24250_RESET_SHIFT 7
#define BQ24250_RESET_MASK 0x80
#define BQ24250_ILIMIT_SHIFT 4
#define BQ24250_ILIMIT_MASK 0x70
#define BQ24250_CHARGE_ENABLE_SHIFT 1
#define BQ24250_CHARGE_ENABLE_MASK 0x02
#define BQ24250_HZ_MODE_SHIFT 0
#define BQ24250_HZ_MODE_MAST 0x1

/*register #3*/
#define BQ24250_VBATREG_5  0x80 /*640mv*/
#define BQ24250_VBATREG_4  0x40 /*320mv*/
#define BQ24250_VBATREG_3  0x20 /*160mv*/
#define BQ24250_VBATREG_2  0x10 /*80mv*/
#define BQ24250_VBATREG_1  0x08 /*40mv*/
#define BQ24250_VBATREG_0  0x04 /*20mv*/

#define BQ24250_VBATREG_DEFAULT_VALUE 350
#define BQ24250_VBATREG_5_VALUE  64 /*640mv*/
#define BQ24250_VBATREG_4_VALUE  32 /*320mv*/
#define BQ24250_VBATREG_3_VALUE  16 /*160mv*/
#define BQ24250_VBATREG_2_VALUE  8 /*80mv*/
#define BQ24250_VBATREG_1_VALUE  4 /*40mv*/
#define BQ24250_VBATREG_0_VALUE  2 /*20mv*/

#define BQ24250_VBATREG_MASK 0xfc
#define BQ24250_USB_DET_MASK 0x03

/* register #4*/
#define BQ24250_ITERM_MASK     0x07
#define BQ24250_ITERM_SHIFT    0
#define BQ24250_ITERM0         0x01 /*Limit 25ma*/
#define BQ24250_ITERM1         0x02 /*Limit 50ma*/
#define BQ24250_ITERM2         0x04 /*Limit 100ma*/
#define BQ24250_ICHG_MASK     0xF8
#define BQ24250_ICHG_SHIFT    3
#define BQ24250_ICHG_4         0x10 /*max 800ma*/
#define BQ24250_ICHG_3         0x08 /*max 400ma*/
#define BQ24250_ICHG_2         0x04 /*max 200ma*/
#define BQ24250_ICHG_1         0x02 /*max 100ma*/
#define BQ24250_ICHG_0         0x01 /*max 50ma*/

/* register #5*/
#define BQ24250_LOW_CHG         0x20

/* register #6*/
#define BQ24250_SYSOFF_SHIFT    4
#define BQ24250_SYSOFF_MASK     0x10
#define BQ24250_TSEN_SHIFT      3
#define BQ24250_TSEN_MASK       0x08

/*memory location*/
#define BQ24250_STAT_REG 0x00
#define BQ24250_CHARGE_REG 0x01
#define BQ24250_VOLTAGE_REG 0x02
#define BQ24250_CURRENT_REG 0x03
#define BQ24250_REG5 0x04
#define BQ24250_REG6 0x05
#define BQ24250_MODE_REG 0x06

typedef enum
{
    BQ2425StateReady = 0x00,
    BQ2425StateChargeProgress,
    BQ2425StateChargeDone,
    BQ2425StateFault,
    
    /* --- */
    BQ2425State_I2CError,
}BQ2425State_e;

typedef enum
{
    BQ2425StFaultReason_Normal = 0x00,
    BQ2425StFaultReason_InputOVP,
    BQ2425StFaultReason_InputUVLO,
    BQ2425StFaultReason_Sleep,
    BQ2425StFaultReason_BattTempFault,
    BQ2425StFaultReason_BattOVP,
    BQ2425StFaultReason_ThermalShutdown,
    BQ2425StFaultReason_TimerFault,
    BQ2425StFaultReason_NoBattConnect,
    BQ2425StFaultReason_ISETShort,
    BQ2425StFaultReason_InputFault_LDOLow,
    
    /* --- */
    BQ2425StFaultReason_I2CError,
}BQ2425StFaultReason_e;

typedef enum
{
    BQ24250Current100MA = 0x00,
    BQ24250Current150MA,
    BQ24250Current500MA,
    BQ24250Current900MA,
    BQ24250Current1500MA,
    BQ24250Current2000MA,
    BQ24250CurrentEXTERNAL,
    BQ24250CurrentPTM
}BQ24250CURRENT_LIMIT_e;


/*charge in return true
  charge out reutrn false*/
BQ2425State_e BQ24250CheckCharge(void);
void BQ24250Init(void);

/*
 * Get fault reason if get BQ2425StateFault
 */
BQ2425StFaultReason_e BQ24250GetFaultReason(void);

/*  1(true) is enable HZ
    0(false) is disable HZ*/
void BQ24250HighImpedance(bool state);

/*  
    voltage is value * 100
    example: 3.5 voltage set 350
    miniature 3.5 voltage
    step 0.20v
    maximum 4.76 voltage*/
bool BQ24250SetChargeVoltage(uint16_t voltage);

/*  1(true) is enable SYSOFF
    0(false) is disable SYSOFF*/
void BQ24250SYSOFF(bool state);
uint8_t BQ24250GetSYSOFF(void);

/* reference BQ24250CURRENT_LIMIT_e*/
void BQ24250SetLimitCurrent(BQ24250CURRENT_LIMIT_e value);

/*  1(true) is enable watch dog
    0(false) is disable watch dog*/
void BQ24250EnableWD(bool state);

/* reference BQ24250CURRENT_LIMIT_e*/
void BQ24250SetChargeCurrent(uint8_t value);

/*
set value:
example:BQ24250_ITERM0|BQ24250_ITERM1*/
void BQ24250SetIterm(uint8_t value);

/*
set value:
example:BQ24250_ICHG_0|BQ24250_ICHG_1*/
void BQ24250SetIchg(uint8_t value);

/*  1(true) is disable charge
    0(false) is enable charge*/
void BQ24250DisableCharge(uint8_t state);

/*
    set TS_EN value
 */
void BQ24250SetTSEN(uint8_t value);


/*!
 * Read TS_EN value
 * Author: Crux
 * \param [IN]      N/A
 * \param [OUT]     N/A
 * \param [return]  TS_EN value
 */
uint8_t BQ24250GetTSEN(void);

#endif  // __BQ24250_H__
