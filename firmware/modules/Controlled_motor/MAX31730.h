/*
 * MAX31730.h
 *
 *  Created on: 10 avr. 2020
 *      Author: Cyril SAGONERO
 *		Company: Koncepto (https://koncepto.io)
 */

#ifndef INC_MAX31730_H_
#define INC_MAX31730_H_
#include "main.h"
#include "i2c.h"

typedef struct {
	void (*Read) (float *);
//	void (*SetThr) (float *);
}MAX31730_functions;

extern MAX31730_functions MAX31730;

#define MAX31730_addr 0x38

// MAX31730 registers address
#define LocalTempMSB_Addr   0x00
#define LocalTempLSB_Addr   0x01
#define Remote1TempMSB_Addr 0x02
#define Remote1TempLSB_Addr 0x03
#define Remote2TempMSB_Addr 0x04
#define Remote2TempLSB_Addr 0x05
#define Remote3TempMSB_Addr 0x06
#define Remote3TempLSB_Addr 0x07
#define HighestTempMSB_Addr 0x10
#define HighestTempLSB_Addr 0x11
#define HighestTempEn_Addr  0x12
#define Configuration_Addr  0x13
#define CustomIdFact_Addr   0x14
#define CustomIdEn_Addr     0x15
#define CustomOffset_Addr   0x16
#define CustomOffsetEn_Addr 0x17
#define FilterEnable_Addr   0x18
#define BetaCompEnable_Addr 0x19
#define BetaValueChan1_Addr 0x1A
#define BetaValueChan2_Addr 0x1B
#define BetaValueChan3_Addr 0x1C
#define LocT_HighL_MSB_Addr 0x20
#define LocT_HighL_LSB_Addr 0x21
#define R1T_HiLimitMSB_Addr 0x22
#define R1L_HiLimitLSB_Addr 0x23
#define R2T_HiLimitMSB_Addr 0x24
#define R2L_HiLimitLSB_Addr 0x25
#define R3T_HiLimitMSB_Addr 0x26
#define R3L_HiLimitLSB_Addr 0x27
#define T_LowLimitMSB_Addr  0x30
#define T_LowLimitLSB_Addr  0x31
#define T_StatusHigh_Addr   0x32
#define T_StatusLow_Addr    0x33
#define ThermPinMask_Addr   0x34
#define TempChanEn_Addr     0x35
#define DiodeFaultStat_Addr 0x36
#define LocRefTempMSB_Addr  0x40
#define LocRefTempLSB_Addr  0x41
#define Rem1RefTempMSB_Addr 0x42
#define Rem1RefTempLSB_Addr 0x43
#define Rem2RefTempMSB_Addr 0x44
#define Rem2RefTempLSB_Addr 0x45
#define Rem3RefTempMSB_Addr 0x46
#define Rem3RefTempLSB_Addr 0x47
#define ManufacturerID_Addr 0x50
#define RevisionCode_Addr   0x51

void ReadTemp(float *);

#endif /* INC_MAX31730_H_ */
