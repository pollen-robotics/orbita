/*
 * MAX31730.c
 *
 *  Created on: 10 avr. 2020
 *      Author: Cyril SAGONERO
 *		Company: Koncepto (https://koncepto.io)
 */


#include "MAX31730.h"

static void MAX31730_Read(uint8_t , uint8_t *, uint16_t );
static float temp_conv(uint16_t );

MAX31730_functions MAX31730={
		ReadTemp,
//		SetTreshold,
};

static void MAX31730_Read(uint8_t RegAddr, uint8_t *RecBuffer, uint16_t Length){
	HAL_I2C_Master_Transmit(&hi2c3, MAX31730_addr, &RegAddr, 1, 1000); 			// Register to read
	HAL_I2C_Master_Receive(&hi2c3, MAX31730_addr, RecBuffer, Length, 1000);		// Read
}


static float temp_conv(uint16_t raw_temp){
	uint16_t temp = ((((raw_temp) & 0x00FF)<<8) | (((raw_temp)&0xFF00)>>8))>>4;
	volatile float realtemp=(temp&0x01)*0.0625 + \
			((temp&0x02)>>1)*0.125 + \
			((temp&0x04)>>2)*0.25 + \
			((temp&0x08)>>3)*0.5 + \
			((temp&0x10)>>4)*1 + \
			((temp&0x20)>>5)*2 + \
			((temp&0x40)>>6)*4 + \
			((temp&0x80)>>7)*8 + \
			((temp&0x100)>>8)*16 + \
			((temp&0x200)>>9)*32 + \
			((temp&0x400)>>10)*64;
	if(raw_temp&0x800){
		realtemp=0-realtemp;
	}
	return realtemp;
}

void ReadTemp(float *Temp){
	uint16_t temperature[3]={0};
	MAX31730_Read(Remote1TempMSB_Addr,(uint8_t *) &temperature[0], 2);
	MAX31730_Read(Remote2TempMSB_Addr,(uint8_t *) &temperature[1], 2);
	MAX31730_Read(Remote3TempMSB_Addr,(uint8_t *) &temperature[2], 2);
	Temp[0]=temp_conv(temperature[0]);
	Temp[1]=temp_conv(temperature[1]);
	Temp[2]=temp_conv(temperature[2]);
}

