/*
 * AS5045B.h
 *
 *  Created on: Mar 16, 2020
 *      Author: Cyril SAGONERO
 *		Company: Koncepto (https://koncepto.io)
 */

#ifndef INC_AS5045B_H_
#define INC_AS5045B_H_

#include "main.h"

#define Nb_AS5045B_Chip	3		// Daisy Chain list - Nb AS5045B Chip

// Do not touch below - Default functions tested

#define Default_SSI_delay	50000

#define SSI_Delay AS5045B_Delay(Default_SSI_delay);
#define SSI_SS_High HAL_GPIO_WritePin(AS5045B_SS_GPIO_Port, AS5045B_SS_Pin, GPIO_PIN_SET);
#define SSI_SS_Low HAL_GPIO_WritePin(AS5045B_SS_GPIO_Port, AS5045B_SS_Pin, GPIO_PIN_RESET);
#define SSI_PROG_High HAL_GPIO_WritePin(AS5045B_MOSI_GPIO_Port, AS5045B_MOSI_Pin, GPIO_PIN_SET);
#define SSI_PROG_Low HAL_GPIO_WritePin(AS5045B_MOSI_GPIO_Port, AS5045B_MOSI_Pin, GPIO_PIN_RESET);
#define SSI_CLK_High HAL_GPIO_WritePin(AS5045B_SCK_GPIO_Port, AS5045B_SCK_Pin, GPIO_PIN_SET);
#define SSI_CLK_Low HAL_GPIO_WritePin(AS5045B_SCK_GPIO_Port, AS5045B_SCK_Pin, GPIO_PIN_RESET);

typedef enum{
	Volatile=0,
	Permanent,
}ProgMode;

// Absolute Angular Position Bits (18 Bits)
typedef union AbsAng{
	uint32_t Raw;
	struct{
		uint32_t Parity 	:1;
		uint32_t MagDec 	:1;
		uint32_t MagInc 	:1;
		uint32_t Lin		:1;
		uint32_t COF		:1;
		uint32_t OCF		:1;
		uint32_t AngPos 	:12;
		uint32_t			:14;
	}Bits;
}AbsAng_struct_t;


typedef union OTP_type{
	uint64_t Raw;
	struct {
		uint64_t M0				 :1;
		uint64_t ChipID          :18;
		uint64_t FactoryBit      :11;
		uint64_t RA 		     :5;
		uint64_t CCW             :1;
		uint64_t ZeroPos         :12;
		uint64_t reserved        :2;
		uint64_t pwmDIS          :1;
		uint64_t MagCompEn       :1;
		uint64_t PWMHalfEN_Index :1;
		uint64_t M1				 :1;
		uint64_t				 :10;
	}OTP;
}OTP_struct_t;

typedef struct {
	void (*OTPRead) (OTP_struct_t *, uint8_t);
	void (*OTPWrite) (OTP_struct_t *, uint8_t);
	void (*ReadAngle) (AbsAng_struct_t *, uint8_t);
	void (*ZeroCal) (ProgMode);
}AS5045B_functions;

#endif /* INC_AS5045B_H_ */
