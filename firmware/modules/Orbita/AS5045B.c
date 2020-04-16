/*
 * AS5045B.c
 *
 *  Created on: Mar 16, 2020
 *      Author: Cyril SAGONERO
 *		Company: Koncepto (https://koncepto.io)
 */

#include "AS5045B.h"

static void AngRead(AbsAng_struct_t *, uint8_t);
static void OTPRead(OTP_struct_t *, uint8_t);
static void OTPWrite(OTP_struct_t *, uint8_t);
static void ZeroCal(ProgMode );


AS5045B_functions AS5045={
		OTPRead,
		OTPWrite,
		AngRead,
		ZeroCal,
};

#ifndef DEBUG
static void debug_pop(void);
#endif

static void AS5045B_Delay(uint32_t);
static void SSI_MOSI_HighImpedance(void);
static void SSI_MOSI_LowImpedance(void);
static void SSI_ClockPulses(uint8_t nbClock);
static uint8_t SSI_SingleClockPulse_Read(GPIO_TypeDef *,uint16_t );
static void SSI_SingleClockPulse_Write(GPIO_TypeDef *, uint16_t , uint8_t );

// SSI operation functions
static void SetupCondition(void);
static void exitCondition(void);
static void operationModeRead(void);
static void operationModeWrite(void);
//static void operationModeProg(void);

// Utilities functions
static uint8_t ParityCheck(AbsAng_struct_t *);
static uint16_t ReverseBits(uint32_t );


static void ZeroCal(ProgMode Perm){
	OTP_struct_t TempOTP[Nb_AS5045B_Chip]={0};
	AbsAng_struct_t TempAbs[Nb_AS5045B_Chip]={0};
	uint8_t GlobalParity=1;
	do{
		AngRead(TempAbs,Nb_AS5045B_Chip);							// Read Abs Angle of all chip
		for(uint8_t NbChip=Nb_AS5045B_Chip; NbChip!=0; NbChip--){
			GlobalParity&=ParityCheck(&TempAbs[NbChip-1]);			// Check parity of all chip
		}
	}while(!GlobalParity);
	AS5045.OTPRead(TempOTP,Nb_AS5045B_Chip);
	for(uint8_t NbChip=Nb_AS5045B_Chip; NbChip!=0; NbChip--){		// Set Zero pos if all chip
		TempOTP[NbChip-1].OTP.ZeroPos=ReverseBits(TempAbs[NbChip-1].Bits.AngPos);
		TempOTP[NbChip-1].OTP.pwmDIS=1;
	}
	AS5045.OTPWrite(TempOTP,Nb_AS5045B_Chip);						// Write Zero pos into OTP register
	AS5045.OTPRead(TempOTP,Nb_AS5045B_Chip);
}

static uint8_t ParityCheck(AbsAng_struct_t *Buffer){
	uint8_t parity_bit=0;
	AbsAng_struct_t LocalAbsAng={0};
	LocalAbsAng.Raw=Buffer->Raw;
	for(uint8_t Nbits=18-2; Nbits!=0 ; Nbits--){
		LocalAbsAng.Raw>>=1;
		parity_bit=Buffer->Raw & 0x01;
	}
	if(parity_bit!=Buffer->Bits.Parity){
		return 0;
	}
	return 1;
}

/*
 *  AS5045B Read Absolute Angular Position Data
 */
static void AngRead(AbsAng_struct_t *Buffer, uint8_t NbChip){
	Buffer->Raw=0;
	SSI_SS_Low;
	for(uint8_t Chip=NbChip; Chip!=0 ; Chip--){
		SSI_Delay;
		SSI_CLK_Low;
		SSI_Delay;
		for(uint8_t i=(18-1) ; i!=0 ; i--){
			Buffer->Raw |= SSI_SingleClockPulse_Read(AS5045B_MISO_GPIO_Port, AS5045B_MISO_Pin);
			Buffer->Raw <<=1;
		}
		SSI_CLK_High;
		SSI_Delay;
		Buffer->Raw |= HAL_GPIO_ReadPin(AS5045B_MISO_GPIO_Port, AS5045B_MISO_Pin);
#ifndef DEBUG
		debug_pop();
#endif
		SSI_CLK_Low;
		SSI_Delay;
		if(NbChip != 1){
			SSI_ClockPulses(1);
		}
		Buffer++;
		Buffer->Raw=0;
	}
	SSI_SS_High;
	SSI_Delay;
	return;
}

/*
 * AS5045B On Time Programming Register Write
 */
static void OTPWrite(OTP_struct_t *Buffer, uint8_t NbChip){
	SetupCondition();
	operationModeWrite();
	SSI_Delay;
	for(uint8_t Chip=NbChip; Chip !=0; Chip--){
		for(uint8_t i=54 ; i!=0 ; i--){
			SSI_SingleClockPulse_Write(AS5045B_MOSI_GPIO_Port, AS5045B_MOSI_Pin, (Buffer->Raw>>(i-1))&0x01); // Pulse clock + Read bit
		}
		Buffer++;
	}
	SSI_ClockPulses(1);		// Data Latched
	exitCondition();
}

/*
 * AS5045B On Time Programming Register Read
 */
static void OTPRead(OTP_struct_t *Buffer, uint8_t NbChip){
	SetupCondition();
	operationModeRead();
	for(uint8_t Chip=NbChip; Chip!=0; Chip--){
		Buffer->Raw=0;
		for(uint8_t i=54 ; i!=0 ; i--){
			Buffer->Raw |= SSI_SingleClockPulse_Read(AS5045B_MOSI_GPIO_Port, AS5045B_MOSI_Pin);
			if(i>1){
				Buffer->Raw<<=1;
			}
		}
		if(Chip!=NbChip){
			Buffer->Raw<<=1;
			Buffer->Raw |= SSI_SingleClockPulse_Read(AS5045B_MOSI_GPIO_Port, AS5045B_MOSI_Pin);
		}
		Buffer++;
		SSI_Delay;
	}
	exitCondition();
}

static void SetupCondition(void){
	SSI_CLK_Low;
	SSI_Delay;
	SSI_SS_Low;
	SSI_Delay;
	SSI_PROG_High;
	SSI_Delay;
	SSI_SS_High;
	SSI_Delay;
	SSI_SS_Low;
	SSI_Delay;
	SSI_CLK_High;
	SSI_Delay;
	SSI_CLK_Low;
	SSI_Delay;
	return;
}

static void exitCondition(void){
	SSI_MOSI_LowImpedance();
	SSI_Delay;
	SSI_SS_Low;
	SSI_Delay;
	SSI_CLK_High;
	SSI_Delay;
	SSI_CLK_Low;
	SSI_Delay;
	SSI_CLK_High;
	SSI_Delay;
	SSI_SS_High;
	SSI_Delay;
	SSI_PROG_Low;
	SSI_Delay;
}

//static void operationModeLoad(void){
//	SSI_PROG_Low;
//	SSI_Delay;
//	SSI_SS_High;
//	SSI_Delay;
//	SSI_ClockPulses(4);
//	return;
//}

static void operationModeRead(void){
	SSI_PROG_Low;
	SSI_Delay;
	SSI_CLK_High;
	SSI_Delay;
	SSI_SS_High;
	SSI_Delay;
	SSI_CLK_Low;
	SSI_Delay;
	SSI_ClockPulses(1);
	SSI_MOSI_HighImpedance();
	return;
}

static void operationModeWrite(void){
	SSI_SS_High;
	SSI_Delay;
	SSI_ClockPulses(3); //
}

//static void operationModeProg(void){
//	SSI_CLK_High;
//	SSI_Delay;
//	SSI_SS_High;
//	SSI_Delay;
//	SSI_CLK_Low;
//	SSI_Delay;
//	SSI_ClockPulses(4);
//}

static void SSI_MOSI_LowImpedance(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = AS5045B_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void SSI_MOSI_HighImpedance(void){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = AS5045B_MOSI_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void SSI_ClockPulses(uint8_t nbClock){
	do{
		SSI_CLK_High;
		SSI_Delay;
		SSI_CLK_Low;
		SSI_Delay;
		nbClock--;
	}while(nbClock !=0);
}

static void AS5045B_Delay(uint32_t time){
	do{
		time--;
	}while(time!=0);
}

static void SSI_SingleClockPulse_Write(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin, uint8_t value){
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, value);
#ifndef DEBUG
	debug_pop();
#endif
	SSI_Delay;
	SSI_CLK_High;	// Rising Edge Validation
	SSI_Delay;
	SSI_CLK_Low;
	SSI_Delay;
}

static uint8_t SSI_SingleClockPulse_Read(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin){
	uint8_t Value=0;
	SSI_CLK_High;
	SSI_Delay;
	SSI_CLK_Low;
	Value = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
#ifndef DEBUG
	debug_pop();
#endif
	SSI_Delay;
	return Value;
}

static uint16_t ReverseBits(uint32_t data){
	volatile uint8_t tab[2]="";
	volatile uint8_t Zero=0;
	volatile uint8_t temp=0;
	volatile uint16_t *result=NULL;

	temp=(data>>8)&0x0F;
	for(uint8_t i=7; i!=0 ; i--){
		Zero|=(temp & 0x1);
		Zero<<=1;
		temp>>=1;
	}
	Zero|=(temp & 0x1);
	tab[0]=Zero;
	Zero=0;
	temp=data&0xFF;
	for(uint8_t i=7; i!=0 ; i--){
		Zero|=(temp & 0x1);
		Zero<<=1;
		temp>>=1;
	}
	Zero|=(temp & 0x1);
	tab[1]=Zero;
	result=(uint16_t *)tab;
	return (*result>>4);
}

#ifndef DEBUG
static void debug_pop (void){
	volatile uint8_t delai=40;
	HAL_GPIO_TogglePin(POP_GPIO_Port, POP_Pin);
	do{
		delai--;
	}while(delai);
	HAL_GPIO_TogglePin(POP_GPIO_Port, POP_Pin);
}
#endif
