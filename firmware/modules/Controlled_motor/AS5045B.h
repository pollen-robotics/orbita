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

#define Nb_AS5045B_Chip 3 // Daisy Chain list - Nb AS5045B Chip

#define Default_SSI_delay 5000
/// Do not touch below - Default functions tested
#define SSI_Delay AS5045B_Delay(Default_SSI_delay);
#define SSI_SS_High HAL_GPIO_WritePin(AS5045B_SS_GPIO_Port, AS5045B_SS_Pin, GPIO_PIN_SET);
#define SSI_SS_Low HAL_GPIO_WritePin(AS5045B_SS_GPIO_Port, AS5045B_SS_Pin, GPIO_PIN_RESET);
#define SSI_PROG_High HAL_GPIO_WritePin(AS5045B_MOSI_GPIO_Port, AS5045B_MOSI_Pin, GPIO_PIN_SET);
#define SSI_PROG_Low HAL_GPIO_WritePin(AS5045B_MOSI_GPIO_Port, AS5045B_MOSI_Pin, GPIO_PIN_RESET);
#define SSI_CLK_High HAL_GPIO_WritePin(AS5045B_SCK_GPIO_Port, AS5045B_SCK_Pin, GPIO_PIN_SET);
#define SSI_CLK_Low HAL_GPIO_WritePin(AS5045B_SCK_GPIO_Port, AS5045B_SCK_Pin, GPIO_PIN_RESET);

#define AS5045B_SS_Pin GPIO_PIN_15
#define AS5045B_SS_GPIO_Port GPIOA
#define AS5045B_SCK_Pin GPIO_PIN_10
#define AS5045B_SCK_GPIO_Port GPIOC
#define AS5045B_MISO_Pin GPIO_PIN_11
#define AS5045B_MISO_GPIO_Port GPIOC
#define AS5045B_MOSI_Pin GPIO_PIN_12
#define AS5045B_MOSI_GPIO_Port GPIOC

// Absolute Angular Position Bits (18 Bits)
typedef union AbsAng {
    uint32_t Raw;
    struct
    {
        uint32_t Parity : 1;
        uint32_t MagDec : 1;
        uint32_t MagInc : 1;
        uint32_t Lin : 1;
        uint32_t COF : 1;
        uint32_t OCF : 1;
        uint32_t AngPos : 12;
        uint32_t : 14;
    } Bits;
} AbsAng_struct_t;

typedef struct
{
    void (*ReadAngle)(AbsAng_struct_t *);
} AS5045B_functions;

uint8_t ParityCheck(AbsAng_struct_t *Buffer);
extern AS5045B_functions AS5045;

#endif /* INC_AS5045B_H_ */
