/*
 * AS5045B.c
 *
 *  Created on: Mar 16, 2020
 *      Author: Cyril SAGONERO
 *		Company: Koncepto (https://koncepto.io)
 */

#include "AS5045B.h"

static void AngRead(AbsAng_struct_t *);

AS5045B_functions AS5045 = {
    AngRead,
};

static void AS5045B_Delay(volatile uint32_t);
static void SSI_MOSI_HighImpedance(void);
static void SSI_MOSI_LowImpedance(void);
static void SSI_ClockPulses(uint8_t nbClock);
static uint8_t SSI_SingleClockPulse_Read(GPIO_TypeDef *, uint16_t);
static void SSI_SingleClockPulse_Write(GPIO_TypeDef *, uint16_t, uint8_t);

// SSI operation functions
static void SetupCondition(void);
static void exitCondition(void);
static void operationModeRead(void);
static void operationModeWrite(void);
//static void operationModeProg(void);

// Utilities functions
static uint16_t ReverseBits(uint32_t);

uint8_t ParityCheck(AbsAng_struct_t *Buffer)
{
    uint8_t parity_bit = 0;
    AbsAng_struct_t LocalAbsAng = {0};
    LocalAbsAng.Raw = Buffer->Raw;
    for (uint8_t Nbits = 18 - 2; Nbits != 0; Nbits--)
    {
        LocalAbsAng.Raw >>= 1;
        parity_bit = Buffer->Raw & 0x01;
    }
    if (parity_bit != Buffer->Bits.Parity)
    {
        return 0;
    }
    return 1;
}

/*
 *  AS5045B Read Absolute Angular Position Data
 */
static void AngRead(AbsAng_struct_t *Buffer)
{
    SSI_SS_Low;
    SSI_Delay;
    SSI_CLK_Low;
    SSI_Delay;
    for (uint8_t nb = 0; nb < 3; nb++)
    {
        Buffer->Raw = 0;
        for (uint8_t i = 0; i < 18; i++)
        {
            Buffer->Raw |= SSI_SingleClockPulse_Read(AS5045B_MISO_GPIO_Port, AS5045B_MISO_Pin);
            Buffer->Raw <<= 1;
        }
        SSI_SingleClockPulse_Read(AS5045B_MISO_GPIO_Port, AS5045B_MISO_Pin); // Pulse
        Buffer++;
    }
    SSI_CLK_High;
    SSI_Delay;
    SSI_SS_High;
    SSI_Delay;
    return;
}

static void SSI_MOSI_LowImpedance(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AS5045B_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void SSI_MOSI_HighImpedance(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = AS5045B_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

static void SSI_ClockPulses(uint8_t nbClock)
{
    do
    {
        SSI_CLK_High;
        SSI_Delay;
        SSI_CLK_Low;
        SSI_Delay;
        nbClock--;
    } while (nbClock != 0);
}

static void AS5045B_Delay(volatile uint32_t time)
{
    do
    {
        time--;
    } while (time != 0);
}

static uint8_t SSI_SingleClockPulse_Read(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
    uint8_t Value = 0;
    SSI_CLK_High;
    SSI_Delay;
    SSI_CLK_Low;
    Value = HAL_GPIO_ReadPin(GPIOx, GPIO_Pin);
    SSI_Delay;
    return Value;
}
