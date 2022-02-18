/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

UART_HandleTypeDef huart1;


/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = RS485_TX_Pin|RS485_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, RS485_TX_Pin|RS485_RX_Pin);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

// void MX_USART1_UART_Init(void)
// {
//   LL_USART_InitTypeDef USART_InitStruct = {0};

//   LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

//   /* Peripheral clock enable */
//   LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
//   LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
//   /**USART1 GPIO Configuration  
//   PA9   ------> USART1_TX
//   PA10   ------> USART1_RX 
//   */
//   GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
//   GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//   GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
//   LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
//   GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
//   GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
//   GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
//   GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
//   GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
//   LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//   /* USART1 interrupt Init */
//   NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
//   NVIC_EnableIRQ(USART1_IRQn);

//   USART_InitStruct.PrescalerValue = LL_USART_PRESCALER_DIV1;
//   USART_InitStruct.BaudRate = 1000000;
//   USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
//   USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
//   USART_InitStruct.Parity = LL_USART_PARITY_NONE;
//   USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
//   USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
//   USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
//   LL_USART_Init(USART1, &USART_InitStruct);
//   LL_USART_SetTXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
//   LL_USART_SetRXFIFOThreshold(USART1, LL_USART_FIFOTHRESHOLD_1_8);
//   LL_USART_DisableFIFO(USART1);
//   LL_USART_ConfigAsyncMode(USART1);

//   /* USER CODE BEGIN WKUPType USART1 */

//   /* USER CODE END WKUPType USART1 */

//   LL_USART_Enable(USART1);

//   /* Polling USART1 initialisation */
//   while((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))))
//   {
//   }

// }

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
