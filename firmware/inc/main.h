/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_dma.h"

#include "stm32g4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOT1_nSLEEP_Pin GPIO_PIN_13
#define MOT1_nSLEEP_GPIO_Port GPIOC
#define MOT1_PMODE_Pin GPIO_PIN_14
#define MOT1_PMODE_GPIO_Port GPIOC
#define HeartBeat_Pin GPIO_PIN_15
#define HeartBeat_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_10
#define RESET_GPIO_Port GPIOG
#define MOT1_EN_Pin GPIO_PIN_0
#define MOT1_EN_GPIO_Port GPIOC
#define MOT1_IPROP_Pin GPIO_PIN_3
#define MOT1_IPROP_GPIO_Port GPIOC
#define ENC1_TIM2_CH1_Pin GPIO_PIN_0
#define ENC1_TIM2_CH1_GPIO_Port GPIOA
#define ENC1_TIM2_CH2_Pin GPIO_PIN_1
#define ENC1_TIM2_CH2_GPIO_Port GPIOA
#define MOT2_IPROP_Pin GPIO_PIN_2
#define MOT2_IPROP_GPIO_Port GPIOA
#define MOT3_IPROP_Pin GPIO_PIN_3
#define MOT3_IPROP_GPIO_Port GPIOA
#define MOT2_nSLEEP_Pin GPIO_PIN_4
#define MOT2_nSLEEP_GPIO_Port GPIOA
#define ENC1_TIM2_ETR_Pin GPIO_PIN_5
#define ENC1_TIM2_ETR_GPIO_Port GPIOA
#define MOT2_nFAULT_Pin GPIO_PIN_6
#define MOT2_nFAULT_GPIO_Port GPIOA
#define MOT2_PMODE_Pin GPIO_PIN_7
#define MOT2_PMODE_GPIO_Port GPIOA
#define MOT3_nFAULT_Pin GPIO_PIN_4
#define MOT3_nFAULT_GPIO_Port GPIOC
#define MOT3_nSLEEP_Pin GPIO_PIN_5
#define MOT3_nSLEEP_GPIO_Port GPIOC
#define MOT3_PMODE_Pin GPIO_PIN_0
#define MOT3_PMODE_GPIO_Port GPIOB
#define ROBUS_VOLTAGE_Pin GPIO_PIN_1
#define ROBUS_VOLTAGE_GPIO_Port GPIOB
#define MOT3_EN_Pin GPIO_PIN_2
#define MOT3_EN_GPIO_Port GPIOB
#define ROBUS_LVL_DOWN_Pin GPIO_PIN_11
#define ROBUS_LVL_DOWN_GPIO_Port GPIOB
#define ROBUS_LVL_UP_Pin GPIO_PIN_12
#define ROBUS_LVL_UP_GPIO_Port GPIOB
#define PTPB_Pin GPIO_PIN_13
#define PTPB_GPIO_Port GPIOB
#define ROBUS_RE_Pin GPIO_PIN_14
#define ROBUS_RE_GPIO_Port GPIOB
#define ROBUS_DE_Pin GPIO_PIN_15
#define ROBUS_DE_GPIO_Port GPIOB
#define MOT2_EN_Pin GPIO_PIN_6
#define MOT2_EN_GPIO_Port GPIOC
#define PTPA_Pin GPIO_PIN_8
#define PTPA_GPIO_Port GPIOA
#define ROBUS_USART1_TX_Pin GPIO_PIN_9
#define ROBUS_USART1_TX_GPIO_Port GPIOA
#define ROBUS_USART1_RX_Pin GPIO_PIN_10
#define ROBUS_USART1_RX_GPIO_Port GPIOA
#define AS5045B_SS_Pin GPIO_PIN_15
#define AS5045B_SS_GPIO_Port GPIOA
#define AS5045B_SCK_Pin GPIO_PIN_10
#define AS5045B_SCK_GPIO_Port GPIOC
#define AS5045B_MISO_Pin GPIO_PIN_11
#define AS5045B_MISO_GPIO_Port GPIOC
#define AS5045B_MOSI_Pin GPIO_PIN_12
#define AS5045B_MOSI_GPIO_Port GPIOC
#define ENC2_TIM3_ETR_Pin GPIO_PIN_2
#define ENC2_TIM3_ETR_GPIO_Port GPIOD
#define ENC3_TIM4_ETR_Pin GPIO_PIN_3
#define ENC3_TIM4_ETR_GPIO_Port GPIOB
#define ENC2_TIM3_CH1_Pin GPIO_PIN_4
#define ENC2_TIM3_CH1_GPIO_Port GPIOB
#define ENC2_TIM3_CH2_Pin GPIO_PIN_5
#define ENC2_TIM3_CH2_GPIO_Port GPIOB
#define ENC3_TIM4_CH1_Pin GPIO_PIN_6
#define ENC3_TIM4_CH1_GPIO_Port GPIOB
#define ENC3_TIM4_CH2_Pin GPIO_PIN_7
#define ENC3_TIM4_CH2_GPIO_Port GPIOB
#define BOOT0_Pin GPIO_PIN_8
#define BOOT0_GPIO_Port GPIOB
#define MOT1_nFAULT_Pin GPIO_PIN_9
#define MOT1_nFAULT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
