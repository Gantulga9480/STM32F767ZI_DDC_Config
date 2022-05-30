/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DDC2_PD2_Pin GPIO_PIN_2
#define DDC2_PD2_GPIO_Port GPIOE
#define DDC2_PD3_Pin GPIO_PIN_3
#define DDC2_PD3_GPIO_Port GPIOE
#define DDC2_PD4_Pin GPIO_PIN_4
#define DDC2_PD4_GPIO_Port GPIOE
#define DDC2_PD5_Pin GPIO_PIN_5
#define DDC2_PD5_GPIO_Port GPIOE
#define DDC2_PD6_Pin GPIO_PIN_6
#define DDC2_PD6_GPIO_Port GPIOE
#define USER_BTN_Pin GPIO_PIN_13
#define USER_BTN_GPIO_Port GPIOC
#define PAR_OUT0_Pin GPIO_PIN_0
#define PAR_OUT0_GPIO_Port GPIOF
#define PAR_OUT1_Pin GPIO_PIN_1
#define PAR_OUT1_GPIO_Port GPIOF
#define PAR_OUT2_Pin GPIO_PIN_2
#define PAR_OUT2_GPIO_Port GPIOF
#define PAR_OUT3_Pin GPIO_PIN_3
#define PAR_OUT3_GPIO_Port GPIOF
#define PAR_OUT4_Pin GPIO_PIN_4
#define PAR_OUT4_GPIO_Port GPIOF
#define PAR_OUT5_Pin GPIO_PIN_5
#define PAR_OUT5_GPIO_Port GPIOF
#define PAR_OUT6_Pin GPIO_PIN_6
#define PAR_OUT6_GPIO_Port GPIOF
#define PAR_OUT7_Pin GPIO_PIN_7
#define PAR_OUT7_GPIO_Port GPIOF
#define PAR_OUT8_Pin GPIO_PIN_8
#define PAR_OUT8_GPIO_Port GPIOF
#define PAR_OUT9_Pin GPIO_PIN_9
#define PAR_OUT9_GPIO_Port GPIOF
#define PAR_OUT10_Pin GPIO_PIN_10
#define PAR_OUT10_GPIO_Port GPIOF
#define DAC_SYNC_Pin GPIO_PIN_0
#define DAC_SYNC_GPIO_Port GPIOC
#define DAC_SCLK_Pin GPIO_PIN_2
#define DAC_SCLK_GPIO_Port GPIOC
#define DAC_DIN_Pin GPIO_PIN_3
#define DAC_DIN_GPIO_Port GPIOC
#define DAC_LDAC_Pin GPIO_PIN_0
#define DAC_LDAC_GPIO_Port GPIOA
#define SBUF_DATA_Pin GPIO_PIN_3
#define SBUF_DATA_GPIO_Port GPIOA
#define OSC_EN_Pin GPIO_PIN_4
#define OSC_EN_GPIO_Port GPIOA
#define SBUF_CLR_Pin GPIO_PIN_5
#define SBUF_CLR_GPIO_Port GPIOA
#define SBUF_SCLK_Pin GPIO_PIN_6
#define SBUF_SCLK_GPIO_Port GPIOA
#define SBUF_LCLK_Pin GPIO_PIN_1
#define SBUF_LCLK_GPIO_Port GPIOB
#define DDC1_SYNC_RCF_Pin GPIO_PIN_14
#define DDC1_SYNC_RCF_GPIO_Port GPIOF
#define DDC1_SYNC_CNC_Pin GPIO_PIN_15
#define DDC1_SYNC_CNC_GPIO_Port GPIOF
#define DDC_CD0_Pin GPIO_PIN_0
#define DDC_CD0_GPIO_Port GPIOG
#define DDC_CD1_Pin GPIO_PIN_1
#define DDC_CD1_GPIO_Port GPIOG
#define DDC2_PD7_Pin GPIO_PIN_7
#define DDC2_PD7_GPIO_Port GPIOE
#define DDC2_PD8_Pin GPIO_PIN_8
#define DDC2_PD8_GPIO_Port GPIOE
#define DDC2_PD9_Pin GPIO_PIN_9
#define DDC2_PD9_GPIO_Port GPIOE
#define DDC2_PD10_Pin GPIO_PIN_10
#define DDC2_PD10_GPIO_Port GPIOE
#define DDC2_PD11_Pin GPIO_PIN_11
#define DDC2_PD11_GPIO_Port GPIOE
#define DDC2_PD12_Pin GPIO_PIN_12
#define DDC2_PD12_GPIO_Port GPIOE
#define DDC2_PD13_Pin GPIO_PIN_13
#define DDC2_PD13_GPIO_Port GPIOE
#define DDC2_PD14_Pin GPIO_PIN_14
#define DDC2_PD14_GPIO_Port GPIOE
#define DDC2_PD15_Pin GPIO_PIN_15
#define DDC2_PD15_GPIO_Port GPIOE
#define SW_B_Pin GPIO_PIN_11
#define SW_B_GPIO_Port GPIOB
#define SW_A_Pin GPIO_PIN_12
#define SW_A_GPIO_Port GPIOB
#define DDC1_PD8_Pin GPIO_PIN_8
#define DDC1_PD8_GPIO_Port GPIOD
#define DDC1_PD9_Pin GPIO_PIN_9
#define DDC1_PD9_GPIO_Port GPIOD
#define DDC1_PD10_Pin GPIO_PIN_10
#define DDC1_PD10_GPIO_Port GPIOD
#define DDC1_PD11_Pin GPIO_PIN_11
#define DDC1_PD11_GPIO_Port GPIOD
#define DDC1_PD12_Pin GPIO_PIN_12
#define DDC1_PD12_GPIO_Port GPIOD
#define DDC1_PD13_Pin GPIO_PIN_13
#define DDC1_PD13_GPIO_Port GPIOD
#define DDC1_PD14_Pin GPIO_PIN_14
#define DDC1_PD14_GPIO_Port GPIOD
#define DDC1_PD15_Pin GPIO_PIN_15
#define DDC1_PD15_GPIO_Port GPIOD
#define DDC_CD2_Pin GPIO_PIN_2
#define DDC_CD2_GPIO_Port GPIOG
#define DDC_CD3_Pin GPIO_PIN_3
#define DDC_CD3_GPIO_Port GPIOG
#define DDC_CD4_Pin GPIO_PIN_4
#define DDC_CD4_GPIO_Port GPIOG
#define DDC_CD5_Pin GPIO_PIN_5
#define DDC_CD5_GPIO_Port GPIOG
#define DDC_CD6_Pin GPIO_PIN_6
#define DDC_CD6_GPIO_Port GPIOG
#define DDC_CD7_Pin GPIO_PIN_7
#define DDC_CD7_GPIO_Port GPIOG
#define DDC1_DV_Pin GPIO_PIN_8
#define DDC1_DV_GPIO_Port GPIOG
#define DDC1_RST_Pin GPIO_PIN_8
#define DDC1_RST_GPIO_Port GPIOC
#define DDC1_CS_Pin GPIO_PIN_9
#define DDC1_CS_GPIO_Port GPIOC
#define DDC1_RDY_Pin GPIO_PIN_8
#define DDC1_RDY_GPIO_Port GPIOA
#define DDC1_RDY_EXTI_IRQn EXTI9_5_IRQn
#define DDC1_SYNC_NCO_Pin GPIO_PIN_9
#define DDC1_SYNC_NCO_GPIO_Port GPIOA
#define DDC2_CS_Pin GPIO_PIN_12
#define DDC2_CS_GPIO_Port GPIOA
#define DDC_A0_Pin GPIO_PIN_10
#define DDC_A0_GPIO_Port GPIOC
#define DDC_A1_Pin GPIO_PIN_11
#define DDC_A1_GPIO_Port GPIOC
#define DDC_A2_Pin GPIO_PIN_12
#define DDC_A2_GPIO_Port GPIOC
#define DDC1_PD0_Pin GPIO_PIN_0
#define DDC1_PD0_GPIO_Port GPIOD
#define DDC1_PD1_Pin GPIO_PIN_1
#define DDC1_PD1_GPIO_Port GPIOD
#define DDC1_PD2_Pin GPIO_PIN_2
#define DDC1_PD2_GPIO_Port GPIOD
#define DDC1_PD3_Pin GPIO_PIN_3
#define DDC1_PD3_GPIO_Port GPIOD
#define DDC1_PD4_Pin GPIO_PIN_4
#define DDC1_PD4_GPIO_Port GPIOD
#define DDC1_PD5_Pin GPIO_PIN_5
#define DDC1_PD5_GPIO_Port GPIOD
#define DDC1_PD6_Pin GPIO_PIN_6
#define DDC1_PD6_GPIO_Port GPIOD
#define DDC1_PD7_Pin GPIO_PIN_7
#define DDC1_PD7_GPIO_Port GPIOD
#define DDC_WR_Pin GPIO_PIN_9
#define DDC_WR_GPIO_Port GPIOG
#define DDC_RD_Pin GPIO_PIN_10
#define DDC_RD_GPIO_Port GPIOG
#define PHY_RESET_Pin GPIO_PIN_12
#define PHY_RESET_GPIO_Port GPIOG
#define PHY_GREEN_LED_Pin GPIO_PIN_14
#define PHY_GREEN_LED_GPIO_Port GPIOG
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#define DDC2_DV_Pin GPIO_PIN_4
#define DDC2_DV_GPIO_Port GPIOB
#define DDC2_RDY_Pin GPIO_PIN_5
#define DDC2_RDY_GPIO_Port GPIOB
#define DDC2_RDY_EXTI_IRQn EXTI9_5_IRQn
#define DDC2_RST_Pin GPIO_PIN_7
#define DDC2_RST_GPIO_Port GPIOB
#define DDC2_PD0_Pin GPIO_PIN_0
#define DDC2_PD0_GPIO_Port GPIOE
#define DDC2_PD1_Pin GPIO_PIN_1
#define DDC2_PD1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define CODER_PORT GPIOF
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
