/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "udp_server.h"
#include "ddc.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UDP_SEND_PORT 10
#define UDP_RECEIVE_PORT 11

#define BUFFER_COUNT 8
#define BUFFER_SIZE 500 // 1000 byte
#define HEADER_SIZE 2   // 4 byte
#define FOOTER_SIZE 2   // 4 byte
#define PACKET_SIZE (BUFFER_SIZE + HEADER_SIZE + FOOTER_SIZE)*2 // 16 bit size to 8 bit size

#define HEADER (('B' << 8) + 'A')    // ABAB
#define FOOTER (('D' << 8) + 'C')    // CDCD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim1_ch3;

/* USER CODE BEGIN PV */

struct IP4_Container udp_ip = {10, 3, 4, 28}; // 10.3.4.28:UDP_SEND_PORT

uint8_t dbuf_index = 0;

int16_t DDC_Buffer1[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer2[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer3[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer4[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer5[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer6[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer7[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];
int16_t DDC_Buffer8[HEADER_SIZE + BUFFER_SIZE + FOOTER_SIZE];

int16_t *buffers[] = {DDC_Buffer1, DDC_Buffer2, DDC_Buffer3, DDC_Buffer4, DDC_Buffer5, DDC_Buffer6, DDC_Buffer7, DDC_Buffer8};

int16_t coef3[128] = {
1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 9, 10, 10, 10, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 10, 10, 10, 9, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 5, 5, 5, 5, 4, 4, 4, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1,
}; // 1000

int16_t coef2[128] = {
0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -1, -1, -1, 0, 1, 1, 1, 1, 0, -1, -1, -2, -1, 0, 1, 2, 2, 1, -1, -3, -4, -4, -2, 2, 7, 13, 17, 20, 20, 17, 13, 7, 2, -2, -4, -4, -3, -1, 1, 2, 2, 1, 0, -1, -2, -1, -1, 0, 1, 1, 1, 1, 0, -1, -1, -1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
}; // 100

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */
void init_buffer();
uint16_t get_addr(uint8_t *s, int16_t start);
uint64_t get_value(uint8_t *s, int16_t start);
void DDC_Config_Init();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */

    /* ---------------------------------------------------- SETUP START */

	init_buffer();

	/* ---------------------------------------------------- ETH START */
	USR_UDP_Init(udp_ip, UDP_SEND_PORT, UDP_RECEIVE_PORT);
	/* ---------------------------------------------------- ETH END   */

	/* ---------------------------------------------------- DDC START */
	HAL_GPIO_WritePin(GPIOA, OSC_EN_Pin, GPIO_PIN_RESET);

	/* Switch */
	HAL_GPIO_WritePin(GPIOB, SW_A_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, SW_B_Pin, GPIO_PIN_SET);

	/* ADC, DDC Clock 5MHz */
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_1);

	DDC_Config_Init();

	/* DMA start IC */
	/* stm32f7xx_hal_tim.c 2405 */
	if (HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)(buffers[dbuf_index] + HEADER_SIZE), BUFFER_SIZE) != HAL_OK) for(;;);
	else dbuf_index++;
	/* ---------------------------------------------------- DDC END */

	HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
	/* ---------------------------------------------------- SETUP END */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	MX_LWIP_Process();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 3-1;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, PAR_OUT0_Pin|PAR_OUT1_Pin|PAR_OUT2_Pin|PAR_OUT3_Pin
                          |PAR_OUT4_Pin|PAR_OUT5_Pin|PAR_OUT6_Pin|PAR_OUT7_Pin
                          |PAR_OUT8_Pin|PAR_OUT9_Pin|PAR_OUT10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DAC_SYNC_Pin|DAC_SCLK_Pin|DAC_DIN_Pin|DDC1_RST_Pin
                          |DDC1_CS_Pin|DDC_A0_Pin|DDC_A1_Pin|DDC_A2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DAC_LDAC_Pin|SBUF_DATA_Pin|OSC_EN_Pin|SBUF_CLR_Pin
                          |SBUF_SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SBUF_LCLK_Pin|SW_B_Pin|SW_A_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DDC_CD0_Pin|DDC_CD1_Pin|DDC_CD2_Pin|DDC_CD3_Pin
                          |DDC_CD4_Pin|DDC_CD5_Pin|DDC_CD6_Pin|DDC_CD7_Pin
                          |DDC_WR_Pin|DDC_RD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_BTN_Pin */
  GPIO_InitStruct.Pin = USER_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAR_OUT0_Pin PAR_OUT1_Pin PAR_OUT2_Pin PAR_OUT3_Pin
                           PAR_OUT4_Pin PAR_OUT5_Pin PAR_OUT6_Pin PAR_OUT7_Pin
                           PAR_OUT8_Pin PAR_OUT9_Pin PAR_OUT10_Pin */
  GPIO_InitStruct.Pin = PAR_OUT0_Pin|PAR_OUT1_Pin|PAR_OUT2_Pin|PAR_OUT3_Pin
                          |PAR_OUT4_Pin|PAR_OUT5_Pin|PAR_OUT6_Pin|PAR_OUT7_Pin
                          |PAR_OUT8_Pin|PAR_OUT9_Pin|PAR_OUT10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_SYNC_Pin DAC_SCLK_Pin DAC_DIN_Pin DDC1_RST_Pin
                           DDC1_CS_Pin DDC_A0_Pin DDC_A1_Pin DDC_A2_Pin */
  GPIO_InitStruct.Pin = DAC_SYNC_Pin|DAC_SCLK_Pin|DAC_DIN_Pin|DDC1_RST_Pin
                          |DDC1_CS_Pin|DDC_A0_Pin|DDC_A1_Pin|DDC_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DAC_LDAC_Pin SBUF_DATA_Pin OSC_EN_Pin SBUF_CLR_Pin
                           SBUF_SCLK_Pin */
  GPIO_InitStruct.Pin = DAC_LDAC_Pin|SBUF_DATA_Pin|OSC_EN_Pin|SBUF_CLR_Pin
                          |SBUF_SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SBUF_LCLK_Pin SW_B_Pin SW_A_Pin LED_Pin */
  GPIO_InitStruct.Pin = SBUF_LCLK_Pin|SW_B_Pin|SW_A_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DDC1_SYNC_RCF_Pin DDC1_SYNC_CNC_Pin */
  GPIO_InitStruct.Pin = DDC1_SYNC_RCF_Pin|DDC1_SYNC_CNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : DDC_CD0_Pin DDC_CD1_Pin DDC_CD2_Pin DDC_CD3_Pin
                           DDC_CD4_Pin DDC_CD5_Pin DDC_CD6_Pin DDC_CD7_Pin
                           DDC_WR_Pin DDC_RD_Pin */
  GPIO_InitStruct.Pin = DDC_CD0_Pin|DDC_CD1_Pin|DDC_CD2_Pin|DDC_CD3_Pin
                          |DDC_CD4_Pin|DDC_CD5_Pin|DDC_CD6_Pin|DDC_CD7_Pin
                          |DDC_WR_Pin|DDC_RD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : DDC1_PD8_Pin DDC1_PD9_Pin DDC1_PD10_Pin DDC1_PD11_Pin
                           DDC1_PD12_Pin DDC1_PD13_Pin DDC1_PD14_Pin DDC1_PD15_Pin
                           DDC1_PD0_Pin DDC1_PD1_Pin DDC1_PD2_Pin DDC1_PD3_Pin
                           DDC1_PD4_Pin DDC1_PD5_Pin DDC1_PD6_Pin DDC1_PD7_Pin */
  GPIO_InitStruct.Pin = DDC1_PD8_Pin|DDC1_PD9_Pin|DDC1_PD10_Pin|DDC1_PD11_Pin
                          |DDC1_PD12_Pin|DDC1_PD13_Pin|DDC1_PD14_Pin|DDC1_PD15_Pin
                          |DDC1_PD0_Pin|DDC1_PD1_Pin|DDC1_PD2_Pin|DDC1_PD3_Pin
                          |DDC1_PD4_Pin|DDC1_PD5_Pin|DDC1_PD6_Pin|DDC1_PD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DDC1_DV_Pin */
  GPIO_InitStruct.Pin = DDC1_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DDC1_DV_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DDC1_RDY_Pin */
  GPIO_InitStruct.Pin = DDC1_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DDC1_RDY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DDC1_SYNC_NCO_Pin */
  GPIO_InitStruct.Pin = DDC1_SYNC_NCO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DDC1_SYNC_NCO_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void DDC_Config_Init()
{
	DDC_ConfigTypeDef ddc_main_conf;
	ddc_main_conf.DDC_Mode 			= 8;
	ddc_main_conf.NCO_Mode 			= 0;
	ddc_main_conf.NCO_SyncMask 		= 0x00FFFFFFFF;
	ddc_main_conf.NCO_Frequency 	= 0x0000DA740E;
	ddc_main_conf.NCO_PhaseOffset 	= 0;
	ddc_main_conf.CIC2_Scale 		= 6;
	ddc_main_conf.CIC2_Decimation 	= 9;
	ddc_main_conf.CIC5_Scale 		= 6;
	ddc_main_conf.CIC5_Decimation 	= 9;
	ddc_main_conf.RCF_Scale 		= 4;
	ddc_main_conf.RCF_Decimation 	= 5;
	ddc_main_conf.RCF_AddressOffset = 0;
	ddc_main_conf.RCF_FilterTaps    = 0;
	USR_DDC_Init(ddc_main_conf);
}

/* @brief This function should handle received UDP packets */
void USR_UDP_ReceiveCallback(struct pbuf *p, const uint32_t addr, const uint16_t port)
{
	struct IP4_Container ip;
	ip = toIP4(addr);
	if (ip.IP4 == 28U)
	{
		//USR_UDP_InsertPostData("HELLO!", 6);
		uint8_t *pptr = (uint8_t *)p->payload;
		if (pptr[0] == 'L')
			HAL_GPIO_TogglePin(GPIOB, LED_Pin);
		if (pptr[0] == 'A')
		{
			uint16_t addr = 0, i = 0;
			uint64_t value = 0;
			DDC_ConfigTypeDef ddc_conf;
			int16_t addr_start = 1;
			int16_t val_start = 4;
			for (; i < 13; i++)
			{
				addr = get_addr(pptr, addr_start);
				value = get_value(pptr, val_start);
				if (addr == DDC_MODE)
					ddc_conf.DDC_Mode = value;
				else if (addr == DDC_NCO_MODE)
					ddc_conf.NCO_Mode = value;
				else if (addr == DDC_NCO_SYNC_MASK)
					ddc_conf.NCO_SyncMask = value;
				else if (addr == DDC_NCO_FREQUENCY)
					ddc_conf.NCO_Frequency = value;
				else if (addr == DDC_NCO_PHASE_OFFSET)
					ddc_conf.NCO_PhaseOffset = value;
				else if (addr == DDC_CIC2_SCALE)
					ddc_conf.CIC2_Scale = value;
				else if (addr == DDC_CIC2_DECIMATION)
					ddc_conf.CIC2_Decimation = value;
				else if (addr == DDC_CIC5_SCALE)
					ddc_conf.CIC5_Scale = value;
				else if (addr == DDC_CIC5_DECIMATION)
					ddc_conf.CIC5_Decimation = value;
				else if (addr == DDC_RCF_SCALE)
					ddc_conf.RCF_Scale = value;
				else if (addr == DDC_RCF_DECIMATION)
					ddc_conf.RCF_Decimation = value;
				else if (addr == DDC_RCF_ADDRESS_OFFSET)
					ddc_conf.RCF_AddressOffset = value;
				else if (addr == DDC_RCF_FILTER_TAPS)
					ddc_conf.RCF_FilterTaps = value;
				addr_start += 16;
				val_start += 16;
			}
			USR_DDC_Init(ddc_conf);
		}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim1.Instance)
	{
		/* Send DDC data to PC vie Ethernet */
		USR_UDP_Send(UDP_SEND_PORT, (uint8_t *)buffers[dbuf_index], PACKET_SIZE);
		dbuf_index++;
		if (dbuf_index == BUFFER_COUNT) dbuf_index = 0;
		HAL_TIM_IC_Start_DMA(&htim1, TIM_CHANNEL_3, (uint32_t *)(buffers[dbuf_index] + HEADER_SIZE), BUFFER_SIZE);
		// if ( != HAL_OK) for(;;);
		// else USR_UDP_Send(UDP_SEND_PORT, (uint8_t *)buffers[dbuf_index], PACKET_SIZE);
	}
}

void init_buffer()
{
	int8_t i = 0, j = 0;
	for (; i < BUFFER_COUNT; i++)
	{
		// HEADER
		for (j = 0; j < HEADER_SIZE; j++) { buffers[i][j] = HEADER; }
		buffers[i][0] = (i + 49);

		// FOOTER
		for (j = 0; j < FOOTER_SIZE; j++) { buffers[i][HEADER_SIZE + BUFFER_SIZE + j] = FOOTER; }
	}
}

uint16_t get_addr(uint8_t *s, int16_t start)
{
	// uint16_t address = 0;
	// address = address +
	return (s[start] - '0') * 10 + (s[start+1] - '0') + 0x300;
}

uint64_t get_value(uint8_t *s, int16_t start)
{
	uint64_t b = 0;
	uint8_t i = 0;
	for (; i < 12; i++)
    {
	   b = b + (s[start+i] - '0') * pow(10, 11-i);
    }
	return b;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

