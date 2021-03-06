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
#include "usr.h"
#include "sbuf.h"
#include "ddc.h"
#include "dac.h"
// #include "coder.h"
#include "defs.h"
#include "vars.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
struct IP4_Container udp_ip = {10, 3, 4, 28}; // 10.3.4.28:UDP_SEND_PORT
GPIO_PinState pmod_state = GPIO_PIN_SET;
uint8_t rx_buffer[5];
uint8_t serial_data_index = 0;
uint8_t serial_init = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */
void UDP_Buffer_Init();
void USR_DDC_Init();
void SW_Set(uint8_t mode);
void USR_Pmod_UdpHandler(uint8_t *udp_data);
void PHY_Init();
USR_StatusTypeDef PHY_Status_Check();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (rx_buffer[0] == 0x83)
	{
		DDC1_Buffer1[2] = (rx_buffer[1] << 8) + rx_buffer[0];
		DDC1_Buffer2[2] = (rx_buffer[1] << 8) + rx_buffer[0];
		DDC1_Buffer1[3] = (rx_buffer[3] << 8) + rx_buffer[2];
		DDC1_Buffer2[3] = (rx_buffer[3] << 8) + rx_buffer[2];
		DDC1_Buffer1[4] = rx_buffer[4];
		DDC1_Buffer2[4] = rx_buffer[4];
	}
	else
	{
		HAL_UART_Abort(&huart1);
		if (serial_init == 0)
		{
			uint8_t i = 1;
			for (; i<5; i++)
			{
				if (rx_buffer[i] == 0x83)
				{
					serial_data_index = i;
					break;
				}
			}
			serial_init = 1;
			if (serial_data_index != 0)
			{
				HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, serial_data_index);
			}
		}
		else
		{
			serial_init = 0;
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, 5);
		}
	}
}
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
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_LWIP_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* ---------------------------------------------------- SETUP START */

  /* ---------------------------------------------------- PHY START */
  PHY_Init();
  /* ---------------------------------------------------- PHY END   */

  /* ---------------------------------------------------- UDP START */
  USR_UDP_Init(udp_ip, UDP_SEND_PORT, UDP_RECEIVE_PORT);
  UDP_Buffer_Init();
  /* ---------------------------------------------------- UDP END   */

  /* ---------------------------------------------------- DDC START */
  /* Switch
   * SW_INT for internal OSC
   * SW_EXT for external OSC
   * */
  SW_Set(SW_INT);

  USR_DDC_Init();
  /* ---------------------------------------------------- DDC END */

  /* ---------------------------------------------------- DAC START */
  USR_DAC_Init();
  /* ---------------------------------------------------- DAC END */

  /* ---------------------------------------------------- SBUF START */
  USR_SBUF_Init();
  /* ---------------------------------------------------- SBUF END */

  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, 5);

  HAL_NVIC_EnableIRQ(EXTI0_IRQn);  // Enable Pmod interrupt
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
  htim1.Init.Period = 60000-1;
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
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 3-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 3-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
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
                          |SBUF_SCLK_Pin|DDC2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SBUF_LCLK_Pin|SW_B_Pin|SW_A_Pin|LED_Pin
                          |DDC2_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, DDC_CD0_Pin|DDC_CD1_Pin|DDC_CD2_Pin|DDC_CD3_Pin
                          |DDC_CD4_Pin|DDC_CD5_Pin|DDC_CD6_Pin|DDC_CD7_Pin
                          |DDC_WR_Pin|DDC_RD_Pin|PHY_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DDC2_PD2_Pin DDC2_PD3_Pin DDC2_PD4_Pin DDC2_PD5_Pin
                           DDC2_PD6_Pin DDC2_PD7_Pin DDC2_PD8_Pin DDC2_PD9_Pin
                           DDC2_PD10_Pin DDC2_PD11_Pin DDC2_PD12_Pin DDC2_PD13_Pin
                           DDC2_PD14_Pin DDC2_PD15_Pin DDC2_PD0_Pin DDC2_PD1_Pin */
  GPIO_InitStruct.Pin = DDC2_PD2_Pin|DDC2_PD3_Pin|DDC2_PD4_Pin|DDC2_PD5_Pin
                          |DDC2_PD6_Pin|DDC2_PD7_Pin|DDC2_PD8_Pin|DDC2_PD9_Pin
                          |DDC2_PD10_Pin|DDC2_PD11_Pin|DDC2_PD12_Pin|DDC2_PD13_Pin
                          |DDC2_PD14_Pin|DDC2_PD15_Pin|DDC2_PD0_Pin|DDC2_PD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

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
                           SBUF_SCLK_Pin DDC2_CS_Pin */
  GPIO_InitStruct.Pin = DAC_LDAC_Pin|SBUF_DATA_Pin|OSC_EN_Pin|SBUF_CLR_Pin
                          |SBUF_SCLK_Pin|DDC2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DDC2_RDY_Pin */
  GPIO_InitStruct.Pin = DDC2_RDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SBUF_LCLK_Pin SW_B_Pin SW_A_Pin LED_Pin
                           DDC2_RST_Pin */
  GPIO_InitStruct.Pin = SBUF_LCLK_Pin|SW_B_Pin|SW_A_Pin|LED_Pin
                          |DDC2_RST_Pin;
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
                           DDC_WR_Pin DDC_RD_Pin PHY_RESET_Pin */
  GPIO_InitStruct.Pin = DDC_CD0_Pin|DDC_CD1_Pin|DDC_CD2_Pin|DDC_CD3_Pin
                          |DDC_CD4_Pin|DDC_CD5_Pin|DDC_CD6_Pin|DDC_CD7_Pin
                          |DDC_WR_Pin|DDC_RD_Pin|PHY_RESET_Pin;
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

  /*Configure GPIO pins : DDC1_DV_Pin PHY_GREEN_LED_Pin */
  GPIO_InitStruct.Pin = DDC1_DV_Pin|PHY_GREEN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

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

  /*Configure GPIO pin : DDC2_DV_Pin */
  GPIO_InitStruct.Pin = DDC2_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DDC2_DV_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  // HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* @brief DDC Initialization Function */
void USR_DDC_Init()
{
	HAL_GPIO_WritePin(DDC_WR_GPIO_Port, DDC_WR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DDC_RD_GPIO_Port, DDC_RD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DDC1_CS_GPIO_Port, DDC1_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DDC2_CS_GPIO_Port, DDC2_CS_Pin, GPIO_PIN_SET);

	/* ADC1, DDC1 Clock enable 30MHz */
	HAL_TIM_Base_Start(&htim8);
	HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_1);

	/* ADC2, DDC2 Clock enable 30MHz */
	HAL_TIM_Base_Start(&htim10);
	HAL_TIM_OC_Start(&htim10, TIM_CHANNEL_1);
	HAL_TIM_Base_Start(&htim11);
	HAL_TIM_OC_Start(&htim11, TIM_CHANNEL_1);

	DDC_ConfigTypeDef ddc_main_conf;

	ddc_main_conf.DDC_Mode 			= MASTER_SINGLE_REAL;
	ddc_main_conf.NCO_Mode 			= NCO_ACTIVE;
	ddc_main_conf.NCO_SyncMask 		= NCO_SYNC_MASK_DEFAULT;
	ddc_main_conf.NCO_Frequency 	= 1517555111;
	ddc_main_conf.NCO_PhaseOffset 	= DDC_RESERVED;
	ddc_main_conf.CIC2_Scale 		= 6;
	ddc_main_conf.CIC2_Decimation 	= 10-1;
	ddc_main_conf.CIC5_Scale 		= 7;
	ddc_main_conf.CIC5_Decimation 	= 10-1;
	ddc_main_conf.RCF_Scale 		= 3;
	ddc_main_conf.RCF_Decimation 	= DDC_RESERVED;
	ddc_main_conf.RCF_AddressOffset = DDC_RESERVED;
	ddc_main_conf.RCF_FilterTaps    = DDC_RESERVED;
	ddc_main_conf.FIR               = false;
	USR_DDC_Config_Init(ddc_main_conf);
}

/* @brief This function should handle received UDP packets */
void USR_UDP_ReceiveCallback(struct pbuf *p, const uint32_t addr, const uint16_t port)
{
	struct IP4_Container ip;
	ip = toIP4(addr);
	if (ip.IP4 == udp_ip.IP4)
	{
		/* Disable DDC data available interrupt */
		HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
		uint8_t *pptr = (uint8_t *)p->payload;
		if (pptr[0] == 'L') HAL_GPIO_TogglePin(GPIOB, LED_Pin);
		else if (pptr[0] == 'A') USR_DDC_UdpHandler(pptr);
		else if (pptr[0] == 'D') USR_DAC_UdpHandler(pptr);
		else if (pptr[0] == 'S') USR_SBUF_UdpHandler(pptr);
		// else if (pptr[0] == 'C') USR_CODER_UdpHandler(pptr);
		else if (pptr[0] == 'P') USR_Pmod_UdpHandler(pptr);
		else if (pptr[0] == 'R')
			/* TODO */
			/* Send DDC configuration to PC using UDP */
			/* Make DDC configuration struct public */
			__NOP();
		/* Enable DDC data available interrupt */
		HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
	}
}

/* @brief Pmod sync operation */
void PmodIRQ_Handler()
{
	/* Pmod stop signal */
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) != pmod_state)
	{
		/* Stop DDC to STM32 Input Capture transfer in Interrupt mode */
#if defined(__DDC_ONE__) || defined(__DDC_BOTH__)
		HAL_TIM_IC_Stop_IT(&htim1, TIM_CHANNEL_4);
#endif
#ifdef __DDC_TWO__
		HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);
#endif
		/* Send buffered DDC data to PC */
		prev_index = dbuf_index;
		dbuf_index++; if (dbuf_index == BUFFER_COUNT) dbuf_index = 0;
		tmp_buffer_index = buffer_index; buffer_index = HEADER_SIZE;
		if (tmp_buffer_index > (BUFFER_SIZE/2 + HEADER_SIZE))
		{
#if defined(__DDC_ONE__) || defined(__DDC_BOTH__)
			USR_UDP_Send(UDP_SEND_PORT, (uint8_t *)buffers1[prev_index], PACKET_SIZE);
#endif
#if defined(__DDC_TWO__) || defined(__DDC_BOTH__)
			USR_UDP_Send(UDP_SEND_PORT, (uint8_t *)buffers2[prev_index], PACKET_SIZE);
#endif
			buffers1[prev_index][tmp_buffer_index] = FOOTER_1;
			buffers1[prev_index][tmp_buffer_index+1] = FOOTER_1;
			buffers2[prev_index][tmp_buffer_index] = FOOTER_2;
			buffers2[prev_index][tmp_buffer_index+1] = FOOTER_2;
#if defined(__DDC_ONE__) || defined(__DDC_BOTH__)
			USR_UDP_Send(UDP_SEND_PORT,
						 (uint8_t *)(buffers1[prev_index] + BUFFER_SIZE/2 + HEADER_SIZE),
						 (tmp_buffer_index - BUFFER_SIZE/2 - HEADER_SIZE + FOOTER_SIZE) * 2);
#endif
#if defined(__DDC_TWO__) || defined(__DDC_BOTH__)
			USR_UDP_Send(UDP_SEND_PORT,
						 (uint8_t *)(buffers2[prev_index] + BUFFER_SIZE/2 + HEADER_SIZE),
						 (tmp_buffer_index - BUFFER_SIZE/2 - HEADER_SIZE + FOOTER_SIZE) * 2);
#endif
		}
		else
		{
#if defined(__DDC_ONE__) || defined(__DDC_BOTH__)
			USR_UDP_Send(UDP_SEND_PORT, (uint8_t *)buffers1[prev_index], (tmp_buffer_index)*2);
#endif
#if defined(__DDC_TWO__) || defined(__DDC_BOTH__)
			USR_UDP_Send(UDP_SEND_PORT, (uint8_t *)buffers2[prev_index], (tmp_buffer_index)*2);
#endif
		}
	}
	/* Pmod start signal */
	else
	{
		/* Start DDC to STM32 Input Capture transfer in Interrupt mode */
#if defined(__DDC_ONE__) || defined(__DDC_BOTH__)
		HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
#endif
#ifdef __DDC_TWO__
		HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
#endif
	}
}

/* @brief DDC data buffer init, Insert header footer */
void UDP_Buffer_Init()
{
	int8_t i = 0, j = 0;
	for (; i < BUFFER_COUNT; i++)
	{
		/* HEADER */
		/* Serial data added to header section */
		for (j = 0; j < HEADER_SIZE-3; j++) { buffers1[i][j] = HEADER_1; }
		for (j = 0; j < HEADER_SIZE-3; j++) { buffers2[i][j] = HEADER_2; }

		/* Insert buffer index in header */
		// buffers[i][0] = (i + 49);

		/* FOOTER */
//		for (j = 0; j < FOOTER_SIZE; j++) { buffers1[i][HEADER_SIZE + BUFFER_SIZE + j] = FOOTER_1; }
//		for (j = 0; j < FOOTER_SIZE; j++) { buffers2[i][HEADER_SIZE + BUFFER_SIZE + j] = FOOTER_2; }
	}
}

/* @brief Resets PHY chip if necessary */
void PHY_Init()
{
	while (PHY_Status_Check() != USR_OK)
	{
		/* Reset PHY by reseting PHY RESET pin*/
		HAL_GPIO_WritePin(PHY_RESET_GPIO_Port, PHY_RESET_Pin, GPIO_PIN_RESET);
		/* Toggle LED indicator to see PHY init process */
		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_SET);
		/* Held low for 100ms */
		HAL_Delay(100);
		/* Release PHY RESET pin by setting the PHY RESET pin */
		HAL_GPIO_WritePin(PHY_RESET_GPIO_Port, PHY_RESET_Pin, GPIO_PIN_SET);
		/* Toggle LED indicator to see PHY init process */
		HAL_GPIO_WritePin(GPIOB, LED_Pin, GPIO_PIN_RESET);
		/* Wait for PHY to start */
		HAL_Delay(3000);
	}
}

/* @brief Checks PHY status by checking LAN Green LED state for 2 seconds
 * @brief Samples LAN Green LED Pin 8 times per second
 * @brief If Low detected return error */
USR_StatusTypeDef PHY_Status_Check()
{
	uint8_t count = 0;
	for (;count < 16; count++)
	{
		if (HAL_GPIO_ReadPin(PHY_GREEN_LED_GPIO_Port, PHY_GREEN_LED_Pin))
		{
			/* Pass if LAN green LED high*/
			HAL_Delay(125);
		}
		else
		{
			/* Otherwise return error */
			return USR_ERR;
		}
	}
	/* PHY status normal */
	return USR_OK;
}

/* @brief Configures signal switch mode
 * @brief Enable/Disable internal 10.7MHz OSC */
void SW_Set(uint8_t mode)
{
	/* Internal OSC */
	if (mode == SW_INT)
	{
		/* Enable internal OSC */
		HAL_GPIO_WritePin(GPIOA, OSC_EN_Pin, GPIO_PIN_SET);
		/* Switch to internal OSC signal pin */
		HAL_GPIO_WritePin(GPIOB, SW_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, SW_B_Pin, GPIO_PIN_RESET);
	}
	/* External OSC */
	else if (mode == SW_EXT)
	{
		/* Disable internal OSC */
		HAL_GPIO_WritePin(GPIOA, OSC_EN_Pin, GPIO_PIN_RESET);
		/* Switch to external signal pin */
		HAL_GPIO_WritePin(GPIOB, SW_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, SW_B_Pin, GPIO_PIN_SET);
	}
}

void USR_Pmod_UdpHandler(uint8_t *udp_data)
{
	if (udp_data[1] == '1')
		pmod_state = GPIO_PIN_SET;
	else
		pmod_state = GPIO_PIN_RESET;
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

