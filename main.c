/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
double 						att_steps[6] = {16, 8, 4, 2, 1, 0.5};
double 						hmc936_steps[6] = {180, 90, 45, 22.5, 11.25, 5.625};

uint8_t 					*UART_RX_BUFF;
uint8_t 					*UART_TX_BUFF;
uint8_t						UART_first_byte;
std_union					UART_length;
uint8_t 					uart_command;

uint8_t						*I2C_TX_BUFF;
uint8_t						*I2C_RX_BUFF;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	DAC_control_MCP(hi2c2, 3.3, CHANNEL_1, MCP_HMC573_GATE_ADDRESS);
	DAC_init_LMP(hi2c2, LMP_HMC952_GATE_ADDRESS);
	DWT_Init();
	
	HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_SET);
	
	port_extension_initial_powerup();
	DWT_Delay(1);
	port_extension_init();
	DWT_Delay(1);
	for(int i = 0; i < 4; i++)
	{
		port_extension_ctrl((0x44 + (i * 8)), 0x00, DD8);
	}
	for(int i = 0; i < 4; i++)
	{
		port_extension_ctrl((0x44 + (i * 8)), 0x00, DD9);
	}
	//supply_ctrl(0x00, 0x01, 0x00, 0x00, 0x00, 0x00);
	//DWT_Delay(1);
	//supply_ctrl(0x00, 0x01, 0x00, 0x00, 0x08, 0x00);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
		//HAL_Delay(1000);
		//HAL_GPIO_TogglePin(Drain_ON_1_GPIO_Port, Drain_ON_1_Pin);
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 12;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 1;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x307075B1;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  __HAL_RCC_GPIOK_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, RX_HMC8073_LE_Pin|TX_HMC8073_LE_Pin|LO_HMC8073_LE_Pin|RX_HMC642_V4_STM_3_Pin
                          |RX_HMC642_V5_STM_3_Pin|RX_HMC642_V6_STM_3_Pin|TX_HMC642_V1_STM_3_Pin|TX_HMC642_V2_STM_3_Pin
                          |TX_HMC642_V3_STM_3_Pin|TX_HMC642_V4_STM_3_Pin|TX_HMC642_V5_STM_3_Pin|TX_HMC642_V6_STM_3_Pin
                          |TX_HMC642_V1_STM_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOI, PLUS_5V_CTRL_SUP_TX_Pin|PLUS_6V_1_CTRL_Pin|RX_HMC642_V6_STM_4_Pin|RX_HMC642_V5_STM_4_Pin|RX_HMC642_V4_STM_4_Pin|RX_HMC642_V3_STM_4_Pin
                          |RX_HMC642_V2_STM_4_Pin|RX_HMC642_V1_STM_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PLUS_5V_CTRL_SUP_LO_Pin|PLUS_5V_CTRL_SUP_RX_Pin|PLUS_6V_2_CTRL_Pin|TX_POWER_2_Pin
                          |RX_POWER_2_Pin|Drain_ON_2_Pin|TX_POWER_3_Pin|RX_HM_936_V1_STM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RX_HMC642_V1_STM_3_GPIO_Port, RX_HMC642_V1_STM_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, RX_HMC642_V2_STM_3_Pin|RX_HMC642_V3_STM_3_Pin|TX_HMC642_V1_STM_2_Pin|RX_HMC642_V6_STM_2_Pin
                          |RX_HMC642_V5_STM_2_Pin|RX_HMC642_V4_STM_2_Pin|RX_HMC642_V3_STM_2_Pin|RX_HMC642_V2_STM_2_Pin
                          |RX_HMC642_V1_STM_2_Pin|SPI_1_CS_1_Pin|TX_HMC936_V6_STM_Pin|TX_HMC936_V5_STM_Pin
                          |TX_HMC936_V4_STM_Pin|TX_HMC642_V4_STM_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_9|Drain_ON_1_Pin|TX_POWER_1_Pin|RX_POWER_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RE_DE_2_Pin|TX_HMC642_V3_STM_4_Pin|TX_HMC642_V2_STM_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, TX_HMC642_V6_STM_1_Pin|TX_HMC642_V5_STM_1_Pin|TX_HMC642_V4_STM_1_Pin|TX_HMC642_V3_STM_1_Pin
                          |TX_HMC642_V2_STM_1_Pin|TX_HMC642_V1_STM_1_Pin|RX_HMC642_V6_STM_1_Pin|RX_HMC642_V5_STM_1_Pin
                          |RX_HM_936_V2_STM_Pin|RX_HM_936_V3_STM_Pin|RX_HM_936_V4_STM_Pin|RX_HM_936_V5_STM_Pin
                          |RX_HM_936_V6_STM_Pin|RE_DE_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOJ, RX_HMC642_V4_STM_1_Pin|RX_HMC642_V3_STM_1_Pin|RX_HMC642_V2_STM_1_Pin|RX_HMC642_V1_STM_1_Pin
                          |TX_HMC642_V6_STM_2_Pin|TX_HMC642_V5_STM_2_Pin|SPI_1_CS_3_Pin|SPI_1_CS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOK, TX_HMC642_V4_STM_2_Pin|TX_HMC642_V3_STM_2_Pin|TX_HMC642_V2_STM_2_Pin|TX_HMC936_V3_STM_Pin
                          |TX_HMC936_V2_STM_Pin|TX_HMC936_V1_STM_Pin|TX_HMC642_V6_STM_4_Pin|TX_HMC642_V5_STM_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RX_POWER_3_Pin|Drain_ON_3_Pin|TX_POWER_4_Pin|RX_POWER_4_Pin
                          |Drain_ON_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RX_HMC8073_LE_Pin TX_HMC8073_LE_Pin LO_HMC8073_LE_Pin RX_HMC642_V4_STM_3_Pin
                           RX_HMC642_V5_STM_3_Pin RX_HMC642_V6_STM_3_Pin TX_HMC642_V1_STM_3_Pin TX_HMC642_V2_STM_3_Pin
                           TX_HMC642_V3_STM_3_Pin TX_HMC642_V4_STM_3_Pin TX_HMC642_V5_STM_3_Pin TX_HMC642_V6_STM_3_Pin
                           TX_HMC642_V1_STM_4_Pin RX_HMC642_V6_STM_4_Pin */
  GPIO_InitStruct.Pin = RX_HMC8073_LE_Pin|TX_HMC8073_LE_Pin|LO_HMC8073_LE_Pin|RX_HMC642_V4_STM_3_Pin
                          |RX_HMC642_V5_STM_3_Pin|RX_HMC642_V6_STM_3_Pin|TX_HMC642_V1_STM_3_Pin|TX_HMC642_V2_STM_3_Pin
                          |TX_HMC642_V3_STM_3_Pin|TX_HMC642_V4_STM_3_Pin|TX_HMC642_V5_STM_3_Pin|TX_HMC642_V6_STM_3_Pin
                          |TX_HMC642_V1_STM_4_Pin|RX_HMC642_V6_STM_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PLUS_5V_CTRL_SUP_TX_Pin PLUS_6V_1_CTRL_Pin RX_HMC642_V4_STM_4_Pin RX_HMC642_V3_STM_4_Pin
                           RX_HMC642_V2_STM_4_Pin RX_HMC642_V1_STM_4_Pin */
  GPIO_InitStruct.Pin = PLUS_5V_CTRL_SUP_TX_Pin|PLUS_6V_1_CTRL_Pin|RX_HMC642_V6_STM_4_Pin|RX_HMC642_V5_STM_4_Pin|RX_HMC642_V4_STM_4_Pin|RX_HMC642_V3_STM_4_Pin
                          |RX_HMC642_V2_STM_4_Pin|RX_HMC642_V1_STM_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pins : PLUS_5V_CTRL_SUP_LO_Pin PLUS_5V_CTRL_SUP_RX_Pin PLUS_6V_2_CTRL_Pin TX_POWER_2_Pin
                           RX_POWER_2_Pin Drain_ON_2_Pin TX_POWER_3_Pin RX_HM_936_V1_STM_Pin */
  GPIO_InitStruct.Pin = PLUS_5V_CTRL_SUP_LO_Pin|PLUS_5V_CTRL_SUP_RX_Pin|PLUS_6V_2_CTRL_Pin|TX_POWER_2_Pin
                          |RX_POWER_2_Pin|Drain_ON_2_Pin|TX_POWER_3_Pin|RX_HM_936_V1_STM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PG_5_5V_Pin PG_3_9V_Pin */
  GPIO_InitStruct.Pin = PG_5_5V_Pin|PG_3_9V_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

  /*Configure GPIO pin : RX_HMC642_V1_STM_3_Pin */
  GPIO_InitStruct.Pin = RX_HMC642_V1_STM_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RX_HMC642_V1_STM_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_HMC642_V2_STM_3_Pin RX_HMC642_V3_STM_3_Pin TX_HMC642_V1_STM_2_Pin RX_HMC642_V6_STM_2_Pin
                           RX_HMC642_V5_STM_2_Pin RX_HMC642_V4_STM_2_Pin RX_HMC642_V3_STM_2_Pin RX_HMC642_V2_STM_2_Pin
                           RX_HMC642_V1_STM_2_Pin SPI_1_CS_1_Pin TX_HMC936_V6_STM_Pin TX_HMC936_V5_STM_Pin
                           TX_HMC936_V4_STM_Pin TX_HMC642_V4_STM_4_Pin */
  GPIO_InitStruct.Pin = RX_HMC642_V2_STM_3_Pin|RX_HMC642_V3_STM_3_Pin|TX_HMC642_V1_STM_2_Pin|RX_HMC642_V6_STM_2_Pin
                          |RX_HMC642_V5_STM_2_Pin|RX_HMC642_V4_STM_2_Pin|RX_HMC642_V3_STM_2_Pin|RX_HMC642_V2_STM_2_Pin
                          |RX_HMC642_V1_STM_2_Pin|SPI_1_CS_1_Pin|TX_HMC936_V6_STM_Pin|TX_HMC936_V5_STM_Pin
                          |TX_HMC936_V4_STM_Pin|TX_HMC642_V4_STM_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PH9 Drain_ON_1_Pin TX_POWER_1_Pin RX_POWER_1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_9|Drain_ON_1_Pin|TX_POWER_1_Pin|RX_POWER_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : RE_DE_2_Pin TX_HMC642_V3_STM_4_Pin TX_HMC642_V2_STM_4_Pin */
  GPIO_InitStruct.Pin = RE_DE_2_Pin|TX_HMC642_V3_STM_4_Pin|TX_HMC642_V2_STM_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_HMC642_V6_STM_1_Pin TX_HMC642_V5_STM_1_Pin TX_HMC642_V4_STM_1_Pin TX_HMC642_V3_STM_1_Pin
                           TX_HMC642_V2_STM_1_Pin TX_HMC642_V1_STM_1_Pin RX_HMC642_V6_STM_1_Pin RX_HMC642_V5_STM_1_Pin
                           RX_HM_936_V2_STM_Pin RX_HM_936_V3_STM_Pin RX_HM_936_V4_STM_Pin RX_HM_936_V5_STM_Pin
                           RX_HM_936_V6_STM_Pin RE_DE_1_Pin */
  GPIO_InitStruct.Pin = TX_HMC642_V6_STM_1_Pin|TX_HMC642_V5_STM_1_Pin|TX_HMC642_V4_STM_1_Pin|TX_HMC642_V3_STM_1_Pin
                          |TX_HMC642_V2_STM_1_Pin|TX_HMC642_V1_STM_1_Pin|RX_HMC642_V6_STM_1_Pin|RX_HMC642_V5_STM_1_Pin
                          |RX_HM_936_V2_STM_Pin|RX_HM_936_V3_STM_Pin|RX_HM_936_V4_STM_Pin|RX_HM_936_V5_STM_Pin
                          |RX_HM_936_V6_STM_Pin|RE_DE_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_HMC642_V4_STM_1_Pin RX_HMC642_V3_STM_1_Pin RX_HMC642_V2_STM_1_Pin RX_HMC642_V1_STM_1_Pin
                           TX_HMC642_V6_STM_2_Pin TX_HMC642_V5_STM_2_Pin SPI_1_CS_3_Pin SPI_1_CS_2_Pin */
  GPIO_InitStruct.Pin = RX_HMC642_V4_STM_1_Pin|RX_HMC642_V3_STM_1_Pin|RX_HMC642_V2_STM_1_Pin|RX_HMC642_V1_STM_1_Pin
                          |TX_HMC642_V6_STM_2_Pin|TX_HMC642_V5_STM_2_Pin|SPI_1_CS_3_Pin|SPI_1_CS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOJ, &GPIO_InitStruct);

  /*Configure GPIO pins : TX_HMC642_V4_STM_2_Pin TX_HMC642_V3_STM_2_Pin TX_HMC642_V2_STM_2_Pin TX_HMC936_V3_STM_Pin
                           TX_HMC936_V2_STM_Pin TX_HMC936_V1_STM_Pin TX_HMC642_V6_STM_4_Pin TX_HMC642_V5_STM_4_Pin */
  GPIO_InitStruct.Pin = TX_HMC642_V4_STM_2_Pin|TX_HMC642_V3_STM_2_Pin|TX_HMC642_V2_STM_2_Pin|TX_HMC936_V3_STM_Pin
                          |TX_HMC936_V2_STM_Pin|TX_HMC936_V1_STM_Pin|TX_HMC642_V6_STM_4_Pin|TX_HMC642_V5_STM_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOK, &GPIO_InitStruct);

  /*Configure GPIO pins : RX_POWER_3_Pin Drain_ON_3_Pin TX_POWER_4_Pin RX_POWER_4_Pin
                           Drain_ON_4_Pin */
  GPIO_InitStruct.Pin = RX_POWER_3_Pin|Drain_ON_3_Pin|TX_POWER_4_Pin|RX_POWER_4_Pin
                          |Drain_ON_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void UART_pack_parser(void)
{
	std_union UART_length;
	UART_length.cstd[1] = UART_RX_BUFF[1];
	UART_length.cstd[0] = UART_RX_BUFF[2];
	l_std_union temp;
	float temp_att = 0;
	float temp_gate = 0;
	float temp_phase = 0;
	temp.listd = 0;

	switch(UART_RX_BUFF[4])
	{ 
		case ECHO:
			HAL_UART_Transmit(&huart1, UART_TX_BUFF, UART_length.istd, 10);
			uart_command = 0;
		break;	
		case	SUPPLY_CTRL:
			supply_ctrl(UART_RX_BUFF[5], UART_RX_BUFF[6], UART_RX_BUFF[7], UART_RX_BUFF[8], UART_RX_BUFF[9], UART_RX_BUFF[10]);
			uart_command = 0x01;
		break;
		case 	ATT_CTRL:
			temp.cstd[0] = UART_RX_BUFF[5];
			temp_att = temp.cstd[0];
			temp_att /= 2;
			attenuator_set_att(temp_att, UART_RX_BUFF[6]);
			uart_command = 0x02;
		break;
		case	MCP_DAC:
			temp.cstd[1] = UART_RX_BUFF[5];
			temp.cstd[0] = UART_RX_BUFF[6];
			temp_gate = temp.listd;
			temp_gate /= 1000;
			switch(UART_RX_BUFF[7])
			{
				case I2C1_BUS:
					DAC_control_MCP(hi2c1, temp_gate, UART_RX_BUFF[8], UART_RX_BUFF[9]);		
				break;
				case I2C2_BUS:
					DAC_control_MCP(hi2c2, temp_gate, UART_RX_BUFF[8], UART_RX_BUFF[9]);		
				break;
				case I2C3_BUS:
					DAC_control_MCP(hi2c3, temp_gate, UART_RX_BUFF[8], UART_RX_BUFF[9]);		
				break;
				default:
				break;
			}			
			uart_command = 0x03;
		break;
		case 	LMP_DAC:
			temp.cstd[1] = UART_RX_BUFF[5];
			temp.cstd[0] = UART_RX_BUFF[6];
			temp_gate = temp.listd;
			temp_gate += -5000;
			temp_gate /= 1000;
			switch(UART_RX_BUFF[7])
			{
				case I2C1_BUS:
					DAC_control_LMP(hi2c1, temp_gate, UART_RX_BUFF[8], UART_RX_BUFF[9]);		
				break;
				case I2C2_BUS:
					DAC_control_LMP(hi2c2, temp_gate, UART_RX_BUFF[8], UART_RX_BUFF[9]);		
				break;
				case I2C3_BUS:
					DAC_control_LMP(hi2c3, temp_gate, UART_RX_BUFF[8], UART_RX_BUFF[9]);		
				break;
				default:
				break;
			}	
			uart_command = 0x04;
		break;
		case	PHASESHIFT:
			temp_phase = UART_RX_BUFF[5];
			temp_phase *= (float)5.625;
			phaseshifter_set_phase(temp_phase, UART_RX_BUFF[6], UART_RX_BUFF[7]);
			uart_command = 0x05;
		break;
		case ATTENUATOR:
			temp_att = UART_RX_BUFF[5];
			temp_att /= 2;
			attenuator_HMC424_ctrl(temp_att, UART_RX_BUFF[6], UART_RX_BUFF[7]);
			uart_command = 0x06;
			break;
		default:
			uart_command = 0xFF;
			break;
	}
}

/**************************************** HMC8073 MAINTAIN FUNCTIONS ***********************************************/

void attenuator_set_att(float attenuation, uint8_t address)
{
	uint8_t SPI_ATT_BUFF[2];
	memset(SPI_ATT_BUFF, 0, 2);
	//uint8_t att_address = address;
	
	for (int i = 0; i < 6; i++)											//D7 and D0 are do not care bits
	{
		attenuation -= att_steps[i];
		if(attenuation >= 0)
		{
			SPI_ATT_BUFF[0] += (1 << (6 - i));
		}
		else
		{
			attenuation += att_steps[i];
		}
	}
	
	SPI_ATT_BUFF[1] = address;
	
	HAL_GPIO_WritePin(LO_HMC8073_LE_GPIO_Port, LO_HMC8073_LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(TX_HMC8073_LE_GPIO_Port, TX_HMC8073_LE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RX_HMC8073_LE_GPIO_Port, RX_HMC8073_LE_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi4, SPI_ATT_BUFF, 2, 1);
	HAL_GPIO_WritePin(LO_HMC8073_LE_GPIO_Port, LO_HMC8073_LE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(TX_HMC8073_LE_GPIO_Port, TX_HMC8073_LE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RX_HMC8073_LE_GPIO_Port, RX_HMC8073_LE_Pin, GPIO_PIN_SET);
}

/**************************************** HMC936 MAINTAIN FUNCTIONS ***********************************************/

uint8_t phaseshifter_set_phase(float phase, uint8_t channel, uint8_t rf_device)
{
	uint8_t temp = 0x00;
	l_std_union temp_1;
	l_std_union temp_2;
	l_std_union temp_3;
	l_std_union temp_4;
	temp_1.listd = 0;
	temp_2.listd = 0;
	temp_3.listd = 0;
	temp_4.listd = 0;
	
	for (int i = 0; i < 6; i++)											//D7 and D0 are do not care bits
	{
		phase -= hmc936_steps[i];
		if(phase >= 0)
		{
			temp |= (1 << (5 - i));
		}
		else
		{
			phase += hmc936_steps[i];
		}
	}	
	
	switch(rf_device)
	{
		case CONVERTER:	
			temp = ~temp;
			if(channel == TX_PHASESHIFT)
			{
				temp_1.listd = GPIOK->BSRR;
				temp_2.listd = GPIOG->BSRR;
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if(i < 3)																		//first 3 bits in TX - PORT K	
					{
						if((temp & (1 << i)) != 0)
						{
							temp_1.istd[0] |= (1 << (5 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_1.istd[1] |= (1 << (5 - i));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else																				//bits 3..5 bits in TX - PORT G	
					{
						if((temp & (1 << i)) != 0)
						{
							temp_2.istd[0] |= (1 << (17 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_2.istd[1] |= (1 << (17 - i));
						}		
						else
						{
							return ERROR_CODE;
						}	
					}
				}
				GPIOK->BSRR = temp_1.listd;
				GPIOG->BSRR = temp_2.listd;
			}
			else if(channel == RX_PHASESHIFT)
			{
				temp_1.listd = GPIOC->BSRR;
				temp_2.listd = GPIOD->BSRR;					
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if(i == 0)																		//first bit in RX - PORT C	
					{
						if((temp & (1 << i)) != 0)
						{
							temp_1.istd[0] |= (1 << (12));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_1.istd[1] |= (1 << (12));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else																				//bits 1..5 bits in TX - PORT D	
					{
						if((temp & (1 << i)) != 0)
						{
							temp_2.istd[0] |= (1 << (i - 1));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_2.istd[1] |= (1 << (i - 1));
						}		
						else
						{
							return ERROR_CODE;
						}	
					}
				}		
				GPIOC->BSRR = temp_1.listd;
				GPIOD->BSRR = temp_2.listd;		
			}
			else
			{
				return ERROR_CODE;
			}
		break;
		case PA_1:
			if(channel == TX_PHASESHIFT)
			{
				temp_1.listd = GPIOD->BSRR;
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if((temp & (1 << i)) != 0)
					{
						temp_1.istd[0] |= (1 << (13 - i));
					}		
					else if((temp & (1 << i)) == 0)
					{
						temp_1.istd[1] |= (1 << (13 - i));
					}		
					else
					{
						return ERROR_CODE;
					}							
				}
				GPIOD->BSRR = temp_1.listd;
			}
			else if(channel == RX_PHASESHIFT)
			{
				temp_1.listd = GPIOJ->BSRR;
				temp_2.listd = GPIOD->BSRR;				
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if(i < 4)																		//first bit in RX - PORT C	
					{
						if((temp & (1 << i)) != 0)
						{
							temp_1.istd[0] |= (1 << (9 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_1.istd[1] |= (1 << (9 - i));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else																				//bits 1..5 bits in TX - PORT D	
					{
						if((temp & (1 << i)) != 0)
						{
							temp_2.istd[0] |= (1 << (19 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_2.istd[1] |= (1 << (19 - i));
						}		
						else
						{
							return ERROR_CODE;
						}	
					}
				}		
				GPIOJ->BSRR = temp_1.listd;
				GPIOD->BSRR = temp_2.listd;		
			}
			else
			{
				return ERROR_CODE;
			}
		break;
		case PA_2:
			if(channel == TX_PHASESHIFT)
			{
				temp_1.listd = GPIOG->BSRR;
				temp_2.listd = GPIOK->BSRR;
				temp_3.listd = GPIOJ->BSRR;			
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if(i == 0)			
					{
						if((temp & (1 << i)) != 0)
						{
							temp_1.istd[0] |= (1 << (2));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_1.istd[1] |= (1 << (2));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else if(i < 4)
					{
						if((temp & (1 << i)) != 0)
						{
							temp_2.istd[0] |= (1 << (3 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_2.istd[1] |= (1 << (3 - i));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else 
					{
						if((temp & (1 << i)) != 0)
						{
							temp_3.istd[0] |= (1 << (15 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_3.istd[1] |= (1 << (15 - i));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
				}
				GPIOG->BSRR = temp_1.listd;
				GPIOK->BSRR = temp_2.listd;
				GPIOJ->BSRR = temp_3.listd;
			}
			else if(channel == RX_PHASESHIFT)
			{
				temp_1.listd = GPIOG->BSRR;			
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if((temp & (1 << i)) != 0)
					{
						temp_1.istd[0] |= (1 << (8 - i));
					}		
					else if((temp & (1 << i)) == 0)
					{
						temp_1.istd[1] |= (1 << (8 - i));
					}		
					else
					{
						return ERROR_CODE;
					}							
				}
				GPIOG->BSRR = temp_1.listd;
			}
			else
			{
				return ERROR_CODE;
			}
		break;		
		case PA_3:
			if(channel == TX_PHASESHIFT)
			{
				temp_1.listd = GPIOE->BSRR;
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if((temp & (1 << i)) != 0)
					{
						temp_1.istd[0] |= (1 << (i + 10));
					}		
					else if((temp & (1 << i)) == 0)
					{
						temp_1.istd[1] |= (1 << (i + 10));
					}		
					else
					{
						return ERROR_CODE;
					}							
				}
				GPIOE->BSRR = temp_1.listd;
			}
			else if(channel == RX_PHASESHIFT)
			{
				temp_1.listd = GPIOF->BSRR;
				temp_2.listd = GPIOG->BSRR;		
				temp_3.listd = GPIOE->BSRR;				
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if(i == 0)			
					{
						if((temp & (1 << i)) != 0)
						{
							temp_1.istd[0] |= (1 << (15));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_1.istd[1] |= (1 << (15));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else if(i < 3)
					{
						if((temp & (1 << i)) != 0)
						{
							temp_2.istd[0] |= (1 << (i - 1));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_2.istd[1] |= (1 << (i - 1));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else 
					{
						if((temp & (1 << i)) != 0)
						{
							temp_3.istd[0] |= (1 << (i + 4));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_3.istd[1] |= (1 << (i + 4));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
				}
				GPIOF->BSRR = temp_1.listd;
				GPIOG->BSRR = temp_2.listd;
				GPIOE->BSRR = temp_3.listd;
			}
			else
			{
				return ERROR_CODE;
			}
		break;		
		case PA_4:
			if(channel == TX_PHASESHIFT)
			{
				temp_1.listd = GPIOE->BSRR;
				temp_2.listd = GPIOB->BSRR;
				temp_3.listd = GPIOG->BSRR;
				temp_4.listd = GPIOK->BSRR;				
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if(i == 0)			
					{
						if((temp & (1 << i)) != 0)
						{
							temp_1.istd[0] |= (1 << (0));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_1.istd[1] |= (1 << (0));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else if(i < 3)
					{
						if((temp & (1 << i)) != 0)
						{
							temp_2.istd[0] |= (1 << (10 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_2.istd[1] |= (1 << (10 - i));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else if(i == 3)
					{
						if((temp & (1 << i)) != 0)
						{
							temp_3.istd[0] |= (1 << (15));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_3.istd[1] |= (1 << (15));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else
					{
						if((temp & (1 << i)) != 0)
						{
							temp_4.istd[0] |= (1 << (11 - i));
						}		
						else if((temp & (1 << i)) == 0)
						{
							temp_4.istd[1] |= (1 << (11 - i));
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
				}
				GPIOE->BSRR = temp_1.listd;
				GPIOB->BSRR = temp_2.listd;
				GPIOG->BSRR = temp_3.listd;
				GPIOK->BSRR = temp_4.listd;
			}
			else if(channel == RX_PHASESHIFT)
			{
				temp_1.listd = GPIOI->BSRR;		
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if((temp & (1 << i)) != 0)
					{
						temp_1.istd[0] |= (1 << (7 - i));
					}		
					else if((temp & (1 << i)) == 0)
					{
						temp_1.istd[1] |= (1 << (7 - i));
					}		
					else
					{
						return ERROR_CODE;
					}							
				}
				GPIOI->BSRR = temp_1.listd;
			}
			else
			{
				return ERROR_CODE;
			}
		break;		
	}
	return OK_CODE;
}

/**************************************** SUPPLY MAINTAIN FUNCTIONS ***********************************************/

void supply_ctrl(uint8_t supply_data_1, uint8_t supply_data_2, uint8_t supply_data_3, uint8_t supply_data_4, uint8_t supply_data_5, uint8_t supply_data_6)
{
	uint8_t supply_data_mass[5];
	supply_data_mass[0] = supply_data_2;
	supply_data_mass[1] = supply_data_3;
	supply_data_mass[2] = supply_data_4;
	supply_data_mass[3] = supply_data_5;
	supply_data_mass[4] = supply_data_6;	
	
	//Byte 1 parcer
	for(int i = 0; i < 5; i++)
	{
		if(supply_data_1 & (1 << i))
		{
			if(i == 0)
			{
				HAL_GPIO_WritePin(GPIOI, (GPIO_PIN_8 << i), GPIO_PIN_SET);
			}
			else if (i != 4)
			{
				HAL_GPIO_WritePin(GPIOC, (GPIO_PIN_13 << (i - 1)), GPIO_PIN_SET);
			}
			else 
			{
				HAL_GPIO_WritePin(GPIOI, (GPIO_PIN_9 << (i - 4)), GPIO_PIN_SET);
			}
		}
		else
		{
			if(i == 0)
			{
				HAL_GPIO_WritePin(GPIOI, (GPIO_PIN_8 << i), GPIO_PIN_RESET);
			}
			else if (i != 4)
			{
				HAL_GPIO_WritePin(GPIOC, (GPIO_PIN_13 << (i - 1)), GPIO_PIN_RESET);
			}
			else 
			{
				HAL_GPIO_WritePin(GPIOI, (GPIO_PIN_9 << (i - 4)), GPIO_PIN_RESET);
			}
		}
	}
	
	//Bytes 2...5 parcer
	for(int i = 0; i < 4; i++)
	{
		port_extension_ctrl((0x44 + (i * 8)), supply_data_mass[i], DD7);
	}
	
	//Byte 5 Drain parser
	for(int i = 4; i < 8; i++)
	{
		if(supply_data_5 & (1 << i))
		{
			if(i == 4)
			{
				HAL_GPIO_WritePin(Drain_ON_1_GPIO_Port, Drain_ON_1_Pin, GPIO_PIN_SET);
			}
			else if (i == 5)
			{
				HAL_GPIO_WritePin(Drain_ON_2_GPIO_Port, Drain_ON_2_Pin, GPIO_PIN_SET);
			}
			else if (i == 6)
			{
				HAL_GPIO_WritePin(Drain_ON_3_GPIO_Port, Drain_ON_3_Pin, GPIO_PIN_SET);
			}
			else if (i == 7) 
			{
				HAL_GPIO_WritePin(Drain_ON_4_GPIO_Port, Drain_ON_4_Pin, GPIO_PIN_SET);
			}
		}
		else
		{
			if(i == 4)
			{
				HAL_GPIO_WritePin(Drain_ON_1_GPIO_Port, Drain_ON_1_Pin, GPIO_PIN_RESET);
			}
			else if (i == 5)
			{
				HAL_GPIO_WritePin(Drain_ON_2_GPIO_Port, Drain_ON_2_Pin, GPIO_PIN_RESET);
			}
			else if (i == 6)
			{
				HAL_GPIO_WritePin(Drain_ON_3_GPIO_Port, Drain_ON_3_Pin, GPIO_PIN_RESET);
			}
			else if (i == 7) 
			{
				HAL_GPIO_WritePin(Drain_ON_4_GPIO_Port, Drain_ON_4_Pin, GPIO_PIN_RESET);
			}
		}
	}
		
	//Byte 6 parcer
	//DD9; PA_1 and PA_2 Both Gates
	if(supply_data_6 & (1 << Gate_ON_1_1))
	{
		port_extension_ctrl((0x3E), ON, DD9);
	}
	else
	{
		port_extension_ctrl((0x3E), OFF, DD9);
	}
	if(supply_data_6 & (1 << Gate_ON_2_1))
	{
		port_extension_ctrl((0x25), ON, DD9);
	}
	else
	{
		port_extension_ctrl((0x25), OFF, DD9);
	}
	if(supply_data_6 & (1 << Gate_ON_1_2))
	{
		port_extension_ctrl((0x3F), ON, DD9);
	}
	else
	{
		port_extension_ctrl((0x3F), OFF, DD9);
	}
	if(supply_data_6 & (1 << Gate_ON_2_2))
	{
		port_extension_ctrl((0x24), ON, DD9);
	}
	else
	{
		port_extension_ctrl((0x24), OFF, DD9);
	}
	
	//DD8; PA_3 and PA_4 Both Gates
	if(supply_data_6 & (1 << Gate_ON_1_3))
	{
		port_extension_ctrl((0x3E), ON, DD8);
	}
	else
	{
		port_extension_ctrl((0x3E), OFF, DD8);
	}
	if(supply_data_6 & (1 << Gate_ON_2_3))
	{
		port_extension_ctrl((0x25), ON, DD8);
	}
	else
	{
		port_extension_ctrl((0x25), OFF, DD8);
	}
	if(supply_data_6 & (1 << Gate_ON_1_4))
	{
		port_extension_ctrl((0x3F), ON, DD8);
	}
	else
	{
		port_extension_ctrl((0x3F), OFF, DD8);
	}
	if(supply_data_6 & (1 << Gate_ON_2_4))
	{
		port_extension_ctrl((0x24), ON, DD8);
	}
	else
	{
		port_extension_ctrl((0x24), OFF, DD8);
	}
}

void port_extension_initial_powerup(void)
{
	//Initial power up (not the init) Sequence as in the datasheet; first byte - reg address, second byte - data
	uint8_t SPI_PORT_EXT_BUFF[2];
	memset(SPI_PORT_EXT_BUFF, 0, 2);
	SPI_PORT_EXT_BUFF[0] = 0x24;
	
	//GPIO Output Low
	for(int i = 0; i < 28; i++)
	{
		HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_RESET);	
		
		HAL_SPI_Transmit(&hspi1, SPI_PORT_EXT_BUFF, 2, 1);
		
		HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_SET);	
		
		SPI_PORT_EXT_BUFF[0]++;
	}
	
	//DWT_Delay(1);
	SPI_PORT_EXT_BUFF[0] = 0x04;
	
	//Shutdown Enabled;	Transition Detection Disabled
	HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_RESET);		
	HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_RESET);	
	
	HAL_SPI_Transmit(&hspi1, SPI_PORT_EXT_BUFF, 2, 1);

	HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_SET);		
	HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_SET);		
	
	//DWT_Delay(1);
	SPI_PORT_EXT_BUFF[0] = 0x06;
	
	//All Clear (Masked Off)
	HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_RESET);		
	HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_RESET);	
	
	HAL_SPI_Transmit(&hspi1, SPI_PORT_EXT_BUFF, 2, 1);

	HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_SET);		
	HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_SET);		
	
	//DWT_Delay(1);
	SPI_PORT_EXT_BUFF[0] = 0x09;
	SPI_PORT_EXT_BUFF[1] = 0xAA;
	
	//P4 to P31: GPIO Inputs Without Pullup
	for(int i = 0; i < 7; i++)
	{
		HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_RESET);	
		
		HAL_SPI_Transmit(&hspi1, SPI_PORT_EXT_BUFF, 2, 1);
		
		HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_SET);

		SPI_PORT_EXT_BUFF[0]++;		
	}	
}

void port_extension_init(void)
{
	//Init; first byte - reg address, second byte - data
	uint8_t SPI_PORT_EXT_BUFF[2];
	memset(SPI_PORT_EXT_BUFF, 0, 2);

	SPI_PORT_EXT_BUFF[0] = 0x04;
	SPI_PORT_EXT_BUFF[1] = 0x01;
	
	//Shutdown Enabled;	Transition Detection Disabled
	HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_RESET);		
	HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_RESET);	
	HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_RESET);	
	
	HAL_SPI_Transmit(&hspi1, SPI_PORT_EXT_BUFF, 2, 1);

	HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_SET);		
	HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_SET);	
	HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_SET);		
	
	//DWT_Delay(1);
	SPI_PORT_EXT_BUFF[0] = 0x09;
	SPI_PORT_EXT_BUFF[1] = 0x55;
	
	//P4 to P31: GPIO output
	for(int i = 0; i < 7; i++)
	{
		HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_RESET);		
		HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_RESET);	
		HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_RESET);	
		
		HAL_SPI_Transmit(&hspi1, SPI_PORT_EXT_BUFF, 2, 1);
		
		HAL_GPIO_WritePin(SPI_1_CS_1_GPIO_Port, SPI_1_CS_1_Pin, GPIO_PIN_SET);		
		HAL_GPIO_WritePin(SPI_1_CS_2_GPIO_Port, SPI_1_CS_2_Pin, GPIO_PIN_SET);	
		HAL_GPIO_WritePin(SPI_1_CS_3_GPIO_Port, SPI_1_CS_3_Pin, GPIO_PIN_SET);

		SPI_PORT_EXT_BUFF[0]++;		
	}	
}

void port_extension_ctrl(uint8_t reg_address, uint8_t pin_state, uint8_t cs_num)
{
	//Init; first byte - reg address, second byte - data
	uint8_t SPI_PORT_EXT_BUFF[2];
	GPIO_TypeDef *CS_PORT;
	uint16_t CS_PIN = 0;
	
	SPI_PORT_EXT_BUFF[0] = reg_address;
	SPI_PORT_EXT_BUFF[1] = pin_state;		
	
	switch(cs_num)
	{
		case DD7:
			CS_PORT = SPI_1_CS_1_GPIO_Port;
			CS_PIN = SPI_1_CS_1_Pin;
		break;
		case DD8:
			CS_PORT = SPI_1_CS_2_GPIO_Port;
			CS_PIN = SPI_1_CS_2_Pin;
		break;
		case DD9:
			CS_PORT = SPI_1_CS_3_GPIO_Port;
			CS_PIN = SPI_1_CS_3_Pin;
		break;	
	}
	
	//register control
	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_RESET);		
	
	HAL_SPI_Transmit(&hspi1, SPI_PORT_EXT_BUFF, 2, 1);

	HAL_GPIO_WritePin(CS_PORT, CS_PIN, GPIO_PIN_SET);		
}

/****************************************** DAC control **************************************************/

/****************************************** LMP control **************************************************/

void DAC_init_LMP(I2C_HandleTypeDef i2c_handle, uint8_t dac_address)
{
	free(I2C_TX_BUFF);
	free(I2C_RX_BUFF);
	I2C_TX_BUFF = (uint8_t *) malloc(4);
	I2C_RX_BUFF = (uint8_t *) malloc(4);
	memset(I2C_RX_BUFF, 0, sizeof(I2C_RX_BUFF));
	memset(I2C_TX_BUFF, 0, sizeof(I2C_TX_BUFF));	
	
	I2C_TX_BUFF[0] = ACC_CNTL;
	I2C_TX_BUFF[1] = PASSWORD_1;
	HAL_I2C_Master_Transmit(&i2c_handle, (dac_address << 1), I2C_TX_BUFF, 2, 1);	
	I2C_TX_BUFF[1] = PASSWORD_2;
	HAL_I2C_Master_Transmit(&i2c_handle, (dac_address << 1), I2C_TX_BUFF, 2, 1);

	I2C_TX_BUFF[0] = 0x02;
	HAL_I2C_Mem_Write(&i2c_handle, (dac_address << 1), OVRD_CNTL, 1, I2C_TX_BUFF, 1, 1);
		
	I2C_TX_BUFF[0] = 0x0C;
	I2C_TX_BUFF[1] = 0xCC;
	I2C_TX_BUFF[2] = 0x0C;
	I2C_TX_BUFF[3] = 0xCC;

	HAL_I2C_Mem_Write(&i2c_handle, (dac_address << 1), DAC0M_OVRD, 1, I2C_TX_BUFF, 4, 1);		

	//HAL_I2C_Mem_Read(&hi2c1, (LM92066_ADDRESS << 1), DAC0M_OVRD, 1, I2C_RX_BUFF, 4, 1);
}

void DAC_control_LMP(I2C_HandleTypeDef i2c_handle, double dac_out, uint8_t channel, uint8_t dac_address)
{
	l_l_std_union temp_dac;
	//uint8_t temp = 0;
	free(I2C_TX_BUFF);
	free(I2C_RX_BUFF);
	I2C_TX_BUFF = (uint8_t *) malloc(4);
	I2C_RX_BUFF = (uint8_t *) malloc(4);
	memset(I2C_RX_BUFF, 0, sizeof(I2C_RX_BUFF));
	memset(I2C_TX_BUFF, 0, sizeof(I2C_TX_BUFF));	
	dac_out /= -5;
	dac_out *= 4095;
	temp_dac.llistd = dac_out;
	
	I2C_TX_BUFF[0] = ACC_CNTL;
	I2C_TX_BUFF[1] = PASSWORD_1;
	HAL_I2C_Master_Transmit(&i2c_handle, (dac_address << 1), I2C_TX_BUFF, 2, 1);	
	I2C_TX_BUFF[1] = PASSWORD_2;
	HAL_I2C_Master_Transmit(&i2c_handle, (dac_address << 1), I2C_TX_BUFF, 2, 1);
	//HAL_I2C_Mem_Read(&hi2c1, (LM92066_ADDRESS << 1), ACC_CNTL, 1, I2C_RX_BUFF, 1, 1);
	//HAL_Delay(1);

	I2C_TX_BUFF[0] = 0x02;
	HAL_I2C_Mem_Write(&i2c_handle, (dac_address << 1), OVRD_CNTL, 1, I2C_TX_BUFF, 1, 1);
	
	//HAL_I2C_Mem_Read(&hi2c1, (LM92066_ADDRESS << 1), OVRD_CNTL, 1, I2C_RX_BUFF, 1, 1);
	
	I2C_TX_BUFF[0] = temp_dac.cstd[1];
	I2C_TX_BUFF[1] = temp_dac.cstd[0];
		
	if(channel == LO_CONV_DAC_LMP)
	{
		HAL_I2C_Mem_Write(&i2c_handle, (dac_address << 1), DAC0M_OVRD, 1, I2C_TX_BUFF, 2, 1);		
	}
	else if(channel == TX_CONV_DAC_LMP)
	{
		HAL_I2C_Mem_Write(&i2c_handle, (dac_address << 1), DAC1M_OVRD, 1, I2C_TX_BUFF, 2, 1);		
	}
	
	
	//HAL_I2C_Mem_Read(&hi2c1, (LM92066_ADDRESS << 1), DAC0M_OVRD, 1, I2C_RX_BUFF, 4, 1);
}

uint8_t DAC_revision_LMP(I2C_HandleTypeDef i2c_handle, uint8_t dac_address)
{
	uint8_t revision = 0;
	free(I2C_TX_BUFF);
	free(I2C_RX_BUFF);
	I2C_TX_BUFF = (uint8_t *) malloc(2);
	I2C_RX_BUFF = (uint8_t *) malloc(2);
	memset(I2C_RX_BUFF, 0, sizeof(I2C_RX_BUFF));
	memset(I2C_TX_BUFF, 0, sizeof(I2C_TX_BUFF));
	
	HAL_I2C_Mem_Read(&i2c_handle, (dac_address << 1), VERSION, 1, I2C_RX_BUFF, 1, 10);
	
	revision = I2C_RX_BUFF[0];
	
	return revision;
}	

/****************************************** MCP control **************************************************/

void DAC_control_MCP(I2C_HandleTypeDef i2c_handle, double dac_out, uint8_t channel, uint8_t dac_address)
{
	l_l_std_union temp_dac;
	//uint8_t temp = 0;
	free(I2C_TX_BUFF);
	free(I2C_RX_BUFF);
	I2C_TX_BUFF = (uint8_t *) malloc(4);
	I2C_RX_BUFF = (uint8_t *) malloc(4);
	memset(I2C_RX_BUFF, 0, sizeof(I2C_RX_BUFF));
	memset(I2C_TX_BUFF, 0, sizeof(I2C_TX_BUFF));	
	dac_out /= 3.3;
	dac_out *= 255;																																								//make sure that not 0xff value, 3.3V will cause 0x100
	temp_dac.llistd = dac_out;
	
	if(channel == CHANNEL_0)
	{
		I2C_TX_BUFF[0] = (((MCP_VOL_DAC_0 << 3) & 0xF8) | ((WRITE_OPERATION << 1) & 0x01));						//Register address shifted left, write operation '00' shifted left, sum operation executed
	}
	else if(channel == CHANNEL_1)
	{
		I2C_TX_BUFF[0] = (((MCP_VOL_DAC_1 << 3) & 0xF8) | ((WRITE_OPERATION << 1) & 0x01));						//Register address shifted left, write operation '00' shifted left, sum operation executed
	}

	I2C_TX_BUFF[1] = 0x00;
	I2C_TX_BUFF[2] = temp_dac.cstd[0];																																				//Stands for +3V
	HAL_I2C_Master_Transmit(&i2c_handle, (dac_address << 1), I2C_TX_BUFF, 3, 1);	
}

/**************************************** HMC424 MAINTAIN FUNCTIONS ***********************************************/

uint8_t attenuator_HMC424_ctrl(float att, uint8_t channel, uint8_t rf_device)
{
	uint8_t temp = 0x00;
	
	for (int i = 0; i < 6; i++)											//D7 and D0 are do not care bits
	{
		att -= att_steps[i];
		if(att >= 0)
		{
			temp |= (1 << (5 - i));
		}
		else
		{
			att += att_steps[i];
		}
	}	
	
	
	
	switch(rf_device)
	{
		case PA_1:
			if(channel == TX_ATT)
			{
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if(i != 5)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 15 - i), ON, DD9);
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 15 - i), OFF, DD9);
						}		
						else
						{
							return ERROR_CODE;
						}									
					}
					else if(i == 5)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 7), ON, DD9);
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 7), OFF, DD9);
						}		
						else
						{
							return ERROR_CODE;
						}		
					}
					else
					{
						return ERROR_CODE;
					}								
				}
			}
			else if(channel == RX_ATT)
			{			
				for(int i = 0; i < 6; i++)										
				{	
					if((i == 0) || (i == 2) || (i == 4))																		
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 10 - (i / 2)), ON, DD9);							
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 10 - (i / 2)), OFF, DD9);	
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else if((i == 1) || (i == 3) || (i == 5))																					
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 6 - ((i - 1) / 2)), ON, DD9);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 6 - ((i - 1) / 2)), OFF, DD9);	
						}		
						else
						{
							return ERROR_CODE;
						}	
					}
					else
					{
						return ERROR_CODE;
					}
				}			
			}
			else
			{
				return ERROR_CODE;
			}
		break;
		case PA_2:
			if(channel == TX_PHASESHIFT)
			{	
				for(int i = 0; i < 6; i++)										
				{	
					if((temp & (1 << i)) != 0)
					{
						port_extension_ctrl((PORT_4_SINGLE_CTRL + 16 + i), ON, DD9);	
					}		
					else if((temp & (1 << i)) == 0)
					{
						port_extension_ctrl((PORT_4_SINGLE_CTRL + 16 + i), OFF, DD9);	
					}		
					else
					{
						return ERROR_CODE;
					}	
				}
			}
			else if(channel == RX_PHASESHIFT)
			{		
				for(int i = 0; i < 6; i++)										
				{	
					if(i < 3)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 22 + i), ON, DD9);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 22 + i), OFF, DD9);	
						}		
						else
						{
							return ERROR_CODE;
						}	
					}
					else if(i == 3)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 3), ON, DD9);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 3), OFF, DD9);	
						}		
						else
						{
							return ERROR_CODE;
						}
					}
					else if(i == 4)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 25), ON, DD9);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 25), OFF, DD9);	
						}		
						else
						{
							return ERROR_CODE;
						}
					}
					else if(i == 5)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 2), ON, DD9);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 2), OFF, DD9);	
						}		
						else
						{
							return ERROR_CODE;
						}
					}					
				}
			}
			else
			{
				return ERROR_CODE;
			}
		break;		
		case PA_3:
			if(channel == TX_ATT)
			{
				for(int i = 0; i < 6; i++)										//look through bits 0..5
				{	
					if((i == 0) || (i == 2) || (i == 4))																		
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 4 + (i / 2)), ON, DD8);							
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 4 + (i / 2)), OFF, DD8);	
						}		
						else
						{
							return ERROR_CODE;
						}			
					}
					else if((i == 1) || (i == 3) || (i == 5))																					
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 6 + ((i - 1) / 2)), ON, DD8);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 6 + ((i - 1) / 2)), OFF, DD8);	
						}		
						else
						{
							return ERROR_CODE;
						}	
					}
					else
					{
						return ERROR_CODE;
					}				
				}
			}
			else if(channel == RX_ATT)
			{			
				for(int i = 0; i < 6; i++)										
				{	
					if(i != 0)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 15 - i), ON, DD8);
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 15 - i), OFF, DD8);
						}		
						else
						{
							return ERROR_CODE;
						}									
					}
					else if(i == 0)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 7), ON, DD8);
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 7), OFF, DD8);
						}		
						else
						{
							return ERROR_CODE;
						}		
					}
					else
					{
						return ERROR_CODE;
					}			
				}			
			}
			else
			{
				return ERROR_CODE;
			}
		break;		
		case PA_4:
			if(channel == TX_PHASESHIFT)
			{	
				for(int i = 0; i < 6; i++)										
				{	
					if((temp & (1 << i)) != 0)
					{
						port_extension_ctrl((PORT_4_SINGLE_CTRL + 16 + i), ON, DD8);	
					}		
					else if((temp & (1 << i)) == 0)
					{
						port_extension_ctrl((PORT_4_SINGLE_CTRL + 16 + i), OFF, DD8);	
					}		
					else
					{
						return ERROR_CODE;
					}	
				}
			}
			else if(channel == RX_PHASESHIFT)
			{		
				for(int i = 0; i < 6; i++)										
				{	
					if(i < 3)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 22 + i), ON, DD8);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 22 + i), OFF, DD8);	
						}		
						else
						{
							return ERROR_CODE;
						}	
					}
					else if(i == 3)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 3), ON, DD8);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 3), OFF, DD8);	
						}		
						else
						{
							return ERROR_CODE;
						}
					}
					else if(i == 4)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 25), ON, DD8);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 25), OFF, DD8);	
						}		
						else
						{
							return ERROR_CODE;
						}
					}
					else if(i == 5)
					{
						if((temp & (1 << i)) != 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 2), ON, DD8);	
						}		
						else if((temp & (1 << i)) == 0)
						{
							port_extension_ctrl((PORT_4_SINGLE_CTRL + 2), OFF, DD8);	
						}		
						else
						{
							return ERROR_CODE;
						}
					}					
				}
			}
			else
			{
				return ERROR_CODE;
			}
		break;		
	}
	return OK_CODE;
}

uint8_t xor_handler(uint8_t *mass)
{
	uint8_t result = 0;
	std_union temp_1;
	temp_1.cstd[1] = mass[1];
	temp_1.cstd[0] = mass[2];
	for(int i = 0; i < (temp_1.istd); i++)
	{
		result ^= mass[i];
	}
	return result;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
