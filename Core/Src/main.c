/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

//#define TEST_CHIPS
//#define TEST_ADC
//#define TEST_ADC_ALL_MSG
//#define TEST_MODEM

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcal_GPIO.h"
#include "hal_HPLPRFSwitch.h"
#include "hal_PoweSensor.h"
#include "hal_ModemPulse.h"
#include "hal_PC.h"
#include "appcb_ModemPulse.h"
#include "appcb_PC.h"
#include "app_RFCU.h"





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
ADC_HandleTypeDef hadc;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	uint8_t ui8TXCheck;
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
	MX_ADC_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	MCAL_GPIO_EnableControl();
	HAL_ModemPulse_Init(PULSE_Pin,APP_ModemPulse_DetecEdgeCallback);
	HAL_PC_Init(APP_PC_RXCallback,APP_PC_TXCallback);

#ifdef TEST_CHIPS
	MCAL_GPIO_EnableControl();
#endif

	//#ifdef TEST_MODEM
	//	if (etEdgeType==FallingEdge)
	//	{
	//
	//		HAL_HPSwitch_Enable();
	//		HAL_LPSwitch_Disable();
	//		HAL_Delay(1000);
	//	}
	//	else if (etEdgeType==RisingEdge)
	//	{
	//
	//		HAL_HPSwitch_Disable();
	//		HAL_LPSwitch_Enable();
	//		HAL_Delay(1000);
	//	}
	//
	//#endif

#ifdef TEST_ADC

	g_aPCTransmitData[0]=0x55;
	g_aPCTransmitData[1]=0x14;
	g_aPCTransmitData[2]=0x13;

	HAL_PowerSensor1_GetValue(&hadc);
//	g_aADCValueReversed[0]=APP_RFCU_ConvertBytes(g_ui32AdcValue[0]);
	HAL_PowerSensor2_GetValue(&hadc);
//	g_aADCValueReversed[1]=APP_RFCU_ConvertBytes(g_ui32AdcValue[1]);
	HAL_PowerSensor3_GetValue(&hadc);
//	g_aADCValueReversed[2]=APP_RFCU_ConvertBytes(g_ui32AdcValue[2]);
	HAL_PowerSensor4_GetValue(&hadc);
//	g_aADCValueReversed[3]=APP_RFCU_ConvertBytes(g_ui32AdcValue[3]);
	//HAL_ADC_Stop(&hadc);
	APP_RFCU_SetBytebyByte(g_ui32AdcValue,g_aPCTransmitData);

//	App_ADCParser(g_aPCTransmitData);
	ui8TXCheck=APP_RFCU_CheckSum(g_aPCTransmitData, 20);

	if(ui8TXCheck==1)
	{
		HAL_PC_Write(&huart2 ,&hdma_usart2_tx,g_aPCTransmitData,20 );
		//			HAL_UART_Transmit_DMA(&huart2, g_aPCTransmitData, 20);
		//				__HAL_DMA_DISABLE_IT(&hdma_usart2_tx, DMA_IT_HT);
		while (g_ui8PCTXFlag == 0);
		g_ui8PCTXFlag = 0;

	}

#endif

#ifdef TEST_ADC_ALL_MSG
	uint32_t ui32adcvalue;
	g_aPCTransmitData[0]=0x55;
	g_aPCTransmitData[1]=0x14;
	g_aPCTransmitData[2]=0x13;

	uint32_t* pui32Sensor1Reading= (uint32_t*)&g_aPCTransmitData[3];
	uint32_t* pui32Sensor2Reading= (uint32_t*)&g_aPCTransmitData[7];
	uint32_t* pui32Sensor3Reading= (uint32_t*)&g_aPCTransmitData[11];
	uint32_t* pui32Sensor4Reading= (uint32_t*)&g_aPCTransmitData[15];

	HAL_PowerSensor1_GetValue(&hadc);
	ui32adcvalue=g_ui32AdcValue[0];
	*pui32Sensor1Reading=ui32adcvalue ;

	HAL_PowerSensor2_GetValue(&hadc);
	ui32adcvalue=g_ui32AdcValue[1];
	*pui32Sensor2Reading=&ui32adcvalue ;

	HAL_PowerSensor3_GetValue(&hadc);
	ui32adcvalue=g_ui32AdcValue[2];
	*pui32Sensor3Reading=&ui32adcvalue ;

	HAL_PowerSensor4_GetValue(&hadc);
	ui32adcvalue=g_ui32AdcValue[3];
	*pui32Sensor4Reading=&ui32adcvalue ;

	ui8TXCheck=APP_RFCU_CheckSum(g_aPCTransmitData, 20);
	if(ui8TXCheck==1)
	{
		HAL_PC_Write(&huart2 ,&hdma_usart2_tx,g_aPCTransmitData,20 );
		while (g_ui8PCTXFlag == 0);
		g_ui8PCTXFlag = 0;

	}


#endif
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
				APP_RFCU_Start();

//#ifdef TEST_ADC
//
//
//		HAL_PowerSensor1_GetValue(&hadc);
//		g_ui8SensorValue[0]=(uint8_t)(((uint32_t)g_ui32AdcValue[0]*SENSOR_MAX_VOLT_VALUE)/ADC_MAXIMUM_VALUE);
//		HAL_PowerSensor2_GetValue(&hadc);
//		g_ui8SensorValue[1]=(uint8_t)(((uint32_t)g_ui32AdcValue[1]*SENSOR_MAX_VOLT_VALUE)/ADC_MAXIMUM_VALUE);
//		HAL_PowerSensor3_GetValue(&hadc);
//		g_ui8SensorValue[2]=(uint8_t)(((uint32_t)g_ui32AdcValue[2]*SENSOR_MAX_VOLT_VALUE)/ADC_MAXIMUM_VALUE);
//		HAL_PowerSensor4_GetValue(&hadc);
//		g_ui8SensorValue[3]=(uint8_t)(((uint32_t)g_ui32AdcValue[3]*SENSOR_MAX_VOLT_VALUE)/ADC_MAXIMUM_VALUE);
//
//		HAL_PC_Write(&huart2 ,&hdma_usart2_tx,g_ui8SensorValue,4 );
//		while (g_ui8PCTXFlag == 0);
//		g_ui8PCTXFlag = 0;
//
//
//#endif

#ifdef TEST_MODEM

		if (etEdgeType==FallingEdge)
		{

			HAL_HPSwitch_Enable();
			HAL_LPSwitch_Disable();
			//HAL_Delay(1000);
		}
		else if (etEdgeType==RisingEdge)
		{

			HAL_HPSwitch_Disable();
			HAL_LPSwitch_Enable();
			//HAL_Delay(1000);
		}

#endif
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC_Init(void)
{

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.OversamplingMode = DISABLE;
	hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ContinuousConvMode = ENABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerFrequencyMode = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	if (HAL_ADC_Init(&hadc) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

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
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, TX__RF_SW_Pin|RX_RF_SW_Pin|MUX_ENABLE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CTRL_4_5V_Pin|CTRL_5V_Pin|CTRL_12V_Pin|CTRL_15V_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : TX__RF_SW_Pin RX_RF_SW_Pin MUX_ENABLE_Pin */
	GPIO_InitStruct.Pin = TX__RF_SW_Pin|RX_RF_SW_Pin|MUX_ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : CTRL_4_5V_Pin CTRL_5V_Pin */
	GPIO_InitStruct.Pin = CTRL_4_5V_Pin|CTRL_5V_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PULSE_Pin */
	GPIO_InitStruct.Pin = PULSE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PULSE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CTRL_12V_Pin CTRL_15V_Pin */
	GPIO_InitStruct.Pin = CTRL_12V_Pin|CTRL_15V_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

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

