/*
 * hal_PowerSensor.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#include "stm32l0xx_hal.h"
#include "hal_PoweSensor.h"
#include "mcal_ADC.h"

extern ADC_HandleTypeDef hadc;

uint32_t g_ui32AdcValue[4]={0,0,0,0};

/*******************************************************************************
 *Functions Definitions                                  *
 *******************************************************************************/
void HAL_PowerSensor1_GetValue(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	g_ui32AdcValue[0]=HAL_ADC_GetValue(hadc);
}

void HAL_PowerSensor2_GetValue(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	g_ui32AdcValue[1]=HAL_ADC_GetValue(hadc);
}

void HAL_PowerSensor3_GetValue(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	g_ui32AdcValue[2]=HAL_ADC_GetValue(hadc);
}

void HAL_PowerSensor4_GetValue(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
	g_ui32AdcValue[3]=HAL_ADC_GetValue(hadc);
}
