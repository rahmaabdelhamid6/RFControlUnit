/*
 * mcal_ADC.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */
#include "stm32l0xx_hal.h"
#include "mcal_ADC.h"

extern ADC_HandleTypeDef hadc;

/*******************************************************************************
 *Functions Definition                               *
 *******************************************************************************/

void MCAL_ADC_SelectCH6(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_6;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void MCAL_ADC_SelectCH7(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_7;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

}

void MCAL_ADC_SelectCH8(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_8;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_3CYCLES_5;
	sConfig.Rank =ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void MCAL_ADC_SelectCH9(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_9;
	hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}
