/*
 * hal_HPLPRFSwitch.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */


#include "stm32l0xx_hal.h"
#include "hal_HPLPRFSwitch.h"
/*******************************************************************************
 *Global Variables                                  *
 *******************************************************************************/

/*******************************************************************************
 *Functions Definition                                *
 *******************************************************************************/

void HAL_HPSwitch_Enable(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
}
void HAL_HPSwitch_Disable(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
}
void HAL_LPSwitch_Enable(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET);
}
void HAL_LPSwitch_Disable(void)
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET);
}


