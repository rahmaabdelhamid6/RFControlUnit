/*
 * mcal_GPIO.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */
#include "stm32l0xx_hal.h"
#include "mcal_GPIO.h"

/*******************************************************************************
 *Functions Definitions                                  *
 *******************************************************************************/

void MCAL_GPIO_EnableControl(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
}
