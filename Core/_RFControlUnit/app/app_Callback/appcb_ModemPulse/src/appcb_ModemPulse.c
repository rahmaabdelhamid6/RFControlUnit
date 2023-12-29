/*
 * appcb_ModemPulse.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#include "stm32l0xx_hal.h"
#include "hal_ModemPulse.h"
#include "appcb_ModemPulse.h"



/*******************************************************************************
 *Functions Definition                                 *
 *******************************************************************************/

void APP_ModemPulse_DetecEdgeCallback(void)
{

	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)==GPIO_PIN_RESET)
	{
		etEdgeType=FallingEdge;
	}
	else if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)==GPIO_PIN_SET)
	{
		etEdgeType=RisingEdge;
	}
	else
	{
		/*do nothing*/
	}
}
