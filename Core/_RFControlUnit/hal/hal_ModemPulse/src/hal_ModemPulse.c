/*
 * hal_ModemPulse.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#include "stm32l0xx_hal.h"
#include "hal_ModemPulse.h"
#include "mcal_isr_GPIO.h"

/*******************************************************************************
 *Global Variables                               *
 *******************************************************************************/
volatile Edgetype_et etEdgeType ;
//uint32_t g_ui32ModemPulsePreviousEdge;
//uint32_t g_ui32ModemPulseCurrentEdge;
/*******************************************************************************
 *Function Definition *
 *******************************************************************************/

void HAL_ModemPulse_Init(uint16_t GPIO_Pin,void (*pfnModemPulseCallback)(void))
{
	/*g_ui32ModemPulsePreviousEdge = GPIO_PIN_RESET;
	g_ui32ModemPulseCurrentEdge =HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);*/
	MCAL_GPIO_DetecEdgeCallback(GPIO_Pin ,pfnModemPulseCallback);

}

/*void HAL_ModemPulse_ReadEdge(void)
{
	if (etEdgeType==FallingEdge)
	{
		HAL_HPSwitch_Enable();
		HAL_HPSwitch_Disable();
		HAL_Delay(1000);
	}
	else if (etEdgeType==RisingEdge)
	{
		HAL_HPSwitch_Disable();
		HAL_HPSwitch_Enable();
		HAL_Delay(1000);
	}

}*/
