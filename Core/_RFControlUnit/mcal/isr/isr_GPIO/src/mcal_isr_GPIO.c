/*
 * mcal_isr_GPIO.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#include "stm32l0xx_hal.h"
#include "mcal_isr_GPIO.h"

void (*g_pfnModemCallback) (void)= NULL;
uint16_t g_ui8GPIOPinNumber;

void MCAL_GPIO_DetecEdgeCallback (uint16_t GPIO_Pin , void (*pfnModemPulseCallback) (void))
{
	g_ui8GPIOPinNumber=GPIO_Pin;
	g_pfnModemCallback = pfnModemPulseCallback;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == g_ui8GPIOPinNumber )
	{
		if (g_pfnModemCallback != NULL)
		{
			g_pfnModemCallback();
		}
		else
		{
			/*do nothing*/
		}
	}
}
