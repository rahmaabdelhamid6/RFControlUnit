/*
 * mcal_isr_UART.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */
#include "stm32l0xx_hal.h"
#include "mcal_isr_UART.h"
/*******************************************************************************
 *Functions Definition                                *
 *******************************************************************************/

extern DMA_HandleTypeDef hdma_usart2_rx;

/*******************************************************************************
 *Functions Definition                                *
 *******************************************************************************/
void (*g_pfnPCRXCallback) (UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_rx,uint16_t Size)= NULL;


void (*g_pfnPCTXCallback) (UART_HandleTypeDef *huart2 );

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart2, uint16_t Size)
{
	if (g_pfnPCRXCallback != NULL)
	{
		g_pfnPCRXCallback(huart2,&hdma_usart2_rx,Size);
	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart2)
{
	if (g_pfnPCTXCallback != NULL)
	{
		g_pfnPCTXCallback(huart2);
	}

}
