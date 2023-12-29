/*
 * hal_PC.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#include "stm32l0xx_hal.h"
#include "hal_PC.h"
#include "mcal_isr_UART.h"

/*******************************************************************************
 *Global Variables                               *
 *******************************************************************************/
uint8_t g_aui8PCBufRX[RXBUF_SIZE];

uint8_t g_ui8SizeBuf;
extern  UART_HandleTypeDef huart2;
extern  DMA_HandleTypeDef hdma_usart2_rx;
/*******************************************************************************
 *Functions Definitions                                  *
 *******************************************************************************/
static void (*gs_pfnHALPCRXCallback) (void)= NULL;
uint8_t* HAL_PC_GetData(uint8_t *pRxDataSize)
{
	*pRxDataSize=g_ui8SizeBuf;
	return g_aui8PCBufRX;
}
void HAL_PC_RXCallback(UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_rx,uint16_t Size)
{
	if (gs_pfnHALPCRXCallback!= NULL)
	{
		gs_pfnHALPCRXCallback();
	}

	g_ui8SizeBuf=Size;
	HAL_UARTEx_ReceiveToIdle_DMA(huart2, g_aui8PCBufRX, RXBUF_SIZE);
	__HAL_DMA_DISABLE_IT(hdma_usart2_rx, DMA_IT_HT);
}

void HAL_PC_Init( void (*pfnPCRXCallback) (void),void (*pfnPCTXCallback) (UART_HandleTypeDef *huart2))
{
	gs_pfnHALPCRXCallback=pfnPCRXCallback;
	g_pfnPCRXCallback=HAL_PC_RXCallback;
	g_pfnPCTXCallback=pfnPCTXCallback;

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, g_aui8PCBufRX, RXBUF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

}
//void HAL_PC_Read(UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_rx)
//{
//	HAL_UARTEx_ReceiveToIdle_DMA(huart2, g_aui8PCBufRX, RXBUF_SIZE);
//	__HAL_DMA_DISABLE_IT(hdma_usart2_rx, DMA_IT_HT);
//}

void HAL_PC_Write(UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_tx,uint8_t *pTransmitData, uint16_t Size)
{
	HAL_UART_Transmit_DMA(huart2, pTransmitData, Size);
	__HAL_DMA_DISABLE_IT(hdma_usart2_tx, DMA_IT_HT);
}
