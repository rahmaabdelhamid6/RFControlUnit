/*
 * hal_PC.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#ifndef RFCONTROLUNIT_HAL_HAL_PC_INC_HAL_PC_H_
#define RFCONTROLUNIT_HAL_HAL_PC_INC_HAL_PC_H_

#define RXBUF_SIZE   4

/*******************************************************************************
 *Functions ProtoTypes                                  *
 *******************************************************************************/
uint8_t* HAL_PC_GetData(uint8_t *pRxDataSize);
void HAL_PC_RXCallback(UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_rx,uint16_t Size);
void HAL_PC_Init( void (*pfnPCRXCallback) (void),void (*pfnPCTXCallback) (UART_HandleTypeDef *huart2));
//void HAL_PC_Read(UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_rx);
void HAL_PC_Write(UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_rx,uint8_t *pTransmitData, uint16_t Size);

#endif /* RFCONTROLUNIT_HAL_HAL_PC_INC_HAL_PC_H_ */
