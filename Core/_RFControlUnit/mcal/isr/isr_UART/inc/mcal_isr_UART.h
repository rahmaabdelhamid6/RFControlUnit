/*
 * mcal_isr_UART.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#ifndef RFCONTROLUNIT_MCAL_ISR_ISR_UART_INC_MCAL_ISR_UART_H_
#define RFCONTROLUNIT_MCAL_ISR_ISR_UART_INC_MCAL_ISR_UART_H_

/*******************************************************************************
 *Global Variable                               *
 *******************************************************************************/
extern void (*g_pfnPCRXCallback) (UART_HandleTypeDef *huart2 , DMA_HandleTypeDef *hdma_usart2_rx,uint16_t Size);
extern void (*g_pfnPCTXCallback) (UART_HandleTypeDef *huart2 );

/*******************************************************************************
 *Function Prototypes                                 *
 *******************************************************************************/

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart2, uint16_t Size);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart2);

#endif /* RFCONTROLUNIT_MCAL_ISR_ISR_UART_INC_MCAL_ISR_UART_H_ */
