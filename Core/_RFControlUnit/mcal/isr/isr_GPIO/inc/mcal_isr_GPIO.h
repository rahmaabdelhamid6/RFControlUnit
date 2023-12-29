/*
 * mcal_isr_GPIO.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#ifndef RFCONTROLUNIT_MCAL_ISR_ISR_GPIO_INC_MCAL_ISR_GPIO_H_
#define RFCONTROLUNIT_MCAL_ISR_ISR_GPIO_INC_MCAL_ISR_GPIO_H_

void MCAL_GPIO_DetecEdgeCallback (uint16_t GPIO_Pin , void (*pfnModemPulseCallback) (void));
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

#endif /* RFCONTROLUNIT_MCAL_ISR_ISR_GPIO_INC_MCAL_ISR_GPIO_H_ */
