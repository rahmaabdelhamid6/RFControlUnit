/*
 * hal_ModemPulse.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#ifndef RFCONTROLUNIT_HAL_HAL_MODEMPULSE_INC_HAL_MODEMPULSE_H_
#define RFCONTROLUNIT_HAL_HAL_MODEMPULSE_INC_HAL_MODEMPULSE_H_

/*******************************************************************************
 *Definition                              *
 *******************************************************************************/
typedef enum{
	FallingEdge=0,
	RisingEdge=1
}Edgetype_et;


extern volatile Edgetype_et etEdgeType;
//extern uint32_t g_ui32ModemPulsePreviousEdge;
//extern uint32_t g_ui32ModemPulseCurrentEdge;
/*******************************************************************************
 *Functions ProtoTypes                                 *
 *******************************************************************************/
void HAL_ModemPulse_Init(uint16_t GPIO_Pin,void (*pfnModemPulseCallback)(void));
void HAL_ModemPulse_ReadEdge(void);

#endif /* RFCONTROLUNIT_HAL_HAL_MODEMPULSE_INC_HAL_MODEMPULSE_H_ */
