/*
 * appcb_PC.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#include "stm32l0xx_hal.h"
#include "appcb_PC.h"

/*******************************************************************************
 *Global Variable                                *
 *******************************************************************************/
volatile uint8_t g_ui8PCRXFlag=0;
volatile uint8_t g_ui8PCTXFlag=0;
/*******************************************************************************
 *Functions Prototype                                                        *
 *******************************************************************************/
void APP_PC_RXCallback()
{
	g_ui8PCRXFlag=1;
}
void APP_PC_TXCallback()
{
	g_ui8PCTXFlag=1;
}
