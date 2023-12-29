/*
 * appcb_PC.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#ifndef RFCONTROLUNIT_APP_APP_CALLBACK_APPCB_PC_INC_APPCB_PC_H_
#define RFCONTROLUNIT_APP_APP_CALLBACK_APPCB_PC_INC_APPCB_PC_H_

/*******************************************************************************
 *Extern variables                                                          *
 *******************************************************************************/
extern volatile uint8_t g_ui8PCRXFlag;
extern volatile uint8_t g_ui8PCTXFlag;
/*******************************************************************************
 *Functions ProtoTypes                                                         *
 *******************************************************************************/
void APP_PC_RXCallback();
void APP_PC_TXCallback();
#endif /* RFCONTROLUNIT_APP_APP_CALLBACK_APPCB_PC_INC_APPCB_PC_H_ */
