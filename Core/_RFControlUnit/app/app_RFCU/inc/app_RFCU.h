/*
 * app_RFCU.h
 *
 *  Created on: Jan 2, 2023
 *      Author: Rahma
 */

#ifndef RFCONTROLUNIT_APP_APP_RFCU_INC_APP_RFCU_H_
#define RFCONTROLUNIT_APP_APP_RFCU_INC_APP_RFCU_H_

#define SENSOR_MAX_VOLT_VALUE     330
#define ADC_MAXIMUM_VALUE         4096
#define PC_TRANSMIT_DATA_SIZE   20

extern uint32_t g_aADCValueReversed[4];
extern uint8_t g_aPCTransmitData[PC_TRANSMIT_DATA_SIZE];
extern uint8_t g_ui8SensorValue[4];
/*******************************************************************************
 *Functions ProtoTypes                                  *
 *******************************************************************************/
uint8_t APP_RFCU_CheckSum(uint8_t aui8buffer[], uint8_t ui8Size);
//void APP_RFCU_ConvertBytes(uint32_t ui32IN0,uint32_t ui32IN1,uint32_t ui32IN2,uint32_t ui32IN3);
uint32_t APP_RFCU_ConvertBytes(uint32_t ui32IN);
void APP_RFCU_SetBytebyByte(uint32_t *ui32IN0, uint8_t *g_aPCTransmitData);

void APP_RFCU_Start(void);

#endif /* RFCONTROLUNIT_APP_APP_RFCU_INC_APP_RFCU_H_ */
