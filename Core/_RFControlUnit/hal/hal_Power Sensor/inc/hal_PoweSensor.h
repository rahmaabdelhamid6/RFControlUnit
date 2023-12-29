/*
 * hal_PoweSensor.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Rahma
 */

#ifndef RFCONTROLUNIT_HAL_HAL_POWER_SENSOR_INC_HAL_POWESENSOR_H_
#define RFCONTROLUNIT_HAL_HAL_POWER_SENSOR_INC_HAL_POWESENSOR_H_

extern uint32_t g_ui32AdcValue[4];

/*******************************************************************************
 *Functions ProtoTypes                                 *
 *******************************************************************************/
void HAL_PowerSensor1_GetValue(ADC_HandleTypeDef *hadc);
void HAL_PowerSensor2_GetValue(ADC_HandleTypeDef *hadc);
void HAL_PowerSensor3_GetValue(ADC_HandleTypeDef *hadc);
void HAL_PowerSensor4_GetValue(ADC_HandleTypeDef *hadc);

#endif /* RFCONTROLUNIT_HAL_HAL_POWER_SENSOR_INC_HAL_POWESENSOR_H_ */
