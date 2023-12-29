/*
 * app_RFCU.c
 *
 *  Created on: Jan 2, 2023
 *      Author: Rahma
 */


#include "stm32l0xx_hal.h"
#include "app_RFCU.h"
#include "hal_ModemPulse.h"
#include "hal_PC.h"
#include "hal_PoweSensor.h"
#include "hal_HPLPRFSwitch.h"
#include "appcb_PC.h"
#include "appcb_ModemPulse.h"




/*******************************************************************************
 *Global Variable                                *
 *******************************************************************************/
uint8_t g_aPCTransmitData[PC_TRANSMIT_DATA_SIZE];
uint8_t g_ui8SensorValue[4]={0,0,0,0};
//uint8_t g_aPCTransmitData[PC_TRANSMIT_DATA_SIZE]={0x55,0x14,0x13,0x0,0x0,0xa,0x63,0x0,0x0,0xa,0x80,0x0,0x0,0xa,0x82,0x0,0x0,0xa,0x71,0x25};
uint32_t g_aADCValueReversed[4];
uint8_t *g_pBufRX;
uint8_t g_DataSizeRx=0;
uint8_t g_ui8Sum= 0;
extern ADC_HandleTypeDef hadc;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
/*******************************************************************************
 *Functions Definitions                                  *
 *******************************************************************************/
uint8_t APP_RFCU_CheckSum(uint8_t aui8buffer[], uint8_t ui8Size)
{
	g_ui8Sum=0;
	for(uint8_t i=1; i<ui8Size-1; i++)
	{
		g_ui8Sum=g_ui8Sum+ aui8buffer[i];
	}
	if (g_ui8Sum>=0x00 && g_ui8Sum<=0xFF && g_ui8Sum== aui8buffer[ui8Size-1])
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
uint32_t APP_RFCU_ConvertBytes(uint32_t ui32IN)
{
	uint32_t ui32OUT;
	uint8_t *p_ui8IN = (uint8_t*) &ui32IN;
	uint8_t *p_ui8OUT = (uint8_t*) &ui32OUT;
	p_ui8OUT[0] = p_ui8IN[3];
	p_ui8OUT[1] = p_ui8IN[2];
	p_ui8OUT[2] = p_ui8IN[1];
	p_ui8OUT[3] = p_ui8IN[0];

	return ui32OUT;

}

void APP_RFCU_SetBytebyByte(uint32_t *ui32IN0, uint8_t *g_aPCTransmitData)
{
	uint8_t j=3;
	uint8_t *p_ui8IN0 = (uint8_t*) ui32IN0;
	for(uint8_t i=0; i<16 ; i++)
	{
		g_aPCTransmitData[j]=p_ui8IN0[i];
		j++;
	}
	APP_RFCU_CheckSum(g_aPCTransmitData, 20);
	g_aPCTransmitData[19] = g_ui8Sum;

	/*for (uint8_t K=1; K<19; K++)
	{
		ui8TXSum+=g_aPCTransmitData[K];
	}
	g_aPCTransmitData[19] = ui8TXSum;*/
}

void APP_RFCU_Start(void)
{
	uint8_t ui8RXCheck;
	uint8_t ui8TXCheck;
	//	uint32_t ui32TempValue1=0;

	if (g_ui8PCRXFlag==1)
	{
		g_ui8PCRXFlag=0;
		g_pBufRX=HAL_PC_GetData(&g_DataSizeRx);
		ui8RXCheck=APP_RFCU_CheckSum(g_pBufRX, g_DataSizeRx);

		if (ui8RXCheck==1)
		{
			HAL_PowerSensor1_GetValue(&hadc);
			g_aADCValueReversed[0]=APP_RFCU_ConvertBytes(g_ui32AdcValue[0]);
			//			ui32TempValue1=(uint32_t)(((uint32_t)g_aADCValueReversed*SENSOR_MAX_VOLT_VALUE)/ADC_MAXIMUM_VALUE);
			HAL_PowerSensor2_GetValue(&hadc);
			g_aADCValueReversed[1]=APP_RFCU_ConvertBytes(g_ui32AdcValue[1]);
			HAL_PowerSensor3_GetValue(&hadc);
			g_aADCValueReversed[2]=APP_RFCU_ConvertBytes(g_ui32AdcValue[2]);
			HAL_PowerSensor4_GetValue(&hadc);
			g_aADCValueReversed[3]=APP_RFCU_ConvertBytes(g_ui32AdcValue[3]);
			//HAL_ADC_Stop(&hadc);
			APP_RFCU_SetBytebyByte(g_aADCValueReversed,g_aPCTransmitData);
			ui8TXCheck=APP_RFCU_CheckSum(g_aPCTransmitData, 20);
			if(ui8TXCheck==1)
			{
				HAL_PC_Write(&huart2 ,&hdma_usart2_rx,g_aPCTransmitData, 20);
				while (g_ui8PCTXFlag == 0);
				g_ui8PCTXFlag = 0;

			}

			//		}
			else
			{
				/*Do nothing*/
			}
		}
	}

		if (etEdgeType==FallingEdge)
		{
			HAL_HPSwitch_Enable();
			HAL_LPSwitch_Disable();
			//HAL_Delay(1000);
		}
		else if (etEdgeType==RisingEdge)
		{
			HAL_HPSwitch_Disable();
			HAL_LPSwitch_Enable();
			//HAL_Delay(1000);
		}
		else
		{
			/*Do nothing*/
		}
}
