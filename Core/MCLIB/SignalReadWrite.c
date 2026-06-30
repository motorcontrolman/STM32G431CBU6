/*
 * signalReadWrite.c
 *
 *  Created on: May 7, 2023
 *      Author: r720r
 */

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "SignalReadWrite.h"
#include "GlobalConstants.h"
#include "GlobalVariables.h"
#include "GeneralFunctions.h"
#include "i2c.h"

float angle_corrected;

static inline uint8_t SPI_TransmitReceive(SPI_HandleTypeDef * hspi, uint16_t TxData, uint16_t *RxData);

float readVdc(void){
	float Vdc;
	uint16_t Vdc_ad = ADC1 -> JDR1;
	Vdc = Vdc_ad * AD2VOLTAGE;
	return Vdc;
}

void readCurrent(uint16_t* Iuvw_AD, float* Iuvw_AD_Offset, float* Iuvw){
	Iuvw_AD[0] = ADC1 -> JDR2; // Iu
	Iuvw_AD[1] = ADC1 -> JDR3; // Iv
	Iuvw_AD[2] = ADC1 -> JDR4; // Iw

	Iuvw[0] = ((float)Iuvw_AD[0] - Iuvw_AD_Offset[0]) * AD2CURRENT;
	Iuvw[1] = ((float)Iuvw_AD[1] - Iuvw_AD_Offset[1]) * AD2CURRENT;
	Iuvw[2] = ((float)Iuvw_AD[2] - Iuvw_AD_Offset[2]) * AD2CURRENT;
}

void writeOutputMode(int8_t outputMode){

	// if the outputMode is OPEN, set Enable Pin to RESET.
	if(outputMode == OUTPUTMODE_RESET )
	{
		HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(PWM_EN_GPIO_Port, PWM_EN_Pin, GPIO_PIN_SET);
	}

}


void writeDuty(float* Duty){
	// TIM1 -> ARR Means Counter Period of TIM1
	TIM1 -> CCR1 = Duty[2] * (TIM1 -> ARR);
	TIM1 -> CCR2 = Duty[1] * (TIM1 -> ARR);
	TIM1 -> CCR3 = Duty[0] * (TIM1 -> ARR);

}

void writeFreeRunCnt(uint16_t Cnt){
	TIM16 -> CNT = Cnt;
}

uint16_t readFreeRunCnt(void){
	uint16_t Cnt;
	Cnt = TIM16 -> CNT;
	return Cnt;
}


float readEncoderAngle(void){
	uint16_t data_t[2];
	uint16_t data_r[2];
	float angle_get;
	//float angle_corrected;
	uint16_t	angle;
	uint16_t	angle_corrected_16bit;
	float encoderAngle;

	data_t[0] = 0x8021;
	data_t[1] = 0xffff;

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	__HAL_SPI_ENABLE(&hspi1);
	SPI_TransmitReceive(&hspi1, data_t[0],&data_r[0]);
	SPI_TransmitReceive(&hspi1, data_t[1],&data_r[1]);

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	angle = (((data_r[1] & 0x7fff) << 1) >> 2);
	angle_get = 16383.0f - angle;//(float)(~(16383 - angle));
    if( angle_get > ANGLEOFFSET )
    {
        angle_corrected = angle_get - ANGLEOFFSET;
    }
    else
    {
        angle_corrected = 16383.0f - (ANGLEOFFSET - angle_get);
    }

    angle_corrected_16bit = (((uint16_t)angle_corrected) << 2 ) * 7;
    encoderAngle = angle_corrected_16bit * TWOPIDIVBITMAX16;
    return encoderAngle;
}

static uint8_t SPI_TransmitReceive(SPI_HandleTypeDef * hspi, uint16_t TxData, uint16_t *RxData)
{
  volatile uint32_t cnt = 0;

   while ((hspi->Instance->SR & SPI_SR_TXE) == 0)
   {
	;
   }
    hspi->Instance->DR = TxData;

    while ((hspi->Instance->SR & SPI_SR_RXNE)==0)
     {
	;
   	}
        if((hspi->Instance->SR & SPI_SR_RXNE))
        {
            *RxData = hspi->Instance->DR;
            return 0;
        }
        cnt++;

    return 1;
	while ((hspi->Instance->SR & SPI_SR_TXE) == 0);
}

void readJoyStickXY(float* joyStickXY){
    uint8_t i2c_address = 0x63;
    i2c_address = ((i2c_address << 1) | 1);
	uint16_t i2c_len = 2;
	uint16_t i2c_success = 0;
	uint8_t i2c_reg = 0x10;
	uint8_t temp_data[2] = {0};

	LL_I2C_Disable(I2C1);
	I2C1_Start();
	i2c_success = I2C_Read_Bytes(i2c_address, i2c_reg, temp_data, i2c_len, 10);
	i2c_success = !i2c_success;

	joyStickXY[0] = ((float)temp_data[0] - 122) * 0.005;
	joyStickXY[1] = ((float)temp_data[1] - 122) * 0.005;
}
