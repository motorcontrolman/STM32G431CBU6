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

static uint16_t sNoInputCaptureCnt = 0;
static uint32_t sHallInputCaptureCnt;
static uint32_t sHallInputCaptureCnt_pre;
static uint16_t sPropoInputCaptureCntRise;
static uint16_t sPropoInputCaptureCntFall;
static uint8_t sPropoState;
static uint8_t sPropoState_pre;
static float sPropoDuty = 0;
float propoInputCaptureCntDiff;

static uint16_t sPropoInputCaptureCntRise2;
static uint16_t sPropoInputCaptureCntFall2;
static uint8_t sPropoState2;
static uint8_t sPropoState_pre2;
static float sPropoDuty2 = 0;
float propoInputCaptureCntDiff2;


static uint16_t inputCaptureCnt8;

uint16_t Bemf_AD[3];

float angle_corrected;

static inline uint8_t SPI_TransmitReceive(SPI_HandleTypeDef * hspi, uint16_t TxData, uint16_t *RxData);

uint8_t readButton1(void){
	volatile uint8_t B1;

	B1 = HAL_GPIO_ReadPin(SYS_SW_GPIO_Port, SYS_SW_Pin);
	B1 = 0b00000001 & (~B1);
	return B1;
}

uint32_t readHallInputCaptureCnt(void){
	// Read Input Capture Count of GPIO
	// CCR1:TIM2 Channel1 = H1, CCR2:Channel2 = H2, CCR3:Channel3 = H3
	volatile uint32_t inputCaptureCnt;

	inputCaptureCnt = TIM2 -> CCR1;

	return inputCaptureCnt;
}

/*
uint16_t readPropoInputCaptureCnt(void){
	// Read Input Capture Count of GPIO
	// CCR1:TIM8 Channel1 = Propo
	volatile uint16_t inputCaptureCnt;

	inputCaptureCnt = TIM8 -> CCR1;
	return inputCaptureCnt;
}


float readPropoDuty(void){
	float propoDuty;



	uint32_t inputCaptureCntMax;
	uint32_t inputCaptureCntHalf;
	float preScaler;

	sPropoState_pre = sPropoState;
	sPropoState = HAL_GPIO_ReadPin(GPIOA, Propo_Pin) & 0b00000001;

	if(sPropoState) // sPropoState = ON
		sPropoInputCaptureCntRise = readPropoInputCaptureCnt();
	else			// sPropoState = OFF
	{
		sPropoInputCaptureCntFall = readPropoInputCaptureCnt();

		// Detect Falling Edge, Update propoDuty
		if(sPropoState == 0 && sPropoState_pre == 1)
		{

			inputCaptureCntMax = TIM8 -> ARR;
			inputCaptureCntHalf = (inputCaptureCntMax + 1) >> 1;
			preScaler = (float)(TIM8 -> PSC);

			propoInputCaptureCntDiff = (float)sPropoInputCaptureCntFall - (float)sPropoInputCaptureCntRise;

			if( propoInputCaptureCntDiff < - (float)inputCaptureCntHalf)
				propoInputCaptureCntDiff += (float)inputCaptureCntMax;

			// Default 1489 Max 1857 Min 1119 Ampritude:370
			sPropoDuty = 1.0f *(propoInputCaptureCntDiff - 1489.0f) * 0.0027f;
			//if(sPropoDuty < 0.0f) sPropoDuty = 0.0f;

		}
	}

	propoDuty = sPropoDuty;
	return propoDuty;

}

uint16_t readPropoInputCaptureCnt2(void){
	// Read Input Capture Count of GPIO
	// CCR4:TIM3 Channel4 = Propo2
	volatile uint16_t inputCaptureCnt;

	inputCaptureCnt = TIM3 -> CCR4;
	inputCaptureCnt8 = inputCaptureCnt;
	return inputCaptureCnt;
}

float readPropoDuty2(void){
	float propoDuty;



	uint32_t inputCaptureCntMax;
	uint32_t inputCaptureCntHalf;
	float preScaler;

	sPropoState_pre2 = sPropoState2;
	sPropoState2 = HAL_GPIO_ReadPin(GPIOB, Propo2_Pin) & 0b00000001;

	if(sPropoState2) // sPropoState = ON
		sPropoInputCaptureCntRise2 = readPropoInputCaptureCnt2();
	else			// sPropoState = OFF
	{
		sPropoInputCaptureCntFall2 = readPropoInputCaptureCnt2();

		// Detect Falling Edge, Update propoDuty
		if(sPropoState2 == 0 && sPropoState_pre2 == 1)
		{

			inputCaptureCntMax = TIM3 -> ARR;
			inputCaptureCntHalf = (inputCaptureCntMax + 1) >> 1;
			preScaler = (float)(TIM3 -> PSC);

			propoInputCaptureCntDiff2 = (float)sPropoInputCaptureCntFall2 - (float)sPropoInputCaptureCntRise2;

			if( propoInputCaptureCntDiff2 < - (float)inputCaptureCntHalf)
				propoInputCaptureCntDiff2 += (float)inputCaptureCntMax;

			// Default 1489 Max 1857 Min 1119 Ampritude:370
			sPropoDuty2 = 1.0f *(propoInputCaptureCntDiff2 - 1489.0f) * 0.0027f;
			//if(sPropoDuty < 0.0f) sPropoDuty = 0.0f;

		}
	}

	propoDuty = sPropoDuty2;
	return propoDuty;

}
*/

float readTimeInterval(uint32_t inputCaptureCnt, uint32_t inputCaptureCnt_pre){

	float inputCaptureCntDiff;
	float timeInterval;
	uint32_t inputCaptureCntMax;
	uint32_t inputCaptureCntHalf;

	// TIM2 -> ARR Means Counter Period of TIM2
	inputCaptureCntMax = TIM2 -> ARR;
	inputCaptureCntHalf = (inputCaptureCntMax + 1) >> 1;


	inputCaptureCntDiff = (float)inputCaptureCnt - (float)inputCaptureCnt_pre;

	if( inputCaptureCntDiff < - (float)inputCaptureCntHalf)
	  inputCaptureCntDiff += (float)inputCaptureCntMax;

	timeInterval = inputCaptureCntDiff * SYSTEMCLOCKCYCLE;

	return timeInterval;
}

float readVolume(void){
	// P-NUCLEO-IHM001(or 002), Volume is connected to PB1(ADC12)
	// BLM_KIT_Ver1_5, Accel is connected  is connected to PC2(ADC8)
	float Volume;
	uint16_t Volume_ad = gAdcValue[1];

	Volume = ((int16_t)Volume_ad - 99)* 0.0002442f;
	//Volume = ((int16_t)Volume_ad - 950) * 0.000573394f;
	if( Volume < 0) Volume = 0;
	return Volume;
}

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


void readElectFreqFromHallSignal(float* electFreq){

	float timeInterval;

	sHallInputCaptureCnt_pre = sHallInputCaptureCnt;
	sHallInputCaptureCnt = readHallInputCaptureCnt();

	// Calculate Electrical Freq From Input Capture Count
	if(sHallInputCaptureCnt != sHallInputCaptureCnt_pre){
		timeInterval = readTimeInterval(sHallInputCaptureCnt, sHallInputCaptureCnt_pre);
		*electFreq = gfDivideAvoidZero(1.0f, timeInterval, 0.0001f);

		sNoInputCaptureCnt = 0;
	}
	// If Input Capture Count keep same value, Set Electrical Freq Zero
	else if(sNoInputCaptureCnt < 2000)
		sNoInputCaptureCnt ++;
	else
		*electFreq = 0;
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
