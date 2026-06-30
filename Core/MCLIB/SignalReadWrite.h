/*
 * signalReadWrite.h
 *
 *  Created on: May 7, 2023
 *      Author: r720r
 */

#ifndef MCLIB_SIGNALREADWRITE_H_
#define MCLIB_SIGNALREADWRITE_H_

#include <stdint.h>
#include "main.h"

#define IU_ADOffSET			1917
#define IV_ADOffSET			1881
#define IW_ADOffSET			1907
#define AD2CURRENT			0.0032f // for roller485 DRV8311:0.25V/A.
#define AD2VOLTAGE			0.00538f; // for roller485
#define ANGLEOFFSET			0.0f

// Global Functions
float readVdc(void);
void readCurrent(uint16_t* Iuvw_AD, float* Iuvw_AD_Offset, float* Iuvw);
void writeOutputMode(int8_t outputMode);
void writeDuty(float* Duty);
void writeFreeRunCnt(uint16_t Cnt);
uint16_t readFreeRunCnt(void);
float readEncoderAngle(void);
void readJoyStickXY(float* joyStickXY);


#endif /* MCLIB_SIGNALREADWRITE_H_ */
