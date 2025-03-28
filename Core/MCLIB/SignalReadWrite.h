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
#define IU2_ADOffSET			1986
#define IV2_ADOffSET			1967
#define IW2_ADOffSET			1974
// #define AD2CURRENT			-0.00193586253f // for IHM07M1
// #define AD2CURRENT			0.085625f // for marutai tekito
#define AD2CURRENT			0.0032f // for roller485 DRV8311:0.25V/A.
//#define AD2CURRENT			-0.358844273 // for CQKIT
#define AD2VOLTAGE			0.0154305f; // for IHM07M1 1/(9.31/(9.31+169)*4096/3.3V)
//#define AD2VOLTAGE			0.025210084f; // for CQKIT


// Global Functions
uint8_t readButton1(void);
uint32_t readHallInputCaptureCnt(void);
uint16_t readPropoInputCaptureCnt(void);
uint16_t readPropoInputCaptureCnt2(void);

float readTimeInterval(uint32_t inputCaptureCnt, uint32_t inputCaptureCnt_pre);
float readPropoDuty(void);
float readPropoDuty2(void);
float readVolume(void);
float readVdc(void);
void readCurrent(uint16_t* Iuvw_AD, float* Iuvw_AD_Offset, float* Iuvw);
void readHallSignal(uint8_t* Hall);
void readElectFreqFromHallSignal(float* electFreq);
// void readCurrent2(uint16_t* Iuvw_AD, float* Iuvw);
void writeOutputMode(int8_t outputMode);
void writeDuty(float* Duty);
void writeFreeRunCnt(uint16_t Cnt);
uint16_t readFreeRunCnt(void);
// void writeDuty8(float* Duty);
// void writeDutyforOpenWinding(float* Duty);

#endif /* MCLIB_SIGNALREADWRITE_H_ */
