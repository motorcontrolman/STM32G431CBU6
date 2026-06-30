/*
 * common.c
 *
 *  Created on: Apr 22, 2023
 *      Author: r720r
 */


#include <stdint.h>
#include "main.h"
#include "GlobalVariables.h"

uint16_t gAdcValue[2];
uint32_t gTIMCounter;
uint32_t gTIMCounter_pre;

float gElectFreq = 0;
float gTheta = 0;
float gElectAngVelo;
float gVdc;
float gTwoDivVdc;
float gVolume;
int8_t gOutputMode[3];
float gDutyRef = 0;
float gDuty[3];
uint16_t gFreerunCnt = 0;
float gProcessingLoad;

