/*
 * Sequence.c
 *
 *  Created on: Sep 2, 2023
 *      Author: r720r
 */

#include "GlobalVariables.h"
#include <stdint.h>
#include "main.h"
#include "SignalReadWrite.h"
#include "GeneralFunctions.h"
#include "GlobalConstants.h"
#include "GlobalStruct.h"
#include "GlobalVariables.h"
#include "Sequence.h"
#include "SixsStep.h"
#include "VectorControl.h"
#include "i2c.h"

static uint8_t sPosMode;
static uint8_t sDrvMode;
static uint16_t sInitCnt = 0;
static int8_t sOutputMode;
static float sDuty[3];
static struct SensorData sSensData;
static struct VectorControlData sVectorControlData;
static struct ElectAngleEstimateData sElectAngleEstimateData = {0.0f, 0.0f, 0.0f};
static float sJoyStickXY[2];

static inline void slctPosMode(float electFreq, uint8_t* posMode);
static inline void slctDrvMode(float electFreq, uint8_t* drvMode);
static inline void slctElectAngleFromPosMode(uint8_t posMode, struct SensorData *sensData);
static inline void slctCntlFromDrvMode(uint8_t drvMode, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode);
static inline void calcCurrentRef(uint8_t drvMode, struct VectorControlData *vectorControlData);

uint8_t temp_data[2] = {0};

void Sequence_Low_Freq(void){

	uint8_t drvMode_pre;

	//read IO signals
	readJoyStickXY(sJoyStickXY);
	gVolume = sJoyStickXY[1];

	sSensData.Vdc = readVdc();
	gLPF(sSensData.Vdc, ANGULARFREQ20Hz, LOWSEQUENCEPERIOD, &sSensData.Vdc_LPF);
	sSensData.twoDivVdc = gfDivideAvoidZero(2.0f, sSensData.Vdc_LPF, 1.0f);

	if(sInitCnt < INITCNTMAX){
		sInitCnt++;
		sPosMode = POSMODE_HALL;
		sDrvMode = DRVMODE_OFFDUTY;

		// Get Current Sensor Offset
		if( sInitCnt <= INITCNTST1){
			sSensData.Iuvw_AD_Offset[0] = 0.0f;
			sSensData.Iuvw_AD_Offset[1] = 0.0f;
			sSensData.Iuvw_AD_Offset[2] = 0.0f;
		}
		else if(sInitCnt <= INITCNTST1 + INITCNTST2){
			sSensData.Iuvw_AD_Offset[0] += (float)sSensData.Iuvw_AD[0] * ONEDIVINITCNTST2;
			sSensData.Iuvw_AD_Offset[1] += (float)sSensData.Iuvw_AD[1] * ONEDIVINITCNTST2;
			sSensData.Iuvw_AD_Offset[2] += (float)sSensData.Iuvw_AD[2] * ONEDIVINITCNTST2;
		}
	}
	else {
		slctPosMode(gElectFreq, &sPosMode);
		slctDrvMode(gElectFreq, &sDrvMode);

		if( drvMode_pre != DRVMODE_VECTORCONTROL && sDrvMode == DRVMODE_VECTORCONTROL){  // Init for VectorControl
			InitVectorControl(sSensData, &sVectorControlData);
		}
		calcCurrentRef(sDrvMode, &sVectorControlData);
	}
}

void Sequence_High_Freq(void){
	gTheta = readEncoderAngle();
	readCurrent(sSensData.Iuvw_AD, sSensData.Iuvw_AD_Offset, sSensData.Iuvw);

	slctElectAngleFromPosMode(sPosMode, &sSensData);
	slctCntlFromDrvMode(sDrvMode, sSensData, &sVectorControlData, sDuty, &sOutputMode);
	writeOutputMode(sOutputMode);

	writeDuty(sDuty);
}
void inline slctPosMode(float electFreq, uint8_t* posMode){
	*posMode = POSMODE_ANGLESENS;
}

void inline slctDrvMode(float electFreq, uint8_t* drvMode){
	*drvMode = DRVMODE_VECTORCONTROL;
}

static inline void slctElectAngleFromPosMode(uint8_t posMode, struct SensorData *sensData){

	switch(posMode){
	case POSMODE_STOP:
		sensData->electAngle = 0.0f;
		sensData->electAngVelo = 0.0f;
		break;
	case POSMODE_ANGLESENS:
		sensData->electAngle = gTheta;
		sensData->electAngVelo = sElectAngleEstimateData.electAngVeloEstimate;
		break;
	default:
		sensData->electAngle = 0.0f;
		sensData->electAngVelo = 0.0f;
		break;
	}
}

void inline slctCntlFromDrvMode(uint8_t drvMode, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode){

	float VamRef;

	switch(drvMode){
		case DRVMODE_OFFDUTY:
			gOffDuty(Duty, outputMode);
			break;
		case DRVMODE_OPENLOOP:
			VamRef = sSensData.Vdc * SQRT3DIV2_DIV2 * gVolume;
			OpenLoopTasks(VamRef, sensData, vectorControlData, Duty, outputMode);
			break;
		case DRVMODE_VECTORCONTROL:
			VectorControlTasks(sensData, vectorControlData, Duty, outputMode);
			break;
		default :
			gOffDuty(Duty, outputMode);
	}
}

static inline void calcCurrentRef(uint8_t drvMode, struct VectorControlData *vectorControlData){
	switch(drvMode){
		case DRVMODE_OFFDUTY:
			vectorControlData->Idq_ref[0] = 0.0f;
			vectorControlData->Idq_ref[1] = 0.0f;
			vectorControlData->Idq_ref_LPF[0] = 0.0f;
			vectorControlData->Idq_ref_LPF[1] = 0.0f;
			break;
		case DRVMODE_OPENLOOP:
			vectorControlData->Idq_ref[0] = vectorControlData->Idq_LPF[0];
			vectorControlData->Idq_ref[1] = vectorControlData->Idq_LPF[1];
			vectorControlData->Idq_ref_LPF[0] = vectorControlData->Idq_LPF[0];
			vectorControlData->Idq_ref_LPF[1] = vectorControlData->Idq_LPF[1];
			break;
		case DRVMODE_VECTORCONTROL:
			vectorControlData->Idq_ref[0] = 0.0f;
			vectorControlData->Idq_ref[1] = IQREFMAX * gVolume;
			gLPF(vectorControlData->Idq_ref[0], ANGULARFREQ5Hz, LOWSEQUENCEPERIOD, &vectorControlData->Idq_ref_LPF[0]);
			gLPF(vectorControlData->Idq_ref[1], ANGULARFREQ5Hz, LOWSEQUENCEPERIOD, &vectorControlData->Idq_ref_LPF[1]);
			break;
		default :
			vectorControlData->Idq_ref[0] = 0.0f;
			vectorControlData->Idq_ref[1] = 0.0f;
			vectorControlData->Idq_ref_LPF[0] = 0.0f;
			vectorControlData->Idq_ref_LPF[1] = 0.0f;
	}
}




