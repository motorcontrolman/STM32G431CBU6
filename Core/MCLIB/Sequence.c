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

static uint8_t sPosMode;
static uint8_t sDrvMode;
static uint16_t sInitCnt = 0;
static float sElectAngleFreerun = 0;
static float sElectAngVeloRefRateLimit = 0;
static int8_t sOutputMode;
static float sDuty[3];
static struct SensorData sSensData;
static struct VectorControlData sVectorControlData;
static struct ElectAngleEstimateData sElectAngleEstimateData = {0.0f, 0.0f, 0.0f};

static inline void slctPosMode(float electFreq, uint8_t* posMode);
static inline void slctDrvMode(float electFreq, uint8_t* drvMode);
static inline void slctPosModeForSensorless(float electAngVelo, uint8_t* posMode);
static inline void slctDrvModeForSensorless(float electAngVelo, uint8_t* drvMode);
static inline void slctElectAngleFromPosMode(uint8_t posMode, struct SensorData *sensData);
static inline void slctCntlFromDrvMode(uint8_t drvMode, struct SensorData sensData, struct VectorControlData *vectorControlData, float* Duty, int8_t* outputMode);
static inline void calcCurrentRef(uint8_t drvMode, struct VectorControlData *vectorControlData);


void Sequence_Low_Freq(void){

	uint8_t posMode_pre;
	uint8_t drvMode_pre;
	float propoDuty;
	float propoDuty2;


	//read IO signals
	//gButton1 = readButton1();
	gVolume = 0; //readVolume();

	propoDuty = readPropoDuty();
	propoDuty2 = readPropoDuty2();
	gLPF(propoDuty, ANGULARFREQ2Hz, CARRIERCYCLE, &gPropoDuty);
	gLPF(propoDuty2, ANGULARFREQ2Hz, CARRIERCYCLE, &gPropoDuty2);

	sSensData.Vdc = 10.0f;//readVdc();
	gLPF(sSensData.Vdc, ANGULARFREQ20Hz, LOWSEQUENCEPERIOD, &sSensData.Vdc_LPF);
	sSensData.twoDivVdc = gfDivideAvoidZero(2.0f, sSensData.Vdc_LPF, 1.0f);

	if(sInitCnt < INITCNTMAX){
		sInitCnt++;
		sPosMode = POSMODE_HALL;
		sDrvMode = DRVMODE_OFFDUTY;
		sElectAngVeloRefRateLimit = 0;

		// Get Current Sensor Offset
		if( sInitCnt <= INITCNTST1){
			sSensData.Iuvw_AD_Offset[0] = 2110.0f;
			sSensData.Iuvw_AD_Offset[1] = 2019.0f;
			sSensData.Iuvw_AD_Offset[2] = 2066.0f;
		}
		else if(sInitCnt <= INITCNTST1 + INITCNTST2){
			//sSensData.Iuvw_AD_Offset[0] += (float)sSensData.Iuvw_AD[0] * ONEDIVINITCNTST2;
			//sSensData.Iuvw_AD_Offset[1] += (float)sSensData.Iuvw_AD[1] * ONEDIVINITCNTST2;
			//sSensData.Iuvw_AD_Offset[2] += (float)sSensData.Iuvw_AD[2] * ONEDIVINITCNTST2;
		}
	}
	else {
		posMode_pre = sPosMode;
		drvMode_pre = sDrvMode;

		slctPosMode(gElectFreq, &sPosMode);
		slctDrvMode(gElectFreq, &sDrvMode);


		if( drvMode_pre == DRVMODE_OPENLOOP && sDrvMode == DRVMODE_VECTORCONTROL){  // Init for VectorControl
			InitVectorControl(sSensData, &sVectorControlData);
		}

		calcCurrentRef(sDrvMode, &sVectorControlData);
	}
}

void Sequence_High_Freq(void){
	// clear Free Running Counter
	// writeFreeRunCnt(ZERO);

	SPI_TransmitReceive_lap();
	readCurrent(sSensData.Iuvw_AD, sSensData.Iuvw_AD_Offset, sSensData.Iuvw);

	// for debug
	sPosMode = POSMODE_ANGLESENS;//POSMODE_FREERUN;//////
	sDrvMode = DRVMODE_VECTORCONTROL;//DRVMODE_OPENLOOP;//
	sElectAngVeloRefRateLimit = TWOPI * 10.0f;


	slctElectAngleFromPosMode(sPosMode, &sSensData);
	slctCntlFromDrvMode(sDrvMode, sSensData, &sVectorControlData, sDuty, &sOutputMode);
	writeOutputMode(sOutputMode);

	writeDuty(sDuty);

	// Calculate ProcessingLoad
	//gFreerunCnt = readFreeRunCnt();
	//gProcessingLoad = (float)gFreerunCnt * ONEDIVCARRIERCNT;
}
void inline slctPosMode(float electFreq, uint8_t* posMode){

	if(*posMode != POSMODE_HALL_PLL){
		if (electFreq > ELECTFREQ_VALIDPLL)
			*posMode = POSMODE_HALL_PLL;
		else
			*posMode = POSMODE_HALL;
	}
	else if(*posMode == POSMODE_HALL_PLL){
		if (electFreq < ELECTFREQ_INVALIDPLL)
			*posMode = POSMODE_HALL;
		else
			*posMode = POSMODE_HALL_PLL;
	}

}

void inline slctDrvMode(float electFreq, uint8_t* drvMode){

	if(*drvMode != DRVMODE_VECTORCONTROL){
		if (electFreq > ELECTFREQ_OPENLOOP2VECTORCONTROL)
			*drvMode = DRVMODE_VECTORCONTROL;
		else
			*drvMode = DRVMODE_OPENLOOP;
	}
	else if(*drvMode == DRVMODE_VECTORCONTROL){
		if (electFreq < ELECTFREQ_VECTORCONTROL2OPENLOOP)
			*drvMode = DRVMODE_OPENLOOP;
		else
			*drvMode = DRVMODE_VECTORCONTROL;
	}
}

static inline void slctPosModeForSensorless(float electAngVelo, uint8_t* posMode){

	if(*posMode != POSMODE_SENSORLESS){
		if (electAngVelo > ELECTANGVELO_FREERUN2SENSORLESS)
			*posMode = POSMODE_SENSORLESS;
		else
			*posMode = POSMODE_FREERUN;
	}
	else if(*posMode == POSMODE_SENSORLESS){
		if (electAngVelo < ELECTANGVELO_SENSORLESS2FREERUN)
			*posMode = POSMODE_FREERUN;
		else
			*posMode = POSMODE_SENSORLESS;
	}
}

static inline void slctDrvModeForSensorless(float electAngVelo, uint8_t* drvMode){

	if(*drvMode != DRVMODE_VECTORCONTROL){
		if (electAngVelo > ELECTANGVELO_OPENLOOP2VECTORCONTROL)
			*drvMode = DRVMODE_VECTORCONTROL;
		else
			*drvMode = DRVMODE_OPENLOOP;
	}
	else if(*drvMode == DRVMODE_VECTORCONTROL){
		if (electAngVelo < ELECTANGVELO_VECTORCONTROL2OPENLOOP)
			*drvMode = DRVMODE_OPENLOOP;
		else
			*drvMode = DRVMODE_VECTORCONTROL;
	}
}

static inline void slctElectAngleFromPosMode(uint8_t posMode, struct SensorData *sensData){
	uint8_t flgInit;
	uint8_t flgPLL;
	float electAngle;
	float electAngVelo;

	switch(posMode){
	case POSMODE_STOP:
		sensData->electAngle = 0.0f;
		sensData->electAngVelo = 0.0f;
		flgPLL = 0;
		calcElectAngle(gHall, gElectFreq, flgPLL, &electAngle, &electAngVelo);
		break;

	case POSMODE_FREERUN:
		sensData->electAngVelo = sElectAngVeloRefRateLimit;
		sElectAngleFreerun = 0;//+= sElectAngVeloRefRateLimit * CARRIERCYCLE ;


		//sensData->electAngle = gfWrapTheta(sElectAngleFreerun);
		sensData->electAngle = 0;

		flgPLL = 1;
		calcElectAngle(gHall, gElectFreq, flgPLL, &electAngle, &electAngVelo);

		break;
	case POSMODE_HALL:
		flgPLL = 0;
		calcElectAngle(gHall, gElectFreq, flgPLL, &electAngle, &electAngVelo);
		sensData->electAngle = electAngle;
		sensData->electAngVelo = electAngVelo;
		break;
	case POSMODE_HALL_PLL:
		flgPLL = 1;
		calcElectAngle(gHall, gElectFreq, flgPLL, &electAngle, &electAngVelo);
		sensData->electAngle = electAngle;
		sensData->electAngVelo = electAngVelo;
		break;
	case POSMODE_SENSORLESS:
		flgInit = 1;
		calcElectAngleEstimate(flgInit, sSensData, sVectorControlData, &sElectAngleEstimateData);
		sensData->electAngle = sElectAngleEstimateData.electAngleEstimate;
		sensData->electAngVelo = sElectAngleEstimateData.electAngVeloEstimate;
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
	float ModRef = 1.13;
	float ModErr;

	vectorControlData->Idq_ref[0] = 2.0f * gPropoDuty2;
	vectorControlData->Idq_ref[1] = 2.0f * gPropoDuty;//IQREFMAX * gVolume;
	vectorControlData->Idq_ref_LPF[0] = vectorControlData->Idq_ref[0];
	vectorControlData->Idq_ref_LPF[1] = vectorControlData->Idq_ref[1];//IQREFMAX * gVolume;

	/*ModErr = ModRef - vectorControlData->Mod;
	sId_ref_i = sId_ref_i + 0.0003 * ModErr;

	if( sId_ref_i > 0)
			sId_ref_i = 0;
	if( sId_ref_i < -1.0f)
				sId_ref_i = -1.0f;

	Idq_ref[0] = sId_ref_i;*/

	switch(drvMode){
		case DRVMODE_OFFDUTY:
			gOffDuty(Duty, outputMode);
			break;
		case DRVMODE_OPENLOOP:
			VamRef = sSensData.Vdc * SQRT3DIV2_DIV2 * gPropoDuty * gButton1;//gVolume;
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

