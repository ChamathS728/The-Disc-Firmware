/*
 * encoder.h
 *
 *  Created on: Apr 14, 2025
 *      Author: chama
 */

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_
#include "stm32g4xx.h"

typedef struct encoderHandle {
	TIM_HandleTypeDef* encTimer;

	int32_t totalPulses;

	float pulseVel;
} encoderHandle_t;

void encoderStart(encoderHandle_t* encoderHandle, uint16_t encPPR, uint16_t gearRatio);

void sampleEncoder(encoderHandle_t* encoderHandle, float dt);



#endif /* INC_ENCODER_H_ */
