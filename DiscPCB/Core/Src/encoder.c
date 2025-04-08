/*
 * encoder.c
 *
 *  Created on: Apr 9, 2025
 *      Author: chama
 */

#include "encoder.h"

void encoderStart(encoderHandle_t* encoderHandle, uint16_t encPPR, uint16_t gearRatio) {
	__HAL_TIM_SET_AUTORELOAD(encoderHandle->encTimer, encPPR);
	__HAL_TIM_SET_PRESCALER(encoderHandle->encTimer, gearRatio);

	// Start the encoder half way
	__HAL_TIM_SET_COUNTER(encoderHandle->encTimer, encoderHandle->encTimer->Instance->ARR/2);


	HAL_TIM_Base_Start(encoderHandle->encTimer);
	HAL_TIM_Encoder_Start(encoderHandle->encTimer, TIM_CHANNEL_ALL);
}

void sampleEncoder(encoderHandle_t* encoderHandle, float dt) {
	// Read from encoder
	int16_t currentEnc = __HAL_TIM_GET_COUNTER(encoderHandle->encTimer) - encoderHandle->encTimer->Instance->ARR/2;

	// Reset encoder
	__HAL_TIM_SET_COUNTER(encoderHandle->encTimer, encoderHandle->encTimer->Instance->ARR/2);

	// Update total pulses appropriately
	encoderHandle->totalPulses += currentEnc;

	// Calculate velocity based on dt
	encoderHandle->pulseVel = (currentEnc)/dt;
}
