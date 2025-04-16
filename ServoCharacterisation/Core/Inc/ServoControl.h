/*
 * ServoControl.h
 *
 *  Created on: Apr 14, 2025
 *      Author: chama
 */

#ifndef INC_SERVOCONTROL_H_
#define INC_SERVOCONTROL_H_
#include "stm32g4xx.h"

#define SERVO_CHANNEL TIM_CHANNEL_1

typedef struct ServoHandle {
	TIM_HandleTypeDef* htim;
	float freqPWM;
	float currentPos;
} ServoHandle_t;


void servo_start(ServoHandle_t* sHandle);
void servo_configure_freq(ServoHandle_t* sHandle, float pulseFreq);
void servo_move(ServoHandle_t* sHandle, float position);
void new_servo_move(ServoHandle_t* sHandle, float position);

#endif /* INC_SERVOCONTROL_H_ */
