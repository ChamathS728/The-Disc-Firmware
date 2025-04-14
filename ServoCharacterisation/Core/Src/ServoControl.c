/*
 * ServoControl.c
 *
 *  Created on: Apr 14, 2025
 *      Author: chama
 */

#include "ServoControl.h"

void servo_start(ServoHandle_t* sHandle) {
	servo_configure_freq(sHandle, sHandle->freqPWM);

	// Start up the servo timer
	HAL_TIM_Base_Start(sHandle->htim);
	HAL_TIM_PWM_Start(sHandle->htim, SERVO_CHANNEL);
}

void servo_configure_freq(ServoHandle_t* sHandle, float pulseFreq) {
	/*
	 * Sets the frequency of pulses using the current clock frequency
	 * Assumes that the prescalar does not change
	 *
	 * pulseFreq should be a value between 10Hz and 100,000Hz
	 * */
	RCC_ClkInitTypeDef clkconfig;
	uint32_t pFLatency;
	HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
	uint32_t APB2TimerClock = HAL_RCC_GetPCLK2Freq();
	if (clkconfig.APB2CLKDivider != RCC_HCLK_DIV1) {
		APB2TimerClock *= 2;
	}
	// Hold prescalar constant and calculate new ARR value
	uint32_t prescaler = (uint32_t) (sHandle->htim->Init.Prescaler + 1);
	uint32_t timerTickFrequency = APB2TimerClock / prescaler;
	uint32_t period = (timerTickFrequency / pulseFreq) - 1;

	// Stop PWM temporarily
    HAL_TIM_PWM_Stop(sHandle->htim, SERVO_CHANNEL);
    HAL_TIM_Base_Stop(sHandle->htim);

    // Update ARR and CCR values to maintain frequency and 50% duty cycle
    __HAL_TIM_SET_AUTORELOAD(sHandle->htim, period);
    uint32_t pulse = ((period + 1) * 50) / 100;
    __HAL_TIM_SET_COMPARE(sHandle->htim, SERVO_CHANNEL, pulse);

    /* Trigger an update event to apply changes immediately */
    HAL_TIM_GenerateEvent(sHandle->htim, TIM_EVENTSOURCE_UPDATE);

    /* Restart the timer */
    HAL_TIM_Base_Start(sHandle->htim);
    HAL_TIM_PWM_Start(sHandle->htim, SERVO_CHANNEL);

    sHandle->freqPWM = pulseFreq;
}

void servo_move(ServoHandle_t* sHandle, float position) {
	/*
	 * Position should be float between 0 and 1
	 * */

	HAL_TIM_PWM_Stop(sHandle->htim, SERVO_CHANNEL);
	HAL_TIM_Base_Stop(sHandle->htim);

	// Write CCR1 to be percentage of ARR
	sHandle->htim->Instance->CCR1 = (uint32_t) (sHandle->htim->Instance->ARR * position);

	HAL_TIM_Base_Start(sHandle->htim);
	HAL_TIM_PWM_Start(sHandle->htim, SERVO_CHANNEL);
}
