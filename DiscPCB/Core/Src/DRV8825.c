/*
 * DRV8825.c
 *
 *  Created on: Jan 30, 2025
 *      Author: chama
 */
#include "DRV8825.h"
#include "math.h"
#include "stdlib.h"
#include "cmsis_os2.h"
#include "math.h"

extern int numOfRevolutions;

stepperHandle_t* DRV_init(stepperConfig_t* sCfgPtr, stepperIO_t* sIOPtr, stepperRotInfo_t* sRotPtr) {
	// Create handle
	stepperHandle_t* sHandle = (stepperHandle_t*) malloc(sizeof(stepperHandle_t));

	sHandle->IO = sIOPtr;
	sHandle->cfg = sCfgPtr;
	sHandle->rotInfo = sRotPtr;

	// Pull sleep pin high to enable device
	HAL_GPIO_WritePin(sHandle->IO->nSleepPort, sHandle->IO->nSleepPin, GPIO_PIN_SET);

	// Reset indexer logic by pulling nReset high, then move it back
	HAL_GPIO_WritePin(sHandle->IO->nResetPort, sHandle->IO->nResetPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(sHandle->IO->nResetPort, sHandle->IO->nResetPin, GPIO_PIN_RESET);

	// Disable driver by pulling nEnable high
	HAL_GPIO_WritePin(sHandle->IO->nEnablePort, sHandle->IO->nEnablePin, GPIO_PIN_SET);

	// Wait 1ms for DRV to stabilise
	osDelay(1);

	// Configure microstep setting
	DRV_microstep_config(sHandle, sCfgPtr->stepRes);

	// Set prescalar for encoder timer to match gear ratio
	__HAL_TIM_SET_PRESCALER(sRotPtr->encPtr, (uint16_t) sRotPtr->gearRatio - 1);

	// Set ARR for encoder timer to match PPR from Encoder
	__HAL_TIM_SET_AUTORELOAD(sRotPtr->encPtr, (uint16_t) sRotPtr->encPPR);

	return sHandle;
}
//void DRV_deinit(stepperHandle_t* sHandlePtr) {
//	//
//
//	__NOP();
//}
void DRV_sleep(stepperHandle_t* sHandlePtr) {
	// Pull sleep pin low to sleep the device
	HAL_GPIO_WritePin(sHandlePtr->IO->nSleepPort, sHandlePtr->IO->nSleepPin, GPIO_PIN_RESET);
}
void DRV_wakeup(stepperHandle_t* sHandlePtr) {
	// Drive nSleep high
	HAL_GPIO_WritePin(sHandlePtr->IO->nSleepPort, sHandlePtr->IO->nSleepPin, GPIO_PIN_SET);

	// Drive NRST high as well
	HAL_GPIO_WritePin(sHandlePtr->IO->nResetPort, sHandlePtr->IO->nResetPin, GPIO_PIN_SET);

	// Wait 1ms for DRV to stabilise
	osDelay(1);

}
void DRV_microstep_config(stepperHandle_t* sHandlePtr, eMicrostepMode microstepSetting) {
	sHandlePtr->cfg->stepRes = microstepSetting;
	switch (microstepSetting) {
		case MICROSTEP_1:
			// 000
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_RESET);

			sHandlePtr->rotInfo->driveRes = REV_1;
			break;
		case MICROSTEP_2:
			// 001
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_SET);

			sHandlePtr->rotInfo->driveRes = REV_2;
			break;
		case MICROSTEP_4:
			// 010
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_RESET);

			sHandlePtr->rotInfo->driveRes = REV_4;
			break;
		case MICROSTEP_8:
			// 011
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_SET);

			sHandlePtr->rotInfo->driveRes = REV_8;
			break;
		case MICROSTEP_16:
			// 100
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_RESET);

			sHandlePtr->rotInfo->driveRes = REV_16;
			break;
		case MICROSTEP_32:
			// 101, 110, 111
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_SET);

			sHandlePtr->rotInfo->driveRes = REV_32;
			break;
//		default:
//			__NOP();
	}
}
void DRV_movement_config(stepperHandle_t* sHandlePtr, eMovementProfile profile);

void DRV_start(stepperHandle_t* sHandlePtr) {
	// REVIEW
	// Enable DRV
	HAL_GPIO_WritePin(sHandlePtr->IO->nEnablePort, sHandlePtr->IO->nEnablePin, GPIO_PIN_RESET);

	// Start Timers

	if (HAL_OK != HAL_TIM_Base_Start_IT(sHandlePtr->rotInfo->PWMStopPtr)) {
		osDelay(100000);
	}

	if (HAL_OK != HAL_TIM_Base_Start_IT(sHandlePtr->rotInfo->PWMPtr)) {
		osDelay(100000);
	}

	if (HAL_OK != HAL_TIM_Base_Start_IT(sHandlePtr->rotInfo->encPtr)) {
		osDelay(100000);
	}

	if (HAL_OK != HAL_TIM_Encoder_Start_IT(sHandlePtr->rotInfo->encPtr, TIM_CHANNEL_ALL)) {
		osDelay(100000);
	}

	if (HAL_OK != HAL_TIM_PWM_Start(sHandlePtr->rotInfo->PWMPtr, STEPPER_CHANNEL)) {
		osDelay(100000);
	}

	if (HAL_OK != HAL_TIM_IC_Start_IT(sHandlePtr->rotInfo->PWMStopPtr, STEPPER_STOP_CHANNEL)) {
		osDelay(100000);
	}

}
void DRV_move_steps(stepperHandle_t* sHandlePtr, uint16_t steps, uint8_t dir) {
	// Set direction
	switch (dir) {
		case 0:
			HAL_GPIO_WritePin(sHandlePtr->IO->dirPort, sHandlePtr->IO->dirPin, GPIO_PIN_RESET);
			sHandlePtr->cfg->stepperDir = 0;
			break;
		case 1:
			HAL_GPIO_WritePin(sHandlePtr->IO->dirPort, sHandlePtr->IO->dirPin, GPIO_PIN_SET);
			sHandlePtr->cfg->stepperDir = 1;
			break;
	}

	// Start PWM timer, it should be stopped in PeriodElapsedCallback in main.c
	HAL_StatusTypeDef qwerty = HAL_TIM_PWM_Stop(sHandlePtr->rotInfo->PWMPtr, STEPPER_CHANNEL);
	qwerty = HAL_TIM_IC_Stop_IT(sHandlePtr->rotInfo->PWMStopPtr, STEPPER_STOP_CHANNEL);

	// Configure ARR of PWMStopTimer to match steps
	__HAL_TIM_SET_AUTORELOAD(sHandlePtr->rotInfo->PWMStopPtr, steps-1);

	osDelay(1);
	qwerty = HAL_TIM_PWM_Start(sHandlePtr->rotInfo->PWMPtr, STEPPER_CHANNEL);
	qwerty = HAL_TIM_IC_Start_IT(sHandlePtr->rotInfo->PWMStopPtr, STEPPER_STOP_CHANNEL);

}

void DRV_move_angle_abs_OL(stepperHandle_t* sHandlePtr, float angle) {
	/*
	 * Moves stepper motor to an absolute angle, measured by the encoder
	 * */

	// Clip angle between min and max
	float ang = (angle > sHandlePtr->rotInfo->minAngle) ? angle : sHandlePtr->rotInfo->minAngle;
	ang = (angle < sHandlePtr->rotInfo->maxAngle) ? ang : sHandlePtr->rotInfo->maxAngle;

	// Get angle requirement: desired - actual
	float currentAngle = numOfRevolutions*360.0 + (float) (sHandlePtr->rotInfo->encPulses % (mtrHandle->rotInfo->encPtr->Instance->ARR * 4));
	int16_t angReq = ang - sHandlePtr->rotInfo->encPulses / (mtrHandle->rotInfo->encPtr->Instance->ARR * 4);

	// Move relative angle
	DRV_move_angle_rel_OL(sHandlePtr, angReq);
}

void DRV_move_angle_rel_OL(stepperHandle_t* sHandlePtr, float relAngle) {
	/*
	 * Moves stepper motor a relative angle in open loop. It uses the microstep mode
	 * to determine how many steps to move and in what direction
	 *
	 * Angle input should be in degrees
	 * */

	// Get direction and number of steps required based on microstep config
	uint8_t dir = 0;
	if (relAngle >= 0) {
		dir = 1;
	}
	else {
		relAngle *= -1;
	}
	uint16_t relStepsUI = (uint16_t) (relAngle/360.0f * sHandlePtr->rotInfo->driveRes);

	// Move required number of steps
	DRV_move_steps(sHandlePtr, relStepsUI, dir);
}

void DRV_set_pulse_freq(stepperHandle_t* sHandlePtr, uint16_t pulseFreq) {
	/*
	 * Sets the frequency of pulses using the current clock frequency
	 * Assumes that the prescalar does not change
	 *
	 * pulseFreq should be a value between 10Hz and 100,000Hz
	 * */
	RCC_ClkInitTypeDef clkconfig;
	uint32_t pFLatency;
	HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
	uint32_t APB1TimerClock = HAL_RCC_GetPCLK1Freq();
	if (clkconfig.APB1CLKDivider != RCC_HCLK_DIV1) {
		APB1TimerClock *= 2;
	}
	// Hold prescalar constant and calculate new ARR value
	uint32_t prescaler = (uint32_t) (sHandlePtr->rotInfo->PWMPtr->Init.Prescaler + 1);
	uint32_t timerTickFrequency = APB1TimerClock / prescaler;
	uint32_t period = (timerTickFrequency / pulseFreq) - 1;

	// Stop PWM temporarily
    HAL_TIM_PWM_Stop(sHandlePtr->rotInfo->PWMPtr, STEPPER_CHANNEL);
    HAL_TIM_Base_Stop(sHandlePtr->rotInfo->PWMPtr);

    // Update ARR and CCR values to maintain frequency and 50% duty cycle
    __HAL_TIM_SET_AUTORELOAD(sHandlePtr->rotInfo->PWMPtr, period);
    uint32_t pulse = ((period + 1) * 50) / 100;
    __HAL_TIM_SET_COMPARE(sHandlePtr->rotInfo->PWMPtr, STEPPER_CHANNEL, pulse);

    /* Trigger an update event to apply changes immediately */
    HAL_TIM_GenerateEvent(sHandlePtr->rotInfo->PWMPtr, TIM_EVENTSOURCE_UPDATE);

    /* Restart the timer */
    HAL_TIM_Base_Start(sHandlePtr->rotInfo->PWMPtr);
    HAL_TIM_PWM_Start(sHandlePtr->rotInfo->PWMPtr, STEPPER_CHANNEL);
}

void DRV_update_angular_pos(stepperHandle_t* sHandlePtr) {
	  // Get encoder
	  uint32_t currentEnc = mtrHandle->rotInfo->encPtr->Instance->CNT;

	  // Get absolute angular position
	  if (numOfRevolutions > 0) {
		  mtrHandle->rotInfo->encPulses = (int16_t) (mtrHandle->rotInfo->encPtr->Instance->ARR * 4 * numOfRevolutions + currentEnc);
	  }
	  else if (numOfRevolutions < 0) {
		  mtrHandle->rotInfo->encPulses = (int16_t) mtrHandle->rotInfo->encPtr->Instance->ARR * -4 * numOfRevolutions - (int16_t) (mtrHandle->rotInfo->encPtr->Instance->ARR - currentEnc);
	  }
	  else {
		  mtrHandle->rotInfo->encPulses = (int16_t) currentEnc;
	  }
}

//void DRV_retract_full(void) {
//	//
//}
//void DRV_extend_full(void) {
//	//
//}

