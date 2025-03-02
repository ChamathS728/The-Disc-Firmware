/*
 * DRV8825.c
 *
 *  Created on: Jan 30, 2025
 *      Author: chama
 */
#include "DRV8825.h"
#include "math.h"

stepperHandle_t* DRV_init(TIM_HandleTypeDef* htimP, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htimE, stepperConfig_t* sCfgPtr, stepperIO_t* sIOPtr) {
	// Create handle
	stepperHandle_t sHandle = {
		.stepTimPtr = htimP,
		.pwrADCPtr = hadc,
		.encPtr = htimE,
		.IO = sIOPtr,
		.cfg = sCfgPtr
	};

	// Pull sleep pin high to enable device
	HAL_GPIO_WritePin(sHandle->IO->nSleepPort, sHandle->IO->nSleepPin, GPIO_PIN_SET);

	// Reset indexer logic by pulling nReset high, then move it back
	HAL_GPIO_WritePin(sHandle->IO->nResetPort, sHandle->IO->nResetPin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(sHandle->IO->nResetPort, sHandle->IO->nResetPin, GPIO_PIN_RESET);

	// Disable driver by pulling nEnable high
	HAL_GPIO_WritePin(sHandle->IO->nEnablePort, sHandle->IO->nEnablePin, GPIO_PIN_SET);

	return &sHandle;
}
void DRV_deinit(stepperHandle_t* sHandlePtr) {
	//

	__NOP();
}
void DRV_sleep(stepperHandle_t* sHandlePtr) {
	// Pull sleep pin low to sleep the device
	HAL_GPIO_WritePin(sHandlePtr->IO->nSleepPort, sHandle->IO->nSleepPin, GPIO_PIN_RESET);
}
void DRV_wakeup(stepperHandle_t* sHandlePtr) {
	// Drive nSleep high
	HAL_GPIO_WritePin(sHandlePtr->IO->nSleepPort, sHandle->IO->nSleepPin, GPIO_PIN_SET);

	// Drive NRST high as well
	HAL_GPIO_WritePin(sHandlePtr->IO->nResetPort, sHandle->IO->nResetPin, GPIO_PIN_SET);

	// Wait 1ms for DRV to stabilise
	osDelay(1);

}
void DRV_microstep_config(stepperHandle_t* sHandlePtr, eMicrostepMode microstepSetting) {
	switch (microstepSetting) {
		case MICROSTEP_1:
			// 000
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_RESET);
		case MICROSTEP_2:
			// 001
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_SET);
		case MICROSTEP_4:
			// 010
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_RESET);
		case MICROSTEP_8:
			// 011
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_SET);
		case MICROSTEP_16:
			// 100
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_RESET);
		case MICROSTEP_32:
			// 101, 110, 111
			HAL_GPIO_WritePin(sHandlePtr->IO->M2Port, sHandlePtr->IO->M2Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M1Port, sHandlePtr->IO->M1Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(sHandlePtr->IO->M0Port, sHandlePtr->IO->M0Pin, GPIO_PIN_SET);
		default:
			_NOP();
	}
}
void DRV_movement_config(stepperHandle_t* sHandlePtr, eMovementProfile profile);

void DRV_start(stepperHandle_t* sHandlePtr) {
	// REVIEW
	// Enable DRV
	HAL_GPIO_WritePin(sHandlePtr->IO->nEnablePort, sHandlePtr->IO->nEnablePin, GPIO_PIN_RESET);

	// Start Timers
	HAL_TIM_Base_Start_IT(sHandlePtr->rotInfo->PWMStopPtr);
	HAL_TIM_Base_Start_IT(sHandlePtr->rotInfo->encPtr);

	HAL_TIM_Encoder_Start(sHandlePtr->rotInfo->encPtr, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(sHandlePtr->rotInfo->PWMPtr, TIM_CHANNEL_1); // Hardcoded channel
	HAL_TIM_Start_IT(sHandlePtr->rotInfo->PWMStopPtr, TIM_CHANNEL_1);
}
void DRV_move_steps(stepperHandle_t* sHandlePtr, uint16_t steps, uint8_t dir) {
	// Set direction
	switch (dir) {
		case 0:
			HAL_GPIO_WritePin(sHandlePtr->IO->dirPort, sHandlePtr->IO->dirPin, GPIO_PIN_RESET);
			sHandlePtr->cfg->stepperDir = 0;
		case 1:
			HAL_GPIO_WritePin(sHandlePtr->IO->dirPort, sHandlePtr->IO->dirPin, GPIO_PIN_SET);
			sHandlePtr->cfg->stepperDir = 1;
		default:
			__NOP();
	}

	// Configure ARR of PWMStopTimer to match steps
	sHandlePtr->rotInfo->PWMStopPtr->Instance->ARR = steps;

	// Start PWM timer, it should be stopped in PeriodElapsedCallback in main.c
	HAL_TIM_PWM_Start(sHandlePtr->rotInfo->PWMPtr, TIM_CHANNEL_1);

}
void DRV_read_batt(ADC_HandleTypeDef* hadc) {
	//

	__NOP();
}
void DRV_move_angle_abs(stepperHandle_t* sHandlePtr, float angle) {
	/*
	 * Moves stepper motor to an absolute angle, measured by the encoder
	 * */

	// Clip angle between min and max
	float ang = (angle > sHandlePtr->rotInfo->minAngle) ? angle : sHandlePtr->rotInfo->minAngle;
	ang = (angle < sHandlePtr->rotInfo->maxAngle) ? ang : sHandlePtr->rotInfo->maxAngle;

	// Get angle requirement: desired - actual
	float angReq = ang - sHandlePtr->rotInfo->encPulses

	__NOP();
}
void DRV_retract_full(void) {
	//

	__NOP();
}
void DRV_extend_full(void) {
	//

	__NOP();
}

