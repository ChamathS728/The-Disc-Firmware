/*
 * DRV8825.h
 *
 *  Created on: Jan 30, 2025
 *      Author: chama
 */

#ifndef INC_DRV8825_H_
#define INC_DRV8825_H_

#include "stm32g4xx_hal.h"

/* Enums */
typedef enum {
	// Binary sequence in M2,M1,M0 order
    MICROSTEP_1 = 1, 	// 000
    MICROSTEP_2 = 2,	// 001
    MICROSTEP_4 = 4,	// 010
    MICROSTEP_8 = 8,	// 011
    MICROSTEP_16 = 16, 	// 100
    MICROSTEP_32 = 32	// 101, 110, 111
} eMicrostepMode;

typedef enum {
	REV_1 = 200,		// 200 steps per revolution with 1.8 deg/step
	REV_2 = 400,
	REV_4 = 800,
	REV_8 = 1600,
	REV_16 = 3200,
	REV_32 = 6400
} eDriverResolution;

// NOTE - May not be needed
typedef enum {
	MOVE_TRAP = 1, 		// Trapezoidal profile, max torque
	MOVE_S_CURVE = 2, 	// S-curve with trapezoidal acceleration
} eMovementProfile;

/* Structs */
typedef struct stepperIO_t {

	// Direction for stepping
	GPIO_TypeDef* dirPort;
	uint16_t dirPin;

	// Input for whether we're at home
	GPIO_TypeDef* nHomePort;
	uint16_t nHomePin;

	// Input for fault detection
	GPIO_TypeDef* nFaultPort;
	uint16_t nFaultPin;

	// Input for whether to reset
	GPIO_TypeDef* nResetPort;
	uint16_t nResetPin;

	// Output for decay mode
	GPIO_TypeDef* decayPort;
	uint16_t decayPin;

	GPIO_TypeDef* nSleepPort;
	uint16_t nSleepPin;

	GPIO_TypeDef* nEnablePort;
	uint16_t nEnablePin;

	// GPIO for Modes
	GPIO_TypeDef* M0Port;
	uint16_t M0Pin;

	GPIO_TypeDef* M1Port;
	uint16_t M1Pin;

	GPIO_TypeDef* M2Port;
	uint16_t M2Pin;

} stepperIO_t;

typedef struct stepperConfig_t {
	eMicrostepMode stepRes;

	uint16_t encoderResolution;

	eMovementProfile moveProfile;

	uint8_t stepperDir;

} stepperConfig_t;

typedef struct stepperRotInfo_t {
	// Timers for stepping
	TIM_HandleTypeDef* PWMPtr;
	TIM_HandleTypeDef* PWMStopPtr;

	// Encoder timer
	TIM_HandleTypeDef* encPtr;

	// Absolute min and max angle, not relative
	float minAngle;
	float maxAngle;

	// Number of steps per revolution, dictated by eMicrostepMode
	eDriverResolution driveRes;

	// Current number of steps according to driver
	int16_t driverSteps;

	// Number of pulses per revolution measured from encoder, eg: 1000
	int16_t encPPR;

	// Current number of pulses measured from encoder
	int16_t encPulses;

} stepperRotInfo_t;

typedef struct stepperHandle_t {
	// ADC handle for motor current sense
	ADC_HandleTypeDef* pwrADCPtr;

	// Stepper motor configuration
	stepperConfig_t* cfg;

	// Stepper motor related IO
	stepperIO_t* IO;

	// Stepper motor rotational information
	stepperRotInfo_t* rotInfo;

} stepperHandle_t;

/* Methods */

stepperHandle_t* DRV_init(TIM_HandleTypeDef* htimP, ADC_HandleTypeDef* hadc, TIM_HandleTypeDef* htimE, stepperConfig_t* sCfgPtr, stepperIO_t* sIOPtr);
void DRV_deinit(stepperHandle_t* stepperHandlePtr);
void DRV_sleep(stepperHandle_t* stepperHandlePtr);
void DRV_wakeup(stepperHandle_t* stepperHandlePtr);
void DRV_microstep_config(stepperHandle_t* stepperHandlePtr, eMicrostepMode microstepSetting);
void DRV_movement_config(stepperHandle_t* stepperHandlePtr, eMovementProfile profile);

void DRV_start(stepperHandle_t* stepperHandlePtr);
void DRV_move_steps(uint16_t steps, uint8_t dir);
void DRV_read_current(ADC_HandleTypeDef* hadc);
void DRV_move_angle(stepperHandle_t* sHandlePtr, float angleAbs);
void DRV_retract_full(void);
void DRV_extend_full(void);

#endif /* INC_DRV8825_H_ */
