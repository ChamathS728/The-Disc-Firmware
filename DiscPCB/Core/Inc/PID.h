/*
 * PID.h
 *
 *  Created on: Apr 5, 2025
 *      Author: chama
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include "stm32g4xx.h"

typedef struct {
    // PID gains
    float Kp;   // Proportional gain
    float Ki;   // Integral gain
    float Kd;	// Derivative gain

    // State variables
    float integral;		// Accumulated integral
    float prev_error;   // Previous error (for derivative)
    float setpoint;     // Desired target value
    float dt;			// Control loop timestep in seconds

    // Output limits
    float output_min;   // Minimum output value
    float output_max;   // Maximum output value

    // Derivative filtering (optional)
    float derivative;   // Last derivative value
    float alpha;        // Derivative low-pass filter coefficient (0 to 1)
} PIDController_t;

float PID_Update(PID_Controller* pid, float measured);

#endif /* INC_PID_H_ */
