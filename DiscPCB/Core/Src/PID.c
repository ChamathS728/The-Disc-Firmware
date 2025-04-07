/*
 * PID.c
 *
 *  Created on: Apr 5, 2025
 *      Author: chama
 */
#include "PID.h"

void PID_Init(PIDController_t* pid,
              float Kp, float Ki, float Kd, float dt,
              float output_min, float output_max,
              float alpha, float setpoint)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->dt = dt;

    pid->output_min = output_min;
    pid->output_max = output_max;

    pid->alpha = alpha;

    // Reset state variables
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->derivative = 0.0f;
    pid->setpoint = setpoint;
}

float PID_Update(PIDController_t* pid, float error)
{
    // Proportional term
    float P = pid->Kp * error;

    // Integral term with anti-windup
    pid->integral += error * pid->dt;
    float I = pid->Ki * pid->integral;

    // Derivative term with optional filtering
    float derivative_raw = (error - pid->prev_error) / pid->dt;
    pid->derivative = pid->alpha * pid->derivative + (1.0f - pid->alpha) * derivative_raw;
    float D = pid->Kd * pid->derivative;

    // Compute total output
    float output = P + I + D;

    // Clamp output
    if (output > pid->output_max) {
        output = pid->output_max;
    } else if (output < pid->output_min) {
        output = pid->output_min;
    } else {
        // Only allow integral to grow when output is not clamped
        pid->integral = pid->integral;  // No change needed, just for clarity
    }

    // Save error for next derivative calculation
    pid->prev_error = error;

    return output;
}
