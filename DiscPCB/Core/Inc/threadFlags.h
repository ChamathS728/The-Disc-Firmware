/*
 * threadFlags.h
 *
 *  Created on: Apr 4, 2025
 *      Author: chama
 */

#ifndef INC_THREADFLAGS_H_
#define INC_THREADFLAGS_H_

#include "cmsis_os2.h"

/* ADC related flags */
uint32_t isADCDone = 1;

/* Stepper Control flags */
uint32_t isTargetNew = 2;

#endif /* INC_THREADFLAGS_H_ */
