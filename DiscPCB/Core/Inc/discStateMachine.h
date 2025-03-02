/*
 * discStateMachine.h
 *
 *  Created on: Feb 2, 2025
 *      Author: chama
 */

#ifndef INC_DISCSTATEMACHINE_H_
#define INC_DISCSTATEMACHINE_H_

typedef enum {
	INITIALISING,
	PAD,
	BURNOUT,
	COASTING,
	RECOVERY,
	IDLE
} eSTATES;


#endif /* INC_DISCSTATEMACHINE_H_ */
