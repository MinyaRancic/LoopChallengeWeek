/*
 * stateMachine.c
 *
 *  Created on: Sep 20, 2019
 *      Author: Minya
 */

#include "stateMachine.h"

enum states getState() {
	return state;
}

void nextState(enum events event) {
	switch(state) {
	case READ_DIGIT: {
		if(event == DIGIT_READ) {
			digitsRead++;
		}
		if(digitsRead == 4) {
			state = SEND_MESSAGE;
		}
		break;
	}

	case SEND_MESSAGE: {
		if(event = CAN_SENT) {
			state = RECEIVE_MESSAGE;
		}
		break;
	}
	case RECEIVE_MESSAGE: {
		if(event = CAN_RECEIVED) {
			state = FINISHED;
		}
		break;
	}
	case FINISHED:
		break;
	default:
		state = READ_DIGIT;
		break;
	}
}
