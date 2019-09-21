/*
 * stateMachine.h
 *
 *  Created on: Sep 20, 2019
 *      Author: Minya
 */

#ifndef STATEMACHINE_H_
#define STATEMACHINE_H_

enum states {
	READ_DIGIT,
	SEND_MESSAGE,
	RECEIVE_MESSAGE,
	FINISHED,
} state;

enum events {
	DIGIT_READ,
	CAN_SENT,
	CAN_RECEIVED,
};

int digitsRead;
void nextState(enum events event);
enum states getState();

#endif /* STATEMACHINE_H_ */
