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
	RECEIVE_MESSAGE
} state;

int digitsRead = 0;
enum states currentState = READ_DIGIT;

void nextState();


#endif /* STATEMACHINE_H_ */
