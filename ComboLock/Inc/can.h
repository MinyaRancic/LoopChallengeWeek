/*
 * can.h
 *
 *  Created on: Sep 19, 2019
 *      Author: Minya
 */

#ifndef CAN_H_
#define CAN_H_

#include "stm32l4xx_hal.h"

#define CAN_ID 1
#define CAN_EXT_ID 0
#define CAN_IS_EXT CAN_ID_STD

int writeToCAN(CAN_HandleTypeDef *hcan, uint8_t data[], uint8_t dataSize);
int readCAN(CAN_HandleTypeDef *hcan, uint8_t *data, CAN_RxHeaderTypeDef *rxHeader);

#endif /* CAN_H_ */
