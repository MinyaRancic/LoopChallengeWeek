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

uint32_t writeToCAN(CAN_HandleTypeDef *hcan, uint8_t data[], uint8_t dataSize);
uint32_t readCAN(uint8_t *data, uint32_t dataSize);

#endif /* CAN_H_ */
