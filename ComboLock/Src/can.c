/*
 * can.c
 *
 *  Created on: Sep 19, 2019
 *      Author: Minya
 */

#include "can.h"

CAN_TxHeaderTypeDef generateTXHeader(uint32_t StdId, uint32_t ExtId, uint32_t IDE, uint32_t RTR,
		uint32_t DLC, FunctionalState TimeTransmit) {
	CAN_TxHeaderTypeDef txHeader;
	txHeader.StdId = StdId;
	txHeader.ExtId = ExtId;
	txHeader.IDE = IDE;
	txHeader.RTR = RTR;
	txHeader.DLC = DLC;
	txHeader.TransmitGlobalTime = DISABLE;

	return txHeader;
}

CAN_RxHeaderTypeDef generateRXHeader(uint32_t StdId, uint32_t ExtId, uint32_t IDE, uint32_t RTR,
		uint32_t DLC) {
	CAN_RxHeaderTypeDef rxHeader;
	rxHeader.StdId = StdId;
	rxHeader.ExtId = ExtId;
	rxHeader.IDE = IDE;
	rxHeader.RTR = RTR;
	rxHeader.DLC = DLC;

	return rxHeader;
}

/**
 * Writes to the CAN Transmit Line and returns the amount of data written (approximately)
 *
 */
int writeToCAN(CAN_HandleTypeDef *hcan, uint8_t data[], uint8_t dataSize) {
	CAN_TxHeaderTypeDef txHeader = generateTXHeader(CAN_ID, CAN_EXT_ID,
	CAN_IS_EXT, CAN_RTR_DATA, dataSize, DISABLE);
	uint32_t mailbox = CAN_TX_MAILBOX0;

	if (HAL_CAN_AddTxMessage(hcan, &txHeader, data, &mailbox) != HAL_OK) {
		return -1;
	}
	while (HAL_CAN_IsTxMessagePending(hcan, mailbox))
		;
	return dataSize;
}

int readCAN(CAN_HandleTypeDef *hcan, uint8_t *data, CAN_RxHeaderTypeDef *rxHeader) {
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
		return -1;
	}
	return rxHeader->DLC;
}
