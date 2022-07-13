/*
 * can.h
 *
 *  Created on: Jun 14, 2022
 *      Author: sangmin_lee
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include "stm32f7xx_hal_can.h"


extern CAN_HandleTypeDef hcan1;

void CanInit(uint32_t id, uint32_t mask);
void sendCan(uint32_t ID, uint8_t *buf, uint8_t len, uint8_t ext);



#endif /* INC_CAN_H_ */
