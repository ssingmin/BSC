/*
 * led.h
 *
 *  Created on: Jul 14, 2022
 *      Author: sangmin_lee
 */

#ifndef INC_LED_H_
#define INC_LED_H_
#include <stdint.h>
typedef struct  {
    uint32_t red;
    uint32_t green;
    uint32_t blue;
}RGB;

void turnOn(RGB rgb);

void turnOff();

#endif /* INC_LED_H_ */
