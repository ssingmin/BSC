/*
 * motor_driver_114.h
 *
 *  Created on: Jul 13, 2022
 *      Author: sangmin_lee
 */

#ifndef INC_MOTOR_DRIVER_114_H_
#define INC_MOTOR_DRIVER_114_H_

#pragma once

#include <stdint.h>
#include "can.h"


#define ENC_RESOLUTION 4096
#define POLE_PAIR 10
#define MAX_RPM 200
#define BREAK_OPTION 1
#define RATED_CURRENT 8
#define MAX_CURRENT 16
#define HALL_OFFSET 240
#define KP_GAIN 550 //default 500
#define KI_GAIN 110 //default 100


    void velocityMode();
    void synchronousMode();

    void settingEnc(int16_t enc);
    void settingPole(int16_t pole);
    void settingBreak(uint8_t enable);
    void settingMaxRPM(int16_t rpm);
    void settingRatedCurrent(double curr);
    void settingMaxCurrent(double curr);
    void settingHallOffset(int16_t dgree);
    void settingKP(int16_t kp);
    void settingKI(int16_t ki);

    void save();

    void settingMotor();
    void startMotor();

    void enable();
    void disable();
    void reset();
    void reqEnc();
    void reqState();
    void control(int16_t lrpm,int16_t rrpm);


#endif /* INC_MOTOR_DRIVER_114_H_ */
