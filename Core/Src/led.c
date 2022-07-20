/*
 * led.c
 *
 *  Created on: Jul 14, 2022
 *      Author: sangmin_lee
 */

#include "led.h"
#include "main.h"

RGB green = {0.0,1.0,0.0};      //for charging2
RGB skyblue = {0.0,1.0,1.0};    //for working, wallfollowing 3, 8
RGB yellow = {1.0,1.0,0.0};     //for stop, W_stop 4, 5
RGB red = {1.0,0.0,0.0};        //for emergency 6
RGB purple = {1.0,0.0,1.0};    //for DOCKING 1
RGB white = {1.0,1.0,1.0};      //for manual 7
RGB blue = {0.0,0.0,1.0};

void turnOn(RGB rgb)
{
    if(rgb.red == 1) {HAL_GPIO_WritePin(Rsig_GPIO_Port, Rsig_Pin, SET);}
    else {HAL_GPIO_WritePin(Rsig_GPIO_Port, Rsig_Pin, RESET);}
    if(rgb.green == 1) {HAL_GPIO_WritePin(Rsig_GPIO_Port, Gsig_Pin, SET);}
    else {HAL_GPIO_WritePin(Rsig_GPIO_Port, Gsig_Pin, RESET);}
    if(rgb.blue == 1) {HAL_GPIO_WritePin(Rsig_GPIO_Port, Bsig_Pin, SET);}
    else {HAL_GPIO_WritePin(Rsig_GPIO_Port, Bsig_Pin, RESET);}
}

void turnOff()
{
    HAL_GPIO_WritePin(Rsig_GPIO_Port, Rsig_Pin, RESET);
    HAL_GPIO_WritePin(Rsig_GPIO_Port, Gsig_Pin, RESET);
    HAL_GPIO_WritePin(Rsig_GPIO_Port, Bsig_Pin, RESET);
}

