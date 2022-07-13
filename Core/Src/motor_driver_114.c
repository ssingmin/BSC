/*
 * motor_driver_114.c
 *
 *  Created on: Jul 13, 2022
 *      Author: sangmin_lee
 */


#include "motor_driver_114.h"
#include "define.h"

void velocityMode()
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x2f;
    buf[index++] = 0x60;
    buf[index++] = 0x60;
    buf[index++] = 0x00;
    buf[index++] = 0x03;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
//    ThisThread::sleep_for(100);
    HAL_Delay(100);
}

void synchronousMode()
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x2b;
    buf[index++] = 0x0f;
    buf[index++] = 0x20;
    buf[index++] = 0x00;
    buf[index++] = 0x01;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
//    ThisThread::sleep_for(100);
    HAL_Delay(100);
}

void settingEnc(int16_t enc)
{
    char buf[8];
    for(int i=0;i<2;i++)
    {
        int index = 0;
        buf[index++] = 0x2b;
        buf[index++] = 0x0e;
        buf[index++] = 0x20;
        buf[index++] = i+1;
        buf[index++] = (enc & 0xff);
        buf[index++] = (enc>>8) & 0xff;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(100);
        HAL_Delay(100);
    }
}

void settingPole(int16_t pole)
{
    char buf[8];
    for(int i=0;i<2;i++)
    {
        int index = 0;
        buf[index++] = 0x2b;
        buf[index++] = 0x0c;
        buf[index++] = 0x20;
        buf[index++] = i+1;
        buf[index++] = (pole & 0xff);
        buf[index++] = (pole>>8) & 0xff;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(100);
        HAL_Delay(100);
    }
}

void settingBreak(uint8_t enable)
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x2b;
    buf[index++] = 0x07;
    buf[index++] = 0x20;
    buf[index++] = 0x00;
    buf[index++] = (char)enable;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
//    ThisThread::sleep_for(100);
    HAL_Delay(100);
}

void settingMaxRPM(int16_t rpm)
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x2b;
    buf[index++] = 0x08;
    buf[index++] = 0x20;
    buf[index++] = 0x00;
    buf[index++] = (rpm & 0xff);
    buf[index++] = (rpm>>8) & 0xff;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
//    ThisThread::sleep_for(100);
    HAL_Delay(100);
}

void settingRatedCurrent(double curr)
{
    char buf[8];
    for(int i=0;i<2;i++)
    {
        int index = 0;
        buf[index++] = 0x2b;
        buf[index++] = 0x14;
        buf[index++] = 0x20;
        buf[index++] = i+1;
        buf[index++] = (int)(curr*10);
        buf[index++] = 0x00;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(100);
        HAL_Delay(100);
    }
}

void settingMaxCurrent(double curr)
{
    char buf[8];
    for(int i=0;i<2;i++)
    {
        int index = 0;
        buf[index++] = 0x2b;
        buf[index++] = 0x15;
        buf[index++] = 0x20;
        buf[index++] = i+1;
        buf[index++] = (int)(curr*10);
        buf[index++] = 0x00;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(100);
        HAL_Delay(100);
    }
}

void settingHallOffset(int16_t degree)
{
    char buf[8];
    for(int i=0;i<2;i++)
    {
        int index = 0;
        buf[index++] = 0x2b;
        buf[index++] = 0x11;
        buf[index++] = 0x20;
        buf[index++] = i+1;
        buf[index++] = (degree & 0xff);
        buf[index++] = (degree>>8) & 0xff;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(100);
        HAL_Delay(100);
    }
}

void save()
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x2b;
    buf[index++] = 0x10;
    buf[index++] = 0x20;
    buf[index++] = 0x00;
    buf[index++] = 0x01;
    buf[index++] =  0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    index = 0;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
//    ThisThread::sleep_for(100);
    HAL_Delay(100);
}



void startMotor()
{
    enable();
    velocityMode();
    synchronousMode();
}

void enable()
{
    char buf[8];
    char seq[4] = {0x00,0x06,0x07,0x0f};
    for(int i =0;i<4;i++)
    {
        int index = 0;
        buf[index++] = 0x2b;
        buf[index++] = 0x40;
        buf[index++] = 0x60;
        buf[index++] = 0x00;
        buf[index++] = seq[i];
        buf[index++] = 0x00;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(100);
        HAL_Delay(100);
    }
}

void disable()
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x2b;
    buf[index++] = 0x40;
    buf[index++] = 0x60;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index++] =  0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    index = 0;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
//    ThisThread::sleep_for(100);
    HAL_Delay(100);
}

void reset()
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x2b;
    buf[index++] = 0x40;
    buf[index++] = 0x60;
    buf[index++] = 0x00;
    buf[index++] = 0x80;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
//    ThisThread::sleep_for(100);
    HAL_Delay(100);
}

void reqEnc()
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x40;
    buf[index++] = 0x6c;
    buf[index++] = 0x60;
    buf[index++] = 0x03;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
}

void reqState()
{
    char buf[8];
    int index = 0;
    buf[index++] = 0x40;
    buf[index++] = 0x3f;
    buf[index++] = 0x60;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index++] = 0x00;
    buf[index] = 0x00;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
}

void control(int16_t lrpm,int16_t rrpm)
{
    lrpm = -1*lrpm;
    char buf[8];
    int index = 0;
    buf[index++] = 0x23;
    buf[index++] = 0xff;
    buf[index++] = 0x60;
    buf[index++] = 0x03;
    buf[index++] = (lrpm & 0xff);
    buf[index++] = (lrpm>>8) & 0xff;
    buf[index++] = rrpm & 0xff;
    buf[index] = (rrpm>>8) & 0xff;
    sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//    can->sendMsg(MOTOR114_REQ_ID,buf,false);
}

void settingKP(int16_t kp)
{
    char buf[8];
    for(int i=0;i<2;i++)
    {
        int index = 0;
        buf[index++] = 0x1d;
        buf[index++] = 0x20;
        buf[index++] = 0x20;
        buf[index++] = i+1;
        buf[index++] = (kp & 0xff);
        buf[index++] = (kp>>8) & 0xff;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(1c0);
        HAL_Delay(100);
    }
}

void settingKI(int16_t ki)
{
    char buf[8];
    for(int i=0;i<2;i++)
    {
        int index = 0;
        buf[index++] = 0x1e;
        buf[index++] = 0x20;
        buf[index++] = 0x20;
        buf[index++] = i+1;
        buf[index++] = (ki & 0xff);
        buf[index++] = (ki>>8) & 0xff;
        buf[index++] = 0x00;
        buf[index] = 0x00;
        sendCan(MOTOR114_REQ_ID, buf, 8, 0);
//        can->sendMsg(MOTOR114_REQ_ID,buf,false);
//        ThisThread::sleep_for(100);
        HAL_Delay(100);
    }
}
void settingMotor()
{
    settingEnc(ENC_RESOLUTION);
    settingPole(POLE_PAIR);
    settingBreak(BREAK_OPTION);
    settingMaxRPM(MAX_RPM);
    settingRatedCurrent(RATED_CURRENT);
    settingMaxCurrent(MAX_CURRENT);
    settingHallOffset(HALL_OFFSET);
    settingKP(KP_GAIN);
    settingKI(KI_GAIN);
    save();
}
