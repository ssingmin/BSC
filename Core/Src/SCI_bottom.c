/*
 * SCI_bottom.c
 *
 *  Created on: Jun 14, 2022
 *      Author: sangmin_lee
 */
#include "main.h"
#include "SCI_bottom.h"
#include <stdint.h>
#include "can.h"
#include "define.h"


uint8_t test = 0;
uint32_t us_Tick = 0;
uint32_t gTick = 0;
uint32_t pre_gTick = 0;
uint32_t Tick_100ms = 0;
uint32_t toggle_seq = 0;
uint32_t cansend_seq = 0;

uint32_t CTLcansend_seq = 0;

uint32_t sendsensor_seq = 0;

uint32_t controlmotor_seq = 0;

uint8_t motor_sw;
int motor_break;
int motor_disable_flag;

uint32_t USS_start = 0;
uint32_t USS_end = 0;
uint32_t USS_tick = 0;

MotorInfo *motor;
SensorState *sensor_state;

extern uint8_t g_uCAN_Rx_Data[8];
extern uint32_t FLAG_RxCplt;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//sequence timer. generate per 1ms
{
  if(htim->Instance == TIM5)
  {
	  us_Tick++;
	  //if(us_Tick>100){us_Tick=0;}
//	  if(us_Tick>2000){
//		  us_Tick=0;
//		  HAL_GPIO_TogglePin(BLUEtest_GPIO_Port, BLUEtest_Pin);
//	  }

  }

  if(htim->Instance == TIM6)
  {
	  gTick++;
	  if((gTick%10) == 0){Tick_100ms++;}
  }
  if(htim->Instance == TIM7)
  {
	  USS_tick++;
	  if(USS_tick>0xffff0000){USS_tick=0;}
  }

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == USS_Data1_Pin) {
    	USS_end = USS_tick;
    }

}

void controlMotor()
{
    static int count = 0;
    if(motor_sw)
    {
        if(motor_disable_flag)
        {
            enable();
            motor_disable_flag = 0;
        }
        if(motor_break == 1)
        {
            control((int)motor->cmd_motor_rpm_left,(int)motor->cmd_motor_rpm_right);
            motor_break = 2;
            count = 0;
        }
        else if(motor_break == 2)
        {
            count ++;
            control((int)motor->cmd_motor_rpm_left,(int)motor->cmd_motor_rpm_right);
            if(count == 20)
                motor_break = 3;

        }
        else if(motor_break == 3)
        {
            control(0,0);
            count = 0;
        }
    }
    else
    {
        disable();
        motor_disable_flag = 1;
    }
}


int toRPM()
{
    motor->cmd_motor_rpm_right = (60/(2*Math_PI*WHEEL_RADIUS)) * (motor->cmd_v + (WHEEL_DISTANCE/2)*motor->cmd_w);
    motor->cmd_motor_rpm_left = (60/(2*Math_PI*WHEEL_RADIUS)) * (motor->cmd_v - (WHEEL_DISTANCE/2)*motor->cmd_w);
    return 0;
}

void parseCmdvel(uint8_t *msg)
{
    /*cmd_v lower/cmd_v upper/cmd_w lower/cmd_w upper/ x / x / x / x */
    int16_t temp;
    temp = ((int16_t)msg[0]|(int16_t)msg[1]<<8);
    motor->cmd_v = (double)temp/SIGNIFICANT_FIGURES;
    temp = ((int16_t)msg[2]|(int16_t)msg[3]<<8);
    motor->cmd_w = (double)temp/SIGNIFICANT_FIGURES;
    motor_sw = msg[4];
    toRPM();
    motor_break = 1;
}



void sendEnc(int id)
{
    /*enc_v lower/enc_v upper/enc_w lower/enc_w upper/Undefined/Undefined/Undefined/Undefined*/
    char packit[8];
    int index=0;

    packit[index++]= ((int16_t)(motor->real_v*SIGNIFICANT_FIGURES)) & 0xff;
    packit[index++]= ((int16_t)(motor->real_v*SIGNIFICANT_FIGURES))>>8 & 0xff;
    packit[index++]= ((int16_t)(motor->real_w*SIGNIFICANT_FIGURES)) & 0xff;
    packit[index++]= ((int16_t)(motor->real_w*SIGNIFICANT_FIGURES))>>8 & 0xff;
    packit[index++]= (sensor_state->motor[1]<<1) | sensor_state->motor[0];
    packit[index++]= motor->RCURR * 100;
    packit[index++]= motor->LCURR * 100;
    packit[index++]=0;

    sendCan(id, packit, 8, 1);//test
}


int toVW(void)
{

    motor->real_motor_rpm_left=(double)motor->LRPM;
    motor->real_motor_rpm_right=(double)motor->RRPM;

    motor->real_v = (motor->real_motor_rpm_left+motor->real_motor_rpm_right)*(Math_PI*WHEEL_RADIUS/60);
    motor->real_w = (motor->real_motor_rpm_right-motor->real_motor_rpm_left)*((Math_PI*WHEEL_RADIUS)/(30*WHEEL_DISTANCE));
    return 0;
}


void parseEnc(uint8_t *msg)
{
    /*PID(216)/Motor1 status/Motor1 rpm lower/Motor1 rpm upper/Motor2 status/Motor2 rpm lower/Motor2 rpm upper/temperature(deg)*/
    if(msg[0]==PID_PNT_MONITOR)
    {

        motor->LRPM=((int16_t)msg[2] | ((int16_t)msg[3]<<8));
        motor->RRPM=((int16_t)msg[5] | ((int16_t)msg[6]<<8));
        motor->LRPM=-1*(motor->LRPM);
        sensor_state->motor[0] = msg[1];
        sensor_state->motor[1] = msg[4];

        toVW();
    }
    else if(msg[0]==PID_MAIN_DATA)
    {
        motor->RCURR=((int16_t)msg[4] | ((int16_t)msg[5]<<8))/10.0;
    }
    else if(msg[0]==PID_MAIN_DATA2)
    {
        motor->LCURR=((int16_t)msg[4] | ((int16_t)msg[5]<<8))/10.0;
    }
}

void parseEnc114(uint8_t *msg)
{
        int16_t rrpm,lrpm;
        lrpm = (int16_t)msg[4] | ((int16_t)msg[5]<<8);
        rrpm = (int16_t)msg[6] | ((int16_t)msg[7]<<8);

        motor->LRPM = (-lrpm) / 10.0;
        motor->RRPM = (rrpm) / 10.0;

        toVW();
}

void parseState114(uint8_t *msg)
{
    sensor_state->motor[0] = (int16_t)msg[4] | ((int16_t)msg[5]<<8);
    sensor_state->motor[1] = (int16_t)msg[6] | ((int16_t)msg[7]<<8);
    if(!(sensor_state->motor[0] == 0x00 && sensor_state->motor[1] == 0x00))
    {
        reset();
        startMotor();
    }
}

void spinonce(void)
{

	uint8_t canbuf[8]={10, 20, 30, 40, 50, 60, 70, 80};
	uint8_t buf[8];
    int index = 0;
//    int tmpindex = 0;

    uint32_t CanId = 0;
//    uint8_t motor_sw = 0;
//    static int count = 0;

//
//
//
//	double cmd_motor_rpm_left;
//	double cmd_motor_rpm_right;
//    double cmd_v;
//    double cmd_w;


	CanInit(0x100,0x1104);//filter id, mask


	while(1)
	{
    	if(Tick_100ms>toggle_seq+5) {		//for monitor iteration.
    		toggle_seq = Tick_100ms;
    		HAL_GPIO_TogglePin(REDtest_GPIO_Port, REDtest_Pin);

    	}



    	if(gTick>controlmotor_seq+4) {		//about controlmotor do it!!!!!
    		controlmotor_seq = gTick;
    		printf("hihi");
    		controlMotor();
            sendEnc(CANID3);
    	}


		if((Tick_100ms>sendsensor_seq)){
			sendsensor_seq = Tick_100ms;

			//printf("hihi: %d\n", USS_tick);

			/////////must need USS of fine Tuning/////////
//			USS_start = USS_tick;
//			HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, SET);
//			pre_gTick = USS_tick;
//			while(gTick<(pre_gTick+50)){;}//wait 500us
//			HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, RESET);
//
//			while(USS_end==0){;}
//
//			while(USS_tick<10){;}
//			USS_tick = 0;
//			printf("sonic value start, end, diff: %d  %d  %d\n", USS_start, USS_end, (USS_end-USS_start));
			//////////////////////////////////////////////

			buf[index++] = 0;
			buf[index++] = 0;
			buf[index++] = 0;
			buf[index++] = 0;
			buf[index++] = 0;
			buf[index++] = 0;
			buf[index++] = 0;
			buf[index] = 0;

			sendCan(2002, buf, 8, 1);//test
			index = 0;


		}
		if(FLAG_RxCplt){
    		for(int i=0;i<8;i++){canbuf[i] = g_uCAN_Rx_Data[i];}
    		FLAG_RxCplt=0;
			if(g_tCan_Rx_Header.StdId>g_tCan_Rx_Header.ExtId){CanId = g_tCan_Rx_Header.StdId;}
			else {CanId = g_tCan_Rx_Header.ExtId;}

    		switch(CanId)
    		{
            case CANID1:
//    		    temp = ((int16_t)canbuf[0]|(int16_t)canbuf[1]<<8);
//    		    cmd_v = (double)temp/SIGNIFICANT_FIGURES;
//    		    temp = ((int16_t)canbuf[2]|(int16_t)canbuf[3]<<8);
//    		    cmd_w = (double)temp/SIGNIFICANT_FIGURES;
//
//    		    cmd_motor_rpm_right = (60/(2*Math_PI*WHEEL_RADIUS)) * (cmd_v + (WHEEL_DISTANCE/2)*cmd_w);
//    		    cmd_motor_rpm_left = (60/(2*Math_PI*WHEEL_RADIUS)) * (cmd_v - (WHEEL_DISTANCE/2)*cmd_w);
//    		    motor_sw = canbuf[4];
            	 parseCmdvel(canbuf);
                break;

            case CANID2:
                //parseState(msg);
                break;
            case CANID5:
                //parseTop(msg);
                break;
            case CANID6:
                //parsePmm(msg);
                break;
            case CANID7:
            	parseEnc(canbuf);
                break;
            case MOTOR114_RES_ID:

                if(canbuf[1] == 0x6c && canbuf[2] == 0x60)
                    parseEnc114(canbuf);
                if(canbuf[1] == 0x3f && canbuf[2] == 0x60)
                    parseState114(canbuf);
                break;
            case MOTOR114_START_ID:
                startMotor();
                break;
    		}

		    g_tCan_Rx_Header.StdId=0;
			g_tCan_Rx_Header.ExtId=0;
			CanId = 0;

		}


//		printf("adc value ch1: %d ch2: %d ch3: %d ch4: %d \n",
//				adcval[0], adcval[1], adcval[2], adcval[3]);


	}
}

