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

uint32_t gTick = 0;
uint32_t pre_gTick = 0;
uint32_t Tick_100ms = 0;
uint32_t toggle_seq = 0;
uint32_t cansend_seq = 0;

uint32_t CTLcansend_seq = 0;

uint32_t send2001 = 0;
uint32_t send2002 = 0;
uint32_t send2003 = 0;

uint32_t controlmotor_seq = 0;



uint32_t USS_start = 0;
uint32_t USS_end = 0;

MotorInfo *motor;
SensorState *sensor_state;

extern uint8_t g_uCAN_Rx_Data[8];
extern uint32_t FLAG_RxCplt;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//sequence timer. generate per 1ms
{
  if(htim->Instance == TIM6)
  {
	  gTick++;
	  if((gTick%10) == 0){Tick_100ms++;}
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == USS_Data1_Pin) {
    	USS_end = gTick;
    }

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

void spinonce(void)
{

	uint8_t canbuf[8]={10, 20, 30, 40, 50, 60, 70, 80};
	uint8_t buf[8];
    int index = 0;
    int tmpindex = 0;
    int16_t temp;
    uint32_t CanId = 0;
    uint8_t motor_sw = 0;
    static int count = 0;

    int motor_disable_flag;
    int motor_break;

	double cmd_motor_rpm_left;
	double cmd_motor_rpm_right;
    double cmd_v;
    double cmd_w;


	CanInit(0x100,0x1104);//filter id, mask


	while(1)
	{
    	if(Tick_100ms>toggle_seq+5) {		//for monitor iteration.
    		toggle_seq = Tick_100ms;
    		HAL_GPIO_TogglePin(REDtest_GPIO_Port, REDtest_Pin);
    	}

    	if(gTick>controlmotor_seq+4) {		//about controlmotor do it!!!!!
    		controlmotor_seq = Tick_100ms;

    		    if(motor_sw)
    		    {
    		        if(motor_disable_flag)
    		        {
    		            enable();
    		            motor_disable_flag = 0;
    		        }
    		        if(motor_break == 1)
    		        {
    		            control((int)cmd_motor_rpm_left,(int)cmd_motor_rpm_right);
    		            motor_break = 2;
    		            count = 0;
    		        }
    		        else if(motor_break == 2)
    		        {
    		            count ++;
    		            control((int)cmd_motor_rpm_left,(int)cmd_motor_rpm_right);
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

            sendEnc(CANID3);
    	}


		if((Tick_100ms>send2002)){
			send2002 = Tick_100ms;

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

				buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index] = 0;

			sendCan(2003, buf, 8, 1);//test
			index = 0;


		}

		if((gTick>cansend_seq+3)){
			cansend_seq = gTick;
			tmpindex++;
						//reqEnc
			if((tmpindex/2)==0){
			    buf[index++] = 0x40;
			    buf[index++] = 0x6c;
			    buf[index++] = 0x60;
			    buf[index++] = 0x03;
			    buf[index++] = 0x00;
			    buf[index++] = 0x00;
			    buf[index++] = 0x00;
			    buf[index] = 0x00;
			    index=0;
			}

			else{
				index=0;
			    buf[index++] = 0x40;
			    buf[index++] = 0x3f;
			    buf[index++] = 0x60;
			    buf[index++] = 0x00;
			    buf[index++] = 0x00;
			    buf[index++] = 0x00;
			    buf[index++] = 0x00;
			    buf[index] = 0x00;
			    index=0;
			}
			sendCan(MOTOR114_REQ_ID, buf, 8, 1);//test
		}
		if((gTick>CTLcansend_seq+4)){
			CTLcansend_seq = gTick;

				cmd_motor_rpm_left = -1*cmd_motor_rpm_left;
			    buf[index++] = 0x23;
			    buf[index++] = 0xff;
			    buf[index++] = 0x60;
			    buf[index++] = 0x03;
			    buf[index++] = ((int)cmd_motor_rpm_left & 0xff);
			    buf[index++] = ((int)cmd_motor_rpm_left>>8) & 0xff;
			    buf[index++] =  ((int)cmd_motor_rpm_right) & 0xff;
			    buf[index] = ((int)cmd_motor_rpm_right>>8) & 0xff;

			sendCan(MOTOR114_REQ_ID, buf, 8, 1);//test
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
    		    temp = ((int16_t)canbuf[0]|(int16_t)canbuf[1]<<8);
    		    cmd_v = (double)temp/SIGNIFICANT_FIGURES;
    		    temp = ((int16_t)canbuf[2]|(int16_t)canbuf[3]<<8);
    		    cmd_w = (double)temp/SIGNIFICANT_FIGURES;

    		    cmd_motor_rpm_right = (60/(2*Math_PI*WHEEL_RADIUS)) * (cmd_v + (WHEEL_DISTANCE/2)*cmd_w);
    		    cmd_motor_rpm_left = (60/(2*Math_PI*WHEEL_RADIUS)) * (cmd_v - (WHEEL_DISTANCE/2)*cmd_w);
    		    motor_sw = canbuf[4];
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

//                if(msg.data[1] == 0x6c && msg.data[2] == 0x60)
//                    parseEnc114(msg);
//                if(msg.data[1] == 0x3f && msg.data[2] == 0x60)
//                    parseState114(msg);
                break;
            case MOTOR114_START_ID:
                //motor114->startMotor();
                break;
    		}















		    g_tCan_Rx_Header.StdId=0;
			g_tCan_Rx_Header.ExtId=0;
			CanId = 0;





		}
		/////////must need USS of fine Tuning/////////
		USS_start = gTick;
		HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, SET);
		pre_gTick = gTick;
		//while(gTick<(pre_gTick+50)){;}//wait 500us
		HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, RESET);

		//while(USS_end==0){;}

		//while(gTick<10){;}
		//gTick = 0;
		//printf("sonic value start, end, diff: %d  %d  %d\n", USS_start, USS_end, (USS_end-USS_start));
		//////////////////////////////////////////////

//		printf("adc value ch1: %d ch2: %d ch3: %d ch4: %d \n",
//				adcval[0], adcval[1], adcval[2], adcval[3]);


	}
}

