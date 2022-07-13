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

#define MOTOR114_RES_ID 0x581
#define MOTOR114_REQ_ID 0x601
#define MOTOR114_START_ID 0x701

#define Math_PI 3.14159265358979323846
#define SIGNIFICANT_FIGURES 100
#define WHEEL_RADIUS 0.065
#define WHEEL_DISTANCE 0.464


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



uint32_t USS_start = 0;
uint32_t USS_end = 0;
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


void spinonce(void)
{

	uint8_t canbuf[8]={10, 20, 30, 40, 50, 60, 70, 80};
	uint8_t buf[8];
    int index = 0;
    int tmpindex = 0;
    int16_t temp;

	double cmd_motor_rpm_left;
	double cmd_motor_rpm_right;
    double cmd_v;
    double cmd_w;


	CanInit(0x3ea,0x7ff);//filter id, mask


	while(1)
	{
    	if(Tick_100ms>toggle_seq+5) {		//for monitor iteration.
    		toggle_seq = Tick_100ms;
    		HAL_GPIO_TogglePin(REDtest_GPIO_Port, REDtest_Pin);
    	}
		//HAL_GPIO_TogglePin(BLUEtest_GPIO_Port, BLUEtest_Pin);

		//HAL_Delay(500);

		//printf("hihi: %d\n",pre_gTick);

		if((gTick>send2001+4)){
			send2001 = gTick;

			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index++] = 0;
			    buf[index] = 0;

			sendCan(2001, buf, 8, 1);//test
			index = 0;
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
			FLAG_RxCplt=0;
			for(int i=0;i<8;i++){canbuf[i]=g_uCAN_Rx_Data[i];}

		    temp = ((int16_t)canbuf[0]|(int16_t)canbuf[1]<<8);
		    cmd_v = (double)temp/SIGNIFICANT_FIGURES;
		    temp = ((int16_t)canbuf[2]|(int16_t)canbuf[3]<<8);
		    cmd_w = (double)temp/SIGNIFICANT_FIGURES;

		    cmd_motor_rpm_right = (60/(2*Math_PI*WHEEL_RADIUS)) * (cmd_v + (WHEEL_DISTANCE/2)*cmd_w);
		    cmd_motor_rpm_left = (60/(2*Math_PI*WHEEL_RADIUS)) * (cmd_v - (WHEEL_DISTANCE/2)*cmd_w);

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

