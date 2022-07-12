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


uint8_t test = 0;

uint32_t gTick = 0;
uint32_t pre_gTick = 0;
uint32_t USS_start = 0;
uint32_t USS_end = 0;
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//sequence timer. generate per 1ms
//{
//  if(htim->Instance == TIM3)
//  {
//	  gTick++;
//  }
//
//}
//

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == USS_Data1_Pin) {
    	USS_end = gTick;
    }

}


void spinonce(void)
{

	uint8_t canbuf[8]={10, 20, 30, 40, 50, 60, 70, 80};

	CanInit(0,0);//filter id, mask


	while(1)
	{
		//HAL_GPIO_TogglePin(BLUEtest_GPIO_Port, BLUEtest_Pin);
		HAL_GPIO_TogglePin(REDtest_GPIO_Port, REDtest_Pin);
		HAL_Delay(500);


		//sendCan(4001, canbuf, 8, 1);//test

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
		gTick = 0;
		//////////////////////////////////////////////

//		printf("adc value ch1: %d ch2: %d ch3: %d ch4: %d \n",
//				adcval[0], adcval[1], adcval[2], adcval[3]);


	}
}

