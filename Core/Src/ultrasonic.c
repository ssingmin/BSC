/*
 * ultrasonic.c
 *
 *  Created on: Jun 14, 2022
 *      Author: sangmin_lee
 */

#include "ultrasonic.h"
#include "main.h"

#define USS1_Trigger_Set HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, SET)
#define USS2_Trigger_Set HAL_GPIO_WritePin(USS_Trigger2_GPIO_Port, USS_Trigger2_Pin, SET)
#define USS3_Trigger_Set HAL_GPIO_WritePin(USS_Trigger3_GPIO_Port, USS_Trigger3_Pin, SET)
#define USS4_Trigger_Set HAL_GPIO_WritePin(USS_Trigger4_GPIO_Port, USS_Trigger4_Pin, SET)
#define USS5_Trigger_Set HAL_GPIO_WritePin(USS_Trigger5_GPIO_Port, USS_Trigger5_Pin, SET)
#define USS6_Trigger_Set HAL_GPIO_WritePin(USS_Trigger6_GPIO_Port, USS_Trigger6_Pin, SET)

#define USS1_Trigger_ReSet HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, RESET)
#define USS2_Trigger_ReSet HAL_GPIO_WritePin(USS_Trigger2_GPIO_Port, USS_Trigger2_Pin, RESET)
#define USS3_Trigger_ReSet HAL_GPIO_WritePin(USS_Trigger3_GPIO_Port, USS_Trigger3_Pin, RESET)
#define USS4_Trigger_ReSet HAL_GPIO_WritePin(USS_Trigger4_GPIO_Port, USS_Trigger4_Pin, RESET)
#define USS5_Trigger_ReSet HAL_GPIO_WritePin(USS_Trigger5_GPIO_Port, USS_Trigger5_Pin, RESET)
#define USS6_Trigger_ReSet HAL_GPIO_WritePin(USS_Trigger6_GPIO_Port, USS_Trigger6_Pin, RESET)



extern TIM_HandleTypeDef htim5;

int32_t USS_start[6] = {0,};
int32_t USS_end[6] = {0,};
int32_t USS_tick = 0;

extern uint32_t us_Tick;
extern uint32_t pre_usTick;



void USSn_Trigger(int USSn)
{
	HAL_TIM_Base_Start_IT (&htim5);//uss timer, 200khz
	switch(USSn){
		case 1: USS1_Trigger_Set; break;
		case 2: USS2_Trigger_Set; break;
		case 3: USS3_Trigger_Set; break;
		case 4: USS4_Trigger_Set; break;
		case 5: USS5_Trigger_Set; break;
		case 6: USS6_Trigger_Set; break;
	}
	USS_start[USSn-1] = us_Tick;//start uss trigger
	pre_usTick = us_Tick;

	for(int i=0;i<1000;i++){;}//wait about 200us

	switch(USSn){
		case 1: USS1_Trigger_ReSet; break;
		case 2: USS2_Trigger_ReSet; break;
		case 3: USS3_Trigger_ReSet; break;
		case 4: USS4_Trigger_ReSet; break;
		case 5: USS5_Trigger_ReSet; break;
		case 6: USS6_Trigger_ReSet; break;
	}
}

uint8_t USSn_DataRead(int USSn)
{
	int32_t USS_tmp = 0;
	int32_t USS_calc = 0;

	USS_tmp = USS_end[USSn-1]-USS_start[USSn-1];
	USS_calc = (0.0361*(float)USS_tmp)*(0.001*(float)USS_tmp)*(0.001*(float)USS_tmp);//x^3, (0.0361*10^-6)*x^3
	//printf("x3[0]: %d \n", USS_calc[0]);
	USS_calc -= (0.108*(float)USS_tmp)*(0.001*(float)USS_tmp);//x^2, (0.108*10^-3)*x^2
	//printf("x2[0]: %d \n", USS_calc[0]);
	USS_calc += (0.933*(float)USS_tmp);//x^1, (0.933)*x^1
	//printf("x1[0]: %d \n", USS_calc[0]);
	USS_calc -= 41;//x^0, constant
	//USS_calc[0]=(USS_end[0]-USS_start[0]);
	//printf("USS_calc[0]: %d \n", USS_calc);
	//printf("sonic value start, end, diff: %d  %d  %d\n", USS_start[0], USS_end[0], (USS_end[0]-USS_start[0]));
	USSn_Trigger(USSn);

	if((USS_calc<=19) || (USS_calc>250)) {return 0;}

	return USS_calc;
}


void USS_init()
{
	for(int i=1;i<7;i++){USSn_Trigger(i);}//다쏘면 안댐...ㅠ 두번 나눠서...ㅠ
}
