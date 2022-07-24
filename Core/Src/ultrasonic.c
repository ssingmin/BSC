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


//harness pin map change 3<<>>4
extern TIM_HandleTypeDef htim5;

int32_t USS_start[6] = {0,};
int32_t USS_end[6] = {0,};
int32_t USS_tick = 0;

extern uint32_t us_Tick;
extern uint32_t pre_usTick;



void USSn_Trigger(int USSn)
{
	uint32_t tmp = 0;
	//printf("flag11121\n");
	//printf("HAL_TIM_Base_Start_IT (&htim5)\n");
	//tmp=HAL_TIM_Base_Start_IT (&htim5);//uss timer, 200khz

	//printf("flag11122: %u\n", tmp);
	USS_start[USSn-1] = us_Tick;//start uss trigger
	switch(USSn){
		case 1:
			USS1_Trigger_Set;
			break;

		case 2:
			USS2_Trigger_Set;
			break;

		case 3:
			USS3_Trigger_Set;
			break;

		case 4:
			USS4_Trigger_Set;
			break;

		case 5:
			USS5_Trigger_Set;
			break;

		case 6:
			USS6_Trigger_Set;
			break;
	}



	//printf("USS_start%d: %d\n",USSn-1 ,USS_start[USSn-1]);
	pre_usTick = us_Tick;
	//printf("flag11123\n");
	for(int i=0;i<1500;i++){;}//wait about 200us
	//printf("flag11124\n");
	switch(USSn){
		case 1:
			USS1_Trigger_ReSet;
			break;

		case 2:
			USS2_Trigger_ReSet;
			break;

		case 3:
			USS3_Trigger_ReSet;
			break;

		case 4:
			USS4_Trigger_ReSet;
			break;

		case 5:
			USS5_Trigger_ReSet;
			break;

		case 6:
			USS6_Trigger_ReSet;
			break;

	}
	//printf("flag11125\n");
}

uint8_t USSn_DataRead(int USSn)
{
	int32_t USS_tmp = 0;
	double USS_calc = 0;
	//printf("flag1111\n");
	USSn_Trigger(USSn);
	HAL_Delay(30);
	USS_tmp = USS_end[USSn-1]-USS_start[USSn-1];

	//USS_calc=1.8*((double)USS_tmp) + 2.433 - 0.008*((double)(USS_tmp*USS_tmp));




	//printf("x3[0]: %d \n", USS_calc[0]);
	USS_calc += (0.01*(double)USS_tmp)*(1*(double)USS_tmp);//x^2, (0.108*10^-3)*x^2
	//printf("1: %f \n", USS_calc);
	USS_calc += (0.182*(double)USS_tmp);//x^1, (0.933)*x^1
	//printf("2: %f \n", USS_calc);
	USS_calc += 0.53;//x^0, constant
	//printf("3: %f \n", USS_calc);
	USS_calc -= (0.042*(double)USS_tmp)*(1*(double)USS_tmp)*(0.001*(double)USS_tmp);//x^3, (0.0361*10^-6)*x^3
	//printf("4: %f \n", USS_calc);

	//printf("USS_calc:%d USS_tmp:%d end:%d start:%d USS%d\n", (uint8_t)USS_calc, USS_tmp, USS_end[USSn-1], USS_start[USSn-1], USSn);
	//USS_calc[0]=(USS_end[0]-USS_start[0]);
	//printf("USS_calc[0]: %d \n", USS_calc);
	//printf("sonic value start, end, diff: %d  %d  %d\n", USS_start[0], USS_end[0], (USS_end[0]-USS_start[0]));
	//printf("flag1112\n");
	//USSn_Trigger(USSn);
	//printf("flag1113\n");
	//printf("\nUSS_tmp:%d USS_calc:%d USSn:%d\n", USS_tmp, USS_calc, USSn);
	if(USS_calc>250) {return 0;}

	return (uint8_t)USS_calc;
}


void USS_init()
{
	for(int i=1;i<7;i++){USSn_Trigger(i);}//다쏘면 안댐...ㅠ 두번 나눠서...ㅠ
}
