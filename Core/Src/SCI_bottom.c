/*
 * SCI_bottom.c
 *
 *  Created on: Jun 14, 2022
 *      Author: sangmin_lee
 */
#include "main.h"
#include <stdint.h>
#include "can.h"
#include "TransmitterIR.h"
#include "RemoteIR.h"
#include "ReceiverIR.h"
#include "charging.h"
#include "state.h"
#include "SCI_bottom.h"
#include "led.h"
#include "ultrasonic.h"

#define debugging 0//must delete 0=debug 1=release

uint8_t test = 0;
uint32_t us_Tick = 0;
uint32_t gTick = 0;
uint32_t pre_usTick = 0;
uint32_t Tick_100ms = 0;
uint32_t Tick_500ms = 0;

uint32_t toggle_seq = 0;
uint32_t state_seq = 0;
uint32_t cansend_seq = 0;
uint32_t FDsen_seq = 0;

uint8_t TIR_setData_flag = 0;

int IR_NEC_Tick = 0;
uint8_t isr_timeout_counter = 0;
uint8_t isr_timeout_flag = 0;


uint32_t CTLcansend_seq = 0;

uint32_t sendsensor_seq = 0;

uint32_t controlmotor_seq = 0;

uint32_t reqmotor_seq = 0;

uint8_t motor_sw=1;
int motor_break;
int motor_disable_flag;




int robot_state;

uint8_t air_sw;
uint8_t uv_sw;
uint8_t charge_relay_sw;
uint8_t charger_sw;
uint8_t check_docking_sig;

uint8_t touch;
uint8_t pir[6];

SensorState *sensor_state;
int battery;


uint8_t ready_flag=0;
uint8_t start_docking_flag;

int check_msg;



MotorInfo *motor;
extern uint8_t testflag;
extern uint8_t rx_data[2];

extern uint8_t g_uCAN_Rx_Data[8];
extern uint32_t FLAG_RxCplt;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern UART_HandleTypeDef huart8;
extern ADC_HandleTypeDef hadc1;

extern uint8_t recv_buf[32];

extern uint8_t robot_standby[4];//RsTb
extern uint8_t start_docking[4];//SsDk
extern uint8_t check_docking[4];//RfIn
extern uint8_t finish_docking[4];//SsEt
extern uint8_t charger_on[4];//
extern uint8_t charger_off[4];//
extern uint8_t battery_full[4];



extern int32_t USS_tick;
extern int32_t USS_end[6];


uint8_t charger_state_temp;
uint8_t check_docking_temp;
//uint8_t ready_flag;//remove

int ir_count = 0;
int inhome_check_cnt = 0;

extern RGB green;    //for charging2
extern RGB skyblue;   //for working, wallfollowing 3, 8
extern RGB yellow;     //for stop, W_stop 4, 5
extern RGB red;     //for emergency 6
extern RGB purple;   //for DOCKING 1
extern RGB white;      //for manual 7
extern RGB blue;

uint8_t inhome=0;
int ir_count_idle = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//sequence timer. generate per 1ms
{
  if(htim->Instance == TIM5)//uss timer, 100khz
  {
	  us_Tick++;
//	  if(us_Tick>0xffff0000){us_Tick=0;}

  }

  if(htim->Instance == TIM6)//system timer, 100hz
  {
	  gTick++;
	  if((gTick%10) == 0){Tick_100ms++;}
	  if((gTick%50) == 0){Tick_500ms++;}

  }

  if(htim->Instance == TIM7)//uss timer, 1khz
  {
	  USS_tick++;
	  if(USS_tick>0xffff0000){USS_tick=0;}
  }

  if(htim->Instance == TIM9)//uss timer, 1779hz
  {
	  if(TIR_setData_flag){tick();}
	  if(isr_timeout_flag){isr_timeout_counter++;}

	  if(isr_timeout_counter>1)//횟수 수정할 것
	  {
		  isr_timeout_counter = 0;
		  isr_timeout();
	  }
  }

  if(htim->Instance == TIM14)//IR NEC timer, 1Mhz
  {
	  IR_NEC_Tick+=4;
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == USS_Data1_Pin) {
    	USS_end[0] = us_Tick;
    	HAL_TIM_Base_Stop_IT (&htim5);//uss timer, 200khz
    }

    if(GPIO_Pin == USS_Data2_Pin) {
    	USS_end[1] = us_Tick;
    	HAL_TIM_Base_Stop_IT (&htim5);//uss timer, 200khz
    }

    if(GPIO_Pin == USS_Data3_Pin) {
    	//USS_end[2] = us_Tick;
    	HAL_TIM_Base_Stop_IT (&htim5);//uss timer, 200khz
    }

    if(GPIO_Pin == USS_Data4_Pin) {
    	USS_end[3] = us_Tick;
    	HAL_TIM_Base_Stop_IT (&htim5);//uss timer, 200khz
    }

    if(GPIO_Pin == USS_Data5_Pin) {
    	USS_end[4] = us_Tick;
    	HAL_TIM_Base_Stop_IT (&htim5);//uss timer, 200khz
    }

    if(GPIO_Pin == USS_Data6_Pin) {
    	//USS_end[5] = us_Tick;
    	HAL_TIM_Base_Stop_IT (&htim5);//uss timer, 200khz
    }



    if(GPIO_Pin == evt_rxpin_Pin){ //check interrupt for specific pin
            if(HAL_GPIO_ReadPin(evt_rxpin_GPIO_Port, evt_rxpin_Pin)){	isr_rise(); }//high edge
             if(!HAL_GPIO_ReadPin(evt_rxpin_GPIO_Port, evt_rxpin_Pin)){ isr_fall();}//low edge
        }
}


void startTTS()
{
    char packit[8];
    int index=0;

    packit[index++]= 0;
    packit[index++]= 0; // temporarily designated
    packit[index++]= 0;
    packit[index++]= 0;
    packit[index++]= 0;
    packit[index++]= 0;
    packit[index++]= 1;
    packit[index++]= 0;

//    if(!can->send8BytePackit(CANID8,packit))
//        can->reset();
    sendCan(5001, packit, 8, 1);
}

void endTTS()
{
    char packit[8];
    int index=0;

    packit[index++]= 0;
    packit[index++]= 0; // temporarily designated
    packit[index++]= 0;
    packit[index++]= 0;
    packit[index++]= 0;
    packit[index++]= 0;
    packit[index++]= 9;
    packit[index++]= 0;

//    if(!can->send8BytePackit(CANID8,packit))
//        can->reset();
    sendCan(5001, packit, 8, 1);
}

void parsePmm(uint8_t *msg)
{
    /* x / x / x / x / x / x / air,uv,relay state/ Battery */
    battery = msg[7];
    sensor_state->air_purifier = (msg[6]&128)>>7;
    sensor_state->uv = (msg[6]&64)>>6;
    sensor_state->relay = (msg[6]&32)>>5;
}


void parseTop(uint8_t *msg)
{
    /* x / x / x / x / x / x /touch sensor/PIR */
    for(int i=0; i<6; i++)
        pir[i] = (msg[7]>>i)&1; // back is 0
    touch = msg[6];
}


void parseState(uint8_t *msg)
{
    /* x / x / x / x / charging relay / air,uv on off / speaker / robot state */
    robot_state = msg[7];
    air_sw = (msg[5] & 128)>>7;
    uv_sw = (msg[5] & 64)>>6;
    charge_relay_sw = (msg[4] & 128)>>7;
    charger_sw = (msg[4] & 64)>>6;
    check_docking_sig = (msg[4] & 32)>>5;
    //fan_duty = msg[3] / 100.0;
    //controlFan(air_sw);
}

void controlMotor()
{
    static int count = 0;
    printf("motor_sw=%d, motor_break=%d\n", motor_sw, motor_break);
    if(motor_sw)
    {
        if(motor_disable_flag)
        {printf("enable\n");
            enable();
            motor_disable_flag = 0;
        }
        if(motor_break == 1)
        {
            control((int)motor->cmd_motor_rpm_left,(int)motor->cmd_motor_rpm_right);
            motor_break = 2;
            count = 0;
            printf("motor_break==1\n");
        }
        else if(motor_break == 2)
        {
            count++;
            control((int)motor->cmd_motor_rpm_left,(int)motor->cmd_motor_rpm_right);
            if(count == 20)
                motor_break = 3;
            printf("motor_break==2\n");
        }
        else if(motor_break == 3)
        {
        	printf("motor_break==3\n");
            control(0,0);
            count = 0;
        }
    }
    else
    {
        disable();
        motor_disable_flag = 1;
        printf("disable==1\n");
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


int stateReady()//이거 전에 ir통신을 받아야 겠는데?
{
	for(inhome_check_cnt=0;inhome_check_cnt<73;inhome_check_cnt++)
	{
		    //check_msg = charging->checkIRdata();
		if(check_msg == 1)
		{
			start_docking_flag = 1;
		}

		if(ir_count++ >= 2)
		{
			sendIRdata(robot_standby);
			ir_count = 0;
			//HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);//이걸로 수신시작할 것
		}

		if(start_docking_flag)
		{
		   startTTS();
		   //State_set(IDLE);
		   ready_flag = 1;
		   start_docking_flag = 0;
		   //mutex.lock();
		   check_msg = 0;
		   return 1;
		   //mutex.unlock();
		}
//		else if(inhome_check_cnt > 73)
//		{
//			inhome_check_cnt = 0;
//			endTTS();
//			//ThisThread::sleep_for(50);
//			NVIC_SystemReset();
//		}
		HAL_GPIO_TogglePin(BLUEtest_GPIO_Port, BLUEtest_Pin);
		check_msg = checkIRdata();
		HAL_Delay(100);
	}
	endTTS();
	HAL_Delay(100);
	//ThisThread::sleep_for(50);
	NVIC_SystemReset();
    //inhome_check_cnt++;
}

void stateThread()
{

	printf("stateThread, robot_state: %d\n", robot_state);
        switch(robot_state)
        {
            case INIT:
                turnOff();
                break;

            case DOCKING:
                turnOn(white);
                inhome = 0;
                break;

            case CHARGING:
                if(battery>95)
                {
                    turnOn(green);
                }
                else
                {
                    turnOn(yellow);
                }
                break;

            case WORKING:
                turnOn(white);
                inhome = 0;
                break;

            case STOP:
                turnOn(purple);
                inhome = 0;
                break;

            case W_STOP:
                turnOn(purple);
                break;

            case EMERGENCY:
                turnOn(red);
                inhome = 0;
                break;

            case MANUAL:
                if(touch)
                    turnOn(blue);
                else
                    turnOn(skyblue);
                inhome = 0;
                break;

            case WALL_FOLLOWING:
                turnOn(white);
                break;
            case 10: //debug
                turnOn(blue);
                break;

			case 11: //for operation test
				turnOn(red);
				break;

			case 12: //for operation test
				turnOn(green);
				break;

        }
//        mutex.lock();
//        check_msg = charging->checkIRdata();
//        mutex.unlock();
//        ThisThread::sleep_for(131);

}


void stateIdle()
{
    //int check_msg = 0;
    //check_msg = charging->checkIRdata();

    if(robot_state == CHARGING)
    {
        if(ir_count_idle++ >= 2)
        {
            if(charger_sw == 1)
            {
                // charging->sendIRdata(charger_on);
            }
            else if(charger_sw == 0)
            {
                // charging->sendIRdata(charger_off);
            }

            if(battery>=95)
            {
                //sendIRdata(battery_full);
            }
            ir_count_idle = 0;
        }
    }
    else
    {
        check_msg = 0;
    }

    if(check_msg == 2)
    {
        inhome = 1;

    }

}


void spinonce(void)
{

	uint8_t canbuf[8]={10, 20, 30, 40, 50, 60, 70, 80};
	uint8_t buf[8];
    int index = 0;

    uint32_t CanId = 0;
    uint16_t FDval[4]={0,};
	//CanInit(0x100,0x1104);//filter id, mask
    CanInit(0,0);//filter id, mask


    HAL_Delay(4000);//must set more 4s

    //ready_flag = 1;
    start_docking_flag = 0;
    check_msg = 0;

    Format format = NEC;

    settingMotor();
    startMotor();

	TransmitterIR_init();
    ReceiverIR_init();
#if debugging
    stateReady();
    HAL_Delay(10000);
#endif

    //USS_init();

	while(1)
	{


		if(Tick_100ms>FDsen_seq+10) {		//for monitor iteration.
			FDsen_seq = Tick_100ms;

		    for(int i=0;i<4;i++){
		    	HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 100);
				FDval[i] = HAL_ADC_GetValue(&hadc1);
		    }
		    HAL_ADC_Stop(&hadc1);

		  //printf("FDval: %d %d %d %d\n", FDval[0], FDval[1], FDval[2], FDval[3]);
		  //HAL_Delay(100);


			}

		if(Tick_100ms>toggle_seq+5) {		//for monitor iteration.
    		toggle_seq = Tick_100ms;
    		HAL_GPIO_TogglePin(REDtest_GPIO_Port, REDtest_Pin);

    		printf("redtest\n");
    	}

    	if(gTick>controlmotor_seq+4) {		//about controlmotor do it!!!!!
    		controlmotor_seq = gTick;
    		//printf("hihi");
    		controlMotor();
            sendEnc(CANID3);
            printf("controlmotor_seq%d\n", gTick);
    	}
    	if(gTick>reqmotor_seq+3) {		//REQ MOTOR
    		reqmotor_seq = gTick;

    		if((reqmotor_seq%8) == 0){reqEnc();}
    		else{reqState();}
    	}
		if(Tick_100ms>state_seq+9) {
			state_seq = Tick_100ms;
        	stateIdle();

    		stateThread();
    	}


		if((Tick_100ms>sendsensor_seq+5)){
			sendsensor_seq = Tick_100ms;
			//for(int i=1;i<7;i++){printf("sonic test %d  ", USSn_DataRead(i));}	printf("\n");
			//printf("sonic test %d\n", USSn_DataRead(4));

//			//printf("hihi: %d\n", USS_tick);
//
//			USS_tmp = USS_end[0]-USS_start[0];
//			USS_calc[0] = (0.0361*(float)USS_tmp)*(0.001*(float)USS_tmp)*(0.001*(float)USS_tmp);//x^3
//			printf("x3[0]: %d \n", USS_calc[0]);
//			USS_calc[0] -= (0.108*(float)USS_tmp)*(0.001*(float)USS_tmp);//x^2
//			printf("x2[0]: %d \n", USS_calc[0]);
//			USS_calc[0] += (0.933*(float)USS_tmp);//x^1
//			printf("x1[0]: %d \n", USS_calc[0]);
//			USS_calc[0] -= 41;//x^0
//			//USS_calc[0]=(USS_end[0]-USS_start[0]);
//			printf("USS_calc[0]: %d \n", USS_calc[0]);
//			USS_calc[0] = 0;
//			//printf("sonic value start, end, diff: %d  %d  %d\n", USS_start[0], USS_end[0], (USS_end[0]-USS_start[0]));
//
//
//			HAL_TIM_Base_Start_IT (&htim5);//uss timer, 200khz
//			HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, SET);
//			USS_start[0] = us_Tick;//start uss trigger
//			pre_usTick = us_Tick;
//			while(us_Tick < pre_usTick+30){;}//wait 150us
//			HAL_GPIO_WritePin(USS_Trigger1_GPIO_Port, USS_Trigger1_Pin, RESET);

			//////////////////////////////////////////////
			if(sendsensor_seq%2==1){for(int i=1;i<7;i+=2){buf[i] = USSn_DataRead(i);}}
			else {for(int i=2;i<7;i+=2){buf[i] = USSn_DataRead(i);}}
//			buf[0] = USSn_DataRead(1);
//			buf[1] = USSn_DataRead(2);
//			buf[1] = 30;
//			buf[3] = USSn_DataRead(4);
//			buf[4] = USSn_DataRead(5);
//			buf[5] = 30;
//			buf[index++] = 0;
//			buf[index++] = 0;
//			buf[index++] = 0;
//			buf[index++] = 0;
//			buf[index++] = 0;
//			buf[index++] = 0;
//			buf[index++] = 0;
			buf[6] = inhome << 1;

			for(int i=0; i<4;i++){
				if(FDval[i]>50){buf[index] |= 1<<i+4;}
				else {buf[index] |= 0<<i+4;}
			}
			//buf[index] = 0;
			sendCan(CANID4, buf, 8, 1);//test
			index = 0;

		}

		if(FLAG_RxCplt>0){
    		for(int i=0;i<8;i++){canbuf[i] = g_uCAN_Rx_Data[i];}
    		FLAG_RxCplt--;
			if(g_tCan_Rx_Header.StdId>g_tCan_Rx_Header.ExtId){CanId = g_tCan_Rx_Header.StdId;}
			else {CanId = g_tCan_Rx_Header.ExtId;}
			if(CanId==1001){printf("canid1001 ready: %d\n", ready_flag);}
			if(ready_flag)
			{
				switch(CanId)
				{
				case CANID1:
					parseCmdvel(canbuf);
					printf("parseCmdvel\n");
					break;

				case CANID2:
					parseState(canbuf);
					break;

				case CANID5:
					parseTop(canbuf);
					break;

				case CANID6:
					parsePmm(canbuf);
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
			}


			g_tCan_Rx_Header.StdId=0;
			g_tCan_Rx_Header.ExtId=0;
			CanId = 0;

		}

	}
}

