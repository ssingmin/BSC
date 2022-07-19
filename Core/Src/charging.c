/*
 * charging.c
 *
 *  Created on: 2022. 7. 15.
 *      Author: sangmin_lee
 */


#include "charging.h"

extern Format format;

    uint8_t recv_buf[32] = {0,};
    uint8_t robot_standby[4] = {0xCA, 0x35, 0x9A, 0x65};//RsTb
    uint8_t start_docking[4] = {0xCE, 0x32, 0x9B, 0x64};//SsDk
    uint8_t check_docking[4] = {0xCA, 0x35, 0x9C, 0x63};//RfIn
    uint8_t finish_docking[4] = {0xCE, 0x32, 0x9D, 0x62};//SsEt
    uint8_t charger_on[4] = {0xCA, 0x35, 0x9E, 0x61};//
    uint8_t charger_off[4] = {0xCA, 0x35, 0x9F, 0x60};//
    uint8_t battery_full[4] = {0xCA, 0x35, 0xAA, 0x55};

void sendIRdata(uint8_t send_data[])
{
    if(getState() == Idle)
    {
        //ir_rx->evt.disable_irq();
        setData(NEC, send_data, 32);
        //ir_rx->evt.enable_irq();
    }
}

int checkIRdata()
{
    int bitcount = 0;
    int check_count = 0;
    int start_docking_count = 0;
    int finish_docking_count = 0;

    if(getState() == Received)
    {
        for(int i = 0; i < 32; i++)
        {
            recv_buf[i] = '0';
        }
        bitcount = getData(NEC, recv_buf, sizeof(recv_buf)*8);
    }
    else if(getState() != Received)
    {
    	printf("getState() != Received ");
        return 0;
    }

    for(int i = 0; i<4; i++)
    {
    	printf("hihihihi111111111: ");
        if(recv_buf[i] == robot_standby[i]) {start_docking_count++;}
        		//if(recv_buf[i] == start_docking[i]) {start_docking_count++;}
        if(recv_buf[i] == finish_docking[i]) {finish_docking_count++;}
        printf("%x : %x\n", recv_buf[i], start_docking[i]);
    }

    if(start_docking_count == 4)
    {
        return 1;
    }
    else if(finish_docking_count == 4)
    {
        return 2;
    }

    printf("hihihihi22222start_docking_count: %d", start_docking_count);
        return 0;
}
