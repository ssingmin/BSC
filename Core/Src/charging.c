/*
 * charging.c
 *
 *  Created on: 2022. 7. 15.
 *      Author: sangmin_lee
 */

//
//#include "include/charging.h"
//
//
//uvbot::Charging::Charging()
//{
//    initSw();
//}
//
//uvbot::Charging::~Charging()
//{
//    delete ir_tx;
//    delete ir_rx;
//}
//
//void uvbot::Charging::initSw()
//{
//    ir_tx = new TransmitterIR(PA_5);
//    ir_rx = new ReceiverIR(PE_14);
//}
//
//void uvbot::Charging::sendIRdata(uint8_t send_data[])
//{
//    if(ir_tx->getState() == TransmitterIR::Idle)
//    {
//        ir_rx->evt.disable_irq();
//        ir_tx->setData(format, send_data, 32);
//        ir_rx->evt.enable_irq();
//    }
//}
//
//int uvbot::Charging::checkIRdata()
//{
//    int bitcount = 0;
//    int check_count = 0;
//    int start_docking_count = 0;
//    int finish_docking_count = 0;
//
//    if(ir_rx->getState() == ReceiverIR::Received)
//    {
//        for(int i = 0; i < 32; i++)
//        {
//            recv_buf[i] = '0';
//        }
//        bitcount = ir_rx->getData(&format, recv_buf, sizeof(recv_buf)*8);
//    }
//    else if(ir_rx->getState() != ReceiverIR::Received)
//    {
//        return 0;
//    }
//
//    for(int i = 0; i<4; i++)
//    {
//        if(recv_buf[i] == start_docking[i])
//        {
//            start_docking_count++;
//        }
//        if(recv_buf[i] == finish_docking[i])
//        {
//            finish_docking_count++;
//        }
//    }
//
//    if(start_docking_count == 4)
//    {
//        return 1;
//    }
//    else if(finish_docking_count == 4)
//    {
//        return 2;
//    }
//
//    return 0;
//}
//
//uint8_t *uvbot::Charging::getRecieveBuf()
//{
//    return recv_buf;
//}

