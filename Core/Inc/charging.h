/*
 * charging.h
 *
 *  Created on: 2022. 7. 15.
 *      Author: sangmin_lee
 */

#ifndef INC_CHARGING_H_
#define INC_CHARGING_H_


//#include "mbed.h"
//#include "RemoteIR/TransmitterIR.h"
//#include "RemoteIR/ReceiverIR.h"
//#include <cstdint>
//#include "include/can.h"
//
//
//namespace uvbot
//{
//
//class Charging
//{
//public:
//    Charging();
//    ~Charging();
//    void initSw();
//    void sendIRdata(uint8_t send_data[]);
//    int checkIRdata();
//    uint8_t *getRecieveBuf();
//
//private:
//    ReceiverIR *ir_rx;
//    TransmitterIR *ir_tx;
//    RemoteIR::Format format = RemoteIR::NEC;
//    uint8_t recv_buf[32] = {0,};
//    uint8_t robot_standby[4] = {0xCA, 0x35, 0x9A, 0x65};//RsTb
//    uint8_t start_docking[4] = {0xCE, 0x32, 0x9B, 0x64};//SsDk
//    uint8_t check_docking[4] = {0xCA, 0x35, 0x9C, 0x63};//RfIn
//    uint8_t finish_docking[4] = {0xCE, 0x32, 0x9D, 0x62};//SsEt
//    uint8_t charger_on[4] = {0xCA, 0x35, 0x9E, 0x61};//
//    uint8_t charger_off[4] = {0xCA, 0x35, 0x9F, 0x60};//
//    uint8_t battery_full[4] = {0xCA, 0x35, 0xAA, 0x55};
//
//};
//
//
//
//};//end namespace



#endif /* INC_CHARGING_H_ */
