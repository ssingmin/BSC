/**
 * IR transmitter (Version 0.0.4)
 *
 * Copyright (C) 2010 Shinichiro Nakamura (CuBeatSystems)
 * http://shinta.main.jp/
 */

#ifndef _TRANSMITTER_IR_H_
#define _TRANSMITTER_IR_H_

#include "RemoteIR.h"
#include "stm32f7xx_hal.h"
#include "main.h"

#define IRDUTY0 htim2.Instance->CCR1 = 0
#define IRDUTY50 htim2.Instance->CCR1 = 52


extern TIM_HandleTypeDef htim2;
extern uint8_t TIR_setData_flag;

/**
 * IR transmitter class.
 */

    /**
     * Constructor.
     *
     * @param txpin Pin for transmit IR signal.
     */
    typedef enum {
    	TIR_Idle,
        Leader,
        Data,
        Trailer
    } TIR_State;



    typedef struct {
    	TIR_State state;
        int bitcount;
        int leader;
        int data;
        int trailer;
    } TIR_work_t;

    typedef struct {
        Format format;
        int bitlength;
        uint8_t buffer[64];
    } TIR_data_t;


    /**
     * Get state.
     *
     * @return Current state.
     */
    TIR_State TIR_getState(void);

    /**
     * Set data.
     *
     * @param format Format.
     * @param buf Buffer of a data.
     * @param bitlength Bit length of the data.
     *
     * @return Data bit length.
     */
    int setData(Format format, uint8_t *buf, int bitlength);



    //PwmOut tx;
    //Ticker ticker;



    void tick();

    void TransmitterIR_init();

#endif
