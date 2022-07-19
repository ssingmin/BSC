/*
 * state.h
 *
 *  Created on: 2022. 7. 19.
 *      Author: sangmin_lee
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_


enum STATE
{
    READY = 0x00,
    IDLE = 0x01
};


    void initState();

    void State_set(enum STATE state);
    enum STATE State_get();

#endif /* INC_STATE_H_ */
