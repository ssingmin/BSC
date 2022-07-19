/*
 * state.c
 *
 *  Created on: 2022. 7. 19.
 *      Author: sangmin_lee
 */




#include "state.h"

    enum STATE state;

void initState()
{
    state = READY;
}

void State_set(enum STATE state)
{
    state = state;
}

enum STATE State_get()
{
    return state;
}

