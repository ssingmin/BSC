#include <stdint.h>
#include "TransmitterIR.h"

#define LOCK()
#define UNLOCK()

extern TIM_HandleTypeDef htim9;

TIR_data_t TIR_data;
TIR_work_t TIR_work;


/**
 * Get state.
 *
 * @return Current state.
 */

void TransmitterIR_init()
{




	IRDUTY0;
	//IRDUTY50;

	TIR_work.state = Idle;
	TIR_work.bitcount = 0;
	TIR_work.leader = 0;
	TIR_work.data = 0;
	TIR_work.trailer = 0;

	TIR_data.format = UNKNOWN;
	TIR_data.bitlength = 0;




}

State TIR_getState(void) {
    LOCK();
    State s = TIR_work.state;
    UNLOCK();
    return s;
}

/**
 * Set data.
 *
 * @param format Format.
 * @param buf Buffer of a data.
 * @param bitlength Bit length of the data.
 *
 * @return Data bit length.
 */
int setData(Format format, uint8_t *buf, int bitlength) {
    LOCK();
    if (TIR_work.state != Idle) {
        UNLOCK();
        return -1;
    }

    TIR_work.state = Leader;
    TIR_work.bitcount = 0;
    TIR_work.leader = 0;
    TIR_work.data = 0;
    TIR_work.trailer = 0;

    TIR_data.format = format;
    TIR_data.bitlength = bitlength;
    const int n = bitlength / 8 + (((bitlength % 8) != 0) ? 1 : 0);
    for (int i = 0; i < n; i++) {
        TIR_data.buffer[i] = buf[i];
    }

    switch (format) {
        case NEC:
//            ticker.detach();
//            ticker.attach_us(callback(this, &tick), TUS_NEC);
        		HAL_TIM_Base_Start_IT (&htim9);//uss timer, 1779hz
        		TIR_setData_flag = 1;

            break;
        case AEHA:
//            ticker.detach();
//            ticker.attach_us(callback(this, &tick), TUS_AEHA);
            break;
        case SONY:
//            ticker.detach();
//            ticker.attach_us(callback(this, &tick), TUS_SONY);
            break;
    }

    UNLOCK();
    return bitlength;
}

void tick(void) {
    LOCK();
    switch (TIR_work.state) {
        case Idle:
            TIR_work.bitcount = 0;
            TIR_work.leader = 0;
            TIR_work.data = 0;
            TIR_work.trailer = 0;
            break;
        case Leader:
            if (TIR_data.format == NEC) {
                /*
                 * NEC.
                 */
                static const int LEADER_NEC_HEAD = 16;
                static const int LEADER_NEC_TAIL = 8;
                if (TIR_work.leader < LEADER_NEC_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                TIR_work.leader++;
                if ((LEADER_NEC_HEAD + LEADER_NEC_TAIL) <= TIR_work.leader) {
                    TIR_work.state = Data;
                }
            } else if (TIR_data.format == AEHA) {
                /*
                 * AEHA.
                 */
                static const int LEADER_AEHA_HEAD = 8;
                static const int LEADER_AEHA_TAIL = 4;
                if (TIR_work.leader < LEADER_AEHA_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                TIR_work.leader++;
                if ((LEADER_AEHA_HEAD + LEADER_AEHA_TAIL) <= TIR_work.leader) {
                    TIR_work.state = Data;
                }
            } else if (TIR_data.format == SONY) {
                /*
                 * SONY.
                 */
                static const int LEADER_SONY_HEAD = 4;
                static const int LEADER_SONY_TAIL = 0;
                if (TIR_work.leader < LEADER_SONY_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                TIR_work.leader++;
                if ((LEADER_SONY_HEAD + LEADER_SONY_TAIL) <= TIR_work.leader) {
                    TIR_work.state = Data;
                }
            } else {
            }
            break;
        case Data:
            if (TIR_data.format == NEC) {
                /*
                 * NEC.
                 */
                if (TIR_work.data == 0) {
                    IRDUTY50;
                    TIR_work.data++;
                } else {
                    IRDUTY0;
                    if (0 != (TIR_data.buffer[TIR_work.bitcount / 8] & (1 << TIR_work.bitcount % 8))) {
                        if (3 <= TIR_work.data) {
                            TIR_work.bitcount++;
                            TIR_work.data = 0;
                        } else {
                            TIR_work.data++;
                        }
                    } else {
                        if (1 <= TIR_work.data) {
                            TIR_work.bitcount++;
                            TIR_work.data = 0;
                        } else {
                            TIR_work.data++;
                        }
                    }
                }
                if (TIR_data.bitlength <= TIR_work.bitcount) {
                    TIR_work.state = Trailer;
                }
            } else if (TIR_data.format == AEHA) {
                /*
                 * AEHA.
                 */
                if (TIR_work.data == 0) {
                    IRDUTY50;
                    TIR_work.data++;
                } else {
                    IRDUTY0;
                    if (0 != (TIR_data.buffer[TIR_work.bitcount / 8] & (1 << TIR_work.bitcount % 8))) {
                        if (3 <= TIR_work.data) {
                            TIR_work.bitcount++;
                            TIR_work.data = 0;
                        } else {
                            TIR_work.data++;
                        }
                    } else {
                        if (1 <= TIR_work.data) {
                            TIR_work.bitcount++;
                            TIR_work.data = 0;
                        } else {
                            TIR_work.data++;
                        }
                    }
                }
                if (TIR_data.bitlength <= TIR_work.bitcount) {
                    TIR_work.state = Trailer;
                }
            } else if (TIR_data.format == SONY) {
                /*
                 * SONY.
                 */
                if (TIR_work.data == 0) {
                    IRDUTY0;
                    TIR_work.data++;
                } else {
                    IRDUTY50;
                    if (0 != (TIR_data.buffer[TIR_work.bitcount / 8] & (1 << TIR_work.bitcount % 8))) {
                        if (2 <= TIR_work.data) {
                            TIR_work.bitcount++;
                            TIR_work.data = 0;
                        } else {
                            TIR_work.data++;
                        }
                    } else {
                        if (1 <= TIR_work.data) {
                            TIR_work.bitcount++;
                            TIR_work.data = 0;
                        } else {
                            TIR_work.data++;
                        }
                    }
                }
                if (TIR_data.bitlength <= TIR_work.bitcount) {
                    TIR_work.state = Trailer;
                }
            } else {
            }
            break;
        case Trailer:
            if (TIR_data.format == NEC) {
                /*
                 * NEC.
                 */
                static const int TRAILER_NEC_HEAD = 1;
                static const int TRAILER_NEC_TAIL = 2;
                if (TIR_work.trailer < TRAILER_NEC_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                TIR_work.trailer++;
                if ((TRAILER_NEC_HEAD + TRAILER_NEC_TAIL) <= TIR_work.trailer) {
                    TIR_work.state = Idle;
                    //ticker.detach();
                    HAL_TIM_Base_Stop_IT (&htim9);//uss timer, 1779hz
                }
            } else if (TIR_data.format == AEHA) {
                /*
                 * AEHA.
                 */
                static const int TRAILER_AEHA_HEAD = 1;
                static const int TRAILER_AEHA_TAIL = 8000 / TUS_AEHA;
                if (TIR_work.trailer < TRAILER_AEHA_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                TIR_work.trailer++;
                if ((TRAILER_AEHA_HEAD + TRAILER_AEHA_TAIL) <= TIR_work.trailer) {
                    TIR_work.state = Idle;
                    //ticker.detach();
                }
            } else if (TIR_data.format == SONY) {
                /*
                 * SONY.
                 */
                static const int TRAILER_SONY_HEAD = 0;
                static const int TRAILER_SONY_TAIL = 0;
                if (TIR_work.trailer < TRAILER_SONY_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                TIR_work.trailer++;
                if ((TRAILER_SONY_HEAD + TRAILER_SONY_TAIL) <= TIR_work.trailer) {
                    TIR_work.state = Idle;
                    //ticker.detach();
                }
            } else {
            }
            break;
        default:
            break;
    }
    UNLOCK();
}
