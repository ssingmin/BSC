#include <stdint.h>
#include "TransmitterIR.h"

#define LOCK()
#define UNLOCK()

extern TIM_HandleTypeDef htim9;

data_t data;
work_t work;


/**
 * Get state.
 *
 * @return Current state.
 */

void TransmitterIR_init()
{




	IRDUTY0;
	//IRDUTY50;

	work.state = Idle;
	work.bitcount = 0;
	work.leader = 0;
	work.data = 0;
	work.trailer = 0;

	data.format = UNKNOWN;
	data.bitlength = 0;




}

State getState(void) {
    LOCK();
    State s = work.state;
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
    if (work.state != Idle) {
        UNLOCK();
        return -1;
    }

    work.state = Leader;
    work.bitcount = 0;
    work.leader = 0;
    work.data = 0;
    work.trailer = 0;

    data.format = format;
    data.bitlength = bitlength;
    const int n = bitlength / 8 + (((bitlength % 8) != 0) ? 1 : 0);
    for (int i = 0; i < n; i++) {
        data.buffer[i] = buf[i];
    }

    switch (format) {
        case NEC:
//            ticker.detach();
//            ticker.attach_us(callback(this, &tick), TUS_NEC);
        		HAL_TIM_Base_Start_IT (&htim9);//uss timer, 1779hz


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
    switch (work.state) {
        case Idle:
            work.bitcount = 0;
            work.leader = 0;
            work.data = 0;
            work.trailer = 0;
            break;
        case Leader:
            if (data.format == NEC) {
                /*
                 * NEC.
                 */
                static const int LEADER_NEC_HEAD = 16;
                static const int LEADER_NEC_TAIL = 8;
                if (work.leader < LEADER_NEC_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                work.leader++;
                if ((LEADER_NEC_HEAD + LEADER_NEC_TAIL) <= work.leader) {
                    work.state = Data;
                }
            } else if (data.format == AEHA) {
                /*
                 * AEHA.
                 */
                static const int LEADER_AEHA_HEAD = 8;
                static const int LEADER_AEHA_TAIL = 4;
                if (work.leader < LEADER_AEHA_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                work.leader++;
                if ((LEADER_AEHA_HEAD + LEADER_AEHA_TAIL) <= work.leader) {
                    work.state = Data;
                }
            } else if (data.format == SONY) {
                /*
                 * SONY.
                 */
                static const int LEADER_SONY_HEAD = 4;
                static const int LEADER_SONY_TAIL = 0;
                if (work.leader < LEADER_SONY_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                work.leader++;
                if ((LEADER_SONY_HEAD + LEADER_SONY_TAIL) <= work.leader) {
                    work.state = Data;
                }
            } else {
            }
            break;
        case Data:
            if (data.format == NEC) {
                /*
                 * NEC.
                 */
                if (work.data == 0) {
                    IRDUTY50;
                    work.data++;
                } else {
                    IRDUTY0;
                    if (0 != (data.buffer[work.bitcount / 8] & (1 << work.bitcount % 8))) {
                        if (3 <= work.data) {
                            work.bitcount++;
                            work.data = 0;
                        } else {
                            work.data++;
                        }
                    } else {
                        if (1 <= work.data) {
                            work.bitcount++;
                            work.data = 0;
                        } else {
                            work.data++;
                        }
                    }
                }
                if (data.bitlength <= work.bitcount) {
                    work.state = Trailer;
                }
            } else if (data.format == AEHA) {
                /*
                 * AEHA.
                 */
                if (work.data == 0) {
                    IRDUTY50;
                    work.data++;
                } else {
                    IRDUTY0;
                    if (0 != (data.buffer[work.bitcount / 8] & (1 << work.bitcount % 8))) {
                        if (3 <= work.data) {
                            work.bitcount++;
                            work.data = 0;
                        } else {
                            work.data++;
                        }
                    } else {
                        if (1 <= work.data) {
                            work.bitcount++;
                            work.data = 0;
                        } else {
                            work.data++;
                        }
                    }
                }
                if (data.bitlength <= work.bitcount) {
                    work.state = Trailer;
                }
            } else if (data.format == SONY) {
                /*
                 * SONY.
                 */
                if (work.data == 0) {
                    IRDUTY0;
                    work.data++;
                } else {
                    IRDUTY50;
                    if (0 != (data.buffer[work.bitcount / 8] & (1 << work.bitcount % 8))) {
                        if (2 <= work.data) {
                            work.bitcount++;
                            work.data = 0;
                        } else {
                            work.data++;
                        }
                    } else {
                        if (1 <= work.data) {
                            work.bitcount++;
                            work.data = 0;
                        } else {
                            work.data++;
                        }
                    }
                }
                if (data.bitlength <= work.bitcount) {
                    work.state = Trailer;
                }
            } else {
            }
            break;
        case Trailer:
            if (data.format == NEC) {
                /*
                 * NEC.
                 */
                static const int TRAILER_NEC_HEAD = 1;
                static const int TRAILER_NEC_TAIL = 2;
                if (work.trailer < TRAILER_NEC_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                work.trailer++;
                if ((TRAILER_NEC_HEAD + TRAILER_NEC_TAIL) <= work.trailer) {
                    work.state = Idle;
                    //ticker.detach();
                    HAL_TIM_Base_Stop_IT (&htim9);//uss timer, 1779hz
                }
            } else if (data.format == AEHA) {
                /*
                 * AEHA.
                 */
                static const int TRAILER_AEHA_HEAD = 1;
                static const int TRAILER_AEHA_TAIL = 8000 / TUS_AEHA;
                if (work.trailer < TRAILER_AEHA_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                work.trailer++;
                if ((TRAILER_AEHA_HEAD + TRAILER_AEHA_TAIL) <= work.trailer) {
                    work.state = Idle;
                    //ticker.detach();
                }
            } else if (data.format == SONY) {
                /*
                 * SONY.
                 */
                static const int TRAILER_SONY_HEAD = 0;
                static const int TRAILER_SONY_TAIL = 0;
                if (work.trailer < TRAILER_SONY_HEAD) {
                    IRDUTY50;
                } else {
                    IRDUTY0;
                }
                work.trailer++;
                if ((TRAILER_SONY_HEAD + TRAILER_SONY_TAIL) <= work.trailer) {
                    work.state = Idle;
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
