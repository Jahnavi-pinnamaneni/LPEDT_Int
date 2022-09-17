/*
 * Created on: Sep 8, 2022
 *
 * @file  : timer.h
 * @brief : This file Initializes the LETIMER0 and sets the interrupts and COMP0 and COMP1 values.
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include <stdint.h>
#include "app.h"

/*
 * @desc: This function initializes the LETIMER0 by enabling the interrupt and loading the COMP0 and COMP1 registers
 * @params: none
 * @return: none
 */
void letimer0_init(void);

/*
 * @desc: This function provides a delay for the specified time
 *        The Oscillator used in the EM3 mode is running at 1KHz. This implies that the lowest possible delay that can be achieved
 *        is 1ms.
 *        Hence the range within which this function can provide the delay is 1ms to 3sec
 * @params: the required time delay in microseconds
 * @return: none
 */
void timerWaitUs(uint32_t us_wait);

#endif /* SRC_TIMERS_H_ */
