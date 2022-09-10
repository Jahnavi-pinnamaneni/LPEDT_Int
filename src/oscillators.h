/*
 * Created on: Sep 8, 2022
 *
 * @file  : oscillator.h
 * @brief : This file initializes the Oscillator and Clock required for LETIMER0 based on the required energy mode.
 *          EM3: LFA, ULFRCO
 *          EM0/EM1/EM2: LFA, LFXO
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */

#ifndef SRC_OSCILLATORS_H_
#define SRC_OSCILLATORS_H_

#include "app.h"


void oscillator_init();


#endif /* SRC_OSCILLATORS_H_ */
