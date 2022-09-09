/*
 * timer.h
 *
 *  Created on: Sep 8, 2022
 *      Author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#ifndef SRC_TIMER_H_
#define SRC_TIMER_H_

#include <stdint.h>
#include "app.h"

#define LFXO_FREQ 32768
#define LFXO_DIV 2000


void letimer0_init();

#endif /* SRC_TIMER_H_ */
