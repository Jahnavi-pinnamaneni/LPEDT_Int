/*
 * Created on: Sep 8, 2022
 *
 * @file  : irq.h
 * @brief : This file defines the IRQ Handler for the LETIMER0.
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_


#include "gpio.h"
#include "scheduler.h"
#include "stdbool.h"

void LETIMER0_IRQHandler(void);
long int letimerMilliseconds(void);
void I2C0_IRQHandler(void);

#endif /* SRC_IRQ_H_ */
