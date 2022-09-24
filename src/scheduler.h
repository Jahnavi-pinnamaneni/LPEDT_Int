/*
  * @file: scheduler.h
  * @desc: This file provides the required to handle all actions of the scheduler.
  * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "em_core.h"
#include <stdint.h>
#include "gpio.h"
#include "timers.h"
#include "i2c.h"
#include "sl_power_manager.h"

/* This enum will have values like 0, 2, 4, 8, 16, 32 */
typedef enum {
  evtNoEvent = 0x0u,
  evtTimerUF = 0x1u,
  evtTimerCOMP1 = 0x2u,
  evtI2CComplete = 0x4u
} scheduler_evt_t;

/*
 * @brief: This function sets the event flag pertaining to the UF interrupt that occurred
 * @params: none
 * @return: none
 */
void schedulerSetEventUF(void);

/*
 * @brief: This function sets the event flag pertaining to the COMP1 interrupt that occurred
 * @params: none
 * @return: none
 */
void schedulerSetEventCOMP1(void);

/*
 * @brief: This function sets the event flag pertaining to the I2C Transfer Complete interrupt that occurred
 * @params: none
 * @return: none
 */
void schedulerSetEventI2CComplete(void);

/*
 * @brief: This function returns the event that needs to be handled.
 * @params: none
 * @return: returns the event that has occurred
 */
uint32_t schedulerGetEvent(void);

/*
 * @brief: This function implements a state machine to read temperature from Si7021 in a non-blocking method
 * @params: the most recent event
 * @return: none
 */
void temperature_state_machine(scheduler_evt_t evt);

#endif /* SRC_SCHEDULER_H_ */
