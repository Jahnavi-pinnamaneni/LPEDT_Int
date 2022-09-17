/*
  * @file: scheduler.h
  * @desc: This file provides the required to handle all actions of the scheduler.
  * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include "em_core.h"
#include <stdint.h>

/* This enum will have values like 0, 2, 4, 8, 16, 32 */
typedef enum {
  evtNoEvent = 0x0u,
  evtTimerUF = 0x1u
} scheduler_evt_t;

/*
 * @brief: This function sets the event flag pertaining to the interrupt that occurred
 * @params: none
 * @return: none
 */
void schedulerSetEventUF(void);

/*
 * @brief: This function returns the event that needs to be handled.
 * @params: none
 * @return: returns the event that has occured
 */
uint32_t schedulerGetEvent(void);

#endif /* SRC_SCHEDULER_H_ */
