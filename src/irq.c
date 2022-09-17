/*
 * Created on: Sep 8, 2022
 *
 * @file  : irq.c
 * @brief : This file defines the IRQ Handler for the LETIMER0.
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */
#include "em_letimer.h"
#include "gpio.h"
#include "scheduler.h"

/*
 * @func: This function describes the IRQ Handler for the LETIMER0.
 *        It calls the scheduler function that saves the event that has occured
 */

void LETIMER0_IRQHandler(void)
{
  uint32_t interrupt_flag;
  interrupt_flag = LETIMER_IntGetEnabled (LETIMER0);
  LETIMER_IntClear(LETIMER0, interrupt_flag);

  if(interrupt_flag & _LETIMER_IEN_UF_MASK)
  {
      LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
      schedulerSetEventUF();

  }
}

