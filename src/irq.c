/*
 * irq.c
 *
 *  Created on: Sep 8, 2022
 *      Author: pjahn
 */
#include "em_letimer.h"
#include "gpio.h"

void LETIMER0_IRQHandler(void)
{ //disable interrupts
  uint32_t interrupt_flag;
  int comp0_flag = false;
  int comp1_flag = false;
  interrupt_flag = LETIMER_IntGetEnabled (LETIMER0);
  if(interrupt_flag & _LETIMER_IEN_COMP0_MASK)
    {
      LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);
      comp0_flag = true;
      gpioLed0SetOff();
    }
  if(interrupt_flag & _LETIMER_IEN_COMP1_MASK)
    {
      LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
      comp1_flag = true;
      gpioLed0SetOn();
    }
}

