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

void LETIMER0_IRQHandler(void)
{
  uint32_t interrupt_flag;
  interrupt_flag = LETIMER_IntGetEnabled (LETIMER0);

  if(interrupt_flag & _LETIMER_IEN_COMP0_MASK)
    {
      //Disable interrupts
      LETIMER_IntDisable(LETIMER0, LETIMER_IF_COMP0);
      LETIMER_IntDisable(LETIMER0, LETIMER_IF_COMP1);

      //Handler logic
      LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP0);
      gpioLed0SetOff();

      //Enable interrupts
      LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);
      LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);
    }
  if(interrupt_flag & _LETIMER_IEN_COMP1_MASK)
    {
      //Disable Interrupts
      LETIMER_IntDisable(LETIMER0, LETIMER_IF_COMP0);
      LETIMER_IntDisable(LETIMER0, LETIMER_IF_COMP1);

      //Handler logic
      LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
      gpioLed0SetOn();

      //Enable Interrupts
      LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP0);
      LETIMER_IntEnable(LETIMER0, LETIMER_IF_COMP1);
    }
}

