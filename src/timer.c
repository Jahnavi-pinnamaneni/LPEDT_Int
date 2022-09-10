/*
 * Created on: Sep 8, 2022
 *
 * @file  : timer.c
 * @brief : This file Initializes the LETIMER0 and sets the interrupts and COMP0 and COMP1 values.
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */

#include "timer.h"
#include "em_letimer.h"
#include <stdbool.h>

/*
 * @func: This function initializes the LETIMER0 by enabling the interrupt and loading the COMP0 and COMP1 registers
 */

void letimer0_init()
{
  uint32_t temp_count;
  /******************Borrowed from Prof. David Sluiter during debug session*****************************/
  const LETIMER_Init_TypeDef letimer = {
    false,                 // enable;   don't enable when init completes, we'll enable last
    true,                  // debugRun; useful to have the timer running when stopped in the debugger
    true,                  // comp0Top; load COMP0 into CNT on underflow
    false,                 // bufTop;   don't load COMP1 into COMP0 when REP0==0
    0,                     // out0Pol;  0 default output pin value
    0,                     // out1Pol;  0 default output pin value
    letimerUFOANone,       // ufoa0;    no underflow output action
    letimerUFOANone,       // ufoa1;    no underflow output action
    letimerRepeatFree,     // repMode;  free running mode i.e. load & go forever
    0                      // top Value, we calc this below
  };
  LETIMER_Init (LETIMER0, &letimer);
  /*****************************************************************************************************/

  LETIMER_CompareSet(LETIMER0, 0, COMP0_VALUE);
  LETIMER_CompareSet(LETIMER0, 1, COMP1_VALUE);


  LETIMER_IntClear (LETIMER0, 0xFFFFFFFF);
  LETIMER_IntEnable(LETIMER0,LETIMER_IF_COMP0);
  LETIMER_IntEnable(LETIMER0,LETIMER_IF_COMP1);
  LETIMER_Enable(LETIMER0, true);


  /*temp_count = LETIMER_CounterGet(LETIMER0);
  temp_count = LETIMER_CounterGet(LETIMER0);
  temp_count = LETIMER_CounterGet(LETIMER0);*/

}





