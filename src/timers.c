/*
 * Created on: Sep 8, 2022
 *
 * @file  : timer.c
 * @brief : This file Initializes the LETIMER0 and sets the interrupts and COMP0 and COMP1 values.
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */

#include <src/timers.h>
#include "em_letimer.h"
#include <stdbool.h>

//#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"
#define MS_DIV 1000;
/*
 * @func: This function initializes the LETIMER0 by enabling the interrupt and loading the COMP0 and COMP1 registers
 */

void letimer0_init(void)
{
  //uint32_t temp_count;
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
    0                      // top Value, we calculate this below
  };
  LETIMER_Init (LETIMER0, &letimer);
  /*****************************************************************************************************/

  LETIMER_CompareSet(LETIMER0, 0, COMP0_VALUE);
  //LETIMER_CompareSet(LETIMER0, 1, COMP1_VALUE);


  LETIMER_IntClear (LETIMER0, 0xFFFFFFFF);
  LETIMER_IntEnable(LETIMER0,LETIMER_IF_UF);
  //LETIMER_IntEnable(LETIMER0,LETIMER_IF_COMP1);
  LETIMER_Enable(LETIMER0, true);


  /*temp_count = LETIMER_CounterGet(LETIMER0);
  temp_count = LETIMER_CounterGet(LETIMER0);
  temp_count = LETIMER_CounterGet(LETIMER0);*/

}

/*
 * @desc: This function provides a delay for the specified time
 *        The Oscillator used in the EM3 mode is running at 1KHz. This implies that the lowest possible delay that can be achieved
 *        is 1ms.
 *        Hence the range within which this function can provide the delay is 1ms to 3sec
 */
void timerWaitUs(uint32_t us_wait)
{
  uint32_t prev_cnt = 0;
  uint32_t curr_cnt = 0;
  uint32_t new_wait = 0;
  us_wait = us_wait/MS_DIV;
  if((us_wait == 0) || (us_wait > LETIMER_PERIOD_MS))
  {
      LOG_ERROR("Enter the delay time within the range 1ms to 3secs");
      return;
  }
  prev_cnt = LETIMER_CounterGet(LETIMER0);

  if(us_wait <= prev_cnt)
    new_wait = prev_cnt - us_wait;
  else
    new_wait = LETIMER_PERIOD_MS- us_wait - prev_cnt;

  curr_cnt = LETIMER_CounterGet(LETIMER0);
  while(curr_cnt != new_wait)
    {
      curr_cnt = LETIMER_CounterGet(LETIMER0);
    }

}



