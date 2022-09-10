/*
 * Created on: Sep 8, 2022
 *
 * @file  : oscillator.c
 * @brief : This file initializes the Oscillator and Clock required for LETIMER0 based on the required energy mode.
 *          EM3: LFA, ULFRCO
 *          EM0/EM1/EM2: LFA, LFXO
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */

#include "em_cmu.h"

#include "oscillators.h"

/*
 * @func: This function initializes Oscillator to either LFXO or ULFRCO based in the energy modes.
 */

void oscillator_init()
{
  /*CMU_Clock_TypeDef clock_select_check;
  CMU_ClkDiv_TypeDef clock_div_check;
  uint32_t freq_lfa_check, freq_letimer0_check;*/

  //Initializations
  CMU_OscillatorEnable(CMU_OSC,true, true);
  CMU_ClockSelectSet(CMU_CLOCK, CMU_SELECT);
  CMU_ClockDivSet(cmuClock_LETIMER0, CMU_PRESCALER );
  CMU_ClockEnable(cmuClock_LETIMER0, true);

 /* //Verfications
  clock_select_check = CMU_ClockSelectGet(cmuClock_LFA);
  clock_div_check = CMU_ClockDivGet(cmuClock_LFA);
  freq_lfa_check = CMU_ClockFreqGet(cmuClock_LFA);
  freq_letimer0_check = CMU_ClockFreqGet(cmuClock_LETIMER0);*/

}

