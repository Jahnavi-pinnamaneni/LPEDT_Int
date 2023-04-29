/***************************************************************************//**
 * @file
 * @brief Application interface provided to main().
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 *
 * Editor: Feb 26, 2022, Dave Sluiter
 * Change: Added comment about use of .h files.
 *
 *
 *
 ******************************************************************************/

// Students: Remember, a header file (a .h file) generally defines an interface
//           for functions defined within an implementation file (a .c file).
//           The .h file defines what a caller (a user) of a .c file requires.
//           At a minimum, the .h file should define the publicly callable
//           functions, i.e. define the function prototypes. #define and type
//           definitions can be added if the caller requires theses.

#ifndef APP_H
#define APP_H

/**********Assignment 2****************************************************/
//?? Settings should support 7seconds period and 1 second on time
#include "em_cmu.h"

/****************System Settings*************************/
#define LOWEST_ENERGY_MODE 2
#define LETIMER_ON_TIME_MS 1000
#define LETIMER_PERIOD_MS 10
/********************************************************/

/****************System Configuration********************/
#if (LOWEST_ENERGY_MODE == 3)
  #define CMU_OSC cmuOsc_ULFRCO
  #define CMU_CLOCK cmuClock_LFA
  #define CMU_SELECT cmuSelect_ULFRCO
  #define CMU_PRESCALER cmuClkDiv_1

  #define COMP0_VALUE (LETIMER_PERIOD_MS)
  #define COMP1_VALUE (LETIMER_ON_TIME_MS)

#elif (LOWEST_ENERGY_MODE == 0 || LOWEST_ENERGY_MODE == 1 || LOWEST_ENERGY_MODE == 2)
  #define CMU_OSC cmuOsc_LFXO
  #define CMU_CLOCK cmuClock_LFA
  #define CMU_SELECT cmuSelect_LFXO
  #define CMU_PRESCALER cmuClkDiv_8

  #define LFXO_FREQ 32768
  #define LFXO_DIV (CMU_PRESCALER * 1000)
  #define COMP0_VALUE ((LETIMER_PERIOD_MS *LFXO_FREQ)/LFXO_DIV)
  #define COMP1_VALUE ((LETIMER_ON_TIME_MS *LFXO_FREQ)/LFXO_DIV)

#endif
/*************************************************************/

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void app_init(void);


/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
void app_process_action(void);

#endif // APP_H
