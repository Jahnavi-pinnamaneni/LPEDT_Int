/*
 * Created on: Sep 8, 2022
 *
 * @file  : irq.c
 * @brief : This file defines the IRQ Handler for the LETIMER0.
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 *
 */
#include "irq.h"
#include "em_letimer.h"
#include "gpio.h"
#include "em_i2c.h"
#include "log.h"

//#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

#define TIME_RESOLUTION 10
static long int count_flag = 0;

#define PB0_POSITION    0x40 // Interrupt Bit position for PB0 is at the 7th. 0b1000000
#define PB1_POSITION    0x80 // Interrupt Bit position for PB1 is at the 8th. 0b10000000

I2C_TransferReturn_TypeDef I2CTransferReturn;

/*
 * @func: This function describes the IRQ Handler for the LETIMER0.
 *        It calls the scheduler function that saves the event that has occurred
 */

void LETIMER0_IRQHandler(void)
{
  uint32_t interrupt_flag;
  interrupt_flag = LETIMER_IntGetEnabled (LETIMER0);
  LETIMER_IntClear(LETIMER0, interrupt_flag);

  if(interrupt_flag & _LETIMER_IEN_UF_MASK)
  {
      count_flag += 1;
      schedulerSetEventUF();
  }
  if(interrupt_flag & _LETIMER_IEN_COMP1_MASK)
  {
      schedulerSetEventCOMP1();
      LETIMER_IntDisable(LETIMER0,LETIMER_IEN_COMP1);
  }
}

/*
 * @func: This function returns the number of Milliseconds since the start of the system.
 */

long int letimerMilliseconds(void)
{
  return count_flag*TIME_RESOLUTION;
}

/*
 * @desc: This function Handles the I2C interrupt and sets the scheduler event
 */
void I2C0_IRQHandler(void)
{
  I2C_TransferReturn_TypeDef transferStatus;

  transferStatus = I2C_Transfer(I2C0);

  if(transferStatus == i2cTransferDone)
    {
      schedulerSetEventI2CComplete();
    }

  if(transferStatus < 0)
    {
      LOG_ERROR("I2C transfer status: %d", transferStatus);
    }
}

I2C_TransferReturn_TypeDef getI2CTransferReturn()
{
  return I2CTransferReturn;
}

/*
 * @desc: This function Handles the PB0 GPIO interrupt and sets the scheduler event
 */
void GPIO_EVEN_IRQHandler(void)
{
  uint32_t interrupt_flag;
  interrupt_flag = GPIO_IntGet();
  GPIO_IntClear(interrupt_flag);
  if(interrupt_flag & PB0_POSITION)
    {
      schedulerSetEventPB0();
    }
}

/*
 * @desc: This function Handles the PB1 GPIO interrupt and sets the scheduler event
 */
void GPIO_ODD_IRQHandler(void)
{
  uint32_t interrupt_flag;
  interrupt_flag = GPIO_IntGet();
  GPIO_IntClear(interrupt_flag);
  if(interrupt_flag & PB1_POSITION)
    {
      schedulerSetEventPB1();
    }

}
