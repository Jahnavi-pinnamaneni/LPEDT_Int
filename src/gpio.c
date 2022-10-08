/*
  gpio.c
 
   Created on: Dec 12, 2018
       Author: Dan Walkes
   Updated by Dave Sluiter Dec 31, 2020. Minor edits with #defines.

   March 17
   Dave Sluiter: Use this file to define functions that set up or control GPIOs.

 */


// *****************************************************************************
// Students:
// We will be creating additional functions that configure and manipulate GPIOs.
// For any new GPIO function you create, place that function in this file.
// *****************************************************************************

#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>

#include "gpio.h"


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define LED0_port  gpioPortF // change to correct ports and pins
#define LED0_pin   4
#define LED1_port  gpioPortF
#define LED1_pin   5
#define SENSOR_ENABLE_port 3
#define SENSOR_ENABLE_pin 15
#define EXTCOMIN_port gpioPortF
#define EXTCOMIN_pin 18


// Set GPIO drive strengths and modes of operation
void gpioInit()
{

  // Student Edit:

	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

  GPIO_DriveStrengthSet(SENSOR_ENABLE_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(SENSOR_ENABLE_port, SENSOR_ENABLE_pin, gpioModePushPull, false);

  GPIO_DriveStrengthSet(EXTCOMIN_port, gpioDriveStrengthWeakAlternateWeak);
  GPIO_PinModeSet(EXTCOMIN_port, EXTCOMIN_pin, gpioModePushPull, false);


} // gpioInit()


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}

void gpioSensor_enSetOn()
{
  GPIO_PinOutSet(SENSOR_ENABLE_port,SENSOR_ENABLE_pin);
}


void gpioSensor_enSetOff()
{
  GPIO_PinOutClear(SENSOR_ENABLE_port,SENSOR_ENABLE_pin);
}


void gpioSetDisplayExtcomin(bool val)
{
  if(val)
    GPIO_PinOutClear(EXTCOMIN_port,EXTCOMIN_pin);
  else
    GPIO_PinOutClear(EXTCOMIN_port,EXTCOMIN_pin);
}


