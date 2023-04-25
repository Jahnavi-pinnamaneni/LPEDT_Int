/***************************************************************************//**
 * @file
 * @brief Core application logic.
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
 * Date:        02-25-2022
 * Author:      Dave Sluiter
 * Description: This code was created by the Silicon Labs application wizard
 *              and started as "Bluetooth - SoC Empty".
 *              It is to be used only for ECEN 5823 "IoT Embedded Firmware".
 *              The MSLA referenced above is in effect.
 *
 ******************************************************************************/
#include <src/timers.h>
#include "em_common.h"
#include "app_assert.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"


// *************************************************
// Students: It is OK to modify this file.
//           Make edits appropriate for each
//           assignment.
// *************************************************

#include "sl_status.h"             // for sl_status_print()

#include "src/ble_device_type.h"
#include "src/gpio.h"
#include "src/lcd.h"
#include "src/oscillators.h"
#include "src/scheduler.h"
#include "src/i2c.h"
#include "src/ble.h"
#include "src/SparkFun_MAX30101.h"
#include "src/spo2_algo.h"

// Students: Here is an example of how to correctly include logging functions in
//           each .c file.
//           Apply this technique to your other .c files.
//           Do not #include "src/log.h" in any .h file! This logging scheme is
//           designed to be included at the top of each .c file that you want
//           to call one of the LOG_***() functions from.

// Include logging specifically for this .c file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

void HR_init()
{
  // Init steps for MAX
   LOG_INFO("MAX30105_begin\r");
   MAX30105_begin();

   LOG_INFO("MAX30105_setup\r");
   // uint8_t powerLevel = 0x1F, uint8_t sampleAverage = 4, uint8_t ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096
   MAX30105_setup(0x1F, 4, 2, 400, 411, 4096);
 //  MAX30105_init();

   LOG_INFO("MAX30105_setPulseAmplitudes\r");
   MAX30105_setPulseAmplitudeRed(0x0A);
   MAX30105_setPulseAmplitudeGreen(0);
}

// *************************************************
// Power Manager
// *************************************************

// See: https://docs.silabs.com/gecko-platform/latest/service/power_manager/overview
#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

// -----------------------------------------------------------------------------
// defines for power manager callbacks
// -----------------------------------------------------------------------------
// Return values for app_is_ok_to_sleep():
//   Return false to keep sl_power_manager_sleep() from sleeping the MCU.
//   Return true to allow system to sleep when you expect/want an IRQ to wake
//   up the MCU from the call to sl_power_manager_sleep() in the main while (1)
//   loop.
//
// Students: We'll need to modify this for A2 onward so that compile time we
//           control what the lowest EM (energy mode) the MCU sleeps to. So
//           think "#if (expression)".
#if (LOWEST_ENERGY_MODE == 0)
  #define APP_IS_OK_TO_SLEEP      (false)

#elif (LOWEST_ENERGY_MODE == 1 || LOWEST_ENERGY_MODE == 2 || LOWEST_ENERGY_MODE == 3)
  #define APP_IS_OK_TO_SLEEP      (true)

#endif




// Return values for app_sleep_on_isr_exit():
//   SL_POWER_MANAGER_IGNORE; // The module did not trigger an ISR and it doesn't want to contribute to the decision
//   SL_POWER_MANAGER_SLEEP;  // The module was the one that caused the system wakeup and the system SHOULD go back to sleep
//   SL_POWER_MANAGER_WAKEUP; // The module was the one that caused the system wakeup and the system MUST NOT go back to sleep
//
// Notes:
//       SL_POWER_MANAGER_IGNORE, we see calls to app_process_action() on each IRQ. This is the
//       expected "normal" behavior.
//
//       SL_POWER_MANAGER_SLEEP, the function app_process_action()
//       in the main while(1) loop will not be called! It would seem that sl_power_manager_sleep()
//       does not return in this case.
//
//       SL_POWER_MANAGER_WAKEUP, doesn't seem to allow ISRs to run. Main while loop is
//       running continuously, flooding the VCOM port with printf text with LETIMER0 IRQs
//       disabled somehow, LED0 is not flashing.

#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_IGNORE)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_SLEEP)
//#define APP_SLEEP_ON_ISR_EXIT   (SL_POWER_MANAGER_WAKEUP)

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)




// *************************************************
// Power Manager Callbacks
// The values returned by these 2 functions AND
// adding and removing power manage requirements is
// how we control when EM mode the MCU goes to when
// sl_power_manager_sleep() is called in the main
// while (1) loop.
// *************************************************

#if defined(SL_CATALOG_POWER_MANAGER_PRESENT)

bool app_is_ok_to_sleep(void)
{
  return APP_IS_OK_TO_SLEEP;
} // app_is_ok_to_sleep()

sl_power_manager_on_isr_exit_t app_sleep_on_isr_exit(void)
{
  return APP_SLEEP_ON_ISR_EXIT;
} // app_sleep_on_isr_exit()

#endif // defined(SL_CATALOG_POWER_MANAGER_PRESENT)




/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
SL_WEAK void app_init(void)
{
  // Put your application 1-time initialization code here.
  // This is called once during start-up.
  // Don't call any Bluetooth API functions until after the boot event.

  // Student Edit: Add a call to gpioInit() here
  //gpioInit();

  //Assignment 3
  gpioInit();
  oscillator_init();
  //HR_init();
  SPO2_init();
  letimer0_init();

  NVIC_ClearPendingIRQ(LETIMER0_IRQn);
  //NVIC_ClearPendingIRQ(I2C0_IRQn );

  NVIC_EnableIRQ(LETIMER0_IRQn);
  //NVIC_EnableIRQ(I2C0_IRQn);

#if (LOWEST_ENERGY_MODE == 1)
    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
#elif (LOWEST_ENERGY_MODE == 2)
    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM2);
#endif

} // app_init()




/*****************************************************************************
 * delayApprox(), private to this file.
 * A value of 3500000 is ~ 1 second. After assignment 1 you can delete or
 * comment out this function. Wait loops are a bad idea in general.
 * We'll discuss how to do this a better way in the next assignment.
 *****************************************************************************/
/*static void delayApprox(int delay)
{
  volatile int i;

  for (i = 0; i < delay; ) {
      i=i+1;
  }

}*/ // delayApprox()





/**************************************************************************//**
 * Application Process Action.
 *****************************************************************************/
SL_WEAK void app_process_action(void)
{
  // Put your application code here for A1 to A4.
  // This is called repeatedly from the main while(1) loop
  // Notice: This function is not passed or has access to Bluetooth stack events.
  //         We will create/use a scheme that is far more energy efficient in
  //         later assignments.
//  uint32_t evt;
//
//  evt = schedulerGetEvent();
//  temperature_state_machine(evt);

 // SPO2_measure();
} // app_process_action()





/**************************************************************************//**
 * Bluetooth stack event handler.
 * This overrides the dummy weak implementation.
 *
 * @param[in] evt Event coming from the Bluetooth stack.
 *
 * The code here will process events from the Bluetooth stack. This is the only
 * opportunity we will get to act on an event.
 *
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt)
{

  // Just a trick to hide a compiler warning about unused input parameter evt.
  //(void) evt;

  // For A5 onward:
  // Some events require responses from our application code,
  // and don’t necessarily advance our state machines.
  // For A5 uncomment the next 2 function calls
   handle_ble_event(evt); // put this code in ble.c/.h

  // sequence through states driven by events
  // temperature_state_machine(evt);    // put this code in scheduler.c/.h

   pulse_state_machine(evt);
} // sl_bt_on_event()

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

uint8_t pulseLED = 11; //Must be on PWM pin
uint8_t readLED = 13; //Blinks with each data read

void SPO2_measure()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (uint8_t i = 0 ; i < bufferLength ; i++)
  {
    while (MAX30105_available() == false) //do we have new data?
      check(); //Check the sensor for new data

    redBuffer[i] = MAX30105_getRed();
    irBuffer[i] = MAX30105_getIR();
    MAX30105_nextSample(); //We're finished with this sample so move to next sample

    LOG_INFO("red = %d\r", redBuffer[i]);

    LOG_INFO("ir = %d\r", irBuffer[i]);

  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (uint8_t i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (uint8_t i = 75; i < 100; i++)
    {
      while (MAX30105_available() == false) //do we have new data?
        check(); //Check the sensor for new data

//      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = MAX30105_getRed();
      irBuffer[i] = MAX30105_getIR();
      MAX30105_nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      LOG_INFO("red = %d\r", redBuffer[i]);

      LOG_INFO("ir = %d\r", irBuffer[i]);

      LOG_INFO("HR = %d\r", heartRate);

      LOG_INFO("HRvalid = %d\r", validHeartRate);

      LOG_INFO("SPO2 = %d\r", spo2);

      LOG_INFO("SPO2Valid = %d\r", validSPO2);

    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}

void SPO2_init()
{
//    Serial.begin(115200); // initialize serial communication at 115200 bits per second:
//
//   pinMode(pulseLED, OUTPUT);
//   pinMode(readLED, OUTPUT);

   // Initialize sensor
//   if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
//   {
//     Serial.println(F("MAX30105 was not found. Please check wiring/power."));
//     while (1);
//   }
  LOG_INFO("MAX30105_begin\r");
  MAX30105_begin();
//   Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
//   while (Serial.available() == 0) ; //wait until user presses a key
//   Serial.read();

   uint8_t ledBrightness = 60; //Options: 0=Off to 255=50mA
   uint8_t sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
   uint8_t ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
   uint8_t sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
   int pulseWidth = 411; //Options: 69, 118, 215, 411
   int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
   LOG_INFO("MAX30105_setup\r");
   // uint8_t powerLevel = 0x1F, uint8_t sampleAverage = 4, uint8_t ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096
//   MAX30105_setup(0x1F, 4, 2, 400, 411, 4096);
   MAX30105_setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}
