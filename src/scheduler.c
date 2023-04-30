/*
  * @file: scheduler.c
  * @desc: This file provides the required to handle all actions of the scheduler.
  * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#include "scheduler.h"

#include "src/gpio.h"


#define INCLUDE_LOG_DEBUG 1
#include "log.h"

#include "SparkFun_MAX30101.h"
#include "heartRate.h"
#include "spo2_algo.h"

#define CMD_DATA_TO_READ 0xF3
#define MULTIPLIER 175.72
#define MAX_RES 65536
#define CONSTANT 46.85

uint32_t evt_flag = 0;

#define SENSOR_ENABLE_TIME 80000
#define TEMP_MEAS_TIME (125*1000)


uint8_t PB0_flag = false;
/******************************************************************************/
/*
 * Pulse sensor variables
 */
//uint32_t irBuffer[100]; //infrared LED sensor data
//uint32_t redBuffer[100];  //red LED sensor data
//int32_t bufferLength; //data length
//int32_t spo2; //SPO2 value
//int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
//int32_t heartRate; //heart rate value
//int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
//
//#define NO_OF_SAMPLES 275
//int32_t hr_buffer[NO_OF_SAMPLES];
//int32_t spo2_buffer[NO_OF_SAMPLES];
//uint8_t pulseLED = 11; //Must be on PWM pin
//uint8_t readLED = 13; //Blinks with each data read



typedef enum states{
  config_shutdown,
  wait_shutdown,
  start_conversion,
  wait_one_shot,
  i2c_read_tmp,
  display_temp
}state_t;

extern uint16_t read_data;
/*
 * @brief: This function sets the event flag pertaining to the UF interrupt that occurred
 */
void schedulerSetEventUF(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

 // evt_flag |= evtTimerUF;
  sl_bt_external_signal(evtTimerUF);

  CORE_EXIT_CRITICAL();
}

/*
 * @brief: This function sets the event flag pertaining to the COMP1 interrupt that occurred
 */
void schedulerSetEventCOMP1(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  //evt_flag |= evtTimerCOMP1;
  sl_bt_external_signal(evtTimerCOMP1);

  CORE_EXIT_CRITICAL();
}

/*
 * @brief: This function sets the event flag pertaining to the I2C Complete interrupt that occurred
 */
void schedulerSetEventI2CComplete(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();

  //evt_flag |= evtI2CComplete;
  sl_bt_external_signal(evtI2CComplete);

  CORE_EXIT_CRITICAL();
}

/*
 * @brief: This function sets the event flag pertaining to the PB0 GPIO interrupt
 */
void schedulerSetEventPB0(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  ble_data_struct_t *ble_data_ptr;
  ble_data_ptr = return_ble_data_struct();

  ble_data_ptr->PB0_state = gpioReadPB0();

  sl_bt_external_signal(evtPB0);

  CORE_EXIT_CRITICAL();
}

/*
 * @brief: This function sets the event flag pertaining to the PB1 GPIO interrupt
 */
void schedulerSetEventPB1(void)
{
  CORE_DECLARE_IRQ_STATE;
  CORE_ENTER_CRITICAL();
  ble_data_struct_t *ble_data_ptr;
  ble_data_ptr = return_ble_data_struct();

  ble_data_ptr->PB1_state = gpioReadPB1();

  sl_bt_external_signal(evtPB1);

  CORE_EXIT_CRITICAL();
}

/*
 * @brief: This function returns the event that needs to be handled.
 */
uint32_t schedulerGetEvent(void)
{
  uint32_t next_evt = 1;
  uint32_t temp = evt_flag;
  int count = 1;
  if(temp != 0)
    {
      while(temp != 0)
        {
          temp = temp >> 1;
          count++;
        }
      next_evt = next_evt << (count-2);
      evt_flag &= ~(next_evt);
    }
  else
    next_evt = evtNoEvent;

  return next_evt;
}


/*
 * This state machine handles all the events related to temperature measurement
 */
void temperature_state_machine(sl_bt_msg_t *event)
{
  static state_t current_state = config_shutdown;
  static state_t next_state = config_shutdown;
  uint16_t temp_data = 0, read = 0;
  uint32_t temp_in_c = 0;

  sl_status_t sc = 0;
  ble_data_struct_t *ble_data_ptr;
  ble_data_ptr = return_ble_data_struct();

  uint8_t htm_temperature_buffer[5];
  uint8_t *p = htm_temperature_buffer;
  uint32_t htm_temperature_flt;

  uint8_t flags = 0x00;
  scheduler_evt_t evt = 0;

  I2C_TransferReturn_TypeDef I2CTransferReturnStatus;

  // This state machine acts only on the events under the external signal header
  // Any other events are ignored
  if(SL_BT_MSG_ID(event->header) == sl_bt_evt_system_external_signal_id)
    {
      evt = event->data.evt_system_external_signal.extsignals;

      //The temperature measurement must be performed only if the indication is
      //enabled and the connection is open
      if(ble_data_ptr->indication_flag && ble_data_ptr->connection_status)
        {
          //DOS LOG_INFO("\n");
          switch(current_state)
                  {
                    case config_shutdown:
                              sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                              setShutdownModeTMP117();
                              next_state = wait_shutdown;
                              LOG_INFO("To1");
                              break;
                    case wait_shutdown:
                               next_state = wait_shutdown;
                               if(evt == evtI2CComplete)
                                 {
                                   /* Disable IRQ on successful transfer */
                                   I2CTransferReturnStatus = getI2CTransferReturn();
                                   if(I2CTransferReturnStatus == i2cTransferDone)
                                     NVIC_DisableIRQ(I2C0_IRQn);
                                   next_state = start_conversion;
                                   LOG_INFO("To2");
                                   sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                                 }
                               break;
                    case start_conversion:
                              next_state = start_conversion;
                              if(evt == evtTimerUF)
                                {
                                  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                                  setOneShotModeTMP117();
                                  next_state = wait_one_shot;
                                  LOG_INFO("To3");
                                  //DOS gpioLed0SetOn();
                                }
                              break;
                    case wait_one_shot:
                              next_state = wait_one_shot;
                              if(evt == evtI2CComplete)
                                {
                                  /* Disable IRQ on successful transfer */
                                  I2CTransferReturnStatus = getI2CTransferReturn();
                                  if(I2CTransferReturnStatus == i2cTransferDone)
                                    NVIC_DisableIRQ(I2C0_IRQn);

                                  next_state = i2c_read_tmp;
                                  LOG_INFO("To4");
                                  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                                  timerWaitUs_irq(TEMP_MEAS_TIME);
                                }
                              break;
                    case i2c_read_tmp:
                              next_state = i2c_read_tmp;
                              if(evt == evtTimerCOMP1)
                                {
                                  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                                  getTemperatureTMP117();

                                  next_state = display_temp;
                                  LOG_INFO("To5");
                                  //DOS gpioLed0SetOff();
                                }
                              break;
                    case display_temp:
                              next_state = display_temp;
                              if(evt == evtI2CComplete)
                                {
                                  /* Disable IRQ on successful transfer */
                                  I2CTransferReturnStatus = getI2CTransferReturn();
                                  if(I2CTransferReturnStatus == i2cTransferDone)
                                    NVIC_DisableIRQ(I2C0_IRQn);

                                  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                                  temp_data = reportTemperatureTMP117();

                                  //temperature data is converted to a format that can be used
                                  //to transmit over the BLE Stack
                                  temp_in_c = (uint32_t)temp_data;
                                  htm_temperature_flt = UINT32_TO_FLOAT(temp_data*1000, -3);
                                  UINT8_TO_BITSTREAM(p, flags);
                                  UINT32_TO_BITSTREAM(p, htm_temperature_flt);

                                  //Update the temperature value to the gatt database
                                  sc = sl_bt_gatt_server_write_attribute_value(
                                    gattdb_temperature_measurement, // handle from gatt_db.h
                                    0, // offset
                                    4, // length
                                    (uint8_t *)&temp_in_c // pointer to buffer where data is
                                  );
                                  if (sc != SL_STATUS_OK) {
                                      LOG_ERROR("Updating GATT DB unsuccessful");
                                  }

                                  if(!ble_data_ptr->indication_in_flight){
                                    sc = sl_bt_gatt_server_send_indication(
                                                                        ble_data_ptr->connection_handle,
                                                                        gattdb_temperature_measurement, // handle from gatt_db.h
                                                                        sizeof(htm_temperature_buffer),
                                                                        &htm_temperature_buffer[0] // in IEEE-11073 format
                                                                      );
                                    if (sc != SL_STATUS_OK) {
                                        LOG_ERROR("Indication not successful");
                                    }
                                    else {
                                        ble_data_ptr->indication_in_flight = true;
                                    }
                                    displayPrintf(DISPLAY_ROW_TEMPVALUE , "Temp=%d", temp_data);
                                }


                                  next_state = start_conversion;
                                  LOG_INFO("To2");
                                }
                              break;
                  }
          current_state = next_state;
        }
      else
        {
          //If the indication and connection status are false then the state
          //machine reverts to idle state
          if(current_state != config_shutdown)
            {
              current_state = config_shutdown;
              //gpioSensor_enSetOff();
            }
          displayPrintf(DISPLAY_ROW_TEMPVALUE , " ");
        }
    }

}

//void BPM_state_machine(sl_bt_msg_t *event)
//{
//  scheduler_evt_t evt = 0;
//  static int i = 0;
//  if(SL_BT_MSG_ID(event->header) == sl_bt_evt_system_external_signal_id)
//  {
//    evt = event->data.evt_system_external_signal.extsignals;
//
//    if(evt == evtTimerUF)
//    {
////      sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
////      setOneShotModeTMP117();
//        send_button_state_indication(i);
////      next_state = wait_one_shot;
//      LOG_INFO("Sent BPM value");
//      i++;
//      //DOS gpioLed0SetOn();
//    }
//
//  }
//
//}
void SPO2_measure_state_machine(sl_bt_msg_t *event)
{
  sl_status_t sc = 0;
  ble_data_struct_t *ble_data_ptr;
  ble_data_ptr = return_ble_data_struct();
  static uint8_t count = 0;
  static int i = 0;
  static long sum = 0;

  static bool hr_value_sent = false;

  static bool spo2_value_sent = false;
  static bool new_value = false;


  switch (SL_BT_MSG_ID(event->header))
  {
    case sl_bt_evt_system_external_signal_id:
      if(event->data.evt_system_external_signal.extsignals == evtPB0)
        {
          if(ble_data_ptr->PB0_state == 0)
            {
              HR_init();
              SPO2_init();
//              PB0_flag = true;

              SPO2_measure();
              hr_value_sent = false;
              spo2_value_sent = false;


//              send_button_state_indication(get_heart_rate());
//              timerWaitUs_polled(10000);
//              while(ble_data_ptr-> indication_in_flight_PB0);
              LOG_INFO("Here\r");

//              send_spo2_indication(get_spo2());


//              send_spo2_indication(get_spo2());
////              timerWaitUs_polled(10000);
//              while(ble_data_ptr-> indication_in_flight_spo2);
//              send_button_state_indication(get_heart_rate());

            }
        }
      break;
//    case sl_bt_evt_gatt_procedure_completed_id:

  }
//  if(new_value)
//    {
      if(hr_value_sent == false)
        {
          if(send_button_state_indication(get_heart_rate()))
            hr_value_sent = true;
        }
      if(spo2_value_sent == false)
        {
          if(send_spo2_indication(get_spo2()))
            spo2_value_sent = true;
        }
//
//    }

}

//void SPO2_measure_state_machine(sl_bt_msg_t *event)
//{
//  sl_status_t sc = 0;
//  ble_data_struct_t *ble_data_ptr;
//  ble_data_ptr = return_ble_data_struct();
//  static uint8_t count = 0;
//  static int i = 0;
//  static long sum = 0;
//
//  switch (SL_BT_MSG_ID(event->header))
//  {
//    case sl_bt_evt_system_external_signal_id:
//      if(event->data.evt_system_external_signal.extsignals == evtPB0)
//        {
//          if(ble_data_ptr->PB0_state == 0)
//            {
//              HR_init();
//              SPO2_init();
//              PB0_flag = true;
//            }
//        }
//      break;
//
//
//    case sl_bt_evt_system_soft_timer_id:
//          count++;
//
//          if(PB0_flag && ble_data_ptr->connection_status)
//            {
//              bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
//
//                //read the first 100 samples, and determine the signal range
//                for (uint8_t i = 0 ; i < bufferLength ; i++)
//                {
//                  while (MAX30105_available() == false) //do we have new data?
//                    check(); //Check the sensor for new data
//
//                  redBuffer[i] = MAX30105_getRed();
//                  irBuffer[i] = MAX30105_getIR();
//                  MAX30105_nextSample(); //We're finished with this sample so move to next sample
//
//                  LOG_INFO("red = %d\r", redBuffer[i]);
//
//                  LOG_INFO("ir = %d\r", irBuffer[i]);
//
//                }
//
//                //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
//                maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
//
//                //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
//                while (count <= 10 && PB0_flag == true)
//                {
////                    if(event->data.evt_system_soft_timer.handle == SOFT_TIMER_PULSE_SENSOR)
////                    {
////                       count++;
////                    }
//
//                    LOG_INFO("========================count %d\r\n", count);
//                    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
//                    for (uint8_t i = 25; i < 100; i++)
//                    {
//                      redBuffer[i - 25] = redBuffer[i];
//                      irBuffer[i - 25] = irBuffer[i];
//                    }
//
//                    //take 25 sets of samples before calculating the heart rate.
//                    for (uint8_t i = 75; i < 100; i++)
//                    {
//                      while (MAX30105_available() == false) //do we have new data?
//                        check(); //Check the sensor for new data
//
//                //      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
//
//                      redBuffer[i] = MAX30105_getRed();
//                      irBuffer[i] = MAX30105_getIR();
//                      MAX30105_nextSample(); //We're finished with this sample so move to next sample
//
//
//                      hr_buffer[i] = heartRate;
//                      spo2_buffer[i] = spo2;
//                      i++;
//
//                      //send samples and calculation result to terminal program through UART
//  //                    LOG_INFO("red = %d\r", redBuffer[i]);
//  //
//  //                    LOG_INFO("ir = %d\r", irBuffer[i]);
//
//                      LOG_INFO("HR = %d \r", heartRate);
//  //
//  //                    LOG_INFO("HRvalid = %d\r", validHeartRate);
//  //
//                      LOG_INFO("SPO2 = %d \r", spo2);
//  //
//  //                    LOG_INFO("SPO2Valid = %d\r", validSPO2);
//                    }
//
//                    //After gathering 25 new samples recalculate HR and SP02
//                    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
//
//                }
//                for(i = 0; i< NO_OF_SAMPLES; i++)
//                  {
//                    if(hr_buffer[i]== -999)
//                      continue;
//                    sum += hr_buffer[i];
//                    LOG_INFO("Last HR sum = %d %ld\r", i, sum);
//                  }
//                heartRate = sum/NO_OF_SAMPLES;
//                LOG_INFO("Last HR = %d %ld\r", heartRate, sum);
//
//                sum = 0;
//                for(i = 0; i< NO_OF_SAMPLES; i++)
//                  {
//                    if(spo2_buffer[i]== -999)
//                      continue;
//                    sum += spo2_buffer[i];
//                    LOG_INFO("Last SPO2 sum = %d %ld\r", i, sum);
//                  }
//                spo2 = sum/NO_OF_SAMPLES;
//                LOG_INFO(" Last SPO2 = %d %ld\r", spo2, sum);
//
//                PB0_flag = false;
//                if(count > 10)
//                  count = 0;
//                i = 0;
//
//            }
//
//
//  }
//
//
//
//}
