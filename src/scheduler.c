/*
  * @file: scheduler.c
  * @desc: This file provides the required to handle all actions of the scheduler.
  * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#include "scheduler.h"

#include "src/gpio.h"


//#define INCLUDE_LOG_DEBUG 1
#include "log.h"


#define CMD_DATA_TO_READ 0xF3
#define MULTIPLIER 175.72
#define MAX_RES 65536
#define CONSTANT 46.85

uint32_t evt_flag = 0;

#define SENSOR_ENABLE_TIME 80000
#define TEMP_MEAS_TIME (125*1000)


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
