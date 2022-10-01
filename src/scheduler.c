/*
  * @file: scheduler.c
  * @desc: This file provides the required to handle all actions of the scheduler.
  * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#include "scheduler.h"


#define INCLUDE_LOG_DEBUG 1
#include "log.h"


#define CMD_DATA_TO_READ 0xF3
#define MULTIPLIER 175.72
#define MAX_RES 65536
#define CONSTANT 46.85

uint32_t evt_flag = 0;

#define SENSOR_ENABLE_TIME 80000
#define TEMP_MEAS_TIME 11000

typedef enum states{
  idle,
  sensor_enable,
  i2c_write_state,
  i2c_writecomplete,
  i2c_read_state
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

void temperature_state_machine(sl_bt_msg_t *event)
{
  /*static state_t current_state = idle;
  static state_t next_state = idle;
  uint16_t temp_data = 0, read = 0;
  uint32_t temp_in_c = 0;

  sl_status_t sc;
  ble_data_struct_t *ble_data_ptr;
  ble_data_ptr = return_ble_data_struct();
  uint8_t htm_temperature_buffer[5];
  uint8_t *p = htm_temperature_buffer;
  uint32_t htm_temperature_flt;

  scheduler_evt_t evt = 0;


  if(SL_BT_MSG_ID(event->header) == sl_bt_evt_system_external_signal_id)
    {
      evt = event->data.evt_system_external_signal.extsignals;
      if(ble_data_ptr->indication_flag && ble_data_ptr->connection_status)
        {
          switch(current_state)
                  {
                    case idle:
                              next_state = idle;
                              if(evt == evtTimerUF)
                                {
                                  gpioSensor_enSetOn();
                                  timerWaitUs_irq(SENSOR_ENABLE_TIME);
                                  next_state = sensor_enable;
                                }
                              break;
                    case sensor_enable:
                               next_state = sensor_enable;
                               if(evt == evtTimerCOMP1)
                                 {
                                   i2c_write_irq(CMD_DATA_TO_READ);
                                   sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                                   next_state = i2c_write_state;
                                 }
                               break;
                    case i2c_write_state:
                              next_state = i2c_write_state;
                              if(evt == evtI2CComplete)
                                {
                                  NVIC_DisableIRQ(I2C0_IRQn);
                                  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                                  timerWaitUs_irq(TEMP_MEAS_TIME);
                                  next_state = i2c_writecomplete;
                                }
                              break;
                    case i2c_writecomplete:
                              next_state = i2c_writecomplete;
                              if(evt == evtTimerCOMP1)
                                {
                                  i2c_read_irq();
                                  sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                                  next_state = i2c_read_state;
                                }
                              break;
                    case i2c_read_state:
                              next_state = i2c_read_state;
                              if(evt == evtI2CComplete)
                                {
                                  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                                  NVIC_DisableIRQ(I2C0_IRQn);
                                  gpioSensor_enSetOff();
                                  read = read_data;

                                  temp_data = ((read & 0x00FF) << 8) & 0xFF00;
                                  read = (read >> 8) | temp_data;

                                  temp_data = (int)( ((MULTIPLIER * read)/MAX_RES) - CONSTANT );

                                  temp_in_c = (uint32_t)temp_data;
                                  htm_temperature_flt = UINT32_TO_FLOAT(temp_in_c*1000, -3);
                                  UINT32_TO_BITSTREAM(p, htm_temperature_flt);
                                  app_log_info("Temperature %f\r\n", htm_temperature_flt);
                                  sc = sl_bt_gatt_server_write_attribute_value(
                                    gattdb_temperature_measurement, // handle from gatt_db.h
                                    0, // offset
                                    4, // length
                                    &temp_in_c // pointer to buffer where data is
                                  );

                                  if (sc != SL_STATUS_OK) {
                                  LOG_ERROR("ERROR");
                                  }

                                  if (indication ) {
                                    sc = sl_bt_gatt_server_send_indication(
                                    event->data.evt_connection_opened.connection,
                                    gattdb_temperature_measurement, // handle from gatt_db.h
                                    5,
                                    &htm_temperature_buffer[0] // in IEEE-11073 format
                                    );
                                    app_assert_status(sc);

                                    sc = sl_bt_gatt_server_send_indication(
                                    event->data.evt_connection_opened.connection,
                                    gattdb_temperature_measurement, // handle from gatt_db.h
                                    5,
                                    p // in IEEE-11073 format
                                    );
                                    app_assert_status(sc);
                                  }
                                  LOG_INFO("Temperature Reading = %d\r\n",temp_data);

                                  next_state = idle;
                                }
                              break;
                  }
        }
        current_state = next_state;
    }*/
  static state_t current_state = idle;
    static state_t next_state = idle;
    uint16_t temp_data = 0, read = 0;

    if(SL_BT_MSG_ID(event->header) != sl_bt_evt_system_external_signal_id)
      {
        return;
      }
    scheduler_evt_t evt = event->data.evt_system_external_signal.extsignals;
    switch(current_state)
    {
      case idle:
                next_state = idle;
                if(evt == evtTimerUF)
                  {
                    gpioSensor_enSetOn();
                    timerWaitUs_irq(SENSOR_ENABLE_TIME);
                    next_state = sensor_enable;
                  }
                break;
      case sensor_enable:
                 next_state = sensor_enable;
                 if(evt == evtTimerCOMP1)
                   {
                     i2c_write_irq(CMD_DATA_TO_READ);
                     sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                     next_state = i2c_write_state;
                   }
                 break;
      case i2c_write_state:
                next_state = i2c_write_state;
                if(evt == evtI2CComplete)
                  {
                    NVIC_DisableIRQ(I2C0_IRQn);
                    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                    timerWaitUs_irq(TEMP_MEAS_TIME);
                    next_state = i2c_writecomplete;
                  }
                break;
      case i2c_writecomplete:
                next_state = i2c_writecomplete;
                if(evt == evtTimerCOMP1)
                  {
                    i2c_read_irq();
                    sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
                    next_state = i2c_read_state;
                  }
                break;
      case i2c_read_state:
                next_state = i2c_read_state;
                if(evt == evtI2CComplete)
                  {
                    sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
                    NVIC_DisableIRQ(I2C0_IRQn);
                    gpioSensor_enSetOff();
                    read = read_data;

                    temp_data = ((read & 0x00FF) << 8) & 0xFF00;
                    read = (read >> 8) | temp_data;

                    temp_data = (int)( ((MULTIPLIER * read)/MAX_RES) - CONSTANT );

                    LOG_INFO("Temperature Reading = %d\r\n",temp_data);
                    next_state = idle;
                  }
                break;

    }
    current_state = next_state;

}

