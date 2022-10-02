/*
 * @file: ble.c
 * @desc: This file contains all the functions related to Bluetooth Stack Implementation
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */
#include "ble.h"
#define INCLUDE_LOG_DEBUG 1
#include "log.h"

// BLE private data
ble_data_struct_t ble_data;

#define GATTDB_SYSTEM_ID 18
#define ADVERTISING_INTERVAL 400   //This value is converted to suit the format that is required by the function (250ms * 1.6)
#define CONNECTION_INTERVAL 60 //60,60,3,75 This value converted to suit the format that is required by the function (75ms * 1.25)
#define SALVE_LATENCY 3
#define SUPERVISION_TIMEOUT 75  //((1 + slave latency) * (connection_interval * 2) * 10) ms


/*
 * @desc: This function returns a pointer to the ble data structure
 */
ble_data_struct_t * return_ble_data_struct()
{
  return &ble_data;
}


/*
 * @desc: This function performs all the Bluetooth event handling
 * @param: take a pointer to the event as an input
 */
void handle_ble_event(sl_bt_msg_t * evt)
{
  sl_status_t sc = 0;
  bd_addr address;
  uint8_t address_type;
  uint8_t system_id[8];
  ble_data_struct_t *ble_data_ptr;

  ble_data_ptr = return_ble_data_struct();



  if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id)
    {
      return;
    }

  // Handle stack events
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      // Print boot message.
      LOG_INFO("Bluetooth stack booted: v%d.%d.%d-b%d\r\n",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      //app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }

      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      sc = sl_bt_gatt_server_write_attribute_value(GATTDB_SYSTEM_ID,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      //app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }
      LOG_INFO("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                   address_type ? "static random" : "public device",
                   address.addr[5],
                   address.addr[4],
                   address.addr[3],
                   address.addr[2],
                   address.addr[1],
                   address.addr[0]);
      ble_data_ptr->advertisingSetHandle = 0xFF;
      // Create an advertising set.
      sc = sl_bt_advertiser_create_set(&(ble_data_ptr->advertisingSetHandle));
      //app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }
      // Set advertising interval to 250ms.
      sc = sl_bt_advertiser_set_timing(
          ble_data_ptr->advertisingSetHandle, // advertising set handle
        ADVERTISING_INTERVAL, // min. adv. interval (milliseconds * 1.6)
        ADVERTISING_INTERVAL, // max. adv. interval (milliseconds * 1.6)
        0,   // adv. duration
        0);  // max. num. adv. events
      //app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }

      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
          ble_data_ptr->advertisingSetHandle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
     // app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }
      else
        {
          LOG_INFO("Started advertising\r\n");
        }

      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      LOG_INFO("Connection opened \r\n");
      sc = sl_bt_advertiser_stop(ble_data_ptr->advertisingSetHandle);
      //app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }
      //Set connection parameters like the  Connection Interval Max and Min and the Slave Latency
      sc = sl_bt_connection_set_parameters(
          evt->data.evt_connection_opened.connection,
          CONNECTION_INTERVAL,   //Time = Value x 1.25 ms
          CONNECTION_INTERVAL,   //Time = Value x 1.25 ms
          SALVE_LATENCY,
          SUPERVISION_TIMEOUT,
          0,
          0xffff
          );
//      app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }
      else
        LOG_INFO("Set parameters complete\r\n");

      ble_data_ptr->connection_handle = evt->data.evt_connection_opened.connection;
      ble_data_ptr->connection_status = true;

#ifdef SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT
      // Set remote connection power reporting - needed for Power Control
      sc = sl_bt_connection_set_remote_power_reporting(
        evt->data.evt_connection_opened.connection,
        sl_bt_connection_power_reporting_enable);
      app_assert_status(sc);
#endif // SL_CATALOG_BLUETOOTH_FEATURE_POWER_CONTROL_PRESENT

      break;


    // -------------------------------
    // This event indicates that a connection was closed.
    case sl_bt_evt_connection_closed_id:
      LOG_INFO("Connection closed\n");
      ble_data_ptr->connection_status = false;
      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
          ble_data_ptr->advertisingSetHandle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
     // app_assert_status(sc);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR\r\n");
        }
      else{
          LOG_INFO("Started advertising\n");
      }

      break;

    ///////////////////////////////////////////////////////////////////////////
    // Add additional event handlers here as your application requires!      //
    ///////////////////////////////////////////////////////////////////////////
    case sl_bt_evt_connection_parameters_id:
      LOG_INFO("\r\n connection_handle: %d, \r\n interval: %d ms,\r\n latency: %d "
          "\r\n, timeout: %d ms, \r\n security_mode: %d, \r\n txsize: %d\r\n",
           evt->data.evt_connection_parameters.connection,
           evt->data.evt_connection_parameters.interval,
           evt->data.evt_connection_parameters.latency,
           evt->data.evt_connection_parameters.timeout,
           evt->data.evt_connection_parameters.security_mode,
           evt->data.evt_connection_parameters.txsize);
      break;
      /***********************************************************************************
       * Borrowed the following code snippet from Varun Mehta
       ***********************************************************************************/
    case sl_bt_evt_gatt_server_characteristic_status_id:
      if(evt->data.evt_gatt_server_characteristic_status.characteristic == gattdb_temperature_measurement &&
                evt->data.evt_gatt_server_characteristic_status.status_flags == sl_bt_gatt_server_client_config)
              {
                //Indications are enabled
                if(evt->data.evt_gatt_server_characteristic_status.client_config_flags == sl_bt_gatt_server_indication)
                  {
                    ble_data_ptr->connection_handle = evt->data.evt_gatt_server_characteristic_status.connection;
                    ble_data.indication_flag = true;
                    LOG_INFO("Indications Enabled %d\n\r",0);
                  }
                else
                  {
                    ble_data.indication_flag = false;
                    LOG_INFO("Indications Disabled %d\n\r",0);
                  }
              }
      break;

//    case sl_bt_evt_gatt_server_indication_timeout_id:
//      if(ble_data_ptr->indication_in_flight)
//        {
//          LOG_ERROR("Indication in Flight\r\n");
//          ble_data_ptr->indication_in_flight = false;
//        }
//       break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
