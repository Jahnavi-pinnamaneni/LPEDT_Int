/*
 * @file: ble.c
 * @desc: This file contains all the functions related to Bluetooth Stack Implementation
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */
#include "ble.h"

//#define INCLUDE_LOG_DEBUG 1
#include "log.h"

// BLE private data
ble_data_struct_t ble_data;

#define GATTDB_SYSTEM_ID      18
#define ADVERTISING_INTERVAL  400   //This value is converted to suit the format
                                    //that is required by the function (250ms * 1.6)

#define CONNECTION_INTERVAL   60    //Time = Value x 1.25 ms

#define SALVE_LATENCY         3     //which defines how many connection intervals
                                    //the peripheral can skip if it has no data to send

#define SUPERVISION_TIMEOUT   75    //(1 + @p latency) * max_interval * 2


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

  // DOS: It's functionally ok do to this, but you realize that ble.c has direct access
  // to the variable ble_data as it is defined just above. Other files like scheduler.c etc
  // would use this technique to access the ble private data structure fields.
  ble_data_ptr = return_ble_data_struct();


// If the event is related to external signals then it is handled in the
// temperature_state_machine. We ignore these events in this handler in the best
// interest of isolation of functionality
  if(SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id)
    {
      return;
    }

  // State machine to handle all stack events
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      displayInit();
      // Print boot message.
      LOG_INFO("Bluetooth stack booted: v%d.%d.%d-b%d",
                   evt->data.evt_system_boot.major,
                   evt->data.evt_system_boot.minor,
                   evt->data.evt_system_boot.patch,
                   evt->data.evt_system_boot.build);

      displayPrintf(DISPLAY_ROW_NAME , BLE_DEVICE_TYPE_STRING);

      // Extract unique ID from BT Address.
      sc = sl_bt_system_get_identity_address(&address, &address_type);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR: Get_identity address");
        }

      displayPrintf(DISPLAY_ROW_BTADDR , "%02X:%02X:%02X:%02X:%02X:%02X", address.addr[0],
                    address.addr[1],address.addr[2],address.addr[3],
                    address.addr[4],address.addr[5]);
      // Pad and reverse unique ID to get System ID.
      system_id[0] = address.addr[5];
      system_id[1] = address.addr[4];
      system_id[2] = address.addr[3];
      system_id[3] = 0xFF;
      system_id[4] = 0xFE;
      system_id[5] = address.addr[2];
      system_id[6] = address.addr[1];
      system_id[7] = address.addr[0];

      // DOS: I don't believe this is required. At least, I don't do this in my
      // implementation.
      sc = sl_bt_gatt_server_write_attribute_value(GATTDB_SYSTEM_ID,
                                                   0,
                                                   sizeof(system_id),
                                                   system_id);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR: write_attribute_value");
        }

      LOG_INFO("Bluetooth %s address: %02X:%02X:%02X:%02X:%02X:%02X",
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
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR: adverstiser_create_set");
        }

      // Set advertising interval to 250ms.
      sc = sl_bt_advertiser_set_timing(
        ble_data_ptr->advertisingSetHandle, // advertising set handle
        ADVERTISING_INTERVAL,     // min. adv. interval (milliseconds * 1.6)
        ADVERTISING_INTERVAL,     // max. adv. interval (milliseconds * 1.6)
        0,                        // adv. duration
        0);                       // max. num. adv. events
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR");
        }

      // Start general advertising and enable connections.
      sc = sl_bt_advertiser_start(
          ble_data_ptr->advertisingSetHandle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR");
        }
      else
        {
          displayPrintf(DISPLAY_ROW_CONNECTION , "Advertising");
          LOG_INFO("Started advertising");
        }
      displayPrintf(DISPLAY_ROW_ASSIGNMENT , "A6");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened.
    case sl_bt_evt_connection_opened_id:
      LOG_INFO("Connection opened ");
      displayPrintf(DISPLAY_ROW_CONNECTION , "Connected");
      // DOS: grouped these together
      ble_data_ptr->connection_handle = evt->data.evt_connection_opened.connection;
      ble_data_ptr->connection_status = true;
      ble_data_ptr->indication_in_flight = false;

      // Stop the the advertiser once the connection is established
      sc = sl_bt_advertiser_stop(ble_data_ptr->advertisingSetHandle);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR");
        }

      // Set connection parameters like the  Connection Interval Max and Min and
      // the Slave Latency
      sc = sl_bt_connection_set_parameters(
          evt->data.evt_connection_opened.connection,
          CONNECTION_INTERVAL,
          CONNECTION_INTERVAL,
          SALVE_LATENCY,
          SUPERVISION_TIMEOUT,
          0,
          0xffff
          );
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR");
        }
      else
        {
          LOG_INFO("Set parameters complete");
        }




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
      displayPrintf(DISPLAY_ROW_CONNECTION , " ");
      displayPrintf(DISPLAY_ROW_TEMPVALUE , " ");
      // Update the connection status when connection is closed
      ble_data_ptr->connection_status = false;
      ble_data_ptr->indication_flag = false;
      ble_data_ptr->indication_in_flight = false; // DOS

      // Restart advertising after client has disconnected.
      sc = sl_bt_advertiser_start(
        ble_data_ptr->advertisingSetHandle,
        sl_bt_advertiser_general_discoverable,
        sl_bt_advertiser_connectable_scannable);
      if(sc != SL_STATUS_OK)
        {
          LOG_ERROR("ERROR");
        }
      else{
          displayPrintf(DISPLAY_ROW_CONNECTION , "Advertising");
          LOG_INFO("Started advertising\n");
      }

      break;

    // This event occurs once the connection parameters are set.
    case sl_bt_evt_connection_parameters_id:
      LOG_INFO("\n connection_handle: %d, \r\n interval: %d ms,\r\n latency: %d "
          "\r\n, timeout: %d ms, \r\n security_mode: %d, \r\n txsize: %d",
           evt->data.evt_connection_parameters.connection,
           evt->data.evt_connection_parameters.interval,
           evt->data.evt_connection_parameters.latency,
           evt->data.evt_connection_parameters.timeout,
           evt->data.evt_connection_parameters.security_mode,
           evt->data.evt_connection_parameters.txsize);
      break;

    // Every time the client configuration is changed we check if the indication
    // flag is set or if the characteristic confirmation is received.
      /***********************************************************************************
       * Borrowed the following code snippet from Varun Mehta
       ***********************************************************************************/
    case sl_bt_evt_gatt_server_characteristic_status_id:
      // Check if the characteristic is temperature measurement and if Characteristic
      // configuration is changed
      if(evt->data.evt_gatt_server_characteristic_status.characteristic ==
          gattdb_temperature_measurement && evt->data.evt_gatt_server_characteristic_status.
          status_flags == sl_bt_gatt_server_client_config)
              {
                // Check if Indications are enabled
                if(evt->data.evt_gatt_server_characteristic_status.client_config_flags
                    == sl_bt_gatt_server_indication)
                  {
                    ble_data_ptr->connection_handle = evt->data.
                        evt_gatt_server_characteristic_status.connection;
                    ble_data.indication_flag = true;
                    LOG_INFO("Indications Enabled\n\r");
                  }
                else
                  {
                    ble_data.indication_flag = false;
                    LOG_INFO("Indications Disabled \n\r");
                  }
              }
      // Check if the characteristic is temperature measurement and if characteristic
      // confirmation has been received. This implies that the indication has been
      // sent successfully hence the indication_in_flight flag is set to false
      else if(evt->data.evt_gatt_server_characteristic_status.characteristic ==
          gattdb_temperature_measurement && evt->data.evt_gatt_server_characteristic_status.
          status_flags == sl_bt_gatt_server_confirmation)
        {
            LOG_INFO("Indications Complete \n\r");
            ble_data_ptr->indication_in_flight = false;
        }
      break;

    // This event occurs when the confirmation for an indication has not been received
    // The BLE server does not send further indications hence the indication_in_flight
    // flag is set to false
    case sl_bt_evt_gatt_server_indication_timeout_id:
      LOG_ERROR("Indication TIMEOUT occurred");
      //DOS Let the close event take care of this. Better to perform all of the book keeping in 1 place
      // ble_data_ptr->indication_in_flight = false;
       break;

    // -------------------------------
    // Default event handler.
    default:
      break;
  }
}
