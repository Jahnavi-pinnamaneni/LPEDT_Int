/*
 * @file: ble.h
 * @desc: This file contains all the functions related to the Bluetooth Stack
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include "sl_bt_api.h"
//#include "app_log.h"
#include "app_assert.h"
#include <stdbool.h>
#include "gatt_db.h"
#include "lcd.h"
#include "ble_device_type.h"

#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }
#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
      *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }
#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))


// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
  // values that are common to servers and clients
  bd_addr myAddress;

  // values unique for server

  // The advertising set handle allocated from Bluetooth stack.
  uint8_t advertisingSetHandle;

  uint8_t connection_handle;

  bool connection_status;

  bool indication_flag;

  bool indication_flag_PB0;

  bool indication_in_flight;

  bool indication_in_flight_PB0;

  // values unique for client

} ble_data_struct_t;

ble_data_struct_t * return_ble_data_struct();
void handle_ble_event(sl_bt_msg_t * evt);
void send_button_state_indication(uint8_t button_st);

#endif /* SRC_BLE_H_ */
