/*
 * tmp117.h
 *
 *  Created on: Apr 22, 2023
 *      Author: pjahn
 */

#ifndef SRC_TMP117_H_
#define SRC_TMP117_H_

#include "i2c.h"

#define TMP117_I2CADDR    0x48
#define TMP117_EEPROM_UL  0x04
#define TMP117_TEMP       0x00
#define TMP117_CONFIG     0x01

#define TMP117_SD         0x620
#define TMP117_SD_MSB     ((TMP117_SD & 0xFF00) >> 8)
#define TMP117_SD_LSB     (TMP117_SD & 0xFF)

#define TMP117_OS         0xE20
#define TMP117_OS_MSB     ((TMP117_OS & 0xFF00) >> 8)
#define TMP117_OS_LSB     (TMP117_OS & 0xFF)

/* Transfer Data for TMP117*/
uint8_t SD_DataBuffer[3] = {TMP117_CONFIG, TMP117_SD_MSB, TMP117_SD_LSB};
uint8_t OS_DataBuffer[3] = {TMP117_CONFIG, TMP117_OS_MSB, TMP117_OS_LSB};
uint8_t TP_DataBuffer[3] = {TMP117_TEMP, 0, 0};
uint8_t rx_Temp[2];

/*
 * @desc: This functions sends the Shutdown mode command and checks for error
 * @param: None
 * @return: None
 */
void setShutdownModeTMP117();

/*
 * @desc: This functions sets the One-shot mode and waits for 125ms conversion time
 * @param: None
 * @return: None
 */
void setOneShotModeTMP117();

/*
 * @desc: This functions reads the Temperature register, 1 Byte to write and 2 Bytes to read
 * @param: None
 * @return: None
 */
void getTemperatureTMP117();

/*
 * @desc: This functions calculates the temperature in Celsius and prints it via serial port
 * @param: None
 * @return: None
 */
int reportTemperatureTMP117();

#endif /* SRC_TMP117_H_ */
