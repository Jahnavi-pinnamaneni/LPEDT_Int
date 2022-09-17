/*
 * @file: i2c.h
 * @desc: This file defines all the functions required for I2C communications
 * @author: Jahnavi Pinnamaneni; japi8358@colorado.edu
 */

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

#include "sl_i2cspm.h"
#include "gpio.h"
#include "timers.h"
#include <stdint.h>
#define SI7021_DEVICE_ADDR 0x40

/*
 * @desc: This function initializes the SCL and SDA GPIO pins to pull up configuration. Additionally the clock is also set to the High frequency peripheral clock (HFPERCLK)
 * @params: none
 * @return: none
 */
void i2c_init();

/*
 * @desc: This function writes to the I2C peripheral the command that is passed in its argument.
 * @params: data that needs to be transmitted
 * @return: none
 */
void i2c_write(uint8_t cmd);

/*
 * @desc: This function reads from the I2C peripheral
 * @params: none
 * @return: none
 */
void i2c_read(void);

/*
 * @desc: This function performs a series of I2C actions to request temperature from the Si7021 sensor and read the measured temperature.
 * @params: none
 * @return: none
 */
void i2c_read_temp();

#endif /* SRC_I2C_H_ */
