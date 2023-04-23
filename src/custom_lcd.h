/*
 * custom_lcd.h
 *
 *  Created on: Apr 22, 2023
 *      Author: pjahn
 */

#ifndef SRC_CUSTOM_LCD_H_
#define SRC_CUSTOM_LCD_H_

/***************************************************************************//**
 * Initialize example
 ******************************************************************************/
void memlcd_app_init(void);

/***************************************************************************//**
 * Function to display custom images
 ******************************************************************************/
void custom_draw(GLIB_Context_t *   pContext,
                 int32_t   x,
                 int32_t   y,
                 uint32_t  width,
                 uint32_t  height,
                 const uint16_t *   picData );


#endif /* SRC_CUSTOM_LCD_H_ */
