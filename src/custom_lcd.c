/*
 * custom_lcd.c
 *
 *  Created on: Apr 22, 2023
 *      Author: pjahn
 */

#include <stdio.h>

#include "sl_board_control.h"
#include "em_assert.h"
#include "glib.h"
#include "dmd.h"

static GLIB_Context_t glibContext;
static int currentLine = 0;

/***************************************************************************//**
 * Initialize example.
 ******************************************************************************/
void memlcd_app_init(void)
{
  uint32_t status;

  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);

  /* Draw text on the memory lcd display*/
  GLIB_drawStringOnLine(&glibContext,
                        "ESteamD",
                        currentLine++,
                        GLIB_ALIGN_LEFT,
                        5,
                        5,
                        true);

  DMD_updateDisplay();
}

/***************************************************************************//**
 * Function to display custom images
 ******************************************************************************/

void custom_draw(GLIB_Context_t *   pContext,
                 int32_t   x,
                 int32_t   y,
                 uint32_t  width,
                 uint32_t  height,
                 const uint16_t *   picData )
{
  int32_t x_min, x_max, y_min, y_max, i, j;
  uint16_t temp = 0;
  x_min = x;
  y_min = y;
  x_max = x + width;
  y_max = y + height;

  for(i = y_min; i < y_max; i++)
    {
      temp = *(picData + (i - y_min));
      for( j = x_min; j < x_max; j++)
        {
          if(temp == 0x00)
            break;
          if((temp & 0x01)!= 0)
            GLIB_drawPixel(pContext,j,i);
          temp = temp>>1;
        }

    }
}
