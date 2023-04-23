/*
 * tmp117.c
 *
 *  Created on: Apr 22, 2023
 *      Author: pjahn
 */

#include "tmp117.h"

//#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

void setShutdownModeTMP117()
{
  uint8_t ret_status = I2CTransferInitWrapper(TMP117_I2CADDR, &SD_DataBuffer[0], I2C_FLAG_WRITE,
                                              sizeof(SD_DataBuffer)/sizeof(uint8_t), 0);
  if(ret_status)
      return;
}

void setOneShotModeTMP117()
{
  uint8_t ret_status = I2CTransferInitWrapper(TMP117_I2CADDR, &OS_DataBuffer[0], I2C_FLAG_WRITE,
                                              sizeof(OS_DataBuffer)/sizeof(uint8_t), 0);
  if(ret_status)
      return;
}

void getTemperatureTMP117()
{
  uint8_t ret_status = I2CTransferInitWrapper(TMP117_I2CADDR, &TP_DataBuffer[0], I2C_FLAG_WRITE_READ,
                                              1, 2);
  if(ret_status)
      return;
}

int reportTemperatureTMP117()
{
  int Calc_Temp;
  uint16_t Read_Temp;

  Read_Temp = ((TP_DataBuffer[1] << 8) | TP_DataBuffer[2]);

  /*TODO: Placeholder conversion code, will not work below
          1C or above 255C and is an integer*/

  Calc_Temp = Read_Temp/0x80;
  LOG_INFO("Temperature = %dC\n\r", Calc_Temp);
}
