/*
 * spo2_algo.c
 *
 *  Created on: Apr 22, 2023
 *      Author: sanis
 */


/** \file algorithm.cpp ******************************************************
*
* Project: MAXREFDES117#
* Filename: algorithm.cpp
* Description: This module calculates the heart rate/SpO2 level
*
*
* --------------------------------------------------------------------
*
* This code follows the following naming conventions:
*
* char              ch_pmod_value
* char (array)      s_pmod_s_string[16]
* float             f_pmod_value
* int32_t           n_pmod_value
* int32_t (array)   an_pmod_value[16]
* int16_t           w_pmod_value
* int16_t (array)   aw_pmod_value[16]
* uint16_t          uw_pmod_value
* uint16_t (array)  auw_pmod_value[16]
* uint8_t           uch_pmod_value
* uint8_t (array)   auch_pmod_buffer[16]
* uint32_t          un_pmod_value
* int32_t *         pn_pmod_value
*
* ------------------------------------------------------------------------- */
/*******************************************************************************
* Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

//#include "Arduino.h"
#include <math.h>
#include "spo2_algo.h"

#include "SparkFun_MAX30101.h"
#include <stdbool.h>

// Include logging specifically for this .c file
#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

uint32_t avg_bpm_val; //heart rate value
uint32_t avg_spo2_val; //heart rate value

uint8_t pulseLED = 11; //Must be on PWM pin
uint8_t readLED = 13; //Blinks with each data read


const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99,
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97,
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91,
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81,
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67,
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50,
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29,
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5,
              3, 2, 1 } ;
static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated.  Samples become 16-bit data.
void maxim_heart_rate_and_oxygen_saturation(uint16_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint16_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid,
                int32_t *pn_heart_rate, int8_t *pch_hr_valid)
#else
void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer, int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid,
                int32_t *pn_heart_rate, int8_t *pch_hr_valid)
#endif
/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the an_ratio for the SPO2 is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
*               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each an_ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_spo2                - Calculated SpO2 value
* \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
* \param[out]    *pn_heart_rate          - Calculated heart rate value
* \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
{
  uint32_t un_ir_mean;
  int32_t k, n_i_ratio_count;
  int32_t i, n_exact_ir_valley_locs_count, n_middle_idx;
  int32_t n_th1, n_npks;
  int32_t an_ir_valley_locs[15] ;
  int32_t n_peak_interval_sum;

  int32_t n_y_ac, n_x_ac;
  int32_t n_spo2_calc;
  int32_t n_y_dc_max, n_x_dc_max;
  int32_t n_y_dc_max_idx = 0;
  int32_t n_x_dc_max_idx = 0;
  int32_t an_ratio[5], n_ratio_average;
  int32_t n_nume, n_denom ;

  // calculates DC mean and subtract DC from ir
  un_ir_mean =0;
  for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
  un_ir_mean =un_ir_mean/n_ir_buffer_length ;

  // remove DC and invert signal so that we can use peak detector as valley detector
  for (k=0 ; k<n_ir_buffer_length ; k++ )
    an_x[k] = -1*(pun_ir_buffer[k] - un_ir_mean) ;

  // 4 pt Moving Average
  for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
    an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int)4;
  }
  // calculate threshold
  n_th1=0;
  for ( k=0 ; k<BUFFER_SIZE ;k++){
    n_th1 +=  an_x[k];
  }
  n_th1=  n_th1/ ( BUFFER_SIZE);
  if( n_th1<30) n_th1=30; // min allowed
  if( n_th1>60) n_th1=60; // max allowed

  for ( k=0 ; k<15;k++) an_ir_valley_locs[k]=0;
  // since we flipped signal, we use peak detector as valley detector
  maxim_find_peaks( an_ir_valley_locs, &n_npks, an_x, BUFFER_SIZE, n_th1, 4, 15 );//peak_height, peak_distance, max_num_peaks
  n_peak_interval_sum =0;
  if (n_npks>=2){
    for (k=1; k<n_npks; k++) n_peak_interval_sum += (an_ir_valley_locs[k] -an_ir_valley_locs[k -1] ) ;
    n_peak_interval_sum =n_peak_interval_sum/(n_npks-1);
    *pn_heart_rate =(int32_t)( (FreqS*60)/ n_peak_interval_sum );
    *pch_hr_valid  = 1;
  }
  else  {
    *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
    *pch_hr_valid  = 0;
  }

  //  load raw value again for SPO2 calculation : RED(=y) and IR(=X)
  for (k=0 ; k<n_ir_buffer_length ; k++ )  {
      an_x[k] =  pun_ir_buffer[k] ;
      an_y[k] =  pun_red_buffer[k] ;
  }

  // find precise min near an_ir_valley_locs
  n_exact_ir_valley_locs_count =n_npks;

  //using exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration an_ratio
  //finding AC/DC maximum of raw

  n_ratio_average =0;
  n_i_ratio_count = 0;
  for(k=0; k< 5; k++) an_ratio[k]=0;
  for (k=0; k< n_exact_ir_valley_locs_count; k++){
    if (an_ir_valley_locs[k] > BUFFER_SIZE ){
      *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
      *pch_spo2_valid  = 0;
      return;
    }
  }
  // find max between two valley locations
  // and use an_ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2
  for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
    n_y_dc_max= -16777216 ;
    n_x_dc_max= -16777216;
    if (an_ir_valley_locs[k+1]-an_ir_valley_locs[k] >3){
        for (i=an_ir_valley_locs[k]; i< an_ir_valley_locs[k+1]; i++){
          if (an_x[i]> n_x_dc_max) {n_x_dc_max =an_x[i]; n_x_dc_max_idx=i;}
          if (an_y[i]> n_y_dc_max) {n_y_dc_max =an_y[i]; n_y_dc_max_idx=i;}
      }
      n_y_ac= (an_y[an_ir_valley_locs[k+1]] - an_y[an_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_ir_valley_locs[k]); //red
      n_y_ac=  an_y[an_ir_valley_locs[k]] + n_y_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k])  ;
      n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw
      n_x_ac= (an_x[an_ir_valley_locs[k+1]] - an_x[an_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_ir_valley_locs[k]); // ir
      n_x_ac=  an_x[an_ir_valley_locs[k]] + n_x_ac/ (an_ir_valley_locs[k+1] - an_ir_valley_locs[k]);
      n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw
      n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
      n_denom= ( n_x_ac *n_y_dc_max)>>7;
      if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
      {
        an_ratio[n_i_ratio_count]= (n_nume*100)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
        n_i_ratio_count++;
      }
    }
  }
  // choose median value since PPG signal may varies from beat to beat
  maxim_sort_ascend(an_ratio, n_i_ratio_count);
  n_middle_idx= n_i_ratio_count/2;

  if (n_middle_idx >1)
    n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
  else
    n_ratio_average = an_ratio[n_middle_idx ];

  if( n_ratio_average>2 && n_ratio_average <184){
    n_spo2_calc= uch_spo2_table[n_ratio_average] ;
    *pn_spo2 = n_spo2_calc ;
    *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
  }
  else{
    *pn_spo2 =  -999 ; // do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid  = 0;
  }
}


void maxim_find_peaks( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num )
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*
* \retval       None
*/
{
  maxim_peaks_above_min_height( pn_locs, n_npks, pn_x, n_size, n_min_height );
  maxim_remove_close_peaks( pn_locs, n_npks, pn_x, n_min_distance );
  if (*n_npks <= n_max_num)
    *n_npks = *n_npks;
  else
    *n_npks = n_max_num;
//  *n_npks = min( *n_npks, n_max_num );
}

void maxim_peaks_above_min_height( int32_t *pn_locs, int32_t *n_npks,  int32_t  *pn_x, int32_t n_size, int32_t n_min_height )
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
  int32_t i = 1, n_width;
  *n_npks = 0;

  while (i < n_size-1){
    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){      // find left edge of potential peaks
      n_width = 1;
      while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])  // find flat peaks
        n_width++;
      if (pn_x[i] > pn_x[i+n_width] && (*n_npks) < 15 ){      // find right edge of peaks
        pn_locs[(*n_npks)++] = i;
        // for flat peaks, peak location is left edge
        i += n_width+1;
      }
      else
        i += n_width;
    }
    else
      i++;
  }
}

void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{

  int32_t i, j, n_old_npks, n_dist;

  /* Order peaks from large to small */
  maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

  for ( i = -1; i < *pn_npks; i++ ){
    n_old_npks = *pn_npks;
    *pn_npks = i+1;
    for ( j = i+1; j < n_old_npks; j++ ){
      n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
      if ( n_dist > n_min_distance || n_dist < -n_min_distance )
        pn_locs[(*pn_npks)++] = pn_locs[j];
    }
  }

  // Resort indices int32_to ascending order
  maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_sort_ascend(int32_t  *pn_x, int32_t n_size)
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_x[i];
    for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
        pn_x[j] = pn_x[j-1];
    pn_x[j] = n_temp;
  }
}

void maxim_sort_indices_descend(  int32_t  *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/
{
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_indx[i];
    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
      pn_indx[j] = pn_indx[j-1];
    pn_indx[j] = n_temp;
  }
}


void SPO2_init()
{
//    Serial.begin(115200); // initialize serial communication at 115200 bits per second:
//
//   pinMode(pulseLED, OUTPUT);
//   pinMode(readLED, OUTPUT);

   // Initialize sensor
//   if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
//   {
//     Serial.println(F("MAX30105 was not found. Please check wiring/power."));
//     while (1);
//   }
  LOG_INFO("MAX30105_begin\r");
  MAX30105_begin();
//   Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
//   while (Serial.available() == 0) ; //wait until user presses a key
//   Serial.read();

   uint8_t ledBrightness = 60; //Options: 0=Off to 255=50mA
   uint8_t sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
   uint8_t ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
   uint8_t sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
   int pulseWidth = 411; //Options: 69, 118, 215, 411
   int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
   LOG_INFO("MAX30105_setup\r");
   // uint8_t powerLevel = 0x1F, uint8_t sampleAverage = 4, uint8_t ledMode = 3, int sampleRate = 400, int pulseWidth = 411, int adcRange = 4096
//   MAX30105_setup(0x1F, 4, 2, 400, 411, 4096);
   MAX30105_setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void SPO2_measure()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  uint8_t avg_bpm[250] = {0};
  uint8_t avg_spo2[250] = {0};
  uint8_t bpm_idx = 0;
  uint8_t spo2_idx = 0;

  int bpm_sum = 0;
  int spo2_sum = 0;

  //read the first 100 samples, and determine the signal range
  for (uint8_t i = 0 ; i < bufferLength ; i++)
  {
    while (MAX30105_available() == false) //do we have new data?
      check(); //Check the sensor for new data

    redBuffer[i] = MAX30105_getRed();
    irBuffer[i] = MAX30105_getIR();
    MAX30105_nextSample(); //We're finished with this sample so move to next sample

    LOG_INFO("red = %d\r", redBuffer[i]);

    LOG_INFO("ir = %d\r", irBuffer[i]);

  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  uint8_t count = 10;
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (count--)
  {
      LOG_INFO("=======================================COUNT %d\r\n",count);
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (uint8_t i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.
    for (uint8_t i = 75; i < 100; i++)
    {
      while (MAX30105_available() == false) //do we have new data?
        check(); //Check the sensor for new data

//      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = MAX30105_getRed();
      irBuffer[i] = MAX30105_getIR();
      MAX30105_nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      LOG_INFO("red = %d\r", redBuffer[i]);

      LOG_INFO("ir = %d\r", irBuffer[i]);

      LOG_INFO("HR = %d\r", heartRate);

      LOG_INFO("HRvalid = %d\r", validHeartRate);

      LOG_INFO("SPO2 = %d\r", spo2);

      LOG_INFO("SPO2Valid = %d\r\n", validSPO2);

      if (validHeartRate)
      {
          if(heartRate > 50 && heartRate < 150)
            {
              avg_bpm[bpm_idx++] = heartRate;
              bpm_sum += heartRate;
            }

      }
      if (validSPO2)
      {
          if(spo2 > 90 && spo2 < 105)
            {
              avg_spo2[spo2_idx++] = spo2;
              spo2_sum += spo2;
            }

      }




    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
  avg_bpm_val = bpm_sum/bpm_idx;
  LOG_INFO("====================== BPM = %d =================================\r", avg_bpm_val);

  avg_spo2_val = spo2_sum/spo2_idx;
  LOG_INFO("====================== SPO2 = %d =================================\r", avg_spo2_val);

}

uint32_t get_heart_rate()
{
  return avg_bpm_val;
}

uint32_t get_spo2()
{
  return avg_spo2_val;
}


