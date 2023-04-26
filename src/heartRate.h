/*
 * heartRate.h
 *
 *  Created on: Apr 21, 2023
 *      Author: sanis
 */

#ifndef SRC_HEARTRATE_H_
#define SRC_HEARTRATE_H_

#include <stdint.h>
#include <stdbool.h>


bool checkForBeat(int32_t sample);
int16_t averageDCEstimator(int32_t *p, uint16_t x);
int16_t lowPassFIRFilter(int16_t din);
int32_t mul16(int16_t x, int16_t y);
long meanDiff(int M);

#endif /* SRC_HEARTRATE_H_ */
