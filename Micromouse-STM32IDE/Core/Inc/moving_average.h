/*
 * moving_average.h
 *
 *  Created on: Jun 25, 2023
 *      Author: juans
 */

#include "main.h"

#ifndef INC_MOVING_AVERAGE_H_
#define INC_MOVING_AVERAGE_H_

#define MOVING_AVERAGE_LENGTH 100

typedef struct{
	double buffer[MOVING_AVERAGE_LENGTH];
	uint16_t counter;
	double out;
	double sum;
}mov_aver_inst;


void reset_average_filter(mov_aver_inst* instance);
void apply_average_filter(mov_aver_inst* instance, double input, double *out);

#endif /* INC_MOVING_AVERAGE_H_ */
