/*
 * IRS.h
 *
 *  Created on: Jun 12, 2023
 *      Author: juans
 */

#ifndef INC_IRS_H_
#define INC_IRS_H_

// Number of samples to take
#define NUM_SAMPLES 128


//Enumeration for each IR Sensor
typedef enum
{
	IR_LEFT = 0,
	IR_FRONT_LEFT = 1,
	IR_FRONT_RIGHT = 2,
	IR_RIGHT = 3
}IR;


uint16_t readIR(IR ir);
uint16_t analogRead(IR ir);

#endif /* INC_IRS_H_ */
