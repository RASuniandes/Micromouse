/*
 * encoders.h
 */
#include "main.h"
#include "stdint.h"

#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

typedef struct{
	int16_t velocity;

	int64_t position;
	uint32_t last_counter_value;

	double velocity_rpm;
	double position_mm;

	double last_position_mm;

}encoder_instance;

void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim);
void reset_encoder(encoder_instance *encoder_value);

#endif /* INC_ENCODER_H_ */
