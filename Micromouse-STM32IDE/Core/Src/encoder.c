/*
 * encoders.c
 */

#include <encoder.h>
#include "main.h"

double wheel_circumference = 94.2478; //IN MM


void update_encoder(encoder_instance *encoder_value, TIM_HandleTypeDef *htim){
	uint32_t temp_counter = __HAL_TIM_GET_COUNTER(htim);
	static uint8_t first_time = 0;
	if(!first_time)
	{
		encoder_value -> velocity = 0;
		first_time = 1;
	}

	else
	{
		if(temp_counter == encoder_value -> last_counter_value)
		{
			encoder_value -> velocity = 0;
		}
		else if(temp_counter > encoder_value ->  last_counter_value)
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			{
				encoder_value -> velocity = -encoder_value -> last_counter_value -
						(__HAL_TIM_GET_AUTORELOAD(htim) - temp_counter);
			}
			else
			{
				encoder_value -> velocity = temp_counter -  encoder_value -> last_counter_value;
			}
		}
		else
		{
			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim)){
				encoder_value -> velocity = temp_counter - encoder_value -> last_counter_value;
			}
			else{
				encoder_value -> velocity = temp_counter +
						(__HAL_TIM_GET_AUTORELOAD(htim) - encoder_value -> last_counter_value);
			}
		}
	}

	encoder_value -> position += encoder_value -> velocity;
	encoder_value -> last_counter_value = temp_counter;

	//Medidas reales
	encoder_value -> position_mm = (double)(encoder_value -> position) / ( (double) 360.0/ wheel_circumference) ;
	encoder_value -> velocity_rpm = (encoder_value -> velocity) * (1000.0/(12.0*29.86))*60.0;
	encoder_value -> last_position_mm = (double)(encoder_value -> last_counter_value) / ( (double) 360.0/ wheel_circumference) ;

}
/*
 * This function has already been implemented for you. Enjoy! :)
 */
void reset_encoder(encoder_instance *encoder_value) {
	encoder_value -> velocity = 0;
	encoder_value -> position = 0;
	encoder_value -> last_counter_value = 0;
	encoder_value -> velocity_rpm = 0;
	encoder_value -> position_mm = 0;
	encoder_value -> last_position_mm = 0;
}
