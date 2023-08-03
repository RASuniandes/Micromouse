#include "moving_average.h"
#include "main.h"

void reset_average_filter(mov_aver_inst* instance){
	instance -> counter = 0;
	instance -> sum = 0;
	instance -> out = 0;

	for(int i = 0; i < MOVING_AVERAGE_LENGTH; i++){
		instance -> buffer[i] = 0;
	}
}
void apply_average_filter(mov_aver_inst* instance, double input, double *out){
	instance -> sum += input - instance->buffer[instance -> counter];
	instance -> buffer[instance -> counter] = input;
	instance -> counter++;

	if(instance -> counter == MOVING_AVERAGE_LENGTH){
		instance -> counter = 0;
	}

	instance -> out = instance -> sum / MOVING_AVERAGE_LENGTH;

	*out = instance -> out;
}
