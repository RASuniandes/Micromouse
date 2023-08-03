/*
 * systick.c
 */

#include "main.h"
//#include "pid.h"
#include <stdio.h>
#include "encoder.h"
#include "pid.h"
#include "moving_average.h"
#include "motors.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim8;
extern encoder_instance enc_right;
extern encoder_instance enc_left;

extern mov_aver_inst filter_speedMR;
extern mov_aver_inst filter_speedML;

extern double vel_rightM_RPM;
extern double vel_leftM_RPM;

extern PID_TypeDef speedMR_PID;
extern PID_TypeDef speedML_PID;
extern PIDCD_TypeDef speedDifference_PID;

extern double speedMR_output;
extern double speedML_output;
extern double speedDifference_output;

extern double speedDifference;

extern uint8_t enable_motors;

#define MAX_CAL_DISTANCE 1000
double PI = 3.14159265358979323846;


void SysTickFunction(void) {
	/*
	 * Anything in this function body will be executed every millisecond.
	 * Call you PID update function here.
	 */


	//Medir la velocidad y la posición
	update_encoder(&enc_right, &htim1);
	update_encoder(&enc_left, &htim8);

	// Aplicar el filtro
	apply_average_filter(&filter_speedMR, enc_right.velocity_rpm, &vel_rightM_RPM);
	apply_average_filter(&filter_speedML, enc_left.velocity_rpm, &vel_leftM_RPM);

	//Encontrar diferencia
	speedDifference = vel_rightM_RPM - vel_leftM_RPM;


	if(enable_motors == 1
			&& HAL_GetTick() > 10000){
		 // PID apply
		 PID_Compute(&speedMR_PID);
		 PID_Compute(&speedML_PID);
		 PID_Compute(&speedDifference_PID);


		 //speedMR_output = speedMR_output * 0.015 * ((2*PI)/60);
		 //speedML_output = speedML_output * 0.015 * ((2*PI)/60);

		 double pwm_MR = (speedMR_output);
		 double pwm_ML =(speedML_output);




		 setMotorRPWM(pwm_MR);
		 setMotorLPWM(pwm_ML);
	}
	else{
		 setMotorRPWM(0);
		 setMotorLPWM(0);
	}







}
