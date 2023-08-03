#include "mpu6500.h"
#include "main.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c2;

uint8_t data_buffer[14];
float sensor_data[7];

void mpu6500_init(){

	  //IS ON
	  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, (DEVICE_ADDRESS << 1) + 0, 1, 100);
	  if(ret == HAL_OK){
		  printf("MPU6500 connected!");
	  }
	  else{
		  printf("MPU6500 not connected, check connections.");
	  }


	  //WAKE UP!
	  uint8_t temp_reg = 0;
	  HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, PWR_MGMT_1_REG, 1,&temp_reg, 1, 1000);


	  // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	  temp_reg = 0x07;
	  HAL_I2C_Mem_Write(&hi2c2,  (DEVICE_ADDRESS << 1) + 0, SMPLRT_DIV , 1, &temp_reg, 1, 1000);


	  // GYRO SENS
	  temp_reg = FS_GYRO_250;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_GYRO, 1, &temp_reg, 1, 100); //+/- 500deg/s
	  if(ret == HAL_OK){
		  printf("Setting Gyro sensibility.");
	  }
	  else{
		  printf("Unable to write to register 27. Gyro sens failed.");
	  }


	  // ACC SENS
	  temp_reg = FS_ACC_2G;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_ACC, 1, &temp_reg, 1, 100); //+/- 500deg/s
	  if(ret == HAL_OK){
		  printf("Setting Accel sensibility.");
	  }
	  else{
		  printf("Unable to write to register 28. Accel sens failed.");
	  }
  }


/* Función que realiza la lectura   */
int16_t * mpu6500_read(){

	HAL_I2C_Mem_Read (&hi2c2, (DEVICE_ADDRESS << 1) + 1, GYRO_XOUT_H_REG, 1, data_buffer, 6, 1000);

	int16_t Accel_X_RAW = (int16_t)(data_buffer[0] << 8 | data_buffer [1]);
	int16_t Accel_Y_RAW = (int16_t)(data_buffer[2] << 8 | data_buffer [3]);
	int16_t Accel_Z_RAW = (int16_t)(data_buffer[4] << 8 | data_buffer [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	sensor_data[0] = Accel_X_RAW/16384.0;  // get the float g
	sensor_data[1] = Accel_Y_RAW/16384.0;
	sensor_data[2]= Accel_Z_RAW/16384.0;



	HAL_I2C_Mem_Read (&hi2c2, (DEVICE_ADDRESS << 1) + 1, GYRO_XOUT_H_REG, 1, data_buffer, 6, 1000);

	int16_t Gyro_X_RAW = (int16_t)(data_buffer[0] << 8 | data_buffer [1]);
	int16_t Gyro_Y_RAW = (int16_t)(data_buffer[2] << 8 | data_buffer [3]);
	int16_t Gyro_Z_RAW = (int16_t)(data_buffer[4] << 8 | data_buffer [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	sensor_data[4] = Gyro_X_RAW/131.0;
	sensor_data[5] = Gyro_Y_RAW/131.0;
	sensor_data[6] = Gyro_Z_RAW/131.0;



	return sensor_data;
}
