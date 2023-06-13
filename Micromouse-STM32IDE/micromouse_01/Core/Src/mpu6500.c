#include "mpu6500.h"
#include "main.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c2;

uint8_t data_buffer[14];
int16_t sensor_data[7];

void mpu6500_init(){
	  HAL_StatusTypeDef ret = HAL_I2C_IsDeviceReady(&hi2c2, (DEVICE_ADDRESS << 1) + 0, 1, 100);
	  if(ret == HAL_OK){
		  printf("MPU6500 connected!");
	  }
	  else{
		  printf("MPU6500 not connected, check connections.");
	  }

	  uint8_t temp_reg = FS_GYRO_500;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_GYRO, 1, &temp_reg, 1, 100); //+/- 500deg/s
	  if(ret == HAL_OK){
		  printf("Setting Gyro sensibility.");
	  }
	  else{
		  printf("Unable to write to register 27. Gyro sens failed.");
	  }

	  temp_reg = FS_ACC_4G;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_CONFIG_ACC, 1, &temp_reg, 1, 100); //+/- 500deg/s
	  if(ret == HAL_OK){
		  printf("Setting Accel sensibility.");
	  }
	  else{
		  printf("Unable to write to register 28. Accel sens failed.");
	  }

	  temp_reg = 0;
	  ret = HAL_I2C_Mem_Write(&hi2c2, (DEVICE_ADDRESS << 1) + 0, REG_USR_CTRL, 1, &temp_reg, 1, 100); //+/- 500deg/s
	  if(ret == HAL_OK){
		  printf("Exiting sleep mode.");
	  }
	  else{
		  printf("Unable to exit from sleep mode.");
	  }
}

/* Función que realiza la lectura   */
int16_t * mpu6500_read(){

	HAL_I2C_Mem_Read(&hi2c2, (DEVICE_ADDRESS << 1) + 1, REG_DATA, 1, data_buffer, 14, 100);

	sensor_data[0] = ((int16_t)data_buffer[0] << 8) + data_buffer[1]; //x_acc [0]
	sensor_data[1] = ((int16_t)data_buffer[2] << 8) + data_buffer[3]; //y_acc [1]
	sensor_data[2] = ((int16_t)data_buffer[4] << 8) + data_buffer[5]; //z_acc [2]

	sensor_data[3] = ((int16_t)data_buffer[6] << 8) + data_buffer[7]; //temp [3]

	sensor_data[4] = ((int16_t)data_buffer[8] << 8) + data_buffer[9]; //x_gyro [4]
	sensor_data[5] = ((int16_t)data_buffer[10] << 8) + data_buffer[11]; //y_gyro [5]
	sensor_data[6] = ((int16_t)data_buffer[12] << 8) + data_buffer[13]; //z_gyro [6]

	return sensor_data;
}
