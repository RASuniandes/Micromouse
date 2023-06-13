#include "vl6180x.h"
#include "main.h"
#include "stdio.h"

extern I2C_HandleTypeDef hi2c2;

void vl6180_init(){
	uint8_t param = 0x01;

	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x207, 1, &param, 1, 100);
	param = 0x01;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x208, 1, &param, 1, 100);
	param = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x096, 1, &param, 1, 100);
	param = 0xFD;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x097, 1, &param, 1, 100);
	param = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0E3, 1, &param, 1, 100);
	param = 0x04;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0E4, 1, &param, 1, 100);
	param = 0x02;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0E5, 1, &param, 1, 100);
	param = 0x01;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0E6, 1, &param, 1, 100);
	param = 0x03;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0E7, 1, &param, 1, 100);
	param = 0x02;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0F5, 1, &param, 1, 100);
	param = 0x05;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0D9, 1, &param, 1, 100);
	param = 0xCE;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0DB, 1, &param, 1, 100);
	param = 0x03;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0DC, 1, &param, 1, 100);
	param = 0xF8;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0DD, 1, &param, 1, 100);
	param = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x09F, 1, &param, 1, 100);
	param = 0x3C;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0A3, 1, &param, 1, 100);
	param = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0B7, 1, &param, 1, 100);
	param = 0x3C;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0BB, 1, &param, 1, 100);
	param = 0x09;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0B2, 1, &param, 1, 100);
	param = 0x09;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0CA, 1, &param, 1, 100);
	param = 0x01;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x198, 1, &param, 1, 100);
	param = 0x17;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x1B0, 1, &param, 1, 100);
	param = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x1AD, 1, &param, 1, 100);
	param = 0x05;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x0FF, 1, &param, 1, 100);
	param = 0x05;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x100, 1, &param, 1, 100);
	param = 0x05;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x199, 1, &param, 1, 100);
	param = 0x1B;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x1A6, 1, &param, 1, 100);
	param = 0x3E;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x1AC, 1, &param, 1, 100);
	param = 0x1F;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x1A7, 1, &param, 1, 100);
	param = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, VL_ADDRESS, 0x030, 1, &param, 1, 100);


}


