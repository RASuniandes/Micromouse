#include "stdint.h"

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#define DEVICE_ADDRESS 0x68
#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 16
#define FS_GYRO_2000 24

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 16
#define FS_ACC_16G 24

#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107
#define REG_DATA 59

void mpu6500_init();

int16_t * mpu6500_read();

#endif /* INC_MPU6500_H_ */
