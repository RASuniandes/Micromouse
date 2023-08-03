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
#define REG_USR_CTRL 108
#define REG_DATA 59
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV  0x19

#define ACCEL_XOUT_H_REG 0x3B
#define GYRO_XOUT_H_REG 0x43

void mpu6500_init();

int16_t * mpu6500_read();

#endif /* INC_MPU6500_H_ */
