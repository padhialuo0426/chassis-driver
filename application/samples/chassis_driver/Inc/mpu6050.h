#ifndef __MPU6050_H
#define __MPU6050_H

#include <stdint.h>
#include <string.h>
#include "mpu6050_reg.h"
#include "i2c_hardware.h"

#define MPU6050_ADDR 0x68 // MPU6050 7位地址，I2C通信函数会自动处理读写位

void mpu6050_write_reg(uint8_t reg_addr, uint8_t data);
uint8_t mpu6050_read_reg(uint8_t reg_addr);
uint8_t mpu6050_get_id(void);
void mpu6050_init(void);
void mpu6050_get_data(int16_t *accel_data, int16_t *gyro_data, int16_t *temp_data);

#endif /* __MPU6050_H */