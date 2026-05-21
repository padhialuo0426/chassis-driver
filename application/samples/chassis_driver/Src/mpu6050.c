#include "mpu6050.h"
#include "soc_osal.h"
#include "osal_debug.h"


void mpu6050_write_reg(uint8_t reg_addr, uint8_t data)
{
    // 硬件 I2C 连续写：第一个字节发寄存器地址，第二个字节发数据
    uint8_t tx_buf[2] = {reg_addr, data};
    i2c_write(MPU6050_ADDR, tx_buf, 2);
}

/**
 * @brief 读取 MPU6050 寄存器数据
 * @param reg_addr 寄存器地址
 * @return 寄存器数据
 */
uint8_t mpu6050_read_reg(uint8_t reg_addr)
{
    uint8_t rx_data = 0;
    i2c_writeread(MPU6050_ADDR, &reg_addr, 1, &rx_data, 1);
    return rx_data;
}

/**
 * @brief 获取 MPU6050 设备ID
 * @return 设备ID，正常应为0x68
 */
uint8_t mpu6050_get_id(void)
{
    return mpu6050_read_reg(MPU6050_WHO_AM_I);
}

/**
 * @brief 初始化 MPU6050 传感器
 * 包括复位、配置采样率、量程、滤波等参数
 */
void mpu6050_init(void)
{
    // 硬复位：写入 bit7 触发内部复位，所有寄存器恢复默认值
    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x80);
    osal_msleep(100); // 等待复位完成

    // 校验设备身份，确认 I2C 通信正常
    uint8_t id = mpu6050_get_id();
    if (id != 0x68)
    {
        osal_printk("MPU6050 ID mismatch: expected 0x68, got 0x%02X\n", id);
        return;
    }

    mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x01);   // 解除休眠，使用X轴陀螺仪参考时钟
    mpu6050_write_reg(MPU6050_PWR_MGMT_2, 0x00);   // 使能所有传感器
    mpu6050_write_reg(MPU6050_SMPLRT_DIV, 0x09);   // 设置采样率为 125Hz (8ms周期)
    mpu6050_write_reg(MPU6050_CONFIG, 0x06);       // 设置低通滤波器为5Hz
    mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x18);  // 陀螺仪量程为2000°/s
    mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x18); // 加速度计量程为±16g (AFS_SEL=3)

    // 0x12 = 0001 0010: INT_RD_CLEAR=1 (读任意寄存器清除中断), I2C_BYPASS_EN=1
    // INT引脚高电平有效，推挽输出
    mpu6050_write_reg(MPU6050_INT_PIN_CFG, 0x12);

    // 0x01 = 0000 0001: 开启 Data Ready中断，当新的传感器数据准备好时，INT引脚会被拉高
    mpu6050_write_reg(MPU6050_INT_ENABLE, 0x01);
}

void mpu6050_get_data(int16_t *accel_data, int16_t *gyro_data, int16_t *temp_data)
{
    uint8_t buffer[14] = {0};
    uint8_t start_reg = MPU6050_ACCEL_XOUT_H;

    i2c_writeread(MPU6050_ADDR, &start_reg, 1, buffer, 14);

    accel_data[0] = (buffer[0] << 8) | buffer[1];  // 加速度X轴
    accel_data[1] = (buffer[2] << 8) | buffer[3];  // 加速度Y轴
    accel_data[2] = (buffer[4] << 8) | buffer[5];  // 加速度Z轴
    temp_data[0] = (buffer[6] << 8) | buffer[7];   // 温度
    gyro_data[0] = (buffer[8] << 8) | buffer[9];   // 陀螺仪X轴
    gyro_data[1] = (buffer[10] << 8) | buffer[11]; // 陀螺仪Y轴
    gyro_data[2] = (buffer[12] << 8) | buffer[13]; // 陀螺仪Z轴
}