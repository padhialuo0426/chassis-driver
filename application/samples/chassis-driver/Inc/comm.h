#ifndef __COMM_H
#define __COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "tb6612_chassis.h"

// ============================================================
// UART 配置
// ============================================================
#define COMM_UART_BUS_ID 2
#define COMM_UART_TXD_PIN 8
#define COMM_UART_RXD_PIN 7
#define COMM_UART_BAUDRATE 115200
#define COMM_UART_BUF_SIZE 512

// ============================================================
// I2C 编码器从机配置
// ============================================================
#define ENCODER_I2C_ADDR 0x30
#define ENCODER_NUM_MOTORS 4

// ============================================================
// 帧协议常量
// ============================================================
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0x55
#define FRAME_ESCAPE 0xA5
#define FRAME_MAX_PAYLOAD 32  // 上限考虑 encoder 帧 (type + 16B) = 17B
#define FRAME_PAYLOAD_LEN 10  // 下行帧固定载荷长度: start(1) + dir(1) + speed×4(8)

// 上行帧类型 (扩展时新增即可)
#define UPLINK_TYPE_MPU6050   0x01
#define UPLINK_TYPE_ENCODER   0x02
// #define UPLINK_TYPE_ATTITUDE  0x03   // 预留: 若 MCU 端做姿态解算后上传

// ============================================================
// 通信超时
// ============================================================
#define COMM_TIMEOUT_MS 500

// ============================================================
// 电机指令结构体 (从下行串口帧解码得到)
// ============================================================
typedef struct
{
    uint8_t start;     // 0=硬件急停, 1=正常运行
    uint8_t direction; // 4×2bit 方向编码
    uint16_t speed[4]; // M1~M4 各自的速度 (0~4095)
} motor_cmd_t;

// ============================================================
// 编码器数据结构体 (从 I2C 读取得到)
// ============================================================
typedef struct
{
    int16_t pulse[4]; // M1~M4 绝对计数值
    int16_t delta[4]; // M1~M4 增量值 (本次 - 上次)
} encoder_data_t;

// ============================================================
// MPU6050 原始数据结构体 (用于打包上行帧)
// ============================================================
typedef struct
{
    int16_t accel[3]; // X, Y, Z 加速度原始值
    int16_t gyro[3];  // X, Y, Z 陀螺仪原始值
    int16_t temp;     // 温度原始值
} mpu_data_t;

// ============================================================
// 对外接口
// ============================================================

/**
 * @brief  初始化通信模块 (UART 引脚/参数 + 内部状态清零)
 */
void comm_init(void);

/**
 * @brief  轮询 UART 并喂入状态机解析
 * @return true=解码出新帧，应立刻处理
 */
bool comm_uart_poll(void);

/**
 * @brief  是否收到了新的电机指令 (自上次调用以来)
 */
bool comm_has_new_cmd(void);

/**
 * @brief  获取最新的电机指令
 * @return 指向内部静态结构体的指针，下次解析成功后会被覆盖
 */
const motor_cmd_t *comm_get_cmd(void);

/**
 * @brief  将电机指令应用到底盘 (含 start=0 硬件急停处理)
 */
void comm_execute_cmd(const motor_cmd_t *cmd);

/**
 * @brief  通过 I2C 读取编码器数据并计算增量
 * @return true=读取成功, false=I2C 通信失败
 */
bool comm_read_encoders(encoder_data_t *data);

/**
 * @brief  检查通信是否超时，超时则触发硬件急停
 */
bool comm_check_timeout(void);

/**
 * @brief  将 MPU6050 数据打包成上行帧并通过 UART 发送
 * @note   type = 0x01, 14 字节载荷 (accel×3 + gyro×3 + temp，全部 int16 LE)
 */
void comm_send_mpu_frame(const mpu_data_t *mpu);

/**
 * @brief  将编码器数据打包成上行帧并通过 UART 发送
 * @note   type = 0x02, 16 字节载荷 (pulse×4 + delta×4，全部 int16 LE)
 *         pulse 用于香橙派做里程计积分；delta 是 MCU 端的固定周期增量，
 *         适合直接做速度估计，丢帧时优于香橙派自行减法计算
 */
void comm_send_encoder_frame(const encoder_data_t *enc);

#endif /* __COMM_H */