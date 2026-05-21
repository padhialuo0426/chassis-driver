#ifndef __TB6612_CHASSIS_H
#define __TB6612_CHASSIS_H

#include <stdint.h>
#include "pca9685.h"

#define STBY_PIN 2 // PCA9685 的 STBY 引脚连接到 GPIO2，高电平有效，控制电机工作/待机

// PWM 占空比有效范围上限 (PCA9685 为 12 位分辨率)
#define CHASSIS_SPEED_MAX 4095

// 电机运行状态枚举 (符合 TB6612 真值表)
typedef enum {
    MOTOR_COAST = 0, // 滑行停止 (IN1=0, IN2=0)
    MOTOR_FORWARD,   // 正转 (IN1=1, IN2=0) - 具体正反视实际电机接线而定
    MOTOR_REVERSE,   // 反转 (IN1=0, IN2=1)
    MOTOR_BRAKE      // 刹车 (IN1=1, IN2=1)
} motor_action_t;

// 单个电机结构体
typedef struct {
    motor_action_t action;
    uint16_t speed;        // 占空比 0~4095
} motor_state_t;

// 4驱底盘整体结构体
typedef struct {
    motor_state_t m1;
    motor_state_t m2;
    motor_state_t m3;
    motor_state_t m4;
} chassis_t;

// 底盘底层接口
void chassis_init(uint8_t dev_addr, float pwm_freq);
void chassis_enable(bool enable);
void chassis_update(const chassis_t *chassis);

#endif /* __TB6612_CHASSIS_H */