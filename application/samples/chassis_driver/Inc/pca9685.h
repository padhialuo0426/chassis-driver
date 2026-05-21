#ifndef __PCA9685_H
#define __PCA9685_H

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "i2c_hardware.h"
#include "pinctrl.h"
#include "gpio.h"

// PCA9685 的 OE 引脚连接到 GPIO1，低电平有效，控制PWM输出使能/禁用
#define OE_PIN 1

// 默认 I2C 从机地址 (A0-A5接地时为 0x40)
#define PCA9685_DEFAULT_ADDR 0x40

// 寄存器地址定义
#define PCA9685_MODE1        0x00
#define PCA9685_MODE2        0x01
#define PCA9685_SUBADR1      0x02
#define PCA9685_SUBADR2      0x03
#define PCA9685_SUBADR3      0x04
#define PCA9685_ALLCALLADR   0x05
#define PCA9685_LED0_ON_L    0x06
#define PCA9685_PRESCALE     0xFE

// 模式控制位定义
#define MODE1_RESTART        0x80
#define MODE1_EXTCLK         0x40
#define MODE1_AI             0x20  // Auto-Increment
#define MODE1_SLEEP          0x10
#define MODE1_ALLCALL        0x01

#define MODE2_INVRT          0x10
#define MODE2_OUTDRV         0x04

// 通道数量
#define PCA9685_NUM_CHANNELS 16  // PCA9685 共 16 个 PWM 通道 (0-15)

// 频率设置相关
#define PCA9685_OSC_CLOCK    25000000.0f  // 内部振荡器时钟 25MHz
#define PCA9685_PRESCALE_MIN 3            // 对应约 1526Hz
#define PCA9685_PRESCALE_MAX 255          // 对应约 24Hz

// API 声明
void pca9685_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t pca9685_read_reg(uint8_t dev_addr, uint8_t reg_addr);

void pca9685_init(uint8_t dev_addr, float freq_hz);
void pca9685_enable(bool enable);
void pca9685_set_pwm_freq(uint8_t dev_addr, float freq_hz);
void pca9685_set_pwm(uint8_t dev_addr, uint8_t channel, uint16_t on, uint16_t off);
void pca9685_set_duty(uint8_t dev_addr, uint8_t channel, uint16_t duty);
void pca9685_set_duties_burst(uint8_t dev_addr, uint8_t start_channel, uint16_t *duties, uint8_t num_channels);

#endif /* __PCA9685_H */