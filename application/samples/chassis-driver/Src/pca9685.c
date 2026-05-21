#include "pca9685.h"
#include "soc_osal.h"
#include "osal_debug.h"

void pca9685_enable(bool enable)
{
    if (enable) {
        uapi_gpio_set_val(OE_PIN, GPIO_LEVEL_LOW); // 使能PWM输出
    } else {
        uapi_gpio_set_val(OE_PIN, GPIO_LEVEL_HIGH); // 禁用PWM输出
    }
}

/**
 * @brief  向 PCA9685 写入单个寄存器
 */
void pca9685_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    uint8_t buffer[2] = {reg_addr, data};
    i2c_write(dev_addr, buffer, sizeof(buffer));
}

/**
 * @brief  从 PCA9685 读取单个寄存器
 */
uint8_t pca9685_read_reg(uint8_t dev_addr, uint8_t reg_addr)
{
    uint8_t data;
    i2c_writeread(dev_addr, &reg_addr, 1, &data, 1);
    return data;
}

/**
 * @brief  设置 PCA9685 的 PWM 输出频率
 * @param  freq_hz 目标频率，范围约 24Hz ~ 1526Hz
 */
void pca9685_set_pwm_freq(uint8_t dev_addr, float freq_hz)
{
    // 计算 Prescale 值: prescale_value = round(osc_clock / (4096 * update_rate)) - 1
    float prescaleval = PCA9685_OSC_CLOCK / (4096.0f * freq_hz);
    prescaleval -= 1.0f;
    uint8_t prescale = (uint8_t)(prescaleval + 0.5f); // 加上 0.5 实现四舍五入

    // prescale 范围保护：有效值为 3~255
    if (prescale < PCA9685_PRESCALE_MIN)
    {
        prescale = PCA9685_PRESCALE_MIN;
    }
    // uint8_t 不会超过 255，无需上限钳位

    // PRE_SCALE 寄存器只有在 MODE1 的 SLEEP 位设为 1 时才能被修改
    uint8_t oldmode = pca9685_read_reg(dev_addr, PCA9685_MODE1);

    // 进入睡眠模式 (清 RESTART 位，设 SLEEP 位)
    pca9685_write_reg(dev_addr, PCA9685_MODE1, (oldmode & 0x7F) | MODE1_SLEEP);

    // 设置预分频器
    pca9685_write_reg(dev_addr, PCA9685_PRESCALE, prescale);

    // 退出睡眠模式：先恢复不含 RESTART 的值，确保不会在振荡器未稳定时触发重启
    pca9685_write_reg(dev_addr, PCA9685_MODE1, oldmode & 0x7F);

    // 振荡器退出睡眠后需要至少 500μs 稳定，向上取整到 1ms
    osal_msleep(1);

    // 振荡器稳定后，写入 RESTART=1 恢复 PWM 输出，同时开启 Auto-Increment
    pca9685_write_reg(dev_addr, PCA9685_MODE1, (oldmode & 0x7F) | MODE1_RESTART | MODE1_AI);
}

/**
 * @brief  初始化 PCA9685
 */
void pca9685_init(uint8_t dev_addr, float freq_hz)
{
    // 配置 OE 控制引脚为 GPIO 输出，并设置初始状态为 PWM 使能
    uapi_pin_set_mode(OE_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(OE_PIN, GPIO_DIRECTION_OUTPUT);
    pca9685_enable(true);

    // 软复位：向通用广播地址 0x00 发送 0x06，将所有寄存器恢复默认值
    // 适用于热重启场景 (MCU 复位但 PCA9685 未断电，上次 PWM 状态残留)
    uint8_t swrst_cmd = 0x06;
    i2c_write(0x00, &swrst_cmd, 1);
    osal_msleep(1);

    // 初始化 MODE1：开启 Auto-Increment，ALLCALL 使能
    pca9685_write_reg(dev_addr, PCA9685_MODE1, MODE1_AI | MODE1_ALLCALL);
    
    // 配置 MODE2: 开启推挽输出 (OUTDRV=1)，这通常是驱动外部 LED 或电机驱动器的最佳配置
    pca9685_write_reg(dev_addr, PCA9685_MODE2, MODE2_OUTDRV);

    // 设定初始频率
    pca9685_set_pwm_freq(dev_addr, freq_hz);
}

/**
 * @brief  配置特定通道的 ON 和 OFF 时钟刻度 (0-4095)
 */
void pca9685_set_pwm(uint8_t dev_addr, uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel >= PCA9685_NUM_CHANNELS)
    {
        return;
    }

    // 每个通道由 4 个寄存器控制，LED0 的基址为 0x06
    uint8_t reg_base = PCA9685_LED0_ON_L + (4 * channel);
    uint8_t buffer[5];
    buffer[0] = reg_base;             // 起始寄存器地址
    buffer[1] = on & 0xFF;            // LEDn_ON_L
    buffer[2] = on >> 8;              // LEDn_ON_H
    buffer[3] = off & 0xFF;           // LEDn_OFF_L
    buffer[4] = off >> 8;             // LEDn_OFF_H
    i2c_write(dev_addr, buffer, sizeof(buffer));
}

/**
 * @brief  单维度占空比配置函数
 * @param  duty 占空比 0 (全关) 到 4095 (全开)
 */
void pca9685_set_duty(uint8_t dev_addr, uint8_t channel, uint16_t duty)
{
    if (channel >= PCA9685_NUM_CHANNELS)
    {
        return;
    }

    if (duty == 0) {
        // 完全关闭: LEDn_OFF_H 的 bit 4 设为 1
        pca9685_set_pwm(dev_addr, channel, 0, 4096);
    } else if (duty >= 4095) {
        // 完全开启: LEDn_ON_H 的 bit 4 设为 1
        pca9685_set_pwm(dev_addr, channel, 4096, 0);
    } else {
        // 常规 PWM: 从 0 时刻开始，在 duty 时刻关闭
        pca9685_set_pwm(dev_addr, channel, 0, duty);
    }
}

/**
 * @brief  批量更新连续多个通道的占空比 (利用 Auto-Increment 降低 I2C 开销)
 * @param  dev_addr      I2C设备地址
 * @param  start_channel 起始通道号 (0-15)
 * @param  duties        存储占空比数据的数组指针
 * @param  num_channels  需要连续更新的通道数量
 */
void pca9685_set_duties_burst(uint8_t dev_addr, uint8_t start_channel, uint16_t *duties, uint8_t num_channels)
{
    // 参数校验：空指针、通道范围越界
    if (duties == NULL || num_channels == 0)
    {
        return;
    }
    if (start_channel + num_channels > PCA9685_NUM_CHANNELS)
    {
        return;
    }

    // 固定大小缓冲区，避免 VLA 导致的栈溢出风险 (最大 16 通道 * 4 字节 + 1 字节地址 = 65 字节)
    uint8_t buffer[1 + 4 * PCA9685_NUM_CHANNELS];

    // 计算起始寄存器地址 (LED0_ON_L 为 0x06，每个通道占 4 字节)
    buffer[0] = PCA9685_LED0_ON_L + (4 * start_channel);

    for (uint8_t i = 0; i < num_channels; i++)
    {
        uint16_t duty = duties[i];
        uint16_t on = 0;
        uint16_t off = duty;

        // 处理全开和全关的特殊位逻辑
        if (duty == 0) {
            on = 0;
            off = 4096; // Bit 12 (即 LEDn_OFF_H 的 bit 4) 设为 1
        } else if (duty >= 4095) {
            on = 4096;  // Bit 12 (即 LEDn_ON_H 的 bit 4) 设为 1
            off = 0;
        }

        buffer[1 + i * 4] = on & 0xFF;         // LEDn_ON_L
        buffer[2 + i * 4] = on >> 8;           // LEDn_ON_H
        buffer[3 + i * 4] = off & 0xFF;        // LEDn_OFF_L
        buffer[4 + i * 4] = off >> 8;          // LEDn_OFF_H
    }

    errcode_t ret = i2c_write(dev_addr, buffer, 1 + 4 * num_channels);
    if (ret != ERRCODE_SUCC)
    {
        osal_printk("[PCA9685] burst write failed, err: 0x%X\n", ret);
    }
}