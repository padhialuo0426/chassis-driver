#include "tb6612_chassis.h"

// 模块内部记录设备地址，消除外部重复硬编码
static uint8_t g_dev_addr = PCA9685_DEFAULT_ADDR;

/**
 * @brief 内部辅助函数：根据动作解析 IN1 和 IN2 的值
 */
static void parse_motor_action(motor_action_t action, uint16_t *in1, uint16_t *in2)
{
    switch (action)
    {
    case MOTOR_FORWARD:
        *in1 = 4096; // 满占空比即高电平
        *in2 = 0;    // 0占空比即低电平
        break;
    case MOTOR_REVERSE:
        *in1 = 0;
        *in2 = 4096;
        break;
    case MOTOR_BRAKE:
        *in1 = 4096;
        *in2 = 4096;
        break;
    case MOTOR_COAST:
    default:
        *in1 = 0;
        *in2 = 0;
        break;
    }
}

/**
 * @brief  speed 钳位，防止超出 PCA9685 有效范围
 */
static uint16_t clamp_speed(uint16_t speed)
{
    return (speed > CHASSIS_SPEED_MAX) ? CHASSIS_SPEED_MAX : speed;
}

/**
 * @brief  初始化底盘 (底层调用 PCA9685 初始化)
 */
void chassis_init(uint8_t dev_addr, float pwm_freq)
{
    g_dev_addr = dev_addr;

    // 配置STBY控制引脚为 GPIO 输出，并设置初始状态为非待机
    uapi_pin_set_mode(STBY_PIN, PIN_MODE_0);
    uapi_gpio_set_dir(STBY_PIN, GPIO_DIRECTION_OUTPUT);

    chassis_enable(true); // 默认进入工作状态

    // 初始化 PCA9685，设置 PWM 频率
    pca9685_init(dev_addr, pwm_freq);

    // 初始化时显式设置所有电机为滑行停止状态
    chassis_t stop_state = {
        .m1 = {MOTOR_COAST, 0},
        .m2 = {MOTOR_COAST, 0},
        .m3 = {MOTOR_COAST, 0},
        .m4 = {MOTOR_COAST, 0}};
    chassis_update(&stop_state);
}

/**
 * @brief  使能底盘 (通过控制 STBY 引脚)
 */
void chassis_enable(bool enable)
{
    if (enable)
    {
        uapi_gpio_set_val(STBY_PIN, GPIO_LEVEL_HIGH); // 退出待机
    }
    else
    {
        uapi_gpio_set_val(STBY_PIN, GPIO_LEVEL_LOW); // 进入待机
    }
}

/**
 * @brief  一次性更新底盘所有 4 个电机的状态 (使用 I2C 突发写入)
 * @note   硬件映射关系:
 * M1: PWM=CH0, IN1=CH4, IN2=CH8
 * M2: PWM=CH1, IN1=CH5, IN2=CH9
 * M3: PWM=CH2, IN1=CH6, IN2=CH10
 * M4: PWM=CH3, IN1=CH7, IN2=CH11
 */
void chassis_update(const chassis_t *chassis)
{
    if (chassis == NULL)
    {
        return;
    }

    // 需要连续更新 CH0 到 CH11，共 12 个通道
    uint16_t duties[12] = {0};

    // 1. 填充 PWM 速度信号 (CH0 ~ CH3)，钳位到有效范围
    duties[0] = clamp_speed(chassis->m1.speed);
    duties[1] = clamp_speed(chassis->m2.speed);
    duties[2] = clamp_speed(chassis->m3.speed);
    duties[3] = clamp_speed(chassis->m4.speed);

    // 2. 填充方向控制信号 (CH4 ~ CH11)
    parse_motor_action(chassis->m1.action, &duties[4], &duties[8]);
    parse_motor_action(chassis->m2.action, &duties[9], &duties[5]);
    parse_motor_action(chassis->m3.action, &duties[10], &duties[6]);
    parse_motor_action(chassis->m4.action, &duties[7], &duties[11]);

    // 3. 调用突发写入函数，一次 I2C 通信完成所有配置
    pca9685_set_duties_burst(g_dev_addr, 0, duties, 12);
}
