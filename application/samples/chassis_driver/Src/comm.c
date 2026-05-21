#include "comm.h"
#include "pinctrl.h"
#include "uart.h"
#include "i2c_hardware.h"
#include "osal_debug.h"
#include "soc_osal.h"
#include "cmsis_os2.h"
#include <string.h>

// ============================================================
// UART 引脚复用模式
// ============================================================
#define UART_PIN_MUX_MODE PIN_MODE_2

// ============================================================
// 帧解析状态机
// ============================================================
typedef enum
{
    STATE_WAIT_HEAD1, // 等待 0xAA
    STATE_WAIT_HEAD2, // 等待 0x55
    STATE_RECV_LEN,   // 接收载荷长度
    STATE_RECV_DATA   // 接收转义后的数据 (载荷 + checksum)
} frame_state_t;

// ============================================================
// 模块内部状态 (全部 static，对外不可见)
// ============================================================

// UART 接收缓冲
static uint8_t g_uart_rx_buff[COMM_UART_BUF_SIZE] = {0};
static uart_buffer_config_t g_uart_buf_cfg = {
    .rx_buffer = g_uart_rx_buff,
    .rx_buffer_size = COMM_UART_BUF_SIZE};

// 帧解析状态
static frame_state_t g_frame_state = STATE_WAIT_HEAD1;
static uint8_t g_raw_buf[FRAME_MAX_PAYLOAD + 1]; // 载荷 + checksum
static uint8_t g_raw_index = 0;
static uint8_t g_expected_len = 0; // 转义前的目标长度 (载荷 + 1 字节 checksum)
static bool g_escape_flag = false; // 上一字节是否为转义标记

// 解码结果
static motor_cmd_t g_motor_cmd = {0};
static bool g_new_cmd_flag = false;

// 通信超时
static uint32_t g_last_valid_tick = 0;

// 编码器增量计算
static int16_t g_last_pulse[4] = {0};
static bool g_encoder_first_read = true;

// ============================================================
// UART 硬件初始化 (内部调用)
// ============================================================
static void uart_init_pin(void)
{
    uapi_pin_set_mode(COMM_UART_TXD_PIN, UART_PIN_MUX_MODE);
    uapi_pin_set_mode(COMM_UART_RXD_PIN, UART_PIN_MUX_MODE);
}

static void uart_init_config(void)
{
    uart_attr_t attr = {
        .baud_rate = COMM_UART_BAUDRATE,
        .data_bits = UART_DATA_BIT_8,
        .stop_bits = UART_STOP_BIT_1,
        .parity = UART_PARITY_NONE,
        .flow_ctrl = UART_FLOW_CTRL_NONE};

    uart_pin_config_t pin_cfg = {
        .tx_pin = COMM_UART_TXD_PIN,
        .rx_pin = COMM_UART_RXD_PIN,
        .cts_pin = PIN_NONE,
        .rts_pin = PIN_NONE};

    uapi_uart_deinit(COMM_UART_BUS_ID);
    uapi_uart_init(COMM_UART_BUS_ID, &pin_cfg, &attr, NULL, &g_uart_buf_cfg);
}

// ============================================================
// 帧解码：校验通过后提取电机指令
// ============================================================
static void frame_decode(const uint8_t *buf, uint8_t payload_len)
{
    // 校验和验证：buf[0..payload_len-1] 的累加和应等于 buf[payload_len]
    uint8_t calc_sum = 0;
    for (uint8_t i = 0; i < payload_len; i++)
    {
        calc_sum += buf[i];
    }
    if (calc_sum != buf[payload_len])
    {
        osal_printk("[COMM] Checksum failed: calc=0x%02X, rx=0x%02X\n",
                    calc_sum, buf[payload_len]);
        return;
    }

    // 提取字段
    g_motor_cmd.start = buf[0];
    g_motor_cmd.direction = buf[1];
    g_motor_cmd.speed[0] = (uint16_t)(buf[2] | (buf[3] << 8)); // M1 (LE)
    g_motor_cmd.speed[1] = (uint16_t)(buf[4] | (buf[5] << 8)); // M2
    g_motor_cmd.speed[2] = (uint16_t)(buf[6] | (buf[7] << 8)); // M3
    g_motor_cmd.speed[3] = (uint16_t)(buf[8] | (buf[9] << 8)); // M4

    g_new_cmd_flag = true;
    g_last_valid_tick = osKernelGetTickCount();
}

// ============================================================
// 帧解析状态机：逐字节喂入
// ============================================================
static void frame_parse_byte(uint8_t ch)
{
    switch (g_frame_state)
    {
    case STATE_WAIT_HEAD1:
        if (ch == FRAME_HEADER_1)
            g_frame_state = STATE_WAIT_HEAD2;
        break;

    case STATE_WAIT_HEAD2:
        if (ch == FRAME_HEADER_2)
        {
            g_frame_state = STATE_RECV_LEN;
        }
        else if (ch != FRAME_HEADER_1)
        {
            g_frame_state = STATE_WAIT_HEAD1;
        }
        // ch == 0xAA: 连续帧头，保持等待 0x55
        break;

    case STATE_RECV_LEN:
        if (ch == 0 || ch > FRAME_MAX_PAYLOAD)
        {
            g_frame_state = STATE_WAIT_HEAD1;
        }
        else
        {
            g_expected_len = ch + 1; // 载荷 + checksum
            g_raw_index = 0;
            g_escape_flag = false;
            g_frame_state = STATE_RECV_DATA;
        }
        break;

    case STATE_RECV_DATA:
        // 数据流中出现 0xAA 说明前一帧被截断，新帧开始
        if (ch == FRAME_HEADER_1 && !g_escape_flag)
        {
            g_frame_state = STATE_WAIT_HEAD2;
            break;
        }

        if (g_escape_flag)
        {
            // 还原转义字节
            switch (ch)
            {
            case 0x01:
                ch = 0xAA;
                break;
            case 0x02:
                ch = 0x55;
                break;
            case 0x03:
                ch = 0xA5;
                break;
            default:
                // 非法转义序列，丢弃整帧
                g_frame_state = STATE_WAIT_HEAD1;
                return;
            }
            g_escape_flag = false;
        }
        else if (ch == FRAME_ESCAPE)
        {
            g_escape_flag = true;
            return; // 转义标记本身不存入缓冲区
        }

        g_raw_buf[g_raw_index++] = ch;

        if (g_raw_index >= g_expected_len)
        {
            frame_decode(g_raw_buf, g_expected_len - 1);
            g_frame_state = STATE_WAIT_HEAD1;
        }
        break;
    }
}

// ============================================================
// 上行帧编码：转义并填充单字节
// ============================================================
static inline uint8_t escape_byte(uint8_t *buf, uint8_t idx, uint8_t byte)
{
    switch (byte)
    {
    case FRAME_HEADER_1:
        buf[idx++] = FRAME_ESCAPE;
        buf[idx++] = 0x01;
        break;
    case FRAME_HEADER_2:
        buf[idx++] = FRAME_ESCAPE;
        buf[idx++] = 0x02;
        break;
    case FRAME_ESCAPE:
        buf[idx++] = FRAME_ESCAPE;
        buf[idx++] = 0x03;
        break;
    default:
        buf[idx++] = byte;
        break;
    }
    return idx;
}

// ============================================================
// 上行帧通用编码 + 发送
// 帧结构: [0xAA][0x55][LEN] [escaped(type + data + checksum)]
// LEN 为转义前载荷长度 (type + data 总字节数，不含 checksum)
// ============================================================
static void comm_send_frame(uint8_t type, const uint8_t *data, uint8_t data_len)
{
    if (data == NULL || (uint16_t)(data_len + 1) > FRAME_MAX_PAYLOAD)
    {
        return;
    }

    uint8_t payload_len = 1 + data_len; // type + data

    // 1. 计算 checksum (覆盖 type + data)
    uint8_t checksum = type;
    for (uint8_t i = 0; i < data_len; i++)
    {
        checksum += data[i];
    }

    // 2. 最坏情况发送长度 = 3(头+LEN) + 2 × (payload_len + 1 checksum)
    //    FRAME_MAX_PAYLOAD = 32 时上限 = 3 + 2*33 = 69
    uint8_t tx_buf[3 + 2 * (FRAME_MAX_PAYLOAD + 1)];
    uint8_t idx = 0;

    // 帧头不参与转义
    tx_buf[idx++] = FRAME_HEADER_1;
    tx_buf[idx++] = FRAME_HEADER_2;
    tx_buf[idx++] = payload_len;

    // 3. 转义并填充 type + data + checksum
    idx = escape_byte(tx_buf, idx, type);
    for (uint8_t i = 0; i < data_len; i++)
    {
        idx = escape_byte(tx_buf, idx, data[i]);
    }
    idx = escape_byte(tx_buf, idx, checksum);

    // 4. 阻塞写入 UART
    uapi_uart_write(COMM_UART_BUS_ID, tx_buf, idx, 0);
}

// ============================================================
// 对外接口实现
// ============================================================

void comm_init(void)
{
    // UART 初始化
    uart_init_pin();
    uart_init_config();

    // 清空 UART 缓冲区中的残留数据，确保状态机从干净状态开始
    uint8_t discard;
    while (uapi_uart_read(COMM_UART_BUS_ID, &discard, 1, 0) == 1)
    {
        // 丢弃所有已缓存的字节
    }

    // 状态清零
    g_frame_state = STATE_WAIT_HEAD1;
    g_raw_index = 0;
    g_escape_flag = false;
    g_new_cmd_flag = false;
    g_encoder_first_read = true;
    g_last_valid_tick = osKernelGetTickCount();

    memset(&g_motor_cmd, 0, sizeof(g_motor_cmd));
    memset(g_last_pulse, 0, sizeof(g_last_pulse));

    osal_printk("[COMM] Initialized. UART bus %d, Encoder I2C addr 0x%02X\n",
                COMM_UART_BUS_ID, ENCODER_I2C_ADDR);
}

bool comm_uart_poll(void)
{
    uint8_t ch;
    while (uapi_uart_read(COMM_UART_BUS_ID, &ch, 1, 0) == 1)
    {
        frame_parse_byte(ch);

        if (g_new_cmd_flag)
        {
            return true; // 有新帧，立刻返回
        }
    }
    return false; // 缓冲区已空，无新帧
}

bool comm_has_new_cmd(void)
{
    return g_new_cmd_flag;
}

const motor_cmd_t *comm_get_cmd(void)
{
    g_new_cmd_flag = false;
    return &g_motor_cmd;
}

void comm_execute_cmd(const motor_cmd_t *cmd)
{
    if (cmd == NULL)
        return;

    // start=0：硬件级急停，不依赖 I2C 通信
    if (cmd->start == 0)
    {
        pca9685_enable(false); // OE → HIGH，PWM 禁用
        chassis_enable(false); // STBY → LOW，TB6612 待机
        return;
    }

    // start=1：确保硬件处于工作状态
    pca9685_enable(true);
    chassis_enable(true);

    // 位运算提取方向，直接映射枚举值
    chassis_t car = {
        .m1 = {.action = (motor_action_t)((cmd->direction >> 0) & 0x03), .speed = cmd->speed[0]},
        .m2 = {.action = (motor_action_t)((cmd->direction >> 2) & 0x03), .speed = cmd->speed[1]},
        .m3 = {.action = (motor_action_t)((cmd->direction >> 4) & 0x03), .speed = cmd->speed[2]},
        .m4 = {.action = (motor_action_t)((cmd->direction >> 6) & 0x03), .speed = cmd->speed[3]}};

    chassis_update(&car);
}

bool comm_read_encoders(encoder_data_t *data)
{
    if (data == NULL)
        return false;

    uint8_t rx_buf[8] = {0};

    // 从 STM32 从机读取 8 字节 (4 × int16_t)
    errcode_t ret = i2c_read(ENCODER_I2C_ADDR, rx_buf, sizeof(rx_buf));
    if (ret != ERRCODE_SUCC)
    {
        osal_printk("[COMM] Encoder I2C read failed, err: 0x%X\n", ret);
        return false;
    }

    // 安全地还原 4 个 int16_t (避免对齐问题)
    for (uint8_t i = 0; i < ENCODER_NUM_MOTORS; i++)
    {
        int16_t pulse;
        memcpy(&pulse, &rx_buf[i * 2], sizeof(int16_t));
        data->pulse[i] = pulse;

        // 利用 int16_t 补码减法自动处理计数器溢出
        if (g_encoder_first_read)
        {
            data->delta[i] = 0;
        }
        else
        {
            data->delta[i] = pulse - g_last_pulse[i];
        }

        g_last_pulse[i] = pulse;
    }

    g_encoder_first_read = false;
    return true;
}

bool comm_check_timeout(void)
{
    uint32_t now = osKernelGetTickCount();

    if ((now - g_last_valid_tick) > COMM_TIMEOUT_MS)
    {
        // 通信超时，硬件级急停
        pca9685_enable(false);
        chassis_enable(false);
        return true;
    }

    return false;
}

// ============================================================
// MPU6050 上行帧发送 (type = 0x01)
// 载荷布局 (14 字节):
//   [0-1]   accel_x   int16 LE
//   [2-3]   accel_y
//   [4-5]   accel_z
//   [6-7]   gyro_x
//   [8-9]   gyro_y
//   [10-11] gyro_z
//   [12-13] temp
// ============================================================
void comm_send_mpu_frame(const mpu_data_t *mpu)
{
    if (mpu == NULL)
        return;

    // ARM Cortex 默认小端序，memcpy 直接得到 LE 字节序
    uint8_t data[14];
    memcpy(&data[0], &mpu->accel[0], 2);
    memcpy(&data[2], &mpu->accel[1], 2);
    memcpy(&data[4], &mpu->accel[2], 2);
    memcpy(&data[6], &mpu->gyro[0], 2);
    memcpy(&data[8], &mpu->gyro[1], 2);
    memcpy(&data[10], &mpu->gyro[2], 2);
    memcpy(&data[12], &mpu->temp, 2);

    comm_send_frame(UPLINK_TYPE_MPU6050, data, sizeof(data));
}

// ============================================================
// 编码器上行帧发送 (type = 0x02)
// 载荷布局 (16 字节):
//   [0-1]   pulse[0]  M1 绝对计数 int16 LE
//   [2-3]   pulse[1]  M2
//   [4-5]   pulse[2]  M3
//   [6-7]   pulse[3]  M4
//   [8-9]   delta[0]  M1 增量 (本周期 - 上周期)
//   [10-11] delta[1]  M2
//   [12-13] delta[2]  M3
//   [14-15] delta[3]  M4
// ============================================================
void comm_send_encoder_frame(const encoder_data_t *enc)
{
    if (enc == NULL)
        return;

    // 同时发送 pulse 和 delta:
    //   pulse - 香橙派可做里程计积分 (int16 补码减法自动处理溢出)
    //   delta - MCU 端的固定 20ms 周期增量，丢帧下仍代表准确的瞬时速度
    uint8_t data[16];
    memcpy(&data[0],  &enc->pulse[0], 2);
    memcpy(&data[2],  &enc->pulse[1], 2);
    memcpy(&data[4],  &enc->pulse[2], 2);
    memcpy(&data[6],  &enc->pulse[3], 2);
    memcpy(&data[8],  &enc->delta[0], 2);
    memcpy(&data[10], &enc->delta[1], 2);
    memcpy(&data[12], &enc->delta[2], 2);
    memcpy(&data[14], &enc->delta[3], 2);

    comm_send_frame(UPLINK_TYPE_ENCODER, data, sizeof(data));
}