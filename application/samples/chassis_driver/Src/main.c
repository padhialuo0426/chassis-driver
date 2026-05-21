#include "common_def.h"
#include "cmsis_os2.h"
#include "app_init.h"
#include "osal_debug.h"
#include "soc_osal.h"
#include "watchdog.h"
#include "oled.h"
#include "i2c_hardware.h"
#include "tb6612_chassis.h"
#include "mpu6050.h"
#include "comm.h"

// ============================================================
// 配置区
// ============================================================
#define CAR_I2C_ADDR 0x40
#define CAR_PWM_FREQ 1000.0f
#define CONTROL_PERIOD_MS 20

#define COMM_TASK_STACK_SIZE 0x1000
#define CONTROL_TASK_STACK_SIZE 0x1000
#define COMM_TASK_PRIO (osPriority_t)(17)
#define CONTROL_TASK_PRIO (osPriority_t)(18)

// ============================================================
// 硬件初始化 (只执行一次，由先启动的任务调用)
// ============================================================
static volatile bool g_hw_inited = false;

static void hw_init_once(void)
{
    if (g_hw_inited)
        return;

    i2c_init();
    oled_init();
    chassis_init(CAR_I2C_ADDR, CAR_PWM_FREQ);
    mpu6050_init();
    g_hw_inited = true;

    osal_printk("[HW] i2c + chassis + mpu6050 initialized.\n");
}

// ============================================================
// 通信任务
// ============================================================
static void *comm_task(const char *arg)
{
    unused(arg);

    hw_init_once();
    comm_init();

    osal_printk("[CommTask] Running.\n");

    while (1)
    {
        if (comm_uart_poll())
        {
            const motor_cmd_t *cmd = comm_get_cmd();
            comm_execute_cmd(cmd);
        }

        comm_check_timeout();
        uapi_watchdog_kick();
        osal_msleep(1);
    }
    return NULL;
}

// ============================================================
// 控制任务
// 周期: 20ms (50Hz)
// 职责: 读编码器 (I2C) + OLED 显示 + 编码器上行
//       读 MPU6050 (I2C) + MPU 上行
// ============================================================
static void *control_task(const char *arg)
{
    unused(arg);

    hw_init_once();

    encoder_data_t enc;
    mpu_data_t mpu;
    uint32_t last_wake = osKernelGetTickCount();

    osal_printk("[CtrlTask] Running. Period: %d ms.\n", CONTROL_PERIOD_MS);

    while (1)
    {
        last_wake += CONTROL_PERIOD_MS;
        osDelayUntil(last_wake);

        // ---- 编码器读取 + OLED 显示 + 上行 ----
        // 上行 send 放在 if 内: I2C 失败时不发送，保证帧内数据有效
        if (comm_read_encoders(&enc))
        {
            oled_set_signed_num(1, 1, enc.delta[0], 5);
            oled_set_signed_num(1, 8, enc.delta[1], 5);
            oled_set_signed_num(2, 1, enc.delta[2], 5);
            oled_set_signed_num(2, 8, enc.delta[3], 5);
            oled_refresh();

            comm_send_encoder_frame(&enc);
        }

        // ---- MPU6050 读取 + 上行 ----
        // mpu6050_get_data 一次性突发读 14 字节寄存器 (~350µs @ 400kHz I2C)
        mpu6050_get_data(mpu.accel, mpu.gyro, &mpu.temp);
        comm_send_mpu_frame(&mpu);
    }

    return NULL;
}

// ============================================================
// 任务创建入口
// ============================================================
static void app_entry(void)
{
    osThreadAttr_t comm_attr = {0};
    comm_attr.name = "CommTask";
    comm_attr.attr_bits = osThreadDetached;
    comm_attr.cb_mem = NULL;
    comm_attr.cb_size = 0;
    comm_attr.stack_mem = NULL;
    comm_attr.stack_size = COMM_TASK_STACK_SIZE;
    comm_attr.priority = COMM_TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)comm_task, NULL, &comm_attr) == NULL)
    {
        osal_printk("[Main] Failed to create CommTask!\n");
    }

    osThreadAttr_t ctrl_attr = {0};
    ctrl_attr.name = "CtrlTask";
    ctrl_attr.attr_bits = osThreadDetached;
    ctrl_attr.cb_mem = NULL;
    ctrl_attr.cb_size = 0;
    ctrl_attr.stack_mem = NULL;
    ctrl_attr.stack_size = CONTROL_TASK_STACK_SIZE;
    ctrl_attr.priority = CONTROL_TASK_PRIO;

    if (osThreadNew((osThreadFunc_t)control_task, NULL, &ctrl_attr) == NULL)
    {
        osal_printk("[Main] Failed to create CtrlTask!\n");
    }
}

app_run(app_entry);