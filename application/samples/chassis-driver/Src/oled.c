#include "i2c_hardware.h"
#include "oled.h"
#include "font.h"
#include <string.h> // 必须引入，用于 memset 和 memcpy

#define OLED_ADDRESS 0x3C // OLED I2C地址

// =========================================================================
// 核心架构升级：定义全局显存 (Frame Buffer)
// 128列 * 8页 = 1024 字节，完全映射 OLED 屏幕的内部 RAM
// =========================================================================
static uint8_t OLED_GRAM[4][128];

/************************* OLED底层驱动函数 *************************/

/**
 * @brief  OLED写命令
 * @param  Command 要写入的命令
 */
static void oled_write_cmd(const uint8_t Command)
{
    uint8_t buffer[2] = {0x00, Command}; // 0x00表示写命令
    i2c_write(OLED_ADDRESS, buffer, 2);
}

/**
 * @brief  OLED连续写数据（核心提速函数）
 * @param  pData 数据首地址指针
 * @param  len 要发送的数据长度，最大 512 字节（一次发送半屏数据）
 */
static void oled_write_data_burst(uint8_t *pData, uint32_t len)
{
    // 定义为 static，防止 513 字节的大数组把 RTOS 任务栈（Stack）撑爆
    static uint8_t tx_buf[513]; // 1 字节控制 + 512 字节数据，分两次发送

    if (pData == NULL || len == 0 || len > 512)
    {
        return;
    }

    tx_buf[0] = 0x40; // 0x40 表示后面跟着的全部是连续的显示数据
    memcpy(&tx_buf[1], pData, len);
    
    i2c_write(OLED_ADDRESS, tx_buf, len + 1);
}

/*********************** OLED高级功能接口 *************************/

/**
 * @brief  刷新整个屏幕
 * @note   所有的画图函数调用后，屏幕不会立刻更新，必须调用此函数才生效！
 */
void oled_refresh(void)
{
    // 1. 设定页地址范围：Page 0 到 Page 7
    oled_write_cmd(0x22); 
    oled_write_cmd(0x00);
    oled_write_cmd(0x03);
    
    // 2. 设定列地址范围：Column 0 到 Column 127
    oled_write_cmd(0x21); 
    oled_write_cmd(0x00);
    oled_write_cmd(0x7F);

    // 3. 将 512 字节全局显存一次性发送到 OLED
    oled_write_data_burst((uint8_t *)OLED_GRAM, 512);
}

/**
 * @brief  刷新指定的字符区块
 * @param  Line 行号，范围：1-2
 * @param  StartColumn 起始列号，范围：1-16
 * @param  EndColumn 结束列号，范围：1-16
 */
void oled_refresh_block(const uint8_t Line, const uint8_t StartColumn, const uint8_t EndColumn)
{
    if (Line < 1 || Line > 2 || StartColumn < 1 || EndColumn < 1 || EndColumn > 16 || StartColumn > EndColumn) return;

    uint8_t start_page = (Line - 1) * 2;
    uint8_t end_page = start_page + 1;
    
    // 把字符列号转换成 OLED 底层的像素列号 (每个字符宽 8 像素)
    uint8_t start_pixel = (StartColumn - 1) * 8;
    uint8_t end_pixel = (EndColumn * 8) - 1;
    
    // 计算本次需要发送的像素宽度
    uint8_t width = end_pixel - start_pixel + 1;

    // 1. 划定 Y 轴刷新窗口
    oled_write_cmd(0x22); 
    oled_write_cmd(start_page); 
    oled_write_cmd(end_page);

    // 2. 划定 X 轴刷新窗口 (仅锁定你需要刷新的像素列！)
    oled_write_cmd(0x21); 
    oled_write_cmd(start_pixel); 
    oled_write_cmd(end_pixel);

    // 3. 提取显存中对应的碎块数据并发送
    uint8_t tx_buf[129]; 
    tx_buf[0] = 0x40;

    for (uint8_t i = start_page; i <= end_page; i++)
    {
        // 仅拷贝显存中发生变化的这几个字节
        memcpy(&tx_buf[1], &OLED_GRAM[i][start_pixel], width);
        // 按实际宽度发送
        i2c_write(OLED_ADDRESS, tx_buf, width + 1); 
    }
}

/**
 * @brief  OLED清屏 (极速版：仅清空显存内存)
 */
void oled_clear(void)
{
    // 使用 memset 瞬间清空  512 字节内存，耗时约几微秒
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

/**
 * @brief  OLED显示一个字符 (操作显存，无通信延迟)
 * @param  Line 行位置，范围：1-2
 * @param  Column 列位置，范围：1-16
 * @param  Char 要显示的一个字符，范围：ASCII可见字符
 */
void oled_set_char(const uint8_t Line, const uint8_t Column, const char Char)
{
    // 边界保护
    if (Line < 1 || Line > 2 || Column < 1 || Column > 16) return;
    if (Char < ' ' || Char > '~') return;  // 字体表仅覆盖 ASCII 0x20~0x7E

    uint8_t page = (Line - 1) * 2;   // 每行占用 2 个 page
    uint8_t x = (Column - 1) * 8;    // 每个字符宽度 8 列

    for (uint8_t i = 0; i < 8; i++)
    {
        // 纯内存操作，零阻塞！
        OLED_GRAM[page][x + i]     = OLED_F8x16[Char - ' '][i];      // 字符上半部分
        OLED_GRAM[page + 1][x + i] = OLED_F8x16[Char - ' '][i + 8];  // 字符下半部分
    }
}

/**
 * @brief  OLED显示字符串
 * @param  Line 起始行位置，范围：1-2
 * @param  Column 起始列位置，范围：1-16
 * @param  String 要显示的字符串，范围：ASCII可见字符
 */
void oled_set_string(const uint8_t Line, const uint8_t Column, const char *String)
{
    if (String == NULL) return;

    for (uint8_t i = 0; String[i] != '\0'; i++)
    {
        if (Column + i > 16) break;  // 超出屏幕宽度，提前退出
        oled_set_char(Line, Column + i, String[i]);
    }
}

/**
 * @brief  OLED次方函数 (内部调用)
 */
static uint32_t oled_pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    for (uint32_t i = 0; i < Y; i++)
    {
        Result *= X;
    }
    return Result;
}

/**
 * @brief  OLED显示数字（十进制，正数）
 */
void oled_set_num(const uint8_t Line, const uint8_t Column, const uint32_t Number, const uint8_t Length)
{
    if (Length == 0 || Length > 10) return;  // uint32_t 最多 10 位十进制

    for (uint8_t i = 0; i < Length; i++)
    {
        oled_set_char(Line, Column + i, Number / oled_pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED显示数字（十进制，带符号数）
 */
void oled_set_signed_num(const uint8_t Line, const uint8_t Column, const int32_t Number, const uint8_t Length)
{
    if (Length == 0 || Length > 10) return;  // uint32_t 最多 10 位十进制

    uint32_t Number1;
    if (Number >= 0)
    {
        oled_set_char(Line, Column, '+');
        Number1 = (uint32_t)Number;
    }
    else
    {
        oled_set_char(Line, Column, '-');
        // 避免 -INT32_MIN 的有符号溢出 (未定义行为)
        Number1 = (uint32_t)(-(Number + 1)) + 1;
    }
    for (uint8_t i = 0; i < Length; i++)
    {
        oled_set_char(Line, Column + i + 1, Number1 / oled_pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  OLED显示数字（十六进制，正数）
 */
void oled_set_hex_num(const uint8_t Line, const uint8_t Column, const uint32_t Number, const uint8_t Length)
{
    if (Length == 0 || Length > 8) return;  // uint32_t 最多 8 位十六进制

    for (uint8_t i = 0; i < Length; i++)
    {
        uint8_t SingleNumber = Number / oled_pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            oled_set_char(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            oled_set_char(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

/**
 * @brief  OLED显示数字（二进制，正数）
 */
void oled_set_bin_num(const uint8_t Line, const uint8_t Column, const uint32_t Number, const uint8_t Length)
{
    if (Length == 0 || Length > 16) return;  // 屏幕最多 16 列，超过也无法显示

    for (uint8_t i = 0; i < Length; i++)
    {
        oled_set_char(Line, Column + i, Number / oled_pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
 * @brief  OLED初始化
 */
void oled_init(void)
{
    oled_write_cmd(0xAE); // 关闭显示

    oled_write_cmd(0xD5); // 设置显示时钟分频比/振荡器频率
    oled_write_cmd(0x80);

    oled_write_cmd(0xA8); // 设置多路复用率
    oled_write_cmd(0x1F); // 0x1F=31，表示 32 行

    oled_write_cmd(0xD3); // 设置显示偏移
    oled_write_cmd(0x00);

    oled_write_cmd(0x40); // 设置显示开始行

    oled_write_cmd(0xA1); // 设置左、右方向，0xA1正常 0xA0左右反置
    oled_write_cmd(0xC8); // 设置上、下方向，0xC8正常 0xC0上下反置

    oled_write_cmd(0xDA); // 设置COM引脚硬件配置
    oled_write_cmd(0x02);

    oled_write_cmd(0x81); // 设置对比度控制
    oled_write_cmd(0xCF);

    oled_write_cmd(0xD9); // 设置预充电周期
    oled_write_cmd(0xF1);

    oled_write_cmd(0xDB); // 设置VCOMH取消选择级别
    oled_write_cmd(0x30);

    oled_write_cmd(0xA4); // 设置整个显示打开/关闭
    oled_write_cmd(0xA6); // 设置正常/倒转显示

    oled_write_cmd(0x8D); // 设置充电泵
    oled_write_cmd(0x14);

    // =======================================================
    // 关键升级：设置为水平寻址模式 (Horizontal Addressing Mode)
    // 允许光标到达列末尾后自动换行，这是全屏极速刷新的核心前提
    // =======================================================
    oled_write_cmd(0x20); 
    oled_write_cmd(0x00); 

    oled_write_cmd(0xAF); // 开启显示

    oled_clear();         // 初始化清空显存
    oled_refresh();       // 将空显存推送到屏幕上
}