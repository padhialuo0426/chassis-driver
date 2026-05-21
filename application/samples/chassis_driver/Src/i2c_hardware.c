#include "i2c_hardware.h"

errcode_t i2c_init(void)
{
    /* 设置 i2c pinmux */
    uapi_pin_set_mode(CONFIG_I2C_SCL_MASTER_PIN, I2C_MASTER_PIN_MODE);
    uapi_pin_set_mode(CONFIG_I2C_SDA_MASTER_PIN, I2C_MASTER_PIN_MODE);
    uapi_pin_set_pull(CONFIG_I2C_SCL_MASTER_PIN, PIN_PULL_TYPE_UP);
    uapi_pin_set_pull(CONFIG_I2C_SDA_MASTER_PIN, PIN_PULL_TYPE_UP);
    /* 初始化 i2c */
    return uapi_i2c_master_init(CONFIG_I2C_MASTER_BUS_ID, I2C_BAUDRATE, 0);
}

/**
 * @brief  I2C Master writes a buffer to target slave.
 * @param  [in]  addr The target slave address for master to write data.
 * @param  [in]  data The buffer pointer of data to be sent to slave.
 * @param  [in]  len The length of data to be sent to slave.
 * @retval errcode_t type error code, ERRCODE_SUCC on success.
 */
errcode_t i2c_write(uint16_t addr, uint8_t *data, uint32_t tx_len)
{
    i2c_data_t i2c_send_data = {0};
    i2c_send_data.send_buf = data;
    i2c_send_data.send_len = tx_len;
    return uapi_i2c_master_write(CONFIG_I2C_MASTER_BUS_ID, addr, &i2c_send_data);
}

errcode_t i2c_read(uint16_t addr, uint8_t *rx_data, uint32_t rx_len)
{
    i2c_data_t i2c_receive_data = {0};
    i2c_receive_data.receive_buf = rx_data;
    i2c_receive_data.receive_len = rx_len;
    return uapi_i2c_master_read(CONFIG_I2C_MASTER_BUS_ID, addr, &i2c_receive_data);
}

/**
 * @brief  I2C Master writes a buffer to target slave and then reads data from this slave.
 * @param  [in]  addr The target slave address for master to write and read data.
 * @param  [in]  tx_data The buffer pointer of data to be sent to slave.
 * @param  [in]  tx_len The length of data to be sent to slave.
 * @param  [in]  rx_data The buffer pointer of data to be received from slave.
 * @param  [in]  rx_len The length of data to be received from slave.
 * @retval errcode_t type error code, ERRCODE_SUCC on success.
 */
errcode_t i2c_writeread(uint16_t addr, uint8_t *tx_data, uint32_t tx_len, uint8_t *rx_data, uint32_t rx_len)
{
    i2c_data_t i2c_receive_data = {0};
    i2c_receive_data.send_buf = tx_data;
    i2c_receive_data.send_len = tx_len;
    i2c_receive_data.receive_buf = rx_data;
    i2c_receive_data.receive_len = rx_len;
    return uapi_i2c_master_writeread(CONFIG_I2C_MASTER_BUS_ID, addr, &i2c_receive_data);
}

/* @brief  I2C Scanner: Scan the I2C bus for devices and print their addresses.
 * @note   This function is a utility to help identify which I2C devices are present on the bus.
 *         It attempts to read from each possible 7-bit address and checks for ACK responses.
 */
uint8_t i2c_scanner(void)
{
    uint8_t rx_buf[1] = {0};
    uint8_t device_count = 0;

    // Scan all possible 7-bit I2C addresses (0x01 to 0x7F)
    for (uint8_t addr = 1; addr < 0x7F; addr++)
    {
        // If the device at this address responds, increment the device count
        if (i2c_read(addr, rx_buf, 1) == ERRCODE_SUCC)
        {
            device_count++;
        }

        // Add a small delay to avoid overwhelming the bus
        for (volatile int i = 0; i < 10000; i++)
            ;
    }

    return device_count;
}