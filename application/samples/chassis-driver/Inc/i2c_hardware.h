#ifndef __I2C_HARDWARE_H
#define __I2C_HARDWARE_H

#include "i2c.h"
#include "pinctrl.h"

#define I2C_BAUDRATE 400000 /* 400kHz */
#define I2C_MASTER_PIN_MODE PIN_MODE_2

errcode_t i2c_init(void);
errcode_t i2c_write(uint16_t addr, uint8_t *data, uint32_t tx_len);
errcode_t i2c_read(uint16_t addr, uint8_t *rx_data, uint32_t rx_len);
errcode_t i2c_writeread(uint16_t addr, uint8_t *tx_data, uint32_t tx_len, uint8_t *rx_data, uint32_t rx_len);
uint8_t i2c_scanner(void);

#endif /* __I2C_HARDWARE_H */