#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

void oled_init(void);
void oled_clear(void);
void oled_refresh(void);
void oled_refresh_block(const uint8_t Line, const uint8_t StartColumn, const uint8_t EndColumn);
void oled_set_char(const uint8_t Line, const uint8_t Column, const char Char);
void oled_set_string(const uint8_t Line, const uint8_t Column, const char *String);
void oled_set_num(const uint8_t Line, const uint8_t Column, const uint32_t Number, const uint8_t Length);
void oled_set_signed_num(const uint8_t Line, const uint8_t Column, const int32_t Number, const uint8_t Length);
void oled_set_hex_num(const uint8_t Line, const uint8_t Column, const uint32_t Number, const uint8_t Length);
void oled_set_bin_num(const uint8_t Line, const uint8_t Column, const uint32_t Number, const uint8_t Length);

#endif