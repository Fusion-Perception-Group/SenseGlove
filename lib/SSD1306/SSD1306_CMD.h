#ifndef p_SSD1306_CMD_h
#define p_SSD1306_CMD_h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "I2C.h"

#define SSD1306_ADDR 0x78
#define SSD1306_CMD 0x80
#define SSD1306_CMD_STREAM 0x00
#define SSD1306_DATA 0xc0
#define SSD1306_DATA_STREAM 0x40

bool set_contrast(I2C_t *i2c, uint8_t contrast);
bool entire_display_on(I2C_t *i2c, bool ignore_ram);
bool set_normal_inverse_display(I2C_t *i2c, bool inverse);
bool set_display_on_off(I2C_t *i2c, bool on);
bool display_on(I2C_t *i2c);
bool display_off(I2C_t *i2c);

bool hscroll_setup(I2C_t *i2c, bool dir, uint8_t start_page, uint8_t end_page, unsigned interval_fr);
bool hvscroll_setup(I2C_t *i2c, bool dir, uint8_t start_page, uint8_t end_page, unsigned interval_fr, uint8_t vertical_offset);
bool set_scroll_activation(I2C_t *i2c, bool on);
bool activate_scroll(I2C_t *i2c);
bool deactivate_scroll(I2C_t *i2c);
bool set_vertical_scroll_area(I2C_t *i2c, uint8_t top_fixed_rows, uint8_t scroll_rows);

bool set_lower_col_start_addr_for_page_mode(I2C_t *i2c, uint8_t addr);
bool set_higher_col_start_addr_for_page_mode(I2C_t *i2c, uint8_t addr);
bool set_memory_addressing_mode(I2C_t *i2c, uint8_t mode);
bool set_col_addr(I2C_t *i2c, uint8_t start, uint8_t end);
bool set_page_addr(I2C_t *i2c, uint8_t start, uint8_t end);
bool set_page_start_addr_for_page_mode(I2C_t *i2c, uint8_t addr);

bool set_display_start_line(I2C_t *i2c, uint8_t line);
bool set_seg_remap(I2C_t *i2c, bool remap);
bool set_multiplex_ratio(I2C_t *i2c, uint8_t ratio);
bool set_COM_output_scan_dir(I2C_t *i2c, bool dir);
bool set_display_offset(I2C_t *i2c, uint8_t offset);
bool set_COM_pins_hardware_config(I2C_t *i2c, bool sequential, bool remapped);

bool set_display_clock_divide_ratio(I2C_t *i2c, uint8_t ratio, uint8_t freq);
bool set_pre_charge_period(I2C_t *i2c, uint8_t phase1, uint8_t phase2);
bool set_VCOMH_deselect_level(I2C_t *i2c, uint8_t level);

bool set_charge_pump(I2C_t *i2c, bool enable);
bool charge_pump_on(I2C_t *i2c);
bool charge_pump_off(I2C_t *i2c);

bool OLED_NOP(I2C_t *i2c);

#ifdef __cplusplus
}
#endif

#endif