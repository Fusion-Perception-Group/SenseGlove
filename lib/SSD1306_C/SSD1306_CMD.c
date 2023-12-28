#include "SSD1306_CMD.h"
#define CMD SSD1306_CMD
#define CMD_S SSD1306_CMD_STREAM
#define DATA SSD1306_DATA
#define DATA_S SSD1306_DATA_STREAM
#define OLED_ADDR SSD1306_ADDR

/**
 * @brief Set the contrast of the display
 *
 * @param i2c
 * @param contrast: 0x00 - 0xff, reset value is 0x7f
 * @return true
 * @return false
 */
bool set_contrast(I2C_t *i2c, uint8_t contrast)
{
    uint8_t data[] = {
        CMD_S,
        0x81,
        contrast};
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the entire display on
 *
 * @param i2c
 * @param ignore_ram: true - ignore RAM content, false - according to RAM content(RESET)
 * @return true
 * @return false
 */
bool entire_display_on(I2C_t *i2c, bool ignore_ram)
{
    uint8_t data[] = {
        CMD_S,
        (uint8_t)ignore_ram | 0xa4};
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set Normal/Inverse mode
 *
 * @param i2c
 * @param inverse: true - inverse mode(bit 0 -> ON), false - normal mode(bit 1 -> ON)(RESET)
 * @return true
 * @return false
 */
bool set_normal_inverse_display(I2C_t *i2c, bool inverse)
{
    uint8_t data[] = {
        CMD_S,
        (uint8_t)inverse | 0xa6,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the display on/off
 *
 * @param i2c
 * @param on: true - display on, false - display off(RESET)
 * @return true
 * @return false
 */
bool set_display_on_off(I2C_t *i2c, bool on)
{
    uint8_t data[] = {
        CMD_S,
        (uint8_t)on | 0xae,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

bool display_on(I2C_t *i2c)
{
    return set_display_on_off(i2c, true);
}

bool display_off(I2C_t *i2c)
{
    return set_display_on_off(i2c, false);
}

/**
 * @brief Setup continuous horizontal scroll (by 1 column)
 *
 * @param i2c
 * @param dir: true - left, false - right
 * @param start_page: 0x00 - 0x07
 * @param end_page: 0x00 - 0x07, must be >= start_page, otherwise return false
 * @param interval_fr: 2, 3, 4, 5, 25, 64, 128, 256, return false if not match
 * @return true
 * @return false: if interval_fr not match or write_I2C failed or start_page > end_page
 */
bool hscroll_setup(I2C_t *i2c, bool dir, uint8_t start_page, uint8_t end_page, unsigned interval_fr)
{
    uint8_t interval;
    if (start_page > end_page)
        return false;

    switch (interval_fr)
    {
    case 2:
        interval = 0x07;
        break;
    case 3:
        interval = 0x04;
        break;
    case 4:
        interval = 0x05;
        break;
    case 5:
        interval = 0x00;
        break;
    case 25:
        interval = 0x06;
        break;
    case 64:
        interval = 0x01;
        break;
    case 128:
        interval = 0x02;
        break;
    case 256:
        interval = 0x03;
        break;
    default:
        return false;
    }

    uint8_t data[] = {
        CMD_S,
        (uint8_t)dir | 0x26,
        0x00, // dummy byte
        start_page,
        interval,
        end_page,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Setup continuous vertical and horizontal scroll
 *
 * @param i2c
 * @param dir: true - left, false - right
 * @param start_page: 0x00 - 0x07
 * @param end_page: 0x00 - 0x07, must be >= start_page, otherwise return false
 * @param interval_fr: 2, 3, 4, 5, 25, 64, 128, 256, return false if not match
 * @param vertical_offset: 0x00 - 0x3f
 * @return true
 * @return false: if interval_fr not match or write_I2C failed or start_page > end_page
 */
bool hvscroll_setup(I2C_t *i2c, bool dir, uint8_t start_page, uint8_t end_page, unsigned interval_fr, uint8_t vertical_offset)
{
    uint8_t interval;
    if (start_page > end_page)
        return false;

    switch (interval_fr)
    {
    case 2:
        interval = 0x07;
        break;
    case 3:
        interval = 0x04;
        break;
    case 4:
        interval = 0x05;
        break;
    case 5:
        interval = 0x00;
        break;
    case 25:
        interval = 0x06;
        break;
    case 64:
        interval = 0x01;
        break;
    case 128:
        interval = 0x02;
        break;
    case 256:
        interval = 0x03;
        break;
    default:
        return false;
    }

    uint8_t data[] = {
        CMD_S,
        dir + 0x29,
        0x00, // dummy byte
        start_page,
        interval,
        end_page,
        vertical_offset,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set scroll activation
 *
 * @param i2c
 * @param on: true - activate scroll, false - deactivate scroll
 * @return true
 * @return false
 */
bool set_scroll_activation(I2C_t *i2c, bool on)
{
    uint8_t data[] = {
        CMD_S,
        (uint8_t)on | 0x2f};
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

bool activate_scroll(I2C_t *i2c)
{
    return set_scroll_activation(i2c, true);
}

bool deactivate_scroll(I2C_t *i2c)
{
    return set_scroll_activation(i2c, false);
}

/**
 * @brief Set the vertical scroll area
 *
 * @note
(1) A[5:0]+B[6:0] <= MUX ratio
(2) B[6:0] <= MUX ratio
(3a) Vertical scrolling offset (E[5:0] in 29h/2Ah) < B[6:0]
(3b) Set Display Start Line (X5X4X3X2X1X0 of 40h~7Fh) < B[6:0]
(4) The last row of the scroll area shifts to the first row of the scroll area.
(5) For 64d MUX display A[5:0] = 0, B[6:0]=64 : whole area scrolls A[5:0]= 0, B[6:0] < 64 : top area scrolls A[5:0] + B[6:0] < 64 : central area scrolls A[5:0] + B[6:0] = 64 : bottom area scrolls
 * @param i2c
 * @param top_fixed_rows: 0-63, number of rows that are not affected by vertical scroll(RESET = 0x00)
 * @param scroll_rows: 0-64,number of rows that are affected by vertical scroll(RESET = 0x40)
 * @return true
 * @return false
 */
bool set_vertical_scroll_area(I2C_t *i2c, uint8_t top_fixed_rows, uint8_t scroll_rows)
{
    uint8_t data[] = {
        CMD_S,
        0xa3,
        top_fixed_rows,
        scroll_rows};
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the lower column start address for page addressing mode
 *
 * @param i2c
 * @param addr: 0x00 - 0x0f(RESET = 0x00)
 * @return true
 * @return false: if addr > 0x0f or write_I2C failed
 */
bool set_lower_col_start_addr_for_page_mode(I2C_t *i2c, uint8_t addr)
{
    if (addr > 0x0f)
        return false;
    uint8_t data[] = {
        CMD_S,
        addr,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the higher column start address for page addressing mode
 *
 * @param i2c
 * @param addr: 0x00 - 0x0f(RESET = 0x00)
 * @return true
 * @return false: if addr out of range or write_I2C failed
 */
bool set_higher_col_start_addr_for_page_mode(I2C_t *i2c, uint8_t addr)
{
    if (addr > 0x0f)
        return false;
    uint8_t data[] = {
        CMD_S,
        addr | 0x10,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the memory addressing mode
 *
 * @param i2c
 * @param mode: 0x00 - horizontal addressing mode,
 *              0x01 - vertical addressing mode,
 *              0x02 - page addressing mode(RESET)
 * @return true
 * @return false: if mode out of range or write_I2C failed
 */
bool set_memory_addressing_mode(I2C_t *i2c, uint8_t mode)
{
    if (mode > 0x02)
        return false;
    uint8_t data[] = {
        CMD_S,
        0x20,
        mode,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the column start and end address
 *
 * @param i2c
 * @param start: 0x00 - 0x7f(RESET = 0x00)
 * @param end: 0x00 - 0x7f(RESET = 0x7f)
 * @return true
 * @return false
 */
bool set_col_addr(I2C_t *i2c, uint8_t start, uint8_t end)
{
    uint8_t data[] = {
        CMD_S,
        0x21,
        start,
        end,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the page start and end address
 *
 * @param i2c
 * @param start: 0x00 - 0x07(RESET = 0x00)
 * @param end: 0x00 - 0x07(RESET = 0x07)
 * @return true
 * @return false
 */
bool set_page_addr(I2C_t *i2c, uint8_t start, uint8_t end)
{
    uint8_t data[] = {
        CMD_S,
        0x22,
        start,
        end,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the page start address for page addressing mode
 *
 * @param i2c
 * @param addr: 0x00 - 0x07(RESET = 0x00)
 * @return true
 * @return false: if addr out of range or write_I2C failed
 */
bool set_page_start_addr_for_page_mode(I2C_t *i2c, uint8_t addr)
{
    if (addr > 0x07)
        return false;
    uint8_t data[] = {
        CMD_S,
        0xb0 | addr,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set the display start line
 *
 * @param i2c
 * @param line: 0x00 - 0x3f(RESET = 0x00)
 * @return true
 * @return false: if line out of range or write_I2C failed
 */
bool set_display_start_line(I2C_t *i2c, uint8_t line)
{
    if (line > 0x3f)
        return false;
    uint8_t data[] = {
        CMD_S,
        0x40 | line,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set segment re-map
 *
 * @param i2c
 * @param remap: true - column address 127 is mapped to SEG0, false - column address 0 is mapped to SEG0(RESET)
 * @return true
 * @return false: if contrast out of range or write_I2C failed
 */
bool set_seg_remap(I2C_t *i2c, bool remap)
{
    uint8_t data[] = {
        CMD_S,
        0xa0 | (uint8_t)remap};
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set multiplex ratio
 *
 * @param i2c
 * @param ratio: 0x0f - 0x3f(RESET = 0x3f)
 * @return true
 * @return false: if ratio out of range or write_I2C failed
 */
bool set_multiplex_ratio(I2C_t *i2c, uint8_t ratio)
{
    if (ratio > 0x3f || ratio < 0x0f)
        return false;
    uint8_t data[] = {
        CMD_S,
        0xa8,
        ratio,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set COM output scan direction
 *
 * @param i2c
 * @param dir: true - remapped mode, false - normal mode(RESET)
 * @return true
 * @return false: if dir out of range or write_I2C failed
 */
bool set_COM_output_scan_dir(I2C_t *i2c, bool dir)
{
    uint8_t data[] = {
        CMD_S,
        0xc0 | ((uint8_t)dir << 3),
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set display offset
 *
 * @param i2c
 * @param offset: 0x00 - 0x3f(RESET = 0x00)
 * @return true
 * @return false: if offset out of range or write_I2C failed
 */
bool set_display_offset(I2C_t *i2c, uint8_t offset)
{
    if (offset > 0x3f)
        return false;
    uint8_t data[] = {
        CMD_S,
        0xd3,
        offset,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set COM pins hardware configuration
 *
 * @param i2c
 * @param sequential: true - sequential, false - alternative(RESET)
 * @param remapped: COM left/right remap, true - remapped, false - normal(RESET)
 * @return true
 * @return false: if ratio out of range or write_I2C failed
 */
bool set_COM_pins_hardware_config(I2C_t *i2c, bool sequential, bool remapped)
{
    uint8_t data[] = {
        CMD_S,
        0xda,
        0x02 | ((uint8_t)sequential << 4) | ((uint8_t)remapped << 5),
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set display clock divide ratio/oscillator frequency
 *
 * @param i2c
 * @param ratio: 1-16(RESET = 0x1)
 * @param freq: 0-15(RESET = 0x8)
 * @return true
 * @return false: if ratio or freq out of range or write_I2C failed
 */
bool set_display_clock_divide_ratio(I2C_t *i2c, uint8_t ratio, uint8_t freq)
{
    --ratio; // remap 1-16 -> 0-15
    if (ratio > 0x0f || freq > 0x0f)
        return false;
    uint8_t data[] = {
        CMD_S,
        0xd5,
        (freq << 4) | ratio,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set pre-charge period
 *
 * @param i2c
 * @param phase1: 1-15(RESET = 0x2)
 * @param phase2: 1-15(RESET = 0x2)
 * @return true
 * @return false: if phase1 or phase2 out of range or write_I2C failed
 */
bool set_pre_charge_period(I2C_t *i2c, uint8_t phase1, uint8_t phase2)
{
    if (!phase1 || !phase2 || phase1 > 0x0f || phase2 > 0x0f)
        return false;
    uint8_t data[] = {
        CMD_S,
        0xd9,
        (phase2 << 4) | phase1,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set VCOMH deselect level
 *
 * @param i2c
 * @param level: {0x00, 0x02, 0x03}(RESET = 0x02), 0x00 - 0.65*Vcc, 0x02 - 0.77*Vcc, 0x03 - 0.83*Vcc
 * @return true
 * @return false: if level out of range or write_I2C failed
 */
bool set_VCOMH_deselect_level(I2C_t *i2c, uint8_t level)
{
    if (level > 0x03 || level == 0x01)
        return false;
    uint8_t data[] = {
        CMD_S,
        0xdb,
        level << 4,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

bool OLED_NOP(I2C_t *i2c)
{
    uint8_t data[] = {
        CMD_S,
        0xe3,
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

/**
 * @brief Set charge pump
 *
 * @param i2c
 * @param enable: true - enable, false - disable(RESET)
 * @return true
 * @return false: if write_I2C failed
 */
bool set_charge_pump(I2C_t *i2c, bool enable)
{
    uint8_t data[] = {
        CMD_S,
        0x8d,
        0x10 | ((uint8_t)enable << 2),
    };
    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

bool charge_pump_on(I2C_t *i2c)
{
    return set_charge_pump(i2c, true);
}

bool charge_pump_off(I2C_t *i2c)
{
    return set_charge_pump(i2c, false);
}
