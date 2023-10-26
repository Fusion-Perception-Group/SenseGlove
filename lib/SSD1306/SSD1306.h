#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "misc.h"
#include "I2C.h"
#include "font_8x8.h"
#include "SSD1306_CMD.h"

#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64
#define SSD1306_PAGENUM 8

typedef uint8_t SSD1306_Canvas_t[SSD1306_PAGENUM][SSD1306_WIDTH];

#define Canvas SSD1306_Canvas_t
#define WIDTH SSD1306_WIDTH
#define HEIGHT SSD1306_HEIGHT

bool init_display(I2C_t *i2c);

bool set_col_start_addr_for_page_mode(I2C_t *i2c, const uint8_t addr);
bool set_cursor(I2C_t *i2c, uint8_t page, uint8_t col);

bool start_wbyte_stream(I2C_t *i2c, const uint8_t mode);
void end_stream(const I2C_t *i2c);

bool fill_display(I2C_t *i2c, const uint8_t value);
bool clear_display(I2C_t *i2c);

bool print_char(I2C_t *i2c, const char c);
bool print_str(I2C_t *i2c, const char *str);
int dnprintf(I2C_t *i2c, const unsigned max_length, const char *format, ...);
int dnprintf_at(I2C_t *i2c, const unsigned col, const unsigned page, const unsigned max_length, const char *format, ...);

int str_to_raster(const char *str, uint8_t *raster, const unsigned max_length);
int cvnprintf(Canvas canvas, const unsigned max_length, int x, int y, const char *format, ...);

static inline bool get_canvas_dot(const Canvas canvas, const unsigned x, const unsigned y)
{
    if (x >= WIDTH || y >= HEIGHT)
        return false;
    return canvas[y/8U][x] & (uint8_t)(1 << (y%8));
}

static inline bool set_canvas_dot(Canvas canvas, const unsigned x, const unsigned y, const bool value)
{
    if (x >= WIDTH || y >= HEIGHT)
        return false;
    if (value)
        canvas[y/8U][x] |= (uint8_t)(1 << (y%8));
    else
        canvas[y/8U][x] &= (uint8_t)~(1 << (y%8));
    return true;
}

void fill_canvas(Canvas canvas, const bool value);
void clear_canvas(Canvas canvas);

void copy_canvas(Canvas dst, const Canvas src);
void merge_canvas(Canvas dst, const Canvas src, const Canvas mask);
void inverse_canvas(Canvas canvas);
void flip_canvas(Canvas canvas, const bool horizontal, const bool vertical);
void scale_canvas(Canvas dst, const Canvas src, const float scale, const bool fill_value);
void stretch_canvas(Canvas dst, const Canvas src, const float hstretch, const float vstretch, const bool fill_value);
void rotate_canvas(Canvas dst, const Canvas src, const float radian, const bool fill_value);
//void project_canvas(
//    Canvas dst, const Canvas src, const int top_l, const int top_r,
//    const int bottom_l, const int bottom_r, const int left_u, const int left_d,
//    const int right_u, const int right_d, const bool fill_value);
void project_canvas(
    Canvas dst, const Canvas src,
    float top_l_x, float top_l_y,
    float top_r_x, float top_r_y,
    float bottom_l_x, float bottom_l_y,
    float bottom_r_x, float bottom_r_y,
    const int fill_value);
void lr_shift_canvas(Canvas canvas, const int shift, const bool fill_value);
void ud_shift_canvas(Canvas canvas, const int shift, const bool fill_value);
bool flush_canvas(I2C_t *i2c, const Canvas canvas);

bool draw_rect(Canvas canvas, unsigned x, unsigned y, unsigned width, unsigned height, bool value);
bool draw_filled_rect(Canvas canvas, unsigned x, unsigned y, unsigned width, unsigned height, bool value);
bool draw_line(Canvas canvas, unsigned x0, unsigned y0, unsigned x1, unsigned y1, bool value);
bool draw_inf_line(Canvas canvas, unsigned x0, unsigned y0, unsigned x1, unsigned y1, bool value);
bool draw_circle(Canvas canvas, unsigned x0, unsigned y0, unsigned radius, bool value);
bool draw_filled_circle(Canvas canvas, unsigned x0, unsigned y0, unsigned radius, bool value);
bool draw_polygon(Canvas canvas, const unsigned points[][2], unsigned num_points, bool value);
bool draw_filled_polygon(Canvas canvas, const unsigned points[][2], unsigned num_points, bool value);
bool draw_raster(Canvas canvas, unsigned x, unsigned y, unsigned width, unsigned pagenum, const uint8_t *raster);
bool fill_block(Canvas canvas, unsigned x, unsigned y, bool value);

#undef Canvas
#undef WIDTH
#undef HEIGHT

#ifdef __cplusplus
}
#endif
