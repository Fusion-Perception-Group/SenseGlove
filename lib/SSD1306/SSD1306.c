#include "SSD1306.h"

#define CMD SSD1306_CMD
#define CMD_S SSD1306_CMD_STREAM
#define DATA SSD1306_DATA
#define DATA_S SSD1306_DATA_STREAM
#define OLED_ADDR SSD1306_ADDR
#define WIDTH SSD1306_WIDTH
#define HEIGHT SSD1306_HEIGHT
#define PAGENUM SSD1306_PAGENUM
#define Canvas SSD1306_Canvas_t

bool init_display(I2C_t *i2c)
{
    uint8_t data[] = {
        CMD_S,
        0xae, // display off
        0xd5, // set display clock divide ratio/oscillator frequency
        0x80,
        0xa8, // set multiplex ratio
        0x3f,
        0xd3, // set display offset
        0x0,
        0x40, // set display start line
        0xa1, // set segment re-map
        0xc8, // set COM output scan direction
        0xda, // set COM pins hardware configuration
        0x12,
        0x81, // set contrast control
        0xcf,
        0xd9, // set pre-charge period
        0xf1,
        0xdb, // set VCOMH deselect level
        0x30,
        0xa4, // set entire display on/off
        0xa6, // set normal/inverse display
        0x8d, // charge pump setting
        0x14,
        0xaf, // display on
    };

    return init_I2C(i2c) && write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

bool set_col_start_addr_for_page_mode(I2C_t *i2c, const uint8_t addr)
{
    uint8_t data[] = {
        CMD_S,
        0x0f & addr,      // set lower column start address for page addressing mode
        0x10 | addr >> 4, // set higher column start address for page addressing mode
    };

    return write_I2C(i2c, OLED_ADDR, data, sizeof(data));
}

bool set_cursor(I2C_t *i2c, uint8_t page, uint8_t col)
{
    return (
        set_page_start_addr_for_page_mode(i2c, page) && set_col_start_addr_for_page_mode(i2c, col));
}

/**
 * @brief Start to write a byte stream to the OLED display.
 *
 * @param i2c
 * @param mode
 *     @arg SSD1306_CMD
 *     @arg SSD1306_DATA
 *     @arg SSD1306_CMD_STREAM
 *     @arg SSD1306_DATA_STREAM
 * @return true
 * @return false
 */
bool start_wbyte_stream(I2C_t *i2c, const uint8_t mode)
{
    return select_I2C(i2c, OLED_ADDR, I2C_WRITE) && wbyte_I2C(i2c, mode);
}

void end_stream(const I2C_t *i2c)
{
    _terminate_I2C(i2c);
}

bool fill_display(I2C_t *i2c, const uint8_t value)
{
    for (unsigned page = 0; page < PAGENUM; ++page)
    {
        if (!set_cursor(i2c, page, 0))
            return false;
        if (!start_wbyte_stream(i2c, DATA_S))
            return false;
        for (unsigned col = 0; col < WIDTH; ++col)
        {
            if ((!wbyte_I2C(i2c, value) && page && col) || i2c->arb_lost)
                return false;
        }
        end_stream(i2c);
    }
    return true;
}

bool clear_display(I2C_t *i2c)
{
    return fill_display(i2c, 0x0);
}

static inline bool _print_char(I2C_t *i2c, const char c)
{
    if (c >= sizeof(FONT_8x8) / sizeof(FONT_8x8[0]))
        return false;
    for (unsigned i = 0; i < 8; ++i)
    {
        if (!wbyte_I2C(i2c, FONT_8x8[(uint8_t)c][i]))
            return false;
    }
    return true;
}

bool print_char(I2C_t *i2c, const char c)
{
    if (!start_wbyte_stream(i2c, DATA_S))
        return false;
    if (!_print_char(i2c, c))
        return false;
    end_stream(i2c);
    return true;
}

bool print_str(I2C_t *i2c, const char *str)
{
    if (!start_wbyte_stream(i2c, DATA_S))
        return false;
    while (*str)
    {
        if (!_print_char(i2c, *str++))
            return false;
    }
    end_stream(i2c);
    return true;
}

int dnprintf(I2C_t *i2c, const unsigned max_length, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char buf[max_length + 1];
    int ret = vsnprintf(buf, max_length, format, args);
    buf[max_length] = '\0';
    if (!print_str(i2c, buf))
        ret = -1;
    va_end(args);
    return ret;
}

int dnprintf_at(I2C_t *i2c, const unsigned col, const unsigned page, const unsigned max_length, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char buf[max_length + 1];
    int ret = vsnprintf(buf, max_length, format, args);
    buf[max_length] = '\0';
    if (!set_cursor(i2c, page, col) || !print_str(i2c, buf))
        ret = -1;
    va_end(args);
    return ret;
}

int str_to_raster(const char *str, uint8_t *raster, const unsigned max_length)
{
    unsigned length = 0;
    while (*str && length < max_length)
    {
        if (*str > sizeof(FONT_8x8) / sizeof(FONT_8x8[0]))
            return -1;
        memcpy(raster + length * 8, FONT_8x8[(uint8_t)*str++], 8);
        ++length;
    }
    return length;
}

int cvnprintf(Canvas canvas, const unsigned max_length, int x, int y, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    if (x < 0)
        x = WIDTH + x % WIDTH;
    if (y < 0)
        y = HEIGHT + y % HEIGHT;
    char buf[max_length + 1];
    buf[max_length] = '\0';
    int ret = vsnprintf(buf, max_length, format, args);
    if (ret < 0)
        return ret;
    // finished generating str

    // draw raster
    int lines = 1, current_max_len = 0;
    const int MAX_LINE_CHARS = MAX((WIDTH - x) / 8, 1);
    for (int i = 0, chars = 0; buf[i]; ++i)
    {
        ++chars;
        if (chars > current_max_len)
            current_max_len = chars;
        if (buf[i] == '\n' || chars >= MAX_LINE_CHARS)
        {
            chars = 0;
            ++lines;
            continue;
        }
    }
    uint8_t raster[lines][current_max_len][8];
    memset(raster, 0, sizeof(raster));
    for (int i = 0, line = 0, chars = 0; buf[i]; ++i)
    {
        if ((uint8_t)buf[i] > sizeof(FONT_8x8) / sizeof(FONT_8x8[0]))
            memcpy(raster[line][chars++], FONT_8x8[(uint8_t)'?'], 8);
        else
            memcpy(raster[line][chars++], FONT_8x8[(uint8_t)buf[i]], 8);
        if (buf[i] == '\n' || chars >= MAX_LINE_CHARS)
        {
            chars = 0;
            ++line;
            continue;
        }
    }
    // finished generating raster

    if (!draw_raster(canvas, x, y, current_max_len * 8, lines, (uint8_t *)raster))
        ret = -1;

    va_end(args);
    return ret;
}

void fill_canvas(Canvas canvas, const bool value)
{
    memset(canvas, value ? 0xff : 0x0, sizeof(Canvas));
}

void clear_canvas(Canvas canvas)
{
    memset(canvas, 0, sizeof(Canvas));
}

void copy_canvas(Canvas dst, const Canvas src)
{
    memcpy(dst, src, sizeof(Canvas));
}

void inverse_canvas(Canvas canvas)
{
    for (unsigned page = 8; page--;)
    {
        for (unsigned col = 128; col--;)
        {
            canvas[page][col] = ~canvas[page][col];
        }
    }
}

/**
 * @brief Scale the canvas by a factor. Based on nearest neighbor interpolation.
 *
 * @param dst
 * @param src
 * @param scale
 */
void scale_canvas(Canvas dst, const Canvas src, const float scale, const bool fill_value)
{
    if (dst == src)
        return;
    float center_x = WIDTH / 2, center_y = HEIGHT / 2;
    const float elacs = 1.0f / scale;
    for (int row = 0; row < HEIGHT; ++row)
    {
        for (int col = 0; col < WIDTH; ++col)
        {
            float x = (col - center_x) * elacs + center_x, y = (row - center_y) * elacs + center_y;
            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
            {
                set_canvas_dot(dst, col, row, fill_value);
                continue;
            }
            set_canvas_dot(dst, col, row, get_canvas_dot(src, roundf(x), roundf(y)));
        }
    }
}

void stretch_canvas(Canvas dst, const Canvas src, const float hstretch, const float vstretch, const bool fill_value)
{
    if (dst == src)
        return;
    float center_x = WIDTH / 2, center_y = HEIGHT / 2;
    const float h_elacs = 1.0f / hstretch, v_elacs = 1.0f / vstretch;
    for (int row = 0; row < HEIGHT; ++row)
    {
        for (int col = 0; col < WIDTH; ++col)
        {
            float x = (col - center_x) * h_elacs + center_x, y = (row - center_y) * v_elacs + center_y;
            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
            {
                set_canvas_dot(dst, col, row, fill_value);
                continue;
            }
            set_canvas_dot(dst, col, row, get_canvas_dot(src, roundf(x), roundf(y)));
        }
    }
}

void rotate_canvas(Canvas dst, const Canvas src, const float radian, const bool fill_value)
{
    if (dst == src)
        return;
    float center_x = WIDTH / 2, center_y = HEIGHT / 2;
    const float sinf_rad = sinf(radian), cosf_rad = cosf(radian);
    for (int row = 0; row < HEIGHT; ++row)
    {
        for (int col = 0; col < WIDTH; ++col)
        {
            float x = (col - center_x) * cosf_rad - (row - center_y) * sinf_rad + center_x;
            float y = (col - center_x) * sinf_rad + (row - center_y) * cosf_rad + center_y;
            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
            {
                set_canvas_dot(dst, col, row, fill_value);
                continue;
            }
            set_canvas_dot(dst, col, row, get_canvas_dot(src, roundf(x), roundf(y)));
        }
    }
}

///**
// * @brief Project a canvas to another canvas.
// *
// * @param dst
// * @param src
// * @param top_l : top left corner of the projection area
// * @param top_r : top right corner of the projection area
// * @param bottom_l : bottom left corner of the projection area
// * @param bottom_r : bottom right corner of the projection area
// * @param left_u
// * @param left_d
// * @param right_u
// * @param right_d
// * @param d
// * @param fill_value
// */
// void project_canvas(
//    Canvas dst, const Canvas src, const int top_l, const int top_r,
//    const int bottom_l, const int bottom_r, const int left_u, const int left_d,
//    const int right_u, const int right_d, const bool fill_value)
//{
//    if (dst == src)
//        return;
//    for (int row = 0; row < HEIGHT; ++row)
//    {
//        for (int col = 0; col < WIDTH; ++col)
//        {
//            float x = (col - left_u) * (top_r - top_l) / (right_u - left_u) + top_l;
//            float y = (row - left_d) * (bottom_r - bottom_l) / (right_d - left_d) + bottom_l;
//            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
//            {
//                set_canvas_dot(dst, col, row, fill_value);
//                continue;
//            }
//            set_canvas_dot(dst, col, row, get_canvas_dot(src, roundf(x), roundf(y)));
//        }
//    }
//}

void project_canvas(
    Canvas dst, const Canvas src,
    float top_l_x, float top_l_y,
    float top_r_x, float top_r_y,
    float bottom_l_x, float bottom_l_y,
    float bottom_r_x, float bottom_r_y,
    const int fill_value)
{
    for (int row = 0; row < HEIGHT; ++row)
    {
        for (int col = 0; col < WIDTH; ++col)
        {
            float u = (float)col / (float)(WIDTH - 1);
            float v = (float)row / (float)(HEIGHT - 1);

            float x = (1 - u) * ((1 - v) * top_l_x + v * bottom_l_x) +
                      u * ((1 - v) * top_r_x + v * bottom_r_x);
            float y = (1 - v) * ((1 - u) * top_l_y + u * top_r_y) +
                      v * ((1 - u) * bottom_l_y + u * bottom_r_y);

            if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
            {
                set_canvas_dot(dst, col, row, fill_value);
                continue;
            }

            int src_x = (int)x;
            int src_y = (int)y;
            float x_diff = x - src_x;
            float y_diff = y - src_y;

            int interpolated_value =
                (1 - x_diff) * (1 - y_diff) * get_canvas_dot(src, src_x, src_y) +
                x_diff * (1 - y_diff) * get_canvas_dot(src, src_x + 1, src_y) +
                (1 - x_diff) * y_diff * get_canvas_dot(src, src_x, src_y + 1) +
                x_diff * y_diff * get_canvas_dot(src, src_x + 1, src_y + 1);

            set_canvas_dot(dst, col, row, interpolated_value);
        }
    }
}

void flip_canvas(Canvas canvas, const bool horizontal, const bool vertical)
{
    if (horizontal)
    {
        for (int page = 0; page < PAGENUM / 2; ++page)
        {
            for (int col = 0; col < WIDTH; ++col)
            {
                uint8_t tmp;
                tmp = reverseByte(canvas[page][col]);
                canvas[page][col] = reverseByte(canvas[PAGENUM - 1 - page][col]);
                canvas[PAGENUM - 1 - page][col] = tmp;
            }
            if (PAGENUM % 2)
                canvas[PAGENUM / 2][WIDTH / 2] = reverseByte(canvas[PAGENUM / 2][WIDTH / 2]);
        }
    }
    if (vertical)
    {
        for (int page = PAGENUM; page--;)
        {
            for (int col = 0; col < WIDTH / 2; ++col)
            {
                uint8_t tmp;
                tmp = canvas[page][col];
                canvas[page][col] = canvas[page][WIDTH - 1 - col];
                canvas[page][WIDTH - 1 - col] = tmp;
            }
        }
    }
}

/**
 * @brief Shift all the columns of the canvas to the left or right.
 *
 * @param canvas
 * @param shift: Positive for left shift, negative for right shift.
 */
void lr_shift_canvas(Canvas canvas, const int shift, const bool fill_value)
{
    int start = MAX(shift, 0), end = MIN(WIDTH + shift, WIDTH);
    if (start >= end)
        return;

    for (unsigned page = PAGENUM; page--;)
    {
        memmove(canvas[page] + MAX(0, -shift), canvas[page] + start, end - start);
        memset(canvas[page] + ((shift < 0) ? 0 : (WIDTH - shift)), fill_value ? 0xff : 0x00, abs(shift));
    }
}

/**
 * @brief Shift the canvas to the up or down by pixels.
 *
 * @param canvas
 * @param shift: Positive for up shift, negative for down shift.
 */
void ud_shift_canvas(Canvas canvas, const int shift, const bool fill_value)
{
    int page_shift = shift / 8;
    int offset = shift - page_shift * 8;
    if (offset > 0)
    {
        for (int page = page_shift; page < PAGENUM - 1; ++page)
        {
            for (int col = 0; col < WIDTH; ++col)
            {
                canvas[page][col] = (canvas[page][col] >> offset) | (canvas[page + 1][col] << (8U - offset));
            }
        }
        for (int col = 0; col < WIDTH; ++col)
        {
            canvas[PAGENUM - 1][col] >>= offset;
            canvas[PAGENUM - 1][col] |= fill_value ? 0xff << (8U - offset) : 0x0;
        }
    }
    else if (offset < 0)
    {
        offset = -offset;
        for (int page = PAGENUM - 1 - (-page_shift); page > 0; --page)
        {
            for (int col = 0; col < WIDTH; ++col)
            {
                canvas[page][col] = (canvas[page][col] << offset) | (canvas[page - 1][col] >> (8U - offset));
            }
        }
        for (int col = 0; col < WIDTH; ++col)
        {
            canvas[0][col] <<= offset;
            canvas[0][col] |= fill_value ? 0xff >> (8U - offset) : 0x0;
        }
    }

    if (page_shift > 0)
    {
        for (int page = 0; page < PAGENUM - page_shift; ++page)
            memcpy(canvas[page], canvas[page + page_shift], WIDTH);
        memset(canvas + PAGENUM - page_shift, fill_value ? 0xff : 0x00, page_shift * WIDTH);
    }
    else if (page_shift < 0)
    {
        page_shift = -page_shift;
        for (int page = PAGENUM - 1; page >= page_shift; --page)
            memcpy(canvas[page], canvas[page - page_shift], WIDTH);
        memset(canvas, fill_value ? 0xff : 0x00, page_shift * WIDTH);
    }
}

void merge_canvas(Canvas dst, const Canvas src, const Canvas mask)
{
    for (unsigned page = PAGENUM; page--;)
    {
        for (unsigned col = WIDTH; col--;)
        {
            dst[page][col] = (dst[page][col] & ~mask[page][col]) | (src[page][col] & mask[page][col]);
        }
    }
}

bool flush_canvas(I2C_t *i2c, const Canvas canvas)
{
    for (unsigned page = 0; page < PAGENUM; ++page)
    {
        if (!set_cursor(i2c, page, 0))
            return false;
        if (!start_wbyte_stream(i2c, DATA_S))
            return false;
        for (unsigned col = 0; col < WIDTH; ++col)
        {
            if (!wbyte_I2C(i2c, canvas[page][col]))
                return false;
        }
        end_stream(i2c);
    }
    return true;
}

bool draw_filled_rect(Canvas canvas, unsigned x, unsigned y, unsigned width, unsigned height, bool value)
{
    if (x >= WIDTH || y >= HEIGHT)
        return false;
    unsigned filament = value ? 0xff : 0x0, y_end = MIN(y + height, HEIGHT);
    unsigned head = y % 8U, tail = y_end % 8U;
    if (head)
    {
        uint8_t head_mask = 0xff << head;
        if (height < 8U - head)
            head_mask &= 0xff >> (8U - head - height);
        for (unsigned i = x; i < MIN(x + width, WIDTH); ++i)
        {
            canvas[y / 8U][i] &= ~head_mask;
            canvas[y / 8U][i] |= filament & head_mask;
        }
    }
    if (tail && (!head || height > 8U - head))
    {
        uint8_t tail_mask = 0xff >> (8U - tail);
        for (unsigned i = x; i < MIN(x + width, WIDTH); ++i)
        {
            canvas[y_end / 8U][i] &= ~tail_mask;
            canvas[y_end / 8U][i] |= filament & tail_mask;
        }
    }
    for (unsigned page = y / 8U + !!head; page < y_end / 8U; ++page)
    {
        memset(canvas[page] + x, filament, MIN(width, WIDTH - x));
    }
    return true;
}

/**
 * @brief Draw a raster on the canvas. Raster should be in shape of [PAGENUM][WIDTH}, {PAGE0|COL0(8-bit data, MSB facing down)|}{PAGE1...}
 *
 */
bool draw_raster(Canvas canvas, unsigned x, unsigned y, unsigned width, unsigned pagenum, const uint8_t *raster)
{
    if (x >= WIDTH || y >= HEIGHT)
        return false;
    int y_end = MIN(y + pagenum * 8, HEIGHT);
    int offset = y % 8;
    uint8_t(*data)[width] = (uint8_t(*)[width])raster;

    if (offset)
    {
        uint8_t head_mask = 0xff << offset;

        for (int col = x; col < MIN(x + width, WIDTH); ++col)
        {
            canvas[y / 8][col] &= ~head_mask;
            canvas[y / 8][col] |= (data[0][col - x] << offset) & head_mask;
        }

        if (y_end / 8 < PAGENUM)
        {
            uint8_t tail_mask = 0xff >> (8 - offset);
            for (int col = x; col < MIN(x + width, WIDTH); ++col)
            {
                canvas[y_end / 8][col] &= ~tail_mask;
                canvas[y_end / 8][col] |= (data[pagenum - 1][col - x] >> (8 - offset)) & tail_mask;
            }
        }

        for (int page = y / 8 + 1, data_page = 1; page < y_end / 8; ++page, ++data_page)
        {
            for (int col = x; col < MIN(x + width, WIDTH); ++col)
            {
                canvas[page][col] = data[data_page][col - x] << offset | data[data_page - 1][col - x] >> (8 - offset);
            }
        }
    }
    else
    {
        for (int data_page = pagenum, page_start = y / 8; data_page--;)
            memcpy(canvas[page_start + data_page] + x, data[data_page], MIN(width, WIDTH - x));
    }
    return true;
}

bool draw_line(Canvas canvas, unsigned x0, unsigned y0, unsigned x1, unsigned y1, bool value)
{
    if (x0 > x1) // Always start from left to right
    {
        SWAP(x0, x1);
        SWAP(y0, y1);
    }

    int dx = x1 - x0;
    int dy = abs(y1 - y0);
    int step;
    int err = 0;

    if (x0 >= WIDTH) // line outside the canvas
        return false;

    if (y0 >= HEIGHT)
    {
        if (dx * (HEIGHT - 1 - y0) < dy * (WIDTH - 1 - x0))
            return false; // line outside the canvas
        y0 = HEIGHT - 1;
        x0 += dx * (HEIGHT - 1 - y0) / dy;
    } // (x0, y0) outside the canvas, remap to the edge of the canvas

    if (x1 >= WIDTH || y1 >= HEIGHT)
    {
        if (dy * (WIDTH - x0) + y0 >= HEIGHT * dx)
        {
            x1 = x0 + (HEIGHT - y0) * dx / dy;
            y1 = HEIGHT - 1;
        }
        else
        {
            y1 = y0 + (WIDTH - x0) * dy / dx;
            x1 = WIDTH - 1;
        } // (x1, y1) outside the canvas, remap to the edge of the canvas
    }

    if (!dx)
        return draw_filled_rect(canvas, x0, MIN(y0, y1), 1, dy + 1, value);

    if (!dy)
        return draw_filled_rect(canvas, x0, y0, dx + 1, 1, value);

    bool steep = dy > dx;
    if (steep)
    {
        if (y0 > y1) // Always start from bottom to top
        {
            SWAP(x0, x1);
            SWAP(y0, y1);
        }
        step = ((int)(x1 - x0) < 0) ? -1 : 1;
        for (unsigned x = x0, y = y0; y <= y1; ++y, err -= dx)
        {
            set_canvas_dot(canvas, x, y, value);

            if (err <= 0)
            {
                x += step;
                err += dy;
            }
        }
    }
    else
    {
        step = ((int)(y1 - y0) < 0) ? -1 : 1;
        for (unsigned x = x0, y = y0; x <= x1; ++x, err += dy)
        {
            set_canvas_dot(canvas, x, y, value);
            // if ((abs(y-y1)*dx - (int)(x1-x)*dy) > 0)
            //     y += ystep;

            if (err >= 0)
            {
                y += step;
                err -= dx;
            }
        }
    }
    return true;
}

bool draw_inf_line(Canvas canvas, unsigned x0, unsigned y0, unsigned x1, unsigned y1, bool value)
{
    if (x0 == x1)
        return draw_filled_rect(canvas, x0, 0, 1, HEIGHT, value);
    if (y0 == y1)
        return draw_filled_rect(canvas, 0, y0, WIDTH, 1, value);

    if (x0 < x1) // ensures b is positive
    {
        SWAP(x0, x1);
        SWAP(y0, y1);
    }

    int a = y1 - y0, b = x0 - x1, c = x1 * y0 - x0 * y1;

    // if (x0 >= WIDTH || x1 >= WIDTH || y0 >= HEIGHT || y1 >= HEIGHT)
    //     {
    //         int origin_x = WIDTH/2, origin_y = HEIGHT/2;
    //         int max_d2 = origin_x * origin_x + origin_y * origin_y;
    //         int axo_byo_c = a*origin_x + b*origin_y + c;
    //         if ((a*a + b*b) * max_d2 < axo_byo_c * axo_byo_c)
    //             return false;  // line outside the canvas
    //     }

    // remap to the edge of the canvas
    const int below = 0, between = 1, above = 2;
    int left, right;

    if (c > 0) // b is ensured to be positive
        left = below;
    else if (b * (HEIGHT - 1) + c < 0)
        left = above;
    else
        left = between;

    if (a * (WIDTH - 1) + c > 0)
        right = below;
    else if (a * (WIDTH - 1) + b * (HEIGHT - 1) + c < 0)
        right = above;
    else
        right = between;

    // reuse x0, y0, x1, y1 to store the remapped coordinates

    if (left == right)
    {
        if (left != between)
            return false; // line outside the canvas
        x0 = 0;
        y0 = -c / b;
        x1 = WIDTH - 1;
        y1 = -(a * (WIDTH - 1) + c) / b;
    }
    else if (left == above)
    {
        y0 = HEIGHT - 1;
        x0 = -(b * (HEIGHT - 1) + c) / a;
        if (right == below)
        {
            y1 = 0;
            x1 = -c / a;
        }
        else
        {
            x1 = WIDTH - 1;
            y1 = -(a * (WIDTH - 1) + c) / b;
        }
    }
    else // left == below
    {
        y0 = 0;
        x0 = -c / a;
        if (right == above)
        {
            y1 = HEIGHT - 1;
            x1 = -(b * (HEIGHT - 1) + c) / a;
        }
        else
        {
            x1 = WIDTH - 1;
            y1 = -(a * (WIDTH - 1) + c) / b;
        }
    }

    return draw_line(canvas, x0, y0, x1, y1, value);
}

bool draw_rect(Canvas canvas, unsigned x, unsigned y, unsigned width, unsigned height, bool value)
{
    if (x >= WIDTH || y >= HEIGHT)
        return false;

    bool ret = false;

    ret |= draw_line(canvas, x, y, x + width - 1, y, value);
    ret |= draw_line(canvas, x, y + height - 1, x + width - 1, y + height - 1, value);
    ret |= draw_line(canvas, x, y, x, y + height - 1, value);
    ret |= draw_line(canvas, x + width - 1, y, x + width - 1, y + height - 1, value);
    return ret;
}

bool draw_circle(Canvas canvas, unsigned x0, unsigned y0, unsigned radius, bool value)
{
    if (x0 >= WIDTH || y0 >= HEIGHT)
        return false;

    int x = radius;
    int y = 0;
    int radiusError = 1 - x;
    bool ret = false;
    while (x >= y)
    {
        ret |= set_canvas_dot(canvas, x + x0, y + y0, value);
        ret |= set_canvas_dot(canvas, y + x0, x + y0, value);
        ret |= set_canvas_dot(canvas, -x + x0, y + y0, value);
        ret |= set_canvas_dot(canvas, -y + x0, x + y0, value);
        ret |= set_canvas_dot(canvas, -x + x0, -y + y0, value);
        ret |= set_canvas_dot(canvas, -y + x0, -x + y0, value);
        ret |= set_canvas_dot(canvas, x + x0, -y + y0, value);
        ret |= set_canvas_dot(canvas, y + x0, -x + y0, value);
        y++;
        if (radiusError < 0)
        {
            radiusError += 2 * y + 1;
        }
        else
        {
            x--;
            radiusError += 2 * (y - x + 1);
        }
    }
    return ret;
}

bool draw_filled_circle(Canvas canvas, unsigned x0, unsigned y0, unsigned radius, bool value)
{
    if (x0 >= WIDTH || y0 >= HEIGHT)
        return false;

    int x = radius;
    int y = 0;
    int radiusError = 1 - x;
    bool ret = false;
    while (x >= y)
    {
        ret |= draw_line(canvas, x0 - x, y0 + y, x0 + x, y0 + y, value);
        ret |= draw_line(canvas, x0 - y, y0 + x, x0 + y, y0 + x, value);
        ret |= draw_line(canvas, x0 - x, y0 - y, x0 + x, y0 - y, value);
        ret |= draw_line(canvas, x0 - y, y0 - x, x0 + y, y0 - x, value);
        y++;
        if (radiusError < 0)
        {
            radiusError += 2 * y + 1;
        }
        else
        {
            x--;
            radiusError += 2 * (y - x + 1);
        }
    }
    return ret;
}

bool draw_polygon(Canvas canvas, const unsigned points[][2], unsigned num_points, bool value)
{
    bool ret = false;
    if (num_points < 2)
        return false;
    for (unsigned i = 0; i < num_points - 1; ++i)
        ret |= draw_line(canvas, points[i][0], points[i][1], points[i + 1][0], points[i + 1][1], value);

    return ret | draw_line(canvas, points[num_points - 1][0], points[num_points - 1][1], points[0][0], points[0][1], value);
}

bool draw_filled_triangle(Canvas canvas, unsigned x0, unsigned y0, unsigned x1, unsigned y1, unsigned x2, unsigned y2, bool value)
{
    bool ret = false;

    // sort the points by y
    if (y0 > y1)
    {
        SWAP(x0, x1);
        SWAP(y0, y1);
    }
    if (y1 > y2)
    {
        SWAP(x1, x2);
        SWAP(y1, y2);
    }
    if (y0 > y1)
    {
        SWAP(x0, x1);
        SWAP(y0, y1);
    }

    if (y0 >= HEIGHT)
        return false; // triangle outside the canvas

    if (y0 == y2)
        return draw_line(canvas, MIN(x0, MIN(x1, x2)), y0, MAX(x0, MAX(x1, x2)), y1, value);

    int err0 = 0, err1 = 0;
    int dx0 = x1 - x0, dx1 = x2 - x0;
    int dy0 = y1 - y0, dy1 = y2 - y0;
    bool steep0 = abs(dy0) > abs(dx0), steep1 = abs(dy1) > abs(dx1);
    if (steep0)
    {
        SWAP(x0, y0);
        SWAP(dx0, dy0);
    }
    if (steep1)
    {
        SWAP(x0, y0);
        SWAP(dx1, dy1);
    }
    int step0 = ((int)(dx0) < 0) ? -1 : 1;
    int step1 = ((int)(dx1) < 0) ? -1 : 1;
    dx0 = abs(dx0);
    dx1 = abs(dx1);

    for (unsigned y = y0, x0_ = x0, x1_ = x0; y < y1; ++y)
    {
        if (steep0)
            ret |= draw_filled_rect(canvas, y, x0_, x1_ - x0_ + 1, 1, value);
        else
            ret |= draw_filled_rect(canvas, x0_, y, x1_ - x0_ + 1, 1, value);

        err0 += dx0;
        if (err0 > 0)
        {
            x0_ += step0;
            err0 -= dy0;
        }

        err1 += dx1;
        if (err1 > 0)
        {
            x1_ += step1;
            err1 -= dy1;
        }
    }
    err0 = 0;
    dx0 = x2 - x1;
    dy0 = y2 - y1;
    steep0 = abs(dy0) > abs(dx0);
    if (steep0)
    {
        SWAP(x1, y1);
        SWAP(dx0, dy0);
    }
    step0 = ((int)(dx0) < 0) ? -1 : 1;
    dx0 = abs(dx0);

    for (unsigned y = y1, x0_ = x1, x1_ = x1; y <= y2; ++y)
    {
        if (steep0)
            ret |= draw_filled_rect(canvas, y, x0_, x1_ - x0_ + 1, 1, value);
        else
            ret |= draw_filled_rect(canvas, x0_, y, x1_ - x0_ + 1, 1, value);

        err0 += dx0;
        if (err0 > 0)
        {
            x0_ += step0;
            err0 -= dy0;
        }

        err1 += dx1;
        if (err1 > 0)
        {
            x1_ += step1;
            err1 -= dy1;
        }
    }
    return ret;
}

bool draw_filled_polygon(Canvas canvas, const unsigned points[][2], unsigned num_points, bool value)
{
    if (num_points < 2)
        return false;
}

bool fill_block(Canvas canvas, unsigned x, unsigned y, bool value)
{
    if (x >= WIDTH || y >= HEIGHT || get_canvas_dot(canvas, x, y) == value)
        return false;

#define _MAX_DEPTH (WIDTH * HEIGHT)
    uint8_t stack[_MAX_DEPTH][2];
    unsigned stack_top = 1;
    stack[0][0] = x;
    stack[0][1] = y;

    do
    {
        --stack_top;
        x = stack[stack_top][0];
        y = stack[stack_top][1];
        if (get_canvas_dot(canvas, x, y) == value)
            continue;
        set_canvas_dot(canvas, x, y, value);
        if (x > 0 && get_canvas_dot(canvas, x - 1, y) != value)
        {
            stack[stack_top][0] = x - 1;
            stack[stack_top][1] = y;
            ++stack_top;
        }
        if (y > 0 && get_canvas_dot(canvas, x, y - 1) != value)
        {
            stack[stack_top][0] = x;
            stack[stack_top][1] = y - 1;
            ++stack_top;
        }
        if (x < WIDTH - 1 && get_canvas_dot(canvas, x + 1, y) != value)
        {
            stack[stack_top][0] = x + 1;
            stack[stack_top][1] = y;
            ++stack_top;
        }
        if (y < HEIGHT - 1 && get_canvas_dot(canvas, x, y + 1) != value)
        {
            stack[stack_top][0] = x;
            stack[stack_top][1] = y + 1;
            ++stack_top;
        }
        // extern I2C_t i2c;
        // flush_canvas(&i2c, canvas);
        // HAL_Delay(1);
    } while (stack_top);

    return true;
#undef _MAX_DEPTH
}
