#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "GPIO.h"
#include "I2C.h"
#include "CLK_CFG.h"
#include "SSD1306.h"

#define OLED_SDA GPIO_PIN_7
#define OLED_SCL GPIO_PIN_6
#define OLED_PORT GPIOB
#define OLED_ADDR SSD1306_ADDR
#define CMD SSD1306_CMD
#define CMD_S SSD1306_CMD_STREAM
#define DATA SSD1306_DATA
#define DATA_S SSD1306_DATA_STREAM
#define Canvas SSD1306_Canvas_t

I2C_t i2c = {
    .GPIOx = OLED_PORT,
    .sda = OLED_SDA,
    .scl = OLED_SCL,
    .delay_us = 0,
    .arb_lost = false
};

int main()
{
    HAL_Init();
    SystemClock_Config();

    bool ret = true;
    HAL_Delay(100);
    ret = init_display(&i2c);
    set_memory_addressing_mode(&i2c, 0x0);
    clear_display(&i2c);

    //for (char c = 0x20; c < 0x7f; c++)
    //{
    //    ret = print_char(&i2c, c);
    //    HAL_Delay(10);
    //}

    //clear_display(&i2c);

    //dnprintf(&i2c, 32, "Hello, world! %d", 1234567890);
    //HAL_Delay(1000);

    Canvas canvas = {0};

    unsigned points[][2] = {
        {64, 0},
        {70, 18},
        {88, 18},
        {74, 30},
        {81, 49},
        {64, 36},
        {48, 48},
        {54, 30},
        {40, 18},
        {58, 18},
    };

    //draw_rect(canvas, 16, 4, 32, 16, true);
    //draw_filled_rect(canvas, 32, 24, 32, 16, true);
    //draw_line(canvas, 0, 0, 127, 54, true);
    //draw_line(canvas, 0, 54, 127, 0, true);
    //draw_line(canvas, 0, 27, 127, 2, true);
    //draw_line(canvas, 32, 16, 32, 48, true);
    //draw_inf_line(canvas, 64, 36, 65, 39, true);
    //draw_circle(canvas, 96, 32, 16, true);
    //draw_filled_circle(canvas, 96, 32, 8, true);
    draw_polygon(canvas, points, sizeof(points)/2/sizeof(unsigned), true);
    //fill_block(canvas, 64, 32, true);
    //draw_raster(canvas, 5, 28, 8*4, 9, FONT_8x8['!']);
    //draw_raster(canvas, 5, 25, 8, 1, FONT_8x8['T']);
    //draw_raster(canvas, 13, 25, 8, 1, FONT_8x8['O']);
    //draw_raster(canvas, 21, 25, 8, 1, FONT_8x8['P']);
    cvnprintf(canvas, 32, 5, 25, "TOP");
    //draw_raster(canvas, 99, 25, 8, 1, FONT_8x8['G']);
    //draw_raster(canvas, 107, 25, 8, 1, FONT_8x8['U']);
    //draw_raster(canvas, 115, 25, 8, 1, FONT_8x8['N']);
    cvnprintf(canvas, 32, 99, 25, "GUN");
    //draw_filled_polygon(canvas, points, sizeof(points)/2/sizeof(unsigned), true);

    // draw a mesh
    //for (unsigned i = 0; i < 128; i += 8)
    //{
    //    draw_line(canvas, i, 0, i, 63, true);
    //    draw_line(canvas, 0, i, 127, i, true);
    //}
    //draw_line(canvas, 127, 0, 127, 63, true);

    //inverse_canvas(canvas);
    //lr_shift_canvas(canvas, -18, 0);
    //ud_shift_canvas(canvas, -24, 0);

    for (int i = 0; i <= 128; ++i)
    {
        Canvas dummy, dummy1;
        //project_canvas(dummy1, canvas, 14, 2, 114, 7, 2, 62, 126, 60, 0);
        //stretch_canvas(dummy1, canvas, 1.5, .75, 0);
        rotate_canvas(dummy, canvas, i*PI/64.0, 0);
        scale_canvas(dummy1, dummy, i/128.0, 0);
        flush_canvas(&i2c, dummy1);
    }

    //flip_canvas(canvas, true, true);
    flush_canvas(&i2c, canvas);

    //for (uint8_t flag = 0xff;;flag = ~flag)
    //{
    //    for (unsigned page = 8; page--;)
    //    {
    //        for (uint8_t value = 0xff; value;)
    //        {
    //            value >>= 1;
    //            set_cursor(&i2c, page, 0);
    //            select_I2C(&i2c, OLED_ADDR, I2C_WRITE);
    //            wbyte_I2C(&i2c, DATA_S);
    //            for (unsigned col = 128; col--;)
    //                wbyte_I2C(&i2c, flag ^ value);
    //            _terminate_I2C(&i2c);
    //            HAL_Delay(10);
    //        }
    //    }
    //}

    //while (true)
    //{
    //    ret = fill_display(&i2c, 0xFF);
    //    //HAL_Delay(500);
    //    ret = clear_display(&i2c);
    //    //HAL_Delay(500);
    //}

    enable_GPIO_CLK(GPIOC);
    init_GPIO(GPIOC, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (GPIO_PinState)!ret);
    while(true)
    {
        if (!ret)
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}

void SysTick_Handler()
{
    HAL_IncTick();
}