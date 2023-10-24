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
    bool ret = true;
    HAL_Init();
    SystemClock_Config();
    enable_GPIO_CLK(GPIOC);
    init_GPIO(GPIOC, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);

    HAL_Delay(100);
    ret = init_display(&i2c);
    set_memory_addressing_mode(&i2c, 2);
    double result;
    result = benchmark_I2C(&i2c, 100);
    
    Canvas background, text = {0};
    cvnprintf(text, 127, 12, 8, "I2C Benchmark");
    scale_canvas(background, text, 1.15, 0);
    clear_canvas(text);
    cvnprintf(text, 127, 8, 24, "%.3lf kbit/s", result);
    cvnprintf(text, 127, 8, 40, "%.3lf us/bit", 1000 / result);

    merge_canvas(background, text, text);
    flush_canvas(&i2c, background);
    //dnprintf(&i2c, 127, "I2C Benchmark : %.3lf kbit/s Per bit cost %lf us", result, 1000 / result);

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