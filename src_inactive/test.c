#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "GPIO.h"
#include "CLK_CFG.h"
#include "DWT_Delay.h"

#define LED_PIN GPIO_PIN_13
#define LED_GPIO_PORT GPIOC

#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_GPIO_PORT GPIOB

void LED_Init();
void BUTTON_Init();

int main()
{
    HAL_Init();
    SystemClock_Config();
    //MX_RTC_Init();
    LED_Init();
    BUTTON_Init();
    DWT_delay_init();

    while (true)
    {
        HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        //LED_GPIO_PORT->ODR ^= LED_PIN;
        HAL_Delay(1000);
    }
}

void LED_Init()
{
    enable_GPIO_CLK(LED_GPIO_PORT);
    init_GPIO(
        LED_GPIO_PORT,
        LED_PIN,
        GPIO_MODE_OUTPUT_PP,
        GPIO_PULLUP,
        GPIO_SPEED_HIGH
        );
}

void BUTTON_Init()
{
    enable_GPIO_CLK(BUTTON_GPIO_PORT);
    init_GPIO(
        BUTTON_GPIO_PORT,
        BUTTON_PIN,
        GPIO_MODE_INPUT,
        GPIO_PULLUP,
        GPIO_SPEED_HIGH
        );
}

void SysTick_Handler()
{
    HAL_IncTick();
}