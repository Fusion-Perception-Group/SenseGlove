#include <string>
#include "CLK_CFG.h"
#include "time.hpp"
//#include "MCU.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"


#define LED_PIN 13
#define LED_GPIO_PORT PortC

#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_GPIO_PORT GPIOB

void BUTTON_Init();

using std::string;

int main()
{
    HAL_Init();
    SystemClock_Config();
    // MX_RTC_Init();
    //LED_Init();
    BUTTON_Init();
    bool ret = true;

    using namespace vermils;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    HighResTimer timer;

    PinConfig config(
        PinConfig::Output,
        PinConfig::VeryHigh,
        PinConfig::PushPull
        );
    Pin led(LED_GPIO_PORT, LED_PIN, config);

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    timer.delay_ms(100);
    ssd1306::I2CDisplay display(i2c);
    //ret &= display.init();
    //ret &= display.set_entire_display_on(false);
    //ret &= display.set_entire_display_on(true);
    ret &= display.fill(0xAF);

    ssd1306::TexRender render(display);

    render.render(0, 0, "Hello, world!\n");
    render << "Goodbye, world!\n";
    render << "abcdefghijklmnopqrstuvwxyz";
    render << 'a';
    render << 'a';
    render << 123;
    render << string("This is a string");
    render.format("I know the answer {}", 42);
    render.render_char(4, 120, 'b');
    render.render_char(7, 128, 'z');

    while (ret)
    {
        led.toggle();
        //HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        // LED_GPIO_PORT->ODR ^= LED_PIN;
        //HAL_Delay(1000);
        timer.delay_ms(1000);
    }
}

void BUTTON_Init()
{
    //enable_GPIO_CLK(BUTTON_GPIO_PORT);
    //init_GPIO(
    //    BUTTON_GPIO_PORT,
    //    BUTTON_PIN,
    //    GPIO_MODE_INPUT,
    //    GPIO_PULLUP,
    //    GPIO_SPEED_HIGH);
}

extern "C" void SysTick_Handler()
{
    HAL_IncTick();
}