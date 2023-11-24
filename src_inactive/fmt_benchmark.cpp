#include <string>
#include "CLK_CFG.h"
#include "time.hpp"
//#include "MCU.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"

//#include "fmt.hpp"
#include "ffmt.hpp"

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
    timer.delay_ms(100);

    PinConfig config(
        PinConfig::Output,
        PinConfig::VeryHigh,
        PinConfig::PushPull
        );
    Pin led(LED_GPIO_PORT, LED_PIN, config);


    uint32_t start = timer.get_cycles(), test_size=10000;
    
    for (unsigned i = 0; i < test_size; ++i)
    {
        //auto volatile v = fstring("%5{}, %5{}", "Hello", "World").get();
        //auto volatile v = ffmt::format("{}, {} {:+^9.2f}", "Hello", "World", 114.514);
        auto volatile v = ffmt::format("{}, {}", "Hello", "World");
    }
    uint32_t end = timer.get_cycles();
    double sec = double(end - start) / stm32::SystemCoreClock;
    double ksps = test_size / 1000 / sec;

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl, 1);

    ssd1306::I2CDisplay display(i2c);
    //ret = display.init();
    display.clear();

    ssd1306::TexRender render(display);

    render.render(0, 12, "FMT Benchmark!\n");
    //render << fstring("Speed: {} kstrings/s", ksps).get();
    render << ffmt::format("Speed:\n{:.3}\nkstrings/s", ksps);

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
    //HAL_IncTick();
}