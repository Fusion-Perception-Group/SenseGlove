#include <string>
#include <chrono>
#include "CLK_CFG.h"
#include "time.hpp"
//#include "MCU.hpp"
#include "nvic.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "clock.hpp"
#include "dma.hpp"
#include "adc.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    HAL_Init();
    SystemClock_Config();
    bool ret = true;

    using std::chrono::operator ""s;
    using std::chrono::operator ""ms;
    using std::chrono::operator ""ns;
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
    ret &= display.clear();

    ssd1306::TexRender render(display);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    Pin button(PortA, 0, PinConfig(PinConfig::Input,
        PinConfig::PullUp, PinConfig::Interrupt, PinConfig::Falling));

    int count = 0;

    button.on_interrupt = [&render, &count]()
    {
        render.format_at(0, 0, "Button pressed\n{} times!\n", ++count);
    };

    button.enable_irq();
    auto hires_clk = std::chrono::high_resolution_clock();
    auto now = hires_clk.now();
    try
    {
        while(true)
            render.format_at(0, 0, "{}\n", &clock::Tim1);
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
    }

    //render.render("Hello, world!\n", 0, 0);

    while (ret)
    {
        led.toggle();
        timer.delay_ms(1000);
    }
}

extern "C" void SysTick_Handler()
{
    HAL_IncTick();
}