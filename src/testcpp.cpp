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
    try
    {
        auto &timer = clock::Timer4;
        auto &dma = dma::DirectMemAccess2;
        auto &stream = dma.streams[0];
        dma.init();
        char buf1[100] = "Hello, world!\n";
        char buf2[100] = "";
        stream.enable_interrupts();
        stream.enable_irq();
        stream.peri_addr = (uint8_t*)buf1;
        stream.dst0_addr = (uint8_t*)buf2;
        stream.count = 100;
        stream.set_direction(dma::Direction::Mem2Mem);
        stream.set_peri_unit_size(dma::UnitSize::Byte);
        stream.set_peri_inc(true);
        stream.set_dst_unit_size(dma::UnitSize::Byte);
        stream.set_dst_inc(true);
        stream.on_complete = [&render, &buf2]()
        {
            render.format_at(2, 0, buf2);
        };
        stream.enable();
        timer.init();
        timer.set_clock_source(clock::ClockSourceConfig{
            //.source = clock::ClockSource::ExternalMode1,
            .polarity = clock::ClockPolarity::Inverted,
            .prescaler = clock::ClockPrescaler::Div2,
            .extern_freq = 114514,
            .filter = 13U,
        });
        timer.set_time_base(clock::TimeBaseConfig());
        // timer.set_auto_reload(10000 - 1);
        // timer.set_prescaler(1000);
        //timer.set_period_time(1s);
        timer.set_frequency(1);
        timer.on_reload = [&render, &count]()
        {
            render.format_at(0, 0, "Timer count {} {}\n", ++count, timer.direction());
        };
        timer.enable_irq();
        timer.start();
        //if (false
        //    or timer.get_clock_source().source != clock::ClockSource::ExternalMode1
        //    or timer.get_clock_source().polarity != clock::ClockPolarity::Inverted
        //    or timer.get_clock_source().prescaler != clock::ClockPrescaler::Div2
        //    or timer.get_clock_source().extern_freq != 114514
        //    or timer.get_clock_source().filter != 13U
        //    )
        //    throw std::runtime_error("Timer clock source not set correctly");
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