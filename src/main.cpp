#include <string>
#include <chrono>
#include "ffmt.hpp"
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"

using namespace vermils;
using namespace stm32;

int main()
{
    mcu::init();
    auto wdg = wdg::IndependentWatchDog();
    wdg.set_max_hungry_time(1000ms);
    
    gpio::PinConfig usart_cfg(
        gpio::PinConfig::AF,
        gpio::PinConfig::VeryHigh,
        gpio::PinConfig::PushPull,
        8  // USART6 Alternate Function
    );
    gpio::Pin tx(gpio::PortA, 11, usart_cfg), rx(gpio::PortA, 12, usart_cfg);
    auto &usart = usart::Usart6;
    usart.init();
    usart.write("Start!\n");

    try
    {
        gpio::PinConfig pwm_config(
            gpio::PinConfig::AF,
            gpio::PinConfig::VeryHigh,
            gpio::PinConfig::PushPull,
            2  // TIM3 Alternate Function
        );

        gpio::Pin pwm_pin1(gpio::PortB, 0, pwm_config);
        gpio::Pin pwm_pin2(gpio::PortB, 1, pwm_config);

        auto pwm_tim = clock::tim::Tim3;
        pwm_tim.init();

        const double duty_ratio = 0.3;
        const uint32_t pwm_freq = 170;

        pwm_tim.channel3.enable();
        pwm_tim.channel4.enable();

        pwm_tim.channel3.set_pwm(pwm_freq, duty_ratio, true);
        pwm_tim.channel4.set_pwm(pwm_freq, duty_ratio, false);

        pwm_tim.start();
        usart.write("PWM started\n");
        clock::delay(100ms);  // screen needs wait for some time before init
        auto i2c = i2c::SoftMaster(gpio::Pin(gpio::PortB, 7), gpio::Pin(gpio::PortB, 6));
        auto screen = ssd1306::I2CDisplay(i2c);
        screen.clear();
        usart.write("Screen cleared\n");
        auto render = ssd1306::TexRender(screen);
        usart.write("Render created\n");


        while (true)
        {
            render.format_at(0, 0, "counter {:06}", pwm_tim.counter);
            render.format_at(1, 0, "ch3 ccr {:06}", pwm_tim.channel3.capcomreg);
            render.format_at(2, 0, "ch4 ccr {:06}", pwm_tim.channel4.capcomreg);
            clock::delay(50ms);
        }
    }
    catch (const std::exception &e)
    {
        usart.write(ffmt::format("Exception: {}\n", e.what()));
    }
    catch (...)
    {
        usart.write("Unknown exception\n");
    }

    while (true)
    {
        usart.write("End Looping\n");
        clock::delay(10s);
    }

    return 0;
}