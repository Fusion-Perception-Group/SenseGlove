#include <string>
#include <chrono>
#include <array>
#include <algorithm>
#include "units.hpp"
#include "mcu.hpp"
#include "time.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "w25qxx.h"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;
int main()
{
    bool ret = true;

    using namespace vermils;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();

    HighResTimer timer;

    PinConfig config(
        PinConfig::Output,
        PinConfig::VeryHigh,
        PinConfig::PushPull
        );
    Pin led(LED_GPIO_PORT, LED_PIN, config);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    auto &usrt = usart::Usart1;
    [[maybe_unused]]uintptr_t addr = 0x0807000B;
    Pin tx(PortA, 9), rx(PortA, 10);
    PinConfig urtcfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::PushPull, 7);
    tx.load(urtcfg);
    rx.load(urtcfg);
    usrt.init();
    //usrt.baudrate = 230400;
    usrt.set_word_length(usart::WordLength::Bits9);
    usrt.set_stop_bits(usart::StopBits::Two);
    usrt.set_parity(usart::Parity::Even);
    usrt.write(ffmt::format("Start!\n"));

    try
    {
        // usrt.write(ffmt::format("Clock speed: {}KHz\n", hardi2c.clock_speed()/1_KHz));
        // usrt.write(ffmt::format("Rise Time: {}ns\n", hardi2c.max_rise_time().count()));
        // usrt.write(ffmt::format("CCR: {}\n", hardi2c.reg.CCR & 0xFFFU));
        while (true)
        {
            //usrt.write(ffmt::format("{}\n", std::chrono::system_clock::now().time_since_epoch().count()));
            usrt.write(ffmt::format("{}\n", RTC->TR));
            clock::delay(1s);
        }
    }
    catch (const std::exception &e)
    {
        usrt.write(ffmt::format("Exception: {}\n", e.what()));
    }

    while (ret)
    {
        led.toggle();
        clock::delay(1s);
    }
}