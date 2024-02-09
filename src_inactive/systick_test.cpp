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

    using namespace vms;
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

    Pin sclk(PortA, 5), miso(PortA, 6), mosi(PortA, 7);
    PinConfig spicfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::PushPull, 5);
    spicfg.pull = PinConfig::NoPull;
    sclk.load(spicfg);
    miso.load(spicfg);
    mosi.load(spicfg);
    Pin cs(PortA, 4);
    cs.io = PinConfig::Output;
    cs.pull = PinConfig::PullUp;
    cs.speed = PinConfig::VeryHigh;
    cs.out_mode = PinConfig::PushPull;
    cs.reset();
    try
    {
        [[maybe_unused]]auto &hardi2c = i2c::I2c1;
        auto &spi = spi::Spi1;
        spi.init();
        while(true)
        {
            uint32_t last = clock::get_systick_ms();
            uint32_t hal_last = HAL_GetTick();
            clock::delay(1s);
            usrt.write(ffmt::format("hal ticks {}\n", HAL_GetTick()-hal_last));
            usrt.write(ffmt::format("{}ms diff {}ms\n", clock::get_systick_ms(), (clock::get_systick_ms() - last)));
            usrt.write(ffmt::format("ticks {}\n", clock::get_systick()));
            usrt.write(ffmt::format("clock speed {}Mhz\n", clock::SystemCoreClock/1_MHz));
            // usrt.write(ffmt::format("VAL {}\n", SysTick->VAL&0xFFFFFF));
            // usrt.write(ffmt::format("LOAD {}\n", SysTick->LOAD&0xFFFFFF));
        }
        //spi.baudrate = 5_MHz;
        //hardi2c.set_speed(i2c::Speed::FastPlus);
        // usrt.write(ffmt::format("Clock speed: {}KHz\n", hardi2c.clock_speed()/1_KHz));
        // usrt.write(ffmt::format("Rise Time: {}ns\n", hardi2c.max_rise_time().count()));
        // usrt.write(ffmt::format("CCR: {}\n", hardi2c.reg.CCR & 0xFFFU));
        // usrt.write(ffmt::format("TRISE: {}\n", hardi2c.reg.TRISE & 0x3FU));
   
        uint32_t id;
        //id = spi::test();
        usrt.write(ffmt::format("baudrate: {}MHz\n", spi.baudrate/1_MHz));
        spi.put<uint8_t>(0x9F);
        uint8_t manu = spi.get<uint8_t>();
        id = spi.get<uint8_t>() << 8;
        id |= spi.get<uint8_t>();
        usrt.write(ffmt::format("Made by {:.x} ID: {:.x}\n", manu, id));
    }
    catch (const std::exception &e)
    {
        usrt.write(ffmt::format("Exception: {}\n", e.what()));
    }

    //render.render("Hello, world!\n", 0, 0);

    while (ret)
    {
        led.toggle();
        timer.delay_ms(1000);
    }
}