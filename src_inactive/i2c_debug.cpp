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

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    PinConfig i2ccfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::OpenDrain);
    i2ccfg.alternate = 4;
    scl.load(i2ccfg);
    sda.load(i2ccfg);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    auto hires_clk = std::chrono::high_resolution_clock();
    [[maybe_unused]]auto now = hires_clk.now();
    auto &usrt = usart::Usart1;
    uintptr_t addr = 0x0807000B;
    Pin tx(PortA, 9), rx(PortA, 10);
    PinConfig cfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::PushPull);
    cfg.alternate = 7;
    tx.load(cfg);
    rx.load(cfg);
    usrt.init();
    //usrt.baudrate = 230400;
    usrt.set_word_length(usart::WordLength::Bits9);
    usrt.set_stop_bits(usart::StopBits::Two);
    usrt.set_parity(usart::Parity::Even);
    usrt.write(ffmt::format("Start!\n"));
    try
    {
        [[maybe_unused]]auto &hardi2c = i2c::I2c1;
        hardi2c.init();
        //hardi2c.set_speed(i2c::Speed::FastPlus);
        hardi2c.clock_speed = 100_KHz;
        usrt.write(ffmt::format("pclk1: {}MHz\n", clock::rcc::get_pclk1()/1_MHz));
        usrt.write(ffmt::format("Clock speed: {}KHz\n", hardi2c.clock_speed()/1_KHz));
        usrt.write(ffmt::format("Rise Time: {}ns\n", hardi2c.max_rise_time().count()));
        usrt.write(ffmt::format("CCR: {}\n", hardi2c.reg.CCR & 0xFFFU));
        usrt.write(ffmt::format("TRISE: {}\n", hardi2c.reg.TRISE & 0x3FU));
        hardi2c.raise_if_error();
        timer.delay_ms(100);
        ssd1306::I2CDisplay display(hardi2c);
        ret &= display.clear();

        ssd1306::TexRender render(display);
        render.wrap = true;
        //for (int i = 0;; ++i)
        //{
        //    auto c = usrt.get<char>();
        //    render.render_char(c);
        //    usrt.put(c);
        //    //render.format_at(0, 0, "i = {}\n", i);
        //}
        render.format_at(0, 0, "{:.p}", flash::Flash.get<uint64_t>(addr));
        //while(true)
        //    render.format_at(0, 0, "{}\n", now.time_since_epoch().count());
        usrt.write(ffmt::format("Reached the end\n"));
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