#include <string>
#include <chrono>
#include <array>
#include <algorithm>
#include "units.hpp"
#include "mcu/mcu.hpp"
#include "utils/ffmt.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;
int main()
{
    bool ret = true;

    using namespace elfe;
    using namespace stm32;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();

    PinConfig config(
        PinConfig::Output,
        PinConfig::VeryHigh,
        PinConfig::PushPull
        );
    Pin led(LED_GPIO_PORT, LED_PIN, config);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    auto hires_clk = std::chrono::high_resolution_clock();
    [[maybe_unused]]auto now = hires_clk.now();
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

    auto wdg = wdg::IndependentWatchDog();
    wdg.start();
    wdg.set_max_hungry_time(1s);
    try
    {
        while (true)
        {
            usrt.write(ffmt::format("watch dog bites? {}\n", wdg.was_reset_by_me()));
            usrt.write(ffmt::format("watch dog max {}ms\n", wdg.get_max_hungry_time().count()/1000));
            usrt.write(ffmt::format("watch dog reload {:.x}\n", wdg.get_reload()));
            usrt.write(ffmt::format("watch dog prescaler {:.x}\n", (unsigned)wdg.get_prescaler()));
            clock::delay(500ms);
            wdg.feed();
        }
    }
    catch (const std::exception &e)
    {
        usrt.write(ffmt::format("Exception: {}\n", e.what()));
    }

    //render.render("Hello, world!\n", 0, 0);

    while (ret)
    {
        led.toggle();
        clock::delay(1s);
    }
}