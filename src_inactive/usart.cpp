#include <string>
#include <chrono>
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
        uintptr_t addr = 0x0807000B;
        Pin tx(PortA, 9), rx(PortA, 10);
        PinConfig cfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::PushPull);
        cfg.alternate = 7;
        tx.load(cfg);
        rx.load(cfg);

        auto &usrt = usart::Usart1;
        usrt.init();
        //usrt.baudrate = 230400;
        usrt.set_word_length(usart::WordLength::Bits9);
        usrt.set_stop_bits(usart::StopBits::Two);
        usrt.set_parity(usart::Parity::Even);
        //render.format("{}", usrt.baudrate());
        //usrt.set_parity(usart::Parity::Even);
        usrt.write("FUCK!!!");
        render.wrap = true;
        for (int i = 0;; ++i)
        {
            auto c = usrt.get<char>();
            render.render_char(c);
            usrt.put(c);
            //render.format_at(0, 0, "i = {}\n", i);
        }
        render.format_at(0, 0, "{:.p}", flash::Flash.get<uint64_t>(addr));
        while(true)
            render.format_at(0, 0, "{}\n", now.time_since_epoch().count());
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