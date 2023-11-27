#include <string>
#include <chrono>
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

    using std::chrono::operator ""s;
    using std::chrono::operator ""ms;
    using std::chrono::operator ""ns;
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
    //auto hires_clk = std::chrono::high_resolution_clock();
    //auto now = hires_clk.now();
    try
    {
        uintptr_t addr = 0x0807000B;
        // flash::Flash.erase_all();
        //render.format_at(0, 0, "{:.x}", flash::Flash.get<int>(addr));
        flash::Flash.erase(addr, 4);
        //flash::Flash.on_complete = [&render]()
        //{
        //    render.format_at(0, 0, "Flash complete!\n");
        //};
        flash::Flash.enable_interrupts();
        flash::Flash.enable_irq();
        flash::Flash.put<uint64_t>(addr, 0xdeadbeef);
        //flash::test(addr);
        render.format_at(0, 0, "{:.p}", flash::Flash.get<uint64_t>(addr));
        // while(true)
        //     render.format_at(0, 0, "{}\n", now.time_since_epoch().count());
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