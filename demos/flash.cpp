#include <string>
#include <chrono>
#include "mcu/mcu.hpp"
#include "extra/ssd1306/ssd1306.hpp"
#include "extra/ssd1306/texrender.hpp"
#include "utils/ffmt.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;
int main()
{
    bool ret = true;

    using std::chrono::operator ""s;
    using std::chrono::operator ""ms;
    using std::chrono::operator ""ns;
    using namespace elfe;
    using namespace stm32;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init().ok();

    PinConfig config(
        PinConfig::Output,
        PinConfig::VeryHigh,
        PinConfig::PushPull
        );
    Pin led(LED_GPIO_PORT, LED_PIN, config);

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear().ok();

    ssd1306::TexRender render(display);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    Pin button(PortA, 0, PinConfig(PinConfig::Input,
        PinConfig::PullUp, PinConfig::Interrupt, PinConfig::Falling));

    int count = 0;

    button.on_interrupt = [&render, &count]()
    {
        render.format_at(0, 0, "Button pressed\n{} times!\n", ++count).ok();
    };

    button.enable_interrupt().ok();
    //auto hires_clk = std::chrono::high_resolution_clock();
    //auto now = hires_clk.now();
    try
    {
        uintptr_t addr = 0x0807000B;
        // flash::Flash.erase_all();
        //render.format_at(0, 0, "{:.x}", flash::Flash.get<int>(addr));
        flash::Flash.erase(addr, 4).ok();
        //flash::Flash.on_complete = [&render]()
        //{
        //    render.format_at(0, 0, "Flash complete!\n");
        //};
        std::vector<uint8_t> data = {0x12, 0x34, 0x56, 0x78};
        flash::Flash.enable_interrupts().ok();
        flash::Flash.write(addr, data).ok();
        std::array<uint8_t, 4> vc2;
        flash::Flash.read(addr, vc2).ok();
        for (auto &v : vc2)
            render.format("{:.x}\n", v).ok();
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what()).ok();
    }

    while (ret)
    {
        led.toggle();
        clock::delay(1000ms);
    }
}