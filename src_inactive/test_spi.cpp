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

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);

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
        uint32_t id;
        spi.put<uint8_t>(0x9F);
        uint8_t manu = spi.get<uint8_t>();
        id = spi.get<uint8_t>() << 8;
        id |= spi.get<uint8_t>();
        render.format("Made by {:.x} ID: {:.x}\n", manu, id);
    }
    catch (const std::exception &e)
    {
        render.format("Exception: {}\n", e.what());
    }

    //render.render("Hello, world!\n", 0, 0);

    while (ret)
    {
        led.toggle();
        timer.delay_ms(1000);
    }
}