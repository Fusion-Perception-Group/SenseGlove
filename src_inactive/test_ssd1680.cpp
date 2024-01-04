#include <string>
#include <tuple>
#include <chrono>
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "ssd1680.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    using namespace vermils;
    using namespace stm32;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();
    Pin sclk(PortA, 5), miso(PortA, 6), mosi(PortA, 7);
    PinConfig spicfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::PushPull, 5);
    spicfg.pull = PinConfig::NoPull;
    sclk.load(spicfg);
    miso.load(spicfg);
    mosi.load(spicfg);
    clock::delay(1000ms);
    auto &spi = spi::Spi1;
    spi.init();

    gpio::Pin cs_pin(PortB, 10), dc_pin(PortB, 2), busy_pin(PortB, 8);
    // ssd1680::Driver driver(spi, cs_pin, dc_pin, busy_pin);
    ssd1680::Display<250, 122> screen(spi, cs_pin, dc_pin, busy_pin);

    gpio::Switch led(LED_GPIO_PORT, LED_PIN, true);

    gpio::Button button(PortA, 0, false, gpio::Button::Trigger::Press);
    button.on_interrupt = [&button,
                            // &driver,
                            &led] () {
        // driver.update();
        led.toggle();
    };
    button.enable_interrupt();


    auto &canvas = screen.bw;

    try
    {
        // driver.sw_reset();
        // clock::delay(10ms);
        // driver.send_command(ssd1680::Command::DriverOutputControl, 0x27, 0x01, 0x01);
        // driver.send_command(ssd1680::Command::DataEntryModeSetting, 0x06);
        // // driver.send_command(ssd1680::Command::WriteVCOMRegister, 0x36);
        // // driver.send_command(ssd1680::Command::GateDrivingVoltageControl, 0x17);
        // // driver.send_command(ssd1680::Command::SourceDrivingVoltageControl, 0x41, 0x00, 0x32);
        // driver.send_command(ssd1680::Command::BoosterSoftStartControl, 0xff, 0xff, 0xff, 0x00);
        // driver.send_command(ssd1680::Command::SetRAMXStartEndPos, 0x0f, 0x00);
        // driver.send_command(ssd1680::Command::SetRAMYStartEndPos, 0x00, 0x00, 0x27, 0x01);
        // driver.send_command(ssd1680::Command::BorderWaveformControl, 0x05);
        // driver.send_command(ssd1680::Command::DisplayUpdateControl1, 0x00, 0x80);
        // driver.send_command(ssd1680::Command::TemperatureSensorControl, 0x80);
        // driver.set_cursor(0xf, 0);
        // driver.power_on();
        // driver.init();

        // canvas.uni_draw_line(0, 0, 0, 5);
        canvas.uni_draw_line(256, 0, -33, 63);
        // canvas.uni_draw_line(0, 119, 295, 0);
        // canvas.uni_draw_line(0, 0, 127, 0);
        // canvas.uni_draw_line(0, 0, 0, 63);
        // canvas.uni_draw_line(127, 40, 127, 40);
        // canvas.uni_draw_line(15, 16, 15, 37, directy::PX_MAX,directy::PX_MAX, directy::LineType::Infinite);
        // canvas.uni_draw_line(15, 16, 20, 37, directy::PX_MAX,directy::PX_MAX, directy::LineType::Infinite);
        // canvas.uni_draw_line(15, 16, 20, 0, directy::PX_MAX,directy::PX_MAX, directy::LineType::Infinite);
        // canvas.uni_draw_line(11, 32, 20, 32, directy::PX_MAX,directy::PX_MAX, directy::LineType::Infinite);
        canvas.uni_draw_filled_rect(0, 0, 10, 10);
        // canvas.uni_draw_filled_rect(240, 0, 249, 10);
        // canvas.uni_draw_rect(0, 0, 249, 121);
        // canvas.uni_draw_filled_rect(0, 112, 9, 121);
        // canvas.uni_draw_filled_rect(16, 16, 32, 32);
        // canvas.uni_draw_circle(64, 32, 16);
        canvas.create_child<directy::LineNode>(0, 0, 127, 63);
        canvas.create_child<directy::TextBoxNode>("Hello, World! 12345678901234567890");
        canvas.draw();
        screen.red.uni_fill(directy::PX_MIN);
        // canvas.uni_fill(directy::PX_MIN);
        // canvas.uni_fill(directy::PX_MAX);

        screen.flush(true);

        for (int i = 0; i < 250*16; i++)
        {
            // // driver.send_data(0xff);
            // driver.send_red_data(0x00);
            // driver.send_bw_data(0xff);
        }

        // driver.full_update();

        while (true)
        {
            led.toggle();
            clock::delay(100ms);
        }
    }
    catch (const std::exception &e)
    {
        led.on();
        // canvas.create_child<directy::TextBoxNode>(e.what());
        // canvas.draw();
        // screen.flush(true);
    }
}
