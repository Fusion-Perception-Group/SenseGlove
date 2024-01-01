#include <string>
#include <tuple>
#include <chrono>
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "ssd1306_canvas.hpp"

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

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    // PinConfig hw_i2c_cfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::OpenDrain, 4);
    // hw_i2c_cfg.pull = PinConfig::PullUp;
    // Pin scl(PortB, 6, hw_i2c_cfg), sda(PortB, 7, hw_i2c_cfg);
    // auto &i2c = i2c::I2c1;
    // i2c.init();
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);
    render.render("START!");
    auto start = clock::get_systick_ms();

    try
    {
        ssd1306::Canvas canvas(display);
        // for (unsigned h=0; h < ssd1306::PAGES*8; ++h)
        // {
        //     for (unsigned col=0; col < ssd1306::COLS; ++col)
        //     {
        //         canvas.set_pixel(col, h, true);
        //         // clock::delay(1ms);
        //     }
        // }
        // canvas.uni_draw_line(-256, 0, 1297, 63);
        // canvas.uni_draw_line(256, 0, -33, 63);
        // canvas.uni_draw_line(0, 63, 127, 0);
        // canvas.uni_draw_line(0, 0, 127, 0);
        // canvas.uni_draw_line(0, 0, 0, 63);
        // canvas.uni_draw_line(127, 40, 127, 40);
        // canvas.uni_draw_line(15, 16, 15, 37, directy::PX_MAX, directy::LineType::Infinite);
        // canvas.uni_draw_line(15, 16, 20, 37, directy::PX_MAX, directy::LineType::Infinite);
        // canvas.uni_draw_line(15, 16, 20, 0, directy::PX_MAX, directy::LineType::Infinite);
        // canvas.uni_draw_line(11, 32, 20, 32, directy::PX_MAX, directy::LineType::Infinite);
        // canvas.uni_draw_rect(0, 0, 127, 63);
        // canvas.uni_draw_filled_rect(16, 16, 32, 32);
        // canvas.uni_draw_circle(64, 32, 16);
        canvas.create_child<directy::LineNode>(0, 0, 127, 63);
        canvas.draw();
        for (unsigned i = 0; i < 64; ++i)
        {
            canvas.draw(64, 32, 0.1, 0.1, 1.5*i/64.0);
            canvas.flush();
            clock::delay(10ms);
        }
        canvas.draw(64, 32, 0.1, 0.1, 1.5);
        canvas.flush();
        auto end = clock::get_systick_ms();
        render.format_at(0, 0, "Time: {}ms", end - start);
        while (true)
        {
            ;
        }
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
    }
}
