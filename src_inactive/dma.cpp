#include <string>
#include <chrono>
#include "units.hpp"
#include "mcu.hpp"
#include "time.hpp"
#include "nvic.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "dma.hpp"
#include "adc.hpp"

using std::string;

int main()
{
    using namespace vms;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;
    mcu::init();

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    try
    {
        auto &dma = dma::Dma2;
        auto &stream = dma.streams[0];
        dma.init();
        string str;
        //char buf1[100] = "FaQ world!\n";
        char const * buf1 = str.c_str();
        char buf2[100] = "";
        stream.src_addr = (uint8_t*)buf1;
        stream.dst_addr = (uint8_t*)buf2;
        // stream.reg.PAR = (uint8_t*)buf1;
        // stream.reg.M0AR = (uint8_t*)buf2;
        // stream.set_direction(dma::Direction::Ram2Ram);
        if (stream.get_direction() != dma::Direction::Ram2Ram)
            throw std::runtime_error(ffmt::format("Src: {}, Dst: {}, Dir: {}\n",
                stream.src_addr(), stream.dst_addr(), (unsigned)stream.get_direction()));
        stream.count = 100;
        stream.set_src_unit_size(dma::UnitSize::Byte);
        stream.set_src_inc(true);
        stream.set_circular_mode(true);
        stream.set_dst_unit_size(dma::UnitSize::Byte);
        stream.set_dst_inc(true);
        stream.enable_interrupt_complete();
        stream.on_complete = [&render, &buf2, &str]()
        {
            str = ffmt::format("{}\n", clock::get_systick_ms());
            render.format(buf2);
        };
        stream.enable();
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
    }

    while (true)
    {
    }
}
