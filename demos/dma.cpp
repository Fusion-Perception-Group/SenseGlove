#include <string>
#include <chrono>
#include "units.hpp"
#include "mcu/mcu.hpp"
#include "nvic/nvic.hpp"
#include "gpio/gpio.hpp"
#include "i2c/i2c.hpp"
#include "extra/ssd1306/ssd1306.hpp"
#include "extra/ssd1306/texrender.hpp"
#include "utils/ffmt.hpp"
#include "dma/dma.hpp"
#include "adc/adc.hpp"

using std::string;

int main()
{
    using namespace elfe;
    using namespace stm32;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;
    mcu::init().ok();

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear().ok();

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
        stream.set_src_addr((void*)buf1).ok();
        stream.set_dest_addr((void*)buf2).ok();
        // stream.reg.PAR = (uint8_t*)buf1;
        // stream.reg.M0AR = (uint8_t*)buf2;
        // stream.set_direction(dma::Direction::Ram2Ram);
        if (stream.get_direction() != dma::Direction::Ram2Ram)
            throw std::runtime_error(ffmt::format("Src: {}, Dst: {}, Dir: {}\n",
                stream.get_source_addr(), stream.get_dest_addr(), (unsigned)stream.get_direction()));
        stream.count = 100;
        stream.set_src_unit_size(dma::UnitSize::Byte);
        stream.set_src_inc(true);
        stream.set_circular_mode(true);
        stream.set_dst_unit_size(dma::UnitSize::Byte);
        stream.set_dst_inc(true);
        stream.enable_interrupt_complete();
        stream.on_complete = [&render, &buf2, &str](bool full)
        {
            str = ffmt::format("{} fully done {}\n", clock::get_systick_ms(), full);
            render.format(buf2).ok();
        };
        stream.enable();
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what()).ok();
    }

    while (true)
    {
    }
}
