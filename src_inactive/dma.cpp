#include <string>
#include <chrono>
#include "CLK_CFG.h"
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
    HAL_Init();
    SystemClock_Config();
    using namespace vermils;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    HighResTimer timer;

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    timer.delay_ms(100);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    try
    {
        auto &dma = dma::Dma2;
        auto &stream = dma.streams[0];
        dma.init();
        char buf1[100] = "FaQ world!\n";
        char buf2[100] = "";
        stream.enable_interrupts();
        stream.enable_irq();
        stream.peri_addr = (uint8_t*)buf1;
        stream.dst0_addr = (uint8_t*)buf2;
        stream.count = 100;
        stream.set_direction(dma::Direction::Mem2Mem);
        stream.set_peri_unit_size(dma::UnitSize::Byte);
        stream.set_peri_inc(true);
        stream.set_dst_unit_size(dma::UnitSize::Byte);
        stream.set_dst_inc(true);
        stream.on_complete = [&render, &buf2]()
        {
            render.format_at(0, 0, buf2);
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

extern "C" void SysTick_Handler()
{
    HAL_IncTick();
}