#include <string>
#include <tuple>
#include <chrono>
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "w25qxx.hpp"

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
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);

    try
    {
        gpio::PinConfig spi_cfg(
            gpio::PinConfig::AF,
            gpio::PinConfig::VeryHigh,
            gpio::PinConfig::PushPull,
            5  // SPI1 Alternate Function
        );
        spi_cfg.pull = gpio::PinConfig::NoPull;
        gpio::Pin sck(PortA, 5, spi_cfg),
            miso(PortA, 6, spi_cfg),
            mosi(PortA, 7, spi_cfg),
            cs(PortA, 4);
        auto &spi = spi::Spi1;
        spi.init();
        auto fls = w25qxx::Flash(spi, cs);
        [[maybe_unused]]uint32_t tmp;
        // cs.reset();  // active low
        // spi.put<uint8_t>(0x9F);
        // tmp = spi.get<uint8_t>();
        // render.format("Manu: {:.x}\n", tmp);
        // tmp = spi.get<uint8_t>() << 8 | spi.get<uint8_t>();
        // render.format("ID: {:.x}\n", tmp);
        // cs.set();
        render.format("Flash ID: {:.x}\n", fls.get_model_id());
        w25qxx::addr_t test_addr = 0x7002;

        uint32_t test_size = 0x1000;
        fls.erase(test_addr, test_size*4);
        for (uint32_t i = 0; i < test_size; ++i)
        {
            fls.put(test_addr + i*4, i);
            tmp = fls.get<uint32_t>(test_addr + i*4);
            if (tmp != i)
            {
                render.format("Test failed at {:.p}: {:.x} != target {:.x}\n", test_addr + i*4, tmp, i);
                break;
            }
        }
        // for (uint32_t i = 0; i < test_size; ++i)
        // {
        //     tmp = fls.get<uint32_t>(test_addr + i);
        //     if (tmp != i)
        //     {
        //         render.format("Test failed at {:.p}: {:.p}\n", test_addr + i, tmp);
        //         break;
        //     }
        // }
        render.render("Test done\n");


        while (true)
        {
            
        }
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
    }
}
