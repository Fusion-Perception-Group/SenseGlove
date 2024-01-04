#pragma once

#include "directy.hpp"
#include "ssd1680_driver.hpp"

namespace vermils
{
namespace ssd1680
{

template <size_t W, size_t H>
class Display
{
    Driver _driver;
public:
    directy::Canvas1B<W, H> red;
    directy::Canvas1B<W, H> bw;
    Display(spi::BaseInterface &spi, gpio::Pin cs_pin, gpio::Pin dc_pin, gpio::Pin busy_pin)
        : _driver(spi, cs_pin, dc_pin, busy_pin)
    {
        _driver.init();
    }
    void flush(const bool full_update=false) const
    {
        for (size_t row = 0; row < (H+7)/8; ++row)
        {
            _driver.set_cursor(0xf-row-1, 0);
            _driver.send_command(Command::WriteRAMBW);
            for (size_t col = W; col--;)
            {
                uint8_t data = bw.pixels[row][col];
                _driver.send_data(data);
            }
            _driver.set_cursor(0xf-row-1, 0);
            _driver.send_command(Command::WriteRAMRed);
            for (size_t col = W; col--;)
            {
                uint8_t data = red.pixels[row][col];
                _driver.send_data(data);
            }
        }
        if (full_update)
            _driver.full_update();
        else
            _driver.update();
    }
};
    
}
}
