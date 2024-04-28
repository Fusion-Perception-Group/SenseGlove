#pragma once

#include "directy/directy.hpp"
#include "extra/ssd1680/ssd1680_driver.hpp"

namespace elfe {
namespace ssd1680 {

    template <size_t W, size_t H>
    class Display {
        Driver _driver;

    public:
        directy::Canvas1B<W, H> red;
        directy::Canvas1B<W, H> bw;
        Display(spi::BaseInterface& spi, gpio::Pin cs_pin, gpio::Pin dc_pin, gpio::Pin busy_pin)
            : _driver(spi, cs_pin, dc_pin, busy_pin)
        {
            ELFE_PANIC_IF(!_driver.init().ok(), SSD1680Error("init failed"));
        }
        VoidResult<> flush(const bool full_update = false) const
        {
            VoidResult<> r;
            for (size_t row = 0; row < (H + 7) / 8; ++row) {
                r = _driver.set_cursor(0xf - row - 1, 0);
                ELFE_PROP(r, r);
                r = _driver.send_command(Command::WriteRAMBW);
                ELFE_PROP(r, r);
                for (size_t col = W; col--;) {
                    uint8_t data = bw.pixels[row][col];
                    r = _driver.send_data(data);
                    ELFE_PROP(r, r);
                }
                r = _driver.set_cursor(0xf - row - 1, 0);
                ELFE_PROP(r, r);
                r = _driver.send_command(Command::WriteRAMRed);
                ELFE_PROP(r, r);
                for (size_t col = W; col--;) {
                    uint8_t data = red.pixels[row][col];
                    r = _driver.send_data(data);
                    ELFE_PROP(r, r);
                }
            }
            if (full_update)
                return _driver.full_update();
            else
                return _driver.update();
        }
    };

}
}
