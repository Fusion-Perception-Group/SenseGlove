#pragma once

#include <string>
#include "ssd1306.hpp"
#include "font_8x8.hpp"
#include "ffmt.hpp"
#include "directy.hpp"

namespace vms
{
namespace ssd1306
{
    class Canvas : public directy::Canvas1B<COLS, PAGES*PAGE_HEIGHT>
    {
    public:
        const BaseDisplay & display;
        Canvas(const BaseDisplay & display): display(display) {}

        void flush() const
        {
            for (uint8_t page=0; page < PAGES; ++page)
            {
                display.set_cursor(PAGES-1-page, 0);
                display.start_stream();
                for (uint8_t col=0; col < COLS; ++col)
                {
                    auto byte = directy::helper::inv_bits<uint8_t>(pixels[page][col]);
                    display.stream_byte(byte);
                    // reverse the page order to make texrender compatible
                }
                display.end_stream();
            }
        }
    };
}
}