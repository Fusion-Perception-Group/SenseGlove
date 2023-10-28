#pragma once

#include <string>
#include "ssd1306.hpp"
#include "font_8x8.hpp"

namespace vermils
{
namespace ssd1306
{
    using shared::FONT_8x8;
    /**
     * @brief A simple text renderer for SSD1306
     * 
     * @param display The display to render to
     * @param wrap Whether to wrap the text (auto line/page changing)
     * @param font The font to use
     */
    class TexRender
    {
        mutable unsigned _page = 0, _col = 0;
        bool _march(unsigned step=8) const
        {    
            unsigned page_step = (_col + step) / COLS;
            _col = _col + step - page_step * COLS;
            _col = (_col < 8) ? 0 : _col;
            _page = (_page + page_step) % PAGES;

            return bool(page_step);
        }
        void _newline() const
        {
            _march(0);
            _page = (_page + 1) % PAGES;
            _col = 0;
        }
    public:
        enum class Fonts {
            FONT_8x8
        };

        const BaseDisplay & display;
        Fonts font;
        bool wrap;

        TexRender(const BaseDisplay & display, bool wrap=true, Fonts font = Fonts::FONT_8x8): display(display), font(font), wrap(wrap) {}

        bool render_char(char c) const
        {
            display.set_cursor(_page, _col);
            if (_march())
                display.set_cursor(_page, _col);
            return display.write((uint8_t *)FONT_8x8[unsigned(c)], 8);
        }

        bool render_char(char c, uint8_t page, uint8_t col) const
        {
            _page = page;
            _col = col;
            return render_char(c);
        }

        unsigned render(const char * str) const;
        unsigned render(const char * str, uint8_t page, uint8_t col) const;
        unsigned render(const std::string & str) const
        {
            return render(str.c_str());
        }
        unsigned render(const std::string & str, uint8_t page, uint8_t col) const
        {
            return render(str.c_str(), page, col);
        }

        const TexRender & operator << (const char * str) const
        {
            render(str);
            return *this;
        }

        const TexRender & operator << (const std::string & str) const
        {
            render(str);
            return *this;
        }

        const TexRender & operator << (char c) const
        {
            render_char(c);
            return *this;
        }
    };
}
}
