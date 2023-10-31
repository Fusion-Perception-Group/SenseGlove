#include "texrender.hpp"

namespace vermils
{
namespace ssd1306
{
    /**
     * @brief Render a string to the display
     * 
     * @param str The string to render
     * @param page
     * @param col
     * @return int
     */
    unsigned TexRender::render(const char * str) const
    {
        unsigned count = 0;
        _march(0); // Make sure _page and _col are valid
        Init:
        if (!display.set_cursor(_page, _col))
            return count;
        if (!display.start_stream())
            return count;
        while(*str)
        {
            if (_col + 8 > COLS)
            {
                display.end_stream();
                if (!wrap)
                    return count;
                _newline();
                goto Init;
            }

            if (*str == '\n' || *str == '\r')
            {
                display.end_stream();
                _newline();
                ++count;
                ++str;
                goto Init;
            }

            for (unsigned i = 0; i < 8; ++i)
            {
                if (!display.stream_byte(FONT_8x8[unsigned(*str)][i]))
                    goto End;
            }

            ++count;
            ++str;
            if (_march())
            {
                display.end_stream();
                goto Init;
            }

        }
        End:
        display.end_stream();
        return count;
    }

    unsigned TexRender::render(const char * str, uint8_t page, uint8_t col) const
    {
        _page = page;
        _col = col;
        return render(str);
    }

    bool TexRender::render_char(char c) const
    {
        _march(0);
        if (!display.set_cursor(_page, _col))
            return false;
        
        if (_col + 8 > COLS)
        {
            if (!wrap)
                return false;
            _newline();
        }
        
        if (c == '\n' || c == '\r')
        {
            _newline();
            return true;
        }
        else
        {
            _march();
            return display.write(FONT_8x8[unsigned(c)], 8);
        }

    }

}
}
