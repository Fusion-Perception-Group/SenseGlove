#include "extra/ssd1306/texrender.hpp"

namespace elfe {
namespace ssd1306 {
    /**
     * @brief Render a string to the display
     *
     * @param str The string to render
     * @param page
     * @param col
     * @return int
     */
    Result<unsigned> TexRender::render(const char* str) const
    {
        unsigned count = 0;
        VoidResult<> r;
        _march(0); // Make sure _page and _col are valid
    Init:
        if (!display.set_cursor(_page, _col))
            return count;
        if (!display.start_stream())
            return count;
        while (*str) {
            if (_col + 8 > COLS) {
                r = display.end_stream();
                ELFE_PROP(r, Result<unsigned>(count, r.error));
                if (!wrap)
                    return count;
                _newline();
                goto Init;
            }

            if (*str == '\n' || *str == '\r') {
                r = display.end_stream();
                ELFE_PROP(r, Result<unsigned>(count, r.error));
                _newline();
                ++count;
                ++str;
                goto Init;
            }

            for (unsigned i = 0; i < 8; ++i) {
                r = display.stream_byte(FONT_8x8[unsigned(*str)][i]);
                if (!r.ok())
                {
                    display.end_stream().ok();
                    return {count, r.error};
                }
            }

            ++count;
            ++str;
            if (_march()) {
                r = display.end_stream();
                ELFE_PROP(r, Result<unsigned>(count, r.error));
                goto Init;
            }
        }
        r = display.end_stream();
        ELFE_PROP(r, Result<unsigned>(count, r.error));
        return count;
    }

    Result<unsigned> TexRender::render(uint8_t page, uint8_t col, const char* str) const
    {
        _page = page;
        _col = col;
        return render(str);
    }

    VoidResult<> TexRender::render_char(char c) const
    {
        _march(0);
        auto r = display.set_cursor(_page, _col);
        ELFE_PROP(r, r);

        if (_col + 8 > COLS) {
            if (!wrap)  // ignore
                return EC::None;
            _newline();
            r = display.set_cursor(_page, _col);
            ELFE_PROP(r, r);
        }

        if (c == '\n' || c == '\r') {
            _newline();
            return EC::None;
        } else {
            _march();
            return display.write(FONT_8x8[unsigned(c)], 8);
        }
    }

}
}
