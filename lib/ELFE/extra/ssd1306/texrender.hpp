#pragma once

#include "result.hpp"
#include "extra/ssd1306/ssd1306.hpp"
#include "shared/font_8x8.hpp"
#include "utils/ffmt.hpp"
#include <string>

namespace elfe {
namespace ssd1306 {
    using shared::FONT_8x8;
    /**
     * @brief A simple text renderer for SSD1306
     *
     * @param display The display to render to
     * @param wrap Whether to wrap the text (auto line/page changing)
     * @param font The font to use
     */
    class TexRender {
        mutable unsigned _page = 0, _col = 0;
        bool _march(unsigned step = 8) const
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

        const BaseDisplay& display;
        Fonts font;
        bool wrap;

        TexRender(const BaseDisplay& display, bool wrap = true, Fonts font = Fonts::FONT_8x8)
            : display(display)
            , font(font)
            , wrap(wrap)
        {
        }

        VoidResult<> render_char(char c) const;

        VoidResult<> render_char(uint8_t page, uint8_t col, char c) const
        {
            _page = page;
            _col = col;
            return render_char(c);
        }

        Result<unsigned> render(const char* str) const;
        Result<unsigned> render(uint8_t page, uint8_t col, const char* str) const;
        Result<unsigned> render(const std::string& str) const
        {
            return render(str.c_str());
        }
        Result<unsigned> render(uint8_t page, uint8_t col, const std::string& str) const
        {
            return render(page, col, str.c_str());
        }

        template <std::integral T>
        Result<unsigned> render(T t) const
        {
            std::string str = ffmt::itostr(t);
            return render(str.c_str());
        }

        template <std::integral T>
        Result<unsigned> render(uint8_t page, uint8_t col, T t) const
        {
            std::string str = ffmt::itostr(t);
            return render(page, col, str.c_str());
        }

        template <std::floating_point T>
        Result<unsigned> render(T t) const
        {
            char buffer[24];
            ffmt::grisu2::dtoa_milo(t, buffer);
            return render(buffer);
        }

        template <std::floating_point T>
        Result<unsigned> render(uint8_t page, uint8_t col, T t) const
        {
            char buffer[24];
            ffmt::grisu2::dtoa_milo(t, buffer);
            return render(page, col, buffer);
        }

        template <typename... T>
        VoidResult<> formatn_at(uint8_t page, uint8_t col, const std::size_t maxsize, const std::string& src, T&&... args)
        {
            auto str = ffmt::format(maxsize, src, std::forward<T>(args)...);
            return render(page, col, str).error;
        }

        template <typename... T>
        VoidResult<> format_at(uint8_t page, uint8_t col, const std::string& src, T&&... args)
        {
            auto str = ffmt::format(src, std::forward<T>(args)...);
            return render(page, col, str).error;
        }

        template <typename... T>
        VoidResult<> formatn(const std::size_t maxsize, const std::string& src, T&&... args)
        {
            auto str = ffmt::format(maxsize, src, std::forward<T>(args)...);
            return render(str).error;
        }

        template <typename... T>
        VoidResult<> format(const std::string& src, T&&... args)
        {
            auto str = ffmt::format(src, std::forward<T>(args)...);
            return render(str).error;
        }

        Result<const TexRender&> operator<<(const char* str) const
        {
            auto r = render(str);
            ELFE_PROP(r, Result<const TexRender&>(*this, r.error));
            return *this;
        }

        Result<const TexRender&> operator<<(const std::string& str) const
        {
            auto r = render(str);
            ELFE_PROP(r, Result<const TexRender&>(*this, r.error));
            return *this;
        }

        Result<const TexRender&> operator<<(char c) const
        {
            auto r = render_char(c);
            ELFE_PROP(r, Result<const TexRender&>(*this, r.error));
            return *this;
        }

        template <typename T>
        Result<const TexRender&> operator<<(T&& t) const
        {
            auto r = render(std::forward<T>(t));
            ELFE_PROP(r, Result<const TexRender&>(*this, r.error));
            return *this;
        }
    };
}
}
