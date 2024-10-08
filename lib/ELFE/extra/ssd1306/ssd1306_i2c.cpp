#include "extra/ssd1306/ssd1306.hpp"

namespace elfe {
namespace ssd1306 {

    /**
     * @brief Set the contrast of the display.
     *
     * @param contrast
     */
    VoidResult<> I2CDisplay::set_contrast(uint8_t contrast) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0x81,
            contrast
        };

        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the entire display on
     *
     * @param ignore_ram: true - ignore RAM content, false - according to RAM content(RESET)
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_entire_display_on(bool ignore_gram) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(ignore_gram | 0xa4)
        };
        
        return i2c.write_bytes(address, data, sizeof(data)).error;

    }

    /**
     * @brief Set Normal/Inverse mode
     *
     * @param inverse: true - inverse mode(bit 0 -> ON), false - normal mode(bit 1 -> ON)(RESET)
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_inverse_display(bool inverse) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(inverse | 0xa6)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the display on/off
     *
     * @param on: true - display on, false - display off(RESET)
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_display_on_off(bool on) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(on | 0xae)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Setup continuous horizontal scroll (by 1 column)
     *
     * @param dir: true - left, false - right
     * @param start_page: 0x00 - 0x07
     * @param end_page: 0x00 - 0x07, must be >= start_page, otherwise return false
     * @param interval_fr: Enum of I2CDisplay::IntervalFrame
     * @return true
     * @return false: if interval_fr not match or communication failed or start_page > end_page
     */
    VoidResult<> I2CDisplay::hscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, BaseDisplay::IntervalFrame interval_fr) const
    {
        ELFE_ERROR_IF(start_page > end_page,
            EC::InvalidArgument, std::invalid_argument("Invalid page range"));
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(dir | 0x26),
            0x00, // Dummy byte
            start_page,
            static_cast<uint8_t>(interval_fr),
            end_page,
            // 0x00,
            // 0xff
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Setup continuous vertical and horizontal scroll
     *
     * @param dir: true - left, false - right
     * @param start_page: 0x00 - 0x07
     * @param end_page: 0x00 - 0x07, must be >= start_page, otherwise return false
     * @param interval_fr: Enum of I2CDisplay::IntervalFrame
     * @param vertical_offset: 0x00 - 0x3f
     * @return true
     * @return false: if interval_fr not match or communication failed or start_page > end_page
     */
    VoidResult<> I2CDisplay::hvscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, BaseDisplay::IntervalFrame interval_fr, uint8_t vertical_offset) const
    {
        ELFE_ERROR_IF(start_page > end_page,
            EC::InvalidArgument, std::invalid_argument("Invalid page range"));
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(dir + 0x29),
            0x00, // Dummy byte
            start_page,
            static_cast<uint8_t>(interval_fr),
            end_page,
            vertical_offset,
            // 0x00,
            // 0xff
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set scroll activation
     *
     * @param on: true - activate scroll, false - deactivate scroll
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_scroll_on_off(bool on) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(on | 0x2f)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the vertical scroll area
     *
     * @note
    (1) A[5:0]+B[6:0] <= MUX ratio
    (2) B[6:0] <= MUX ratio
    (3a) Vertical scrolling offset (E[5:0] in 29h/2Ah) < B[6:0]
    (3b) Set Display Start Line (X5X4X3X2X1X0 of 40h~7Fh) < B[6:0]
    (4) The last row of the scroll area shifts to the first row of the scroll area.
    (5) For 64d MUX display A[5:0] = 0, B[6:0]=64 : whole area scrolls A[5:0]= 0, B[6:0] < 64 : top area scrolls A[5:0] + B[6:0] < 64 : central area scrolls A[5:0] + B[6:0] = 64 : bottom area scrolls
     * @param top_fixed_rows: 0-63, number of rows that are not affected by vertical scroll(RESET = 0x00)
     * @param scroll_rows: 0-64,number of rows that are affected by vertical scroll(RESET = 0x40)
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_vertical_scroll_area(uint8_t top_fixed_rows, uint8_t scroll_rows) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0xa3,
            top_fixed_rows,
            scroll_rows
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the lower column start address for page addressing mode
     *
     * @param addr: 0x00 - 0x0f(RESET = 0x00)
     * @return true
     * @return false: if addr > 0x0f or communication failed
     */
    VoidResult<> I2CDisplay::set_lower_col_start_addr_for_page_mode(uint8_t addr) const
    {
        ELFE_ERROR_IF(
            addr > 0x0f, EC::InvalidArgument, std::invalid_argument("Invalid column start address")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            addr
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the higher column start address for page addressing mode
     *
     * @param addr: 0x00 - 0x0f(RESET = 0x00)
     * @return true
     * @return false: if addr out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_higher_col_start_addr_for_page_mode(uint8_t addr) const
    {
        ELFE_ERROR_IF(
            addr > 0x0f, EC::InvalidArgument, std::invalid_argument("Invalid column start address")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(0x10 | addr)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    VoidResult<> I2CDisplay::set_col_start_addr_for_page_mode(uint8_t addr) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(0x0f & addr), // lower 4 bits
            static_cast<uint8_t>(0x10 | (addr >> 4)) // higher 4 bits
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the memory addressing mode
     *
     * @param mode: Enum of I2CDisplay::MemoryAddressingMode,
     *              0x00 - horizontal addressing mode,
     *              0x01 - vertical addressing mode,
     *              0x02 - page addressing mode(RESET)
     * @return true
     * @return false: if mode out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_memory_addressing_mode(I2CDisplay::MemoryAddressingMode mode) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0x20,
            mode
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the column start and end address
     *
     * @param start: 0x00 - 0x7f(RESET = 0x00)
     * @param end: 0x00 - 0x7f(RESET = 0x7f)
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_col_addr(uint8_t start, uint8_t end) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0x21,
            start,
            end
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the page start and end address
     *
     * @param start: 0x00 - 0x07(RESET = 0x00)
     * @param end: 0x00 - 0x07(RESET = 0x07)
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_page_addr(uint8_t start, uint8_t end) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0x22,
            start,
            end
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the page start address for page addressing mode
     *
     * @param addr: 0x00 - 0x07(RESET = 0x00)
     * @return true
     * @return false: if addr out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_page_start_addr_for_page_mode(uint8_t addr) const
    {
        ELFE_ERROR_IF(
            addr > 0x07, EC::InvalidArgument, std::invalid_argument("Invalid page start address")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(0xb0 | addr)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set the display start line
     *
     * @param line: 0x00 - 0x3f(RESET = 0x00)
     * @return true
     * @return false: if line out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_display_start_line(uint8_t line) const
    {
        ELFE_ERROR_IF(
            line > 0x3f, EC::InvalidArgument, std::invalid_argument("Invalid display start line")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(0x40 | line)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set segment re-map
     *
     * @param remap: true - column address 127 is mapped to SEG0, false - column address 0 is mapped to SEG0(RESET)
     * @return true
     * @return false: if contrast out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_seg_remap(bool remap) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>(remap | 0xa0)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set multiplex ratio
     *
     * @param ratio: 0x0f - 0x3f(RESET = 0x3f)
     * @return true
     * @return false: if ratio out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_multiplex_ratio(uint8_t ratio) const
    {
        ELFE_ERROR_IF(
            ratio > 0x3f || ratio < 0x0f, EC::InvalidArgument, std::invalid_argument("Invalid multiplex ratio")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            0xa8,
            ratio
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set COM output scan direction
     *
     * @param dir: true - remapped mode, false - normal mode(RESET)
     * @return true
     * @return false: if dir out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_com_output_scan_dir(bool dir) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            static_cast<uint8_t>((dir << 3) | 0xc0)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set display offset
     *
     * @param offset: 0x00 - 0x3f(RESET = 0x00)
     * @return true
     * @return false: if offset out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_display_offset(uint8_t offset) const
    {
        ELFE_ERROR_IF(
            offset > 0x3f, EC::InvalidArgument, std::invalid_argument("Invalid display offset")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            0xd3,
            offset
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set COM pins hardware configuration
     *
     * @param sequential: true - sequential, false - alternative(RESET)
     * @param remapped: COM left/right remap, true - remapped, false - normal(RESET)
     * @return true
     * @return false: if ratio out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_com_pins_hardware_config(bool sequential, bool remapped) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0xda,
            static_cast<uint8_t>(0x02 | (sequential << 4) | (remapped << 5))
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set display clock divide ratio/oscillator frequency
     *
     * @param ratio: 1-16(RESET = 0x1)
     * @param freq: 0-15(RESET = 0x8)
     * @return true
     * @return false: if ratio or freq out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_display_clock_divide_ratio(uint8_t ratio, uint8_t freq) const
    {
        --ratio; // remap 1-16 -> 0-15
        ELFE_ERROR_IF(
            ratio > 0x0f || freq > 0x0f, EC::InvalidArgument, std::invalid_argument("Invalid ratio or frequency")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            0xd5,
            static_cast<uint8_t>((freq << 4) | ratio)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set pre-charge period
     *
     * @param phase1: 1-15(RESET = 0x2)
     * @param phase2: 1-15(RESET = 0x2)
     * @return true
     * @return false: if phase1 or phase2 out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_pre_charge_period(uint8_t phase1, uint8_t phase2) const
    {
        ELFE_ERROR_IF(
            !phase1 || !phase2 || phase1 > 0x0f || phase2 > 0x0f,
            EC::InvalidArgument, std::invalid_argument("Invalid phase")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            0xd9,
            static_cast<uint8_t>((phase2 << 4) | phase1)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set VCOMH deselect level
     *
     * @param level: {0x00, 0x02, 0x03}(RESET = 0x02), 0x00 - 0.65*Vcc, 0x02 - 0.77*Vcc, 0x03 - 0.83*Vcc
     * @return true
     * @return false: if level out of range or communication failed
     */
    VoidResult<> I2CDisplay::set_vcomh_deselect_level(uint8_t level) const
    {
        ELFE_ERROR_IF(
            level > 0x03 || level == 0x01,
            EC::InvalidArgument, std::invalid_argument("Invalid level")
        );
        const uint8_t data[] = {
            CMD_STREAM,
            0xdb,
            static_cast<uint8_t>(level << 4)
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Set charge pump
     *
     * @param enable: true - enable, false - disable(RESET)
     * @return true
     * @return false: if communication failed
     */
    VoidResult<> I2CDisplay::set_charge_pump(bool enable) const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0x8d,
            static_cast<uint8_t>(0x10 | (enable << 2))
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Does nothing
     *
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::nop() const
    {
        const uint8_t data[] = {
            CMD_STREAM,
            0xe3
        };
        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief Write data to the display
     *
     * @param data
     * @param size
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::write(const uint8_t* data, std::size_t size) const
    {
        auto ret = i2c.select(address, false);
        ELFE_PROP(ret, ret.error);
        ELFE_ERROR_IF(
            !ret.value, EC::ExSSD1306Error,
            SSD1306Error("Failed to select slave for i2c"));
        auto ru = i2c.write_byte(DATA_STREAM);
        ELFE_PROP(ru, ru.error);
        for (std::size_t i = 0; i < size; ++i)
        {
            ret = i2c.write_byte(data[i]);
            ELFE_PROP(ret, ret.error);
        }
        i2c.end();
        return EC::None;
    }

    /**
     * @brief Write a single byte to the display
     *
     * @param value
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::set_value(uint8_t value) const
    {
        uint8_t data[] = {
            DATA,
            value
        };

        return i2c.write_bytes(address, data, sizeof(data)).error;
    }

    /**
     * @brief
     *
     * @param value
     * @return const I2CDisplay&
     * @throw SSD1306Exception
     */
    Result<const BaseDisplay&> I2CDisplay::operator<<(uint8_t value) const
    {
        auto ret = set_value(value);
        return { *this, ret.error };
    }

    /**
     * @brief Fill the display with a value
     *
     * @param value
     * @return true
     * @return false
     */
    VoidResult<> I2CDisplay::fill(uint8_t value) const
    {
        VoidResult<> ret;
        for (uint8_t page = 0; page < PAGES; ++page) {
            ret = set_cursor(page, 0);
            if (ret)
            {
                ret = start_stream();
                ELFE_PROP(ret, ret);
            }
            for (uint8_t col = 0; col < COLS; ++col)
            {
                ret = stream_byte(value);
                ELFE_PROP(ret, ret);
            }
            ret = end_stream();
            ELFE_PROP(ret, ret);
        }
        return EC::None;
    }

    /**
     * @brief Get the value at a specified page and column
     *
     * @return uint8_t
     */
    Result<uint8_t> I2CDisplay::get_value() const
    {
        auto ret = i2c.select(address, true);
        ELFE_PROP(ret, Result<uint8_t>(0, ret.error));
        ELFE_ERROR_IF(
            !ret.value, Result<uint8_t>(0, EC::ExSSD1306Error),
            SSD1306Error("Failed to select slave for i2c"));
        auto ru = i2c.read_byte(false);
        ELFE_PROP(ru, Result<uint8_t>(0, ru.error));
        uint8_t value = ru.value;
        i2c.end();
        return value;
    }

    VoidResult<> I2CDisplay::start_stream() const
    {
        auto rb = i2c.select(address, false);
        ELFE_PROP(rb, rb.error);
        ELFE_ERROR_IF(
            !rb.value, EC::ExSSD1306Error,
            SSD1306Error("Failed to select slave for i2c"));
        return i2c.write_byte(DATA_STREAM).error;
    }

    VoidResult<> I2CDisplay::end_stream() const { i2c.end(); return EC::None; }

}
}