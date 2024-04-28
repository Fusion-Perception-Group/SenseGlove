#pragma once

#include "result.hpp"
#include "i2c/i2c.hpp"
#include <exception>

namespace elfe {
namespace ssd1306 {
    using namespace err::extra::ssd1306;
    using EC = err::ErrorCode;

    inline constexpr uint8_t PAGES = 8, COLS = 128, PAGE_HEIGHT = 8;

    class BaseDisplay {
    public:
        enum IntervalFrame : uint8_t {
            F2 = 0x07,
            F3 = 0x04,
            F4 = 0x05,
            F5 = 0x00,
            F25 = 0x06,
            F64 = 0x01,
            F128 = 0x02,
            F256 = 0x03
        };

        enum MemoryAddressingMode : uint8_t {
            Horizontal = 0x00,
            Vertical = 0x01,
            Page = 0x02
        };

        virtual ~BaseDisplay() { }

        virtual VoidResult<> init() const;
        virtual VoidResult<> set_cursor(uint8_t page, uint8_t col) const;
        virtual VoidResult<> write(const uint8_t* data, std::size_t size) const;
        virtual VoidResult<> fill(uint8_t value) const;
        virtual VoidResult<> clear() const;
        virtual VoidResult<> set_value(uint8_t value) const = 0;
        virtual VoidResult<> set_value(uint8_t page, uint8_t col, uint8_t value) const;
        virtual Result<uint8_t> get_value() const = 0;
        virtual Result<uint8_t> get_value(uint8_t page, uint8_t col) const;
        virtual Result<const BaseDisplay&> operator<<(uint8_t) const = 0;
        virtual VoidResult<> start_stream() const { return {}; };
        virtual VoidResult<> stream_byte(uint8_t value) const { return set_value(value); };
        virtual VoidResult<> end_stream() const { return EC::None; };

        virtual VoidResult<> set_contrast(uint8_t contrast) const = 0;
        virtual VoidResult<> set_entire_display_on(bool ignore_gram = false) const = 0;
        virtual VoidResult<> set_inverse_display(bool inverse) const = 0;
        virtual VoidResult<> set_display_on_off(bool on) const = 0;

        virtual VoidResult<> hscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr) const = 0;
        virtual VoidResult<> hvscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr, uint8_t vertical_offset) const = 0;
        virtual VoidResult<> set_scroll_on_off(bool on) const = 0;
        virtual VoidResult<> set_vertical_scroll_area(uint8_t top_fixed_rows, uint8_t scroll_rows) const = 0;

        virtual VoidResult<> set_lower_col_start_addr_for_page_mode(uint8_t addr) const = 0;
        virtual VoidResult<> set_higher_col_start_addr_for_page_mode(uint8_t addr) const = 0;
        virtual VoidResult<> set_col_start_addr_for_page_mode(uint8_t addr) const;
        virtual VoidResult<> set_memory_addressing_mode(MemoryAddressingMode mode) const = 0;
        virtual VoidResult<> set_col_addr(uint8_t start, uint8_t end) const = 0;
        virtual VoidResult<> set_page_addr(uint8_t start, uint8_t end) const = 0;
        virtual VoidResult<> set_page_start_addr_for_page_mode(uint8_t addr) const = 0;

        virtual VoidResult<> set_display_start_line(uint8_t line) const = 0;
        virtual VoidResult<> set_seg_remap(bool remap) const = 0;
        virtual VoidResult<> set_multiplex_ratio(uint8_t ratio) const = 0;
        virtual VoidResult<> set_com_output_scan_dir(bool dir) const = 0;
        virtual VoidResult<> set_display_offset(uint8_t offset) const = 0;
        virtual VoidResult<> set_com_pins_hardware_config(bool sequential, bool remapped = true) const = 0;

        virtual VoidResult<> set_display_clock_divide_ratio(uint8_t ratio, uint8_t freq) const = 0;
        virtual VoidResult<> set_pre_charge_period(uint8_t phase1, uint8_t phase2) const = 0;
        virtual VoidResult<> set_vcomh_deselect_level(uint8_t level) const = 0;

        virtual VoidResult<> set_charge_pump(bool enable) const = 0;

        virtual VoidResult<> nop() const = 0;
    };

    /**
     * @brief SSD1306 display driver for I2C interface.
     *
     * @warning Before initing the display, make sure power is supplied for more than 100ms.
     *
     * @param `i2c::BaseMaster`
     * @param `init` whether to initialize the display after instantiation
     * @throw `I2CException` if I2C communication fails
     */
    class I2CDisplay : public BaseDisplay {
        static inline constexpr uint8_t CMD = 0x80;
        static inline constexpr uint8_t CMD_STREAM = 0x00;
        static inline constexpr uint8_t DATA = 0xc0;
        static inline constexpr uint8_t DATA_STREAM = 0x40;

    public:
        uint8_t address = 0x78;
        const stm32::i2c::BaseMaster& i2c;

        I2CDisplay(const stm32::i2c::BaseMaster& i2c, const bool init = true)
            : i2c(i2c)
        {
            if (init) {
                ELFE_PANIC_IF(!this->init().ok(), std::runtime_error("Failed to initialize display"));
            }
        }

        // bool init() const override;
        VoidResult<> write(const uint8_t* data, std::size_t size) const override;
        VoidResult<> fill(uint8_t value) const override;
        VoidResult<> set_value(uint8_t value) const override;
        Result<uint8_t> get_value() const override;
        Result<const BaseDisplay&> operator<<(uint8_t) const override;
        VoidResult<> start_stream() const override;
        VoidResult<> stream_byte(uint8_t value) const { return i2c.write_byte(value).error; };
        VoidResult<> end_stream() const;

        VoidResult<> set_contrast(uint8_t contrast) const override;
        VoidResult<> set_entire_display_on(bool ignore_gram = false) const override;
        VoidResult<> set_inverse_display(bool inverse) const override;
        VoidResult<> set_display_on_off(bool on) const override;

        VoidResult<> hscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr) const override;
        VoidResult<> hvscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr, uint8_t vertical_offset) const override;
        VoidResult<> set_scroll_on_off(bool on) const override;
        VoidResult<> set_vertical_scroll_area(uint8_t top_fixed_rows, uint8_t scroll_rows) const override;

        VoidResult<> set_lower_col_start_addr_for_page_mode(uint8_t addr) const override;
        VoidResult<> set_higher_col_start_addr_for_page_mode(uint8_t addr) const override;
        VoidResult<> set_col_start_addr_for_page_mode(uint8_t addr) const override;
        VoidResult<> set_memory_addressing_mode(MemoryAddressingMode mode) const override;
        VoidResult<> set_col_addr(uint8_t start, uint8_t end) const override;
        VoidResult<> set_page_addr(uint8_t start, uint8_t end) const override;
        VoidResult<> set_page_start_addr_for_page_mode(uint8_t addr) const override;

        VoidResult<> set_display_start_line(uint8_t line) const override;
        VoidResult<> set_seg_remap(bool remap) const override;
        VoidResult<> set_multiplex_ratio(uint8_t ratio) const override;
        VoidResult<> set_com_output_scan_dir(bool dir) const override;
        VoidResult<> set_display_offset(uint8_t offset) const override;
        VoidResult<> set_com_pins_hardware_config(bool sequential, bool remapped = true) const override;

        VoidResult<> set_display_clock_divide_ratio(uint8_t ratio, uint8_t freq) const override;
        VoidResult<> set_pre_charge_period(uint8_t phase1, uint8_t phase2) const override;
        VoidResult<> set_vcomh_deselect_level(uint8_t level) const override;

        VoidResult<> set_charge_pump(bool enable) const override;

        VoidResult<> nop() const override;
    };

}
}
