#pragma once

#include <exception>
#include "I2C.hpp"

namespace vermils
{
namespace ssd1306
{
    class SSD1306Exception : public std::exception
    {
    public:
        const char *what() const noexcept override
        {
            return "SSD1306Exception";
        }
    };

    class BaseDisplay
    {
    public:
        enum IntervalFrame : uint8_t
        {
            F2 = 0x07,
            F3 = 0x04,
            F4 = 0x05,
            F5 = 0x00,
            F25 = 0x06,
            F64 = 0x01,
            F128 = 0x02,
            F256 = 0x03
        };

        enum MemoryAddressingMode : uint8_t
        {
            Horizontal = 0x00,
            Vertical = 0x01,
            Page = 0x02
        };

        static constexpr uint8_t PAGES = 8, COLS = 128, PAGE_HEIGHT = 8;

        virtual ~BaseDisplay() {}

        virtual bool init() const;
        virtual bool set_cursor(uint8_t page, uint8_t col) const;
        virtual bool fill(uint8_t data) const;
        virtual bool clear() const;
        virtual bool set_value(uint8_t value) const;
        virtual bool set_value(uint8_t page, uint8_t col, uint8_t value) const;
        virtual uint8_t get_value() const;
        virtual uint8_t get_value(uint8_t page, uint8_t col) const;
        virtual bool operator << (uint8_t) const;
        virtual bool operator << (const uint8_t []) const;

        virtual bool set_contrast(uint8_t contrast) const = 0;
        virtual bool set_entire_display_on(bool ignore_gram=false) const = 0;
        virtual bool set_inverse_display(bool inverse) const = 0;
        virtual bool set_display_on_off(bool on) const = 0;

        virtual bool hscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr) const = 0;
        virtual bool hvscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr, uint8_t vertical_offset) const = 0;
        virtual bool set_scroll_on_off(bool on) const = 0;
        virtual bool set_vertical_scroll_area(uint8_t top_fixed_rows, uint8_t scroll_rows) const = 0;

        virtual bool set_lower_col_start_addr_for_page_mode(uint8_t addr) const = 0;
        virtual bool set_higher_col_start_addr_for_page_mode(uint8_t addr) const = 0;
        virtual bool set_col_start_addr_for_page_mode(uint8_t addr) const;
        virtual bool set_memory_addressing_mode(MemoryAddressingMode mode) const = 0;
        virtual bool set_col_addr(uint8_t start, uint8_t end) const = 0;
        virtual bool set_page_addr(uint8_t start, uint8_t end) const = 0;
        virtual bool set_page_start_addr_for_page_mode(uint8_t addr) const = 0;

        virtual bool set_display_start_line(uint8_t line) const = 0;
        virtual bool set_seg_remap(bool remap) const = 0;
        virtual bool set_multiplex_ratio(uint8_t ratio) const = 0;
        virtual bool set_com_output_scan_dir(bool dir) const = 0;
        virtual bool set_display_offset(uint8_t offset) const = 0;
        virtual bool set_com_pins_hardware_config(bool sequential, bool remapped = true) const = 0;

        virtual bool set_display_clock_divide_ratio(uint8_t ratio, uint8_t freq) const = 0;
        virtual bool set_pre_charge_period(uint8_t phase1, uint8_t phase2) const = 0;
        virtual bool set_vcomh_deselect_level(uint8_t level) const = 0;

        virtual bool set_charge_pump(bool enable) const = 0;

        virtual bool nop() const = 0;
    };

    /**
     * @brief SSD1306 display driver for I2C interface.
     * 
     * @warning Before initing the display, make sure power is supplied for more than 100ms.
     * 
     * @param `i2c::BaseMaster`
     * @param `init` whether to initialize the display after instantiation
     */
    class I2CDisplay : public BaseDisplay
    {
        static inline constexpr uint8_t CMD = 0x80;
        static inline constexpr uint8_t CMD_STREAM = 0x00;
        static inline constexpr uint8_t DATA = 0xc0;
        static inline constexpr uint8_t DATA_STREAM = 0x40;

    public:
        uint8_t address = 0x78;
        const stm32::i2c::BaseMaster &i2c;
        
        I2CDisplay(const stm32::i2c::BaseMaster &i2c, const bool init=true): i2c(i2c)
        {
            if (init)
                this->init();
        }

        bool init() const override;

        bool set_contrast(uint8_t contrast) const override;
        bool set_entire_display_on(bool ignore_gram=false) const override;
        bool set_inverse_display(bool inverse) const override;
        bool set_display_on_off(bool on) const override;

        bool hscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr) const override;
        bool hvscroll_setup(bool dir, uint8_t start_page, uint8_t end_page, IntervalFrame interval_fr, uint8_t vertical_offset) const override;
        bool set_scroll_on_off(bool on) const override;
        bool set_vertical_scroll_area(uint8_t top_fixed_rows, uint8_t scroll_rows) const override;

        bool set_lower_col_start_addr_for_page_mode(uint8_t addr) const override;
        bool set_higher_col_start_addr_for_page_mode(uint8_t addr) const override;
        bool set_col_start_addr_for_page_mode(uint8_t addr) const override;
        bool set_memory_addressing_mode(MemoryAddressingMode mode) const override;
        bool set_col_addr(uint8_t start, uint8_t end) const override;
        bool set_page_addr(uint8_t start, uint8_t end) const override;
        bool set_page_start_addr_for_page_mode(uint8_t addr) const override;

        bool set_display_start_line(uint8_t line) const override;
        bool set_seg_remap(bool remap) const override;
        bool set_multiplex_ratio(uint8_t ratio) const override;
        bool set_com_output_scan_dir(bool dir) const override;
        bool set_display_offset(uint8_t offset) const override;
        bool set_com_pins_hardware_config(bool sequential, bool remapped=true) const override;

        bool set_display_clock_divide_ratio(uint8_t ratio, uint8_t freq) const override;
        bool set_pre_charge_period(uint8_t phase1, uint8_t phase2) const override;
        bool set_vcomh_deselect_level(uint8_t level) const override;

        bool set_charge_pump(bool enable) const override;

        bool nop() const override;
    };

}
}
