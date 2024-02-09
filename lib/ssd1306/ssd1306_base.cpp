#include "ssd1306.hpp"

namespace vms
{
namespace ssd1306
{
bool BaseDisplay::init() const
{
    return (
        set_display_on_off(false) &&
        set_display_clock_divide_ratio(1, 8) &&
        set_multiplex_ratio(0x3f) &&
        set_display_offset(0) &&
        set_display_start_line(0) &&
        set_seg_remap(true) &&
        set_com_output_scan_dir(true) &&
        set_com_pins_hardware_config(true, false) &&
        set_memory_addressing_mode(MemoryAddressingMode::Horizontal) &&
        set_contrast(0xcf) &&
        set_pre_charge_period(0x01, 0x0f) &&
        set_vcomh_deselect_level(0x03) &&
        set_entire_display_on(false) &&
        set_inverse_display(false) &&
        set_charge_pump(true) &&
        set_display_on_off(true)
    );
}

bool BaseDisplay::set_col_start_addr_for_page_mode(uint8_t addr) const
{
    return (addr & 0x0f) && set_higher_col_start_addr_for_page_mode(addr >> 4);
}

bool BaseDisplay::set_cursor(uint8_t page, uint8_t col) const
{
    return (
        set_page_start_addr_for_page_mode(page) &&
        set_col_start_addr_for_page_mode(col)
    );
}

bool BaseDisplay::fill(uint8_t value) const
{
    bool ret = true;
    for (uint8_t page = 0; page < PAGES; ++page)
    {
        ret &= set_cursor(page, 0);
        for (uint8_t col = 0; col < COLS; ++col)
        {
            ret &= set_value(value);
        }
    }
    return ret;
}

bool BaseDisplay::clear() const
{
    return fill(0);
}

bool BaseDisplay::set_value(uint8_t page, uint8_t col, uint8_t value) const
{
    return set_cursor(page, col) && set_value(value);
}

/**
 * @brief Get value at specified page and column
 * 
 * @param page 
 * @param col 
 * @return uint8_t 
 * @throw SSD1306Exception
 */
uint8_t BaseDisplay::get_value(uint8_t page, uint8_t col) const
{
    if (set_cursor(page, col))
        return get_value();
    throw SSD1306Exception("Failed to get value");
}

bool BaseDisplay::write(const uint8_t * data, std::size_t size) const
{
    bool ret = true;
    for (std::size_t i = 0; i < size; ++i)
        ret &= set_value(data[i]);

    return ret;
}

}
}