#include "SSD1306.hpp"

namespace vermils
{
namespace ssd1306
{
bool BaseDisplay::init() const
{
    return (
        set_display_on_off(false) &&
        set_display_clock_divide_ratio(0, 8) &&
        set_multiplex_ratio(0x3f) &&
        set_display_offset(0) &&
        set_display_start_line(0) &&
        set_seg_remap(true) &&
        set_com_output_scan_dir(true) &&
        set_com_pins_hardware_config(true, false) &&
        set_memory_addressing_mode(MemoryAddressingMode::Page) &&
        set_contrast(0xcf) &&
        set_pre_charge_period(0xf1, 0x00) &&
        set_vcomh_deselect_level(0x03) &&
        set_entire_display_on(false) &&
        set_inverse_display(false) &&
        set_charge_pump(true) &&
        set_display_on_off(true)
    );
}

bool BaseDisplay::set_col_start_addr_for_page_mode(uint8_t addr) const
    {
        return set_lower_col_start_addr_for_page_mode(addr & 0x0f) && set_higher_col_start_addr_for_page_mode(addr >> 4);
    }
}
}