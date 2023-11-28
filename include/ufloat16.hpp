#pragma once

#include <cstdint>

namespace vermils
{

/**
 * @brief unsigned 16-bit floating point number, 4bit exponent, 12bit mantissa
 * 
 */
class ufloat16_t
{
    static const uint16_t _m_mask = 0xFFFU;
    static const uint16_t _e_mask = 0xF000U;
    uint16_t _data;
public:
    constexpr ufloat16_t() = default;
    constexpr operator float() const
    {
        uint16_t m = _data & _m_mask;
        uint16_t e = _data >> 12;
        return (1 + m) * (1 << e);
    }
};

}
