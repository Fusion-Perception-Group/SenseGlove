#pragma once

#include <cstdint>
#include "userconfig.hpp"
#include "clock_shared.hpp"
#include "clock_tree.hpp"
#include "rcc_f1_detail.hpp"
#include "rcc_f4_detail.hpp"
#include "rcc_h7_detail.hpp"
#include "rcc_generic_detail.hpp"

namespace vermils
{
namespace stm32
{
namespace gpio
{
    namespace hidden
    {
        class _Port;
    }
}
namespace i2c
{
    class HardMaster;
    class HardSlave;
}
namespace adc
{
    class RegularADC;
}
namespace dma
{
    class BaseDMA;
}
namespace flash
{
    class EmbeddedFlash;
}
namespace usart
{
    class HardUart;
    class HardUsart;
}
namespace spi
{
    class HardwareInterface;
}


namespace clock
{
namespace tim
{
class BasicTimer;
}

namespace rcc
{
void enable_clock(const gpio::hidden::_Port& port) noexcept;
void enable_clock(const adc::RegularADC& adc) noexcept;
void enable_clock(const dma::BaseDMA& dma) noexcept;
void enable_clock(const tim::BasicTimer& tim) noexcept;
void enable_clock(const usart::HardUsart& usart) noexcept;
void enable_clock(const i2c::HardMaster& i2c) noexcept;
void enable_clock(const spi::HardwareInterface& spi) noexcept;

void disable_clock(const gpio::hidden::_Port& port) noexcept;
void disable_clock(const adc::RegularADC& adc) noexcept;
void disable_clock(const dma::BaseDMA& dma) noexcept;
void disable_clock(const tim::BasicTimer& tim) noexcept;
void disable_clock(const usart::HardUsart& usart) noexcept;
void disable_clock(const i2c::HardMaster& i2c) noexcept;
void disable_clock(const spi::HardwareInterface& spi) noexcept;

void reset(const gpio::hidden::_Port& port) noexcept;
void reset(const adc::RegularADC& adc) noexcept;
void reset(const dma::BaseDMA& dma) noexcept;
void reset(const tim::BasicTimer& tim) noexcept;
void reset(const flash::EmbeddedFlash& flash) noexcept;
void reset(const usart::HardUsart& usart) noexcept;
void reset(const i2c::HardMaster& i2c) noexcept;
void reset(const spi::HardwareInterface& spi) noexcept;

uint32_t get_pclk1() noexcept;
uint32_t get_pclk2() noexcept;

[[noreturn]] void reset_system() noexcept;
void reset_backup_domain() noexcept;

void set_system_clock(uint64_t hz);

}
}
}
}
