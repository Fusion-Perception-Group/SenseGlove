#include "spi/hardspi.hpp"
#include "./_cpp_config.hpp"
#include "units.hpp"

namespace elfe {
namespace stm32 {
    namespace spi {
        namespace detail {
#ifdef SPI1_BASE
            Register& reg1 = *(Register*)SPI1_BASE;
#endif
#ifdef SPI2_BASE
            Register& reg2 = *(Register*)SPI2_BASE;
#endif
#ifdef SPI3_BASE
            Register& reg3 = *(Register*)SPI3_BASE;
#endif
#ifdef SPI4_BASE
            Register& reg4 = *(Register*)SPI4_BASE;
#endif
#ifdef SPI5_BASE
            Register& reg5 = *(Register*)SPI5_BASE;
#endif
#ifdef SPI6_BASE
            Register& reg6 = *(Register*)SPI6_BASE;
#endif
        }

        uint32_t HardwareInterface::_BaudRate::getter() const
        {
            uint8_t pres = ((owner.reg.CR1 & SPI_CR1_BR) >> 3U) + 1U;
            switch (owner.order) {
            case 1:
            case 2:
                return clock::rcc::get_pclk1() >> pres;
            case 0:
            case 3:
            case 4:
            case 5:
                return clock::rcc::get_pclk2() >> pres;
            default:
                return clock::rcc::get_pclk1() >> pres;
            }
        }
        void HardwareInterface::_BaudRate::setter(uint32_t baud) const
        {
            uint8_t pres = 0;
            uint32_t pclk = 0;
            switch (owner.order) {
            case 1:
                pclk = clock::rcc::get_pclk1();
                break;
            case 0:
            case 2:
            case 3:
            case 4:
            case 5:
                pclk = clock::rcc::get_pclk2();
                break;
            default:
                pclk = clock::rcc::get_pclk1();
            }
            while (pclk > baud) {
                pclk >>= 1U;
                ++pres;
            }
            --pres;
            pres = (pres > 7) ? 7 : pres;
            owner.reg.CR1 = (owner.reg.CR1 & ~SPI_CR1_BR) | (pres << 3U);
        }
        VoidResult<> HardwareInterface::turn_slave(bool yes) noexcept
        {
            if (yes) {
                reg.CR1 &= ~SPI_CR1_MSTR;
            } else {
                reg.CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI;
            }
            return EC::None;
        }
        bool HardwareInterface::is_slave() const noexcept
        {
            return !(reg.CR1 & SPI_CR1_MSTR);
        }
        VoidResult<> HardwareInterface::set_mode(Mode mode) noexcept
        {
            reg.CR1 = (reg.CR1 & ~(SPI_CR1_CPHA | SPI_CR1_CPOL)) | ((uint8_t)mode & 0x3U);
            return EC::None;
        }
        Mode HardwareInterface::get_mode() const noexcept
        {
            return (Mode)(reg.CR1 & (SPI_CR1_CPHA | SPI_CR1_CPOL));
        }
        Result<size_t> HardwareInterface::exchange_bytes(const void* tx, void* rx, size_t size)
        {
#define ELFE_TMP_WAIT_FOR(X)                             \
    do {                                                 \
        auto checker = clock::make_timeout(10ms);        \
        do {                                             \
            auto r = raise_if_error();                   \
            ELFE_PROP(r, Result<size_t>(0, r.error));    \
            r = checker.raise_if_timedout<SPITimeout>(); \
            ELFE_PROP(r, Result<size_t>(0, r.error));    \
        } while (not(X));                                \
    } while (0)

            const size_t original_size = size;
            uint8_t *tx8 = (uint8_t*)tx, *rx8 = (uint8_t*)rx;
            uint16_t *tx16 = (uint16_t*)tx, *rx16 = (uint16_t*)rx;
            const bool use_16bits = is_using_16bits();
            const bool readonly = (tx == nullptr);
            const bool writeonly = (rx == nullptr);
            const bool bidir = is_bidirectional();
            if (readonly and writeonly)
                return 0; // are you kidding me?

            ELFE_TMP_WAIT_FOR(not(reg.SR & SPI_SR_BSY));

            while (size) {

                ELFE_TMP_WAIT_FOR(reg.SR & SPI_SR_TXE);
                if (bidir)
                    reg.CR1 &= ~SPI_CR1_BIDIOE;
                if (readonly) {
                    reg.DR = 0xFFFF;
                } else if (use_16bits) {
                    reg.DR = *tx16++;
                } else {
                    reg.DR = *tx8++;
                }

                ELFE_TMP_WAIT_FOR(reg.SR & SPI_SR_RXNE);

                if (bidir)
                    reg.CR1 &= ~SPI_CR1_BIDIOE;

                if (writeonly) {
                    [[maybe_unused]] volatile uint32_t dummy = reg.DR;
                } else if (use_16bits) {
                    *rx16++ = reg.DR;
                } else {
                    *rx8++ = reg.DR;
                }

                size -= 1 + use_16bits;
            }
            ELFE_TMP_WAIT_FOR(not(reg.SR & SPI_SR_BSY));

            return original_size;
#undef ELFE_TMP_WAIT_FOR
        }

        VoidResult<> HardwareInterface::init()
        {
            clock::rcc::enable_clock(*this);
            reg.CR1 &= ~SPI_CR1_SPE;
            auto r = set_mode(Mode::Mode0);
            ELFE_PROP(r, r);
            r = turn_slave(false);
            ELFE_PROP(r, r);
            baudrate = 6_MHz; // 6 Mbits/s
            r = use_software_ss(true);
            ELFE_PROP(r, r);
            r = set_msb_first(true);
            ELFE_PROP(r, r);
            r = config_half_duplex(false, false);
            ELFE_PROP(r, r);
            r = use_16bits(false);
            ELFE_PROP(r, r);
            reg.CR2 = 0;

            reg.CR1 |= SPI_CR1_SPE;
            return EC::None;
        }
        void HardwareInterface::deinit()
        {
            reg.CR1 &= ~SPI_CR1_SPE;
            clock::rcc::reset(*this);
            clock::rcc::disable_clock(*this);
        }
        VoidResult<> HardwareInterface::use_16bits(bool yes)
        {
            bool original_state = reg.CR1 & SPI_CR1_SPE;
            if (yes) {
                reg.CR1 |= SPI_CR1_DFF;
            } else {
                reg.CR1 &= ~SPI_CR1_DFF;
            }

            if (original_state) {
                reg.CR1 |= SPI_CR1_SPE;
            }
            return EC::None;
        }
        bool HardwareInterface::is_using_16bits() const noexcept
        {
            return reg.CR1 & SPI_CR1_DFF;
        }
        VoidResult<> HardwareInterface::select_as_slave(bool yes)
        {
            if (yes) {
                reg.CR1 |= SPI_CR1_SSI;
            } else {
                reg.CR1 &= ~SPI_CR1_SSI;
            }
            return EC::None;
        }
        bool HardwareInterface::is_selected_as_slave() const noexcept
        {
            return reg.CR1 & SPI_CR1_SSI;
        }
        VoidResult<> HardwareInterface::use_software_ss(bool yes)
        {
            if (yes) {
                reg.CR1 |= SPI_CR1_SSM;
            } else {
                reg.CR1 &= ~(SPI_CR1_SSM | SPI_CR1_SSI);
            }
            return EC::None;
        }
        bool HardwareInterface::is_using_software_ss() const noexcept
        {
            return reg.CR1 & SPI_CR1_SSM;
        }
        VoidResult<> HardwareInterface::set_msb_first(bool yes)
        {
            if (yes) {
                reg.CR1 &= ~SPI_CR1_LSBFIRST;
            } else {
                reg.CR1 |= SPI_CR1_LSBFIRST;
            }
            return EC::None;
        }
        bool HardwareInterface::is_msb_first() const noexcept
        {
            return !(reg.CR1 & SPI_CR1_LSBFIRST);
        }

        VoidResult<> HardwareInterface::config_half_duplex(bool bidirectional, bool rxonly)
        {
            ELFE_ERROR_IF(
                bidirectional and rxonly, EC::InvalidArgument,
                std::invalid_argument("Cannot be bidirectional and rxonly at the same time"));

            if (bidirectional) {
                reg.CR1 |= SPI_CR1_BIDIMODE;
            } else if (rxonly) {
                reg.CR1 |= SPI_CR1_RXONLY;
                reg.CR1 &= ~SPI_CR1_BIDIMODE;
            } else {
                reg.CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY);
            }
            return EC::None;
        }
        bool HardwareInterface::is_half_duplex() const noexcept
        {
            return is_bidirectional() or is_rxonly();
        }
        bool HardwareInterface::is_bidirectional() const noexcept
        {
            return reg.CR1 & SPI_CR1_BIDIOE;
        }
        bool HardwareInterface::is_rxonly() const noexcept
        {
            return reg.CR1 & SPI_CR1_RXONLY;
        }

        VoidResult<> HardwareInterface::raise_if_error() const
        {
            uint32_t sr = reg.SR;
            [[likely]] if (not(sr & (SPI_SR_CRCERR | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_FRE)))
                return EC::None;

            [[maybe_unused]] volatile uint32_t dummy;

            if (sr & SPI_SR_CRCERR) {
                reg.SR &= ~SPI_SR_CRCERR;
                ELFE_ERROR(EC::SPICRCError, CRCError());
            }
            if (sr & SPI_SR_OVR) {
                dummy = reg.DR;
                dummy = reg.SR; // clear OVR flag
                ELFE_ERROR(EC::SPIOverrun, Overrun());
            }
            if (sr & SPI_SR_MODF) {
                dummy = reg.SR;
                reg.CR1 = reg.CR1; // clear MODF flag
                ELFE_ERROR(EC::SPIModeFault, ModeFault());
            }
            if (sr & SPI_SR_UDR) {
                dummy = reg.SR; // clear UDR flag
                ELFE_ERROR(EC::SPIUnderrun, Underrun());
            }
            if (sr & SPI_SR_FRE) {
                dummy = reg.SR; // clear TIFRFE flag
                ELFE_ERROR(EC::SPITIFrameError, TIFrameError());
            }
            ELFE_ERROR(EC::SPIError, SPIError("Unknown SPI error"));
        }

        void HardwareInterface::enable_on_send_interrupt() const noexcept
        {
            reg.CR2 |= SPI_CR2_TXEIE;
            nvic::enable_irq(irqn);
        }
        void HardwareInterface::disable_on_send_interrupt() const noexcept
        {
            reg.CR2 &= ~SPI_CR2_TXEIE;
        }
        void HardwareInterface::enable_on_recv_interrupt() const noexcept
        {
            reg.CR2 |= SPI_CR2_RXNEIE;
            nvic::enable_irq(irqn);
        }
        void HardwareInterface::disable_on_recv_interrupt() const noexcept
        {
            reg.CR2 &= ~SPI_CR2_RXNEIE;
        }
        void HardwareInterface::enable_on_error_interrupt() const noexcept
        {
            reg.CR2 |= SPI_CR2_ERRIE;
            nvic::enable_irq(irqn);
        }
        void HardwareInterface::disable_on_error_interrupt() const noexcept
        {
            reg.CR2 &= ~SPI_CR2_ERRIE;
        }

        void HardwareInterface::on_send_handler() const noexcept
        try {
            if (reg.SR & SPI_SR_TXE and reg.CR2 & SPI_CR2_TXEIE) {
                if (on_send_ready)
                    on_send_ready();
            }
        } catch (...) {
        }
        void HardwareInterface::on_recv_handler() const noexcept
        try {
            if (reg.SR & SPI_SR_RXNE and reg.CR2 & SPI_CR2_RXNEIE) {
                if (on_recv_ready)
                    on_recv_ready();
            }
        } catch (...) {
        }
        void HardwareInterface::on_error_handler() const noexcept
        try {
            if (reg.CR2 & SPI_CR2_ERRIE and reg.SR & (SPI_SR_CRCERR | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_FRE)) {
                if (on_error)
                    on_error();
            }
        } catch (...) {
        }

#ifdef SPI1_BASE
        HardwareInterface Spi1(detail::reg1, 0, nvic::SPI1_IRQn);
#endif
#ifdef SPI2_BASE
        HardwareInterface Spi2(detail::reg2, 1, nvic::SPI2_IRQn);
#endif
#ifdef SPI3_BASE
        HardwareInterface Spi3(detail::reg3, 2, nvic::SPI3_IRQn);
#endif
#ifdef SPI4_BASE
        HardwareInterface Spi4(detail::reg4, 3, nvic::SPI4_IRQn);
#endif
#ifdef SPI5_BASE
        HardwareInterface Spi5(detail::reg5, 4, nvic::SPI5_IRQn);
#endif
#ifdef SPI6_BASE
        HardwareInterface Spi6(detail::reg6, 5, nvic::SPI6_IRQn);
#endif

        extern "C" {
#ifdef SPI1_BASE
        void SPI1_IRQHandler()
        {
            Spi1.global_interrupt_handler();
        }
#endif
#ifdef SPI2_BASE
        void SPI2_IRQHandler()
        {
            Spi2.global_interrupt_handler();
        }
#endif
#ifdef SPI3_BASE
        void SPI3_IRQHandler()
        {
            Spi3.global_interrupt_handler();
        }
#endif
#ifdef SPI4_BASE
        void SPI4_IRQHandler()
        {
            Spi4.global_interrupt_handler();
        }
#endif
#ifdef SPI5_BASE
        void SPI5_IRQHandler()
        {
            Spi5.global_interrupt_handler();
        }
#endif
#ifdef SPI6_BASE
        void SPI6_IRQHandler()
        {
            Spi6.global_interrupt_handler();
        }
#endif
        }

    }
}
}
