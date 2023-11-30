#include "hardspi.hpp"
#include "_config.hpp"
#include "units.hpp"

namespace vermils
{
namespace stm32
{
namespace spi
{
    namespace detail
    {
        #ifdef SPI1_BASE
        Register &reg1 = *(Register*)SPI1_BASE;
        #endif
        #ifdef SPI2_BASE
        Register &reg2 = *(Register*)SPI2_BASE;
        #endif
        #ifdef SPI3_BASE
        Register &reg3 = *(Register*)SPI3_BASE;
        #endif
        #ifdef SPI4_BASE
        Register &reg4 = *(Register*)SPI4_BASE;
        #endif
        #ifdef SPI5_BASE
        Register &reg5 = *(Register*)SPI5_BASE;
        #endif
        #ifdef SPI6_BASE
        Register &reg6 = *(Register*)SPI6_BASE;
        #endif
    }


uint32_t HardwareInterface::_BaudRate::getter() const
{
    uint8_t pres = ((owner.reg.CR1 & SPI_CR1_BR) >> 3U)+1U;
    return clock::rcc::get_pclk2() >> pres;
}
void HardwareInterface::_BaudRate::setter(uint32_t baud) const
{
    uint8_t pres = 0;
    uint32_t pclk = clock::rcc::get_pclk2();
    while (pclk > baud)
    {
        pclk >>= 1U;
        ++pres;
    }
    owner.reg.CR1 = (owner.reg.CR1 & ~SPI_CR1_BR) | ((pres-1U) << 3U);
}
void HardwareInterface::turn_slave(bool yes) noexcept
{
    if (yes)
    {
        reg.CR1 &= ~SPI_CR1_MSTR;
    }
    else
    {
        reg.CR1 |= SPI_CR1_MSTR;
    }
}
bool HardwareInterface::is_slave() const noexcept
{
    return !(reg.CR1 & SPI_CR1_MSTR);
}
void HardwareInterface::set_mode(Mode mode)
{
    reg.CR1 = (reg.CR1 & ~(SPI_CR1_CPHA | SPI_CR1_CPOL)) | ((uint8_t)mode & 0x3U);
}
Mode HardwareInterface::get_mode() const noexcept
{
    return (Mode)(reg.CR1 & (SPI_CR1_CPHA | SPI_CR1_CPOL));
}
size_t HardwareInterface::exchange_bytes(void *tx, void *rx, size_t size)
{
    size_t count=0;
    uint8_t *tx8 = (uint8_t*)tx, *rx8 = (uint8_t*)rx;
    uint16_t *tx16 = (uint16_t*)tx, *rx16 = (uint16_t*)rx;

    const bool use_16bits = is_using_16bits();
    const bool readonly = (tx == nullptr)
                            or (is_half_duplex() 
                                and (
                                    (is_rxonly()
                                    or (is_bidirectional()
                                        and not(reg.CR1 & SPI_CR1_BIDIOE)))));
    const bool writeonly = (rx == nullptr) or (is_half_duplex() and (is_bidirectional() and (reg.CR1 & SPI_CR1_BIDIOE)));
    if (readonly and writeonly)
        return count;  // are you kidding me?
    
    while (reg.SR & SPI_SR_BSY)
        raise_if_error();

    if (not readonly and not writeonly)
    {
        while (count < size)
        {
            while (not (reg.SR & SPI_SR_TXE))
                raise_if_error();
            if (use_16bits)
            {
                reg.DR = *tx16++;
            }
            else
            {
                reg.DR = *tx8++;
            }

            while (not (reg.SR & SPI_SR_RXNE))
                raise_if_error();
            if (use_16bits)
            {
                *rx16++ = reg.DR;
                ++count;
            }
            else
            {
                *rx8++ = reg.DR;
            }
            ++count;
        }

    }
    else if (writeonly)
    {
        while (count < size)
        {
            while (not (reg.SR & SPI_SR_TXE))
                raise_if_error();
            if (use_16bits)
            {
                reg.DR = *tx16++;
                ++count;
            }
            else
            {
                reg.DR = *tx8++;
            }
            ++count;
        }
        while (not (reg.SR & SPI_SR_TXE))
            raise_if_error();
    }
    else // writeonly
    {
        while (count < size)
        {
            while (reg.SR & SPI_SR_BSY)
                raise_if_error();
            reg.DR = 0;

            while (not (reg.SR & SPI_SR_RXNE))
                raise_if_error();
            if (use_16bits)
            {
                *rx16++ = reg.DR;
                ++count;
            }
            else
            {
                *rx8++ = reg.DR;
            }
            ++count;
        }
    }

    while (reg.SR & SPI_SR_BSY)
        raise_if_error();
    
    return count;
}

void HardwareInterface::init()
{
    clock::rcc::enable_clock(*this);
    reg.CR1 &= ~SPI_CR1_SPE;
    set_mode(Mode::Mode1);
    turn_slave(false);
    baudrate = 5_MHz; // 5 Mbits/s
    set_msb_first(true);
    config_half_duplex(false);
    use_16bits(false);

    reg.CR1 |= SPI_CR1_SPE;
    reg.CR2 = 0;
}
void HardwareInterface::deinit()
{
    reg.CR1 &= ~SPI_CR1_SPE;
    clock::rcc::reset(*this);
    clock::rcc::disable_clock(*this);
}
void HardwareInterface::use_16bits(bool yes)
{
    bool original_state = reg.CR1 & SPI_CR1_SPE;
    if (yes)
    {
        reg.CR1 |= SPI_CR1_DFF;
    }
    else
    {
        reg.CR1 &= ~SPI_CR1_DFF;
    }

    if (original_state)
    {
        reg.CR1 |= SPI_CR1_SPE;
    }
}
bool HardwareInterface::is_using_16bits() const noexcept
{
    return reg.CR1 & SPI_CR1_DFF;
}
void HardwareInterface::select_as_slave(bool yes)
{
    if (yes)
    {
        reg.CR1 |= SPI_CR1_SSI;
    }
    else
    {
        reg.CR1 &= ~SPI_CR1_SSI;
    }
}
bool HardwareInterface::is_selected_as_slave() const noexcept
{
    return reg.CR1 & SPI_CR1_SSI;
}
void HardwareInterface::use_software_ss(bool yes)
{
    if (yes)
    {
        reg.CR1 |= SPI_CR1_SSM;
    }
    else
    {
        reg.CR1 &= ~(SPI_CR1_SSM | SPI_CR1_SSI);
    }
}
bool HardwareInterface::is_using_software_ss() const noexcept
{
    return reg.CR1 & SPI_CR1_SSM;
}
void HardwareInterface::set_msb_first(bool yes)
{
    if (yes)
    {
        reg.CR1 &= ~SPI_CR1_LSBFIRST;
    }
    else
    {
        reg.CR1 |= SPI_CR1_LSBFIRST;
    }
}
bool HardwareInterface::is_msb_first() const noexcept
{
    return !(reg.CR1 & SPI_CR1_LSBFIRST);
}

void HardwareInterface::config_half_duplex(bool enable, bool bidirectional, bool rxonly)
{
    if (enable)
    {
        reg.CR1 |= SPI_CR1_BIDIMODE;
        if (rxonly)
        {
            reg.CR1 |= SPI_CR1_RXONLY;
        }
        else
        {
            reg.CR1 &= ~SPI_CR1_RXONLY;
        }
        if (bidirectional)
        {
            reg.CR1 |= SPI_CR1_BIDIOE;
        }
        else
        {
            reg.CR1 &= ~SPI_CR1_BIDIOE;
        }
    }
    else
    {
        reg.CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY);
    }
}
bool HardwareInterface::is_half_duplex() const noexcept
{
    return reg.CR1 & SPI_CR1_BIDIMODE;
}
bool HardwareInterface::is_bidirectional() const noexcept
{
    return reg.CR1 & SPI_CR1_BIDIOE;
}
bool HardwareInterface::is_rxonly() const noexcept
{
    return reg.CR1 & SPI_CR1_RXONLY;
}

void HardwareInterface::raise_if_error() const
{
    if (not (reg.SR & (SPI_SR_CRCERR | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_FRE)))
        return;

    [[maybe_unused]]volatile uint32_t dummy;

    if (reg.SR & SPI_SR_CRCERR)
    {
        reg.SR &= ~SPI_SR_CRCERR;
        throw CRCError();
    }
    if (reg.SR & SPI_SR_OVR)
    {
        dummy = reg.DR;
        dummy = reg.SR;  // clear OVR flag
        throw Overrun();
    }
    if (reg.SR & SPI_SR_MODF)
    {
        dummy = reg.SR;
        reg.CR1 = reg.CR1;  // clear MODF flag
        throw SPIException("SPI Master Mode fault");
    }
    if (reg.SR & SPI_SR_UDR)
    {
        dummy = reg.SR;  // clear UDR flag
        throw Underrun();
    }
    if (reg.SR & SPI_SR_FRE)
    {
        dummy = reg.SR;  // clear TIFRFE flag
        throw TIFrameError();
    }
}


void HardwareInterface::set_on_send_interrupt(bool on) const noexcept
{
    if (on)
    {
        reg.CR2 |= SPI_CR2_TXEIE;
    }
    else
    {
        reg.CR2 &= ~SPI_CR2_TXEIE;
    }
}
void HardwareInterface::set_on_recv_interrupt(bool on) const noexcept
{
    if (on)
    {
        reg.CR2 |= SPI_CR2_RXNEIE;
    }
    else
    {
        reg.CR2 &= ~SPI_CR2_RXNEIE;
    }
}
void HardwareInterface::set_on_error_interrupt(bool on) const noexcept
{
    if (on)
    {
        reg.CR2 |= SPI_CR2_ERRIE;
    }
    else
    {
        reg.CR2 &= ~SPI_CR2_ERRIE;
    }
}

void HardwareInterface::on_send_handler() const noexcept
try{
    if (reg.SR & SPI_SR_TXE)
    {
        if (on_send_ready)
            on_send_ready();
    }
}
catch(...) {}
void HardwareInterface::on_recv_handler() const noexcept
try{
    if (reg.SR & SPI_SR_RXNE)
    {
        if (on_recv_ready)
            on_recv_ready();
    }
}
catch(...) {}
void HardwareInterface::on_error_handler() const noexcept
try{
    if (reg.SR & (SPI_SR_CRCERR | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_FRE))
    {
        if (on_error)
            on_error();
    }
}
catch(...) {}


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

extern "C"
{
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
