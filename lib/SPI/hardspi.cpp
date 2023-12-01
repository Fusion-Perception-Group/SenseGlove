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
    switch (owner.order)
    {
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
    switch (owner.order)
    {
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
    while (pclk > baud)
    {
        pclk >>= 1U;
        ++pres;
    }
    --pres;
    pres = (pres > 7) ? 7 : pres;
    owner.reg.CR1 = (owner.reg.CR1 & ~SPI_CR1_BR) | (pres << 3U);
}
void HardwareInterface::turn_slave(bool yes) noexcept
{
    if (yes)
    {
        reg.CR1 &= ~SPI_CR1_MSTR;
    }
    else
    {
        reg.CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI;
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
size_t HardwareInterface::exchange_bytes(const void *tx, void *rx, size_t size)
{
    #define __SPI_WAIT_FOR(X)\
        do\
        {\
            auto checker = clock::make_timeout(10ms);\
            do\
            {\
                raise_if_error();\
                checker.raise_if_timedout<Timeout>();\
            }\
            while (not (X));\
        } while (0)

    const size_t original_size = size;
    uint8_t *tx8 = (uint8_t*)tx, *rx8 = (uint8_t*)rx;
    uint16_t *tx16 = (uint16_t*)tx, *rx16 = (uint16_t*)rx;
    const bool use_16bits = is_using_16bits();
    const bool readonly = (tx == nullptr);
    const bool writeonly = (rx == nullptr);
    const bool bidir = is_bidirectional();
    if (readonly and writeonly)
        return 0;  // are you kidding me?
    
    __SPI_WAIT_FOR(not (reg.SR & SPI_SR_BSY));

    while (size)
    {

        __SPI_WAIT_FOR(reg.SR & SPI_SR_TXE);
        if (bidir)
            reg.CR1 &= ~SPI_CR1_BIDIOE;
        if (readonly)
        {
            reg.DR = 0xFFFF;
        }
        else if (use_16bits)
        {
            reg.DR = *tx16++;
        }
        else
        {
            reg.DR = *tx8++;
        }

        __SPI_WAIT_FOR(reg.SR & SPI_SR_RXNE);

        if (bidir)
            reg.CR1 &= ~SPI_CR1_BIDIOE;

        if (writeonly)
        {
            [[maybe_unused]]volatile uint32_t dummy = reg.DR;
        }
        else if (use_16bits)
        {
            *rx16++ = reg.DR;
        }
        else
        {
            *rx8++ = reg.DR;
        }

        size -= 1 + use_16bits;
    }
    __SPI_WAIT_FOR(not (reg.SR & SPI_SR_BSY));
    
    return original_size;
}

void HardwareInterface::init()
{
    clock::rcc::enable_clock(*this);
    reg.CR1 &= ~SPI_CR1_SPE;
    set_mode(Mode::Mode0);
    turn_slave(false);
    baudrate = 6_MHz; // 6 Mbits/s
    use_software_ss(true);
    set_msb_first(true);
    config_half_duplex(false, false);
    use_16bits(false);
    reg.CR2 = 0;

    reg.CR1 |= SPI_CR1_SPE;
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

void HardwareInterface::config_half_duplex(bool bidirectional, bool rxonly)
{
    if (bidirectional and rxonly)
        throw std::invalid_argument("Cannot be bidirectional and rxonly at the same time");

    if (bidirectional)
    {
        reg.CR1 |= SPI_CR1_BIDIMODE;
    }
    else if (rxonly)
    {
        reg.CR1 |= SPI_CR1_RXONLY;
        reg.CR1 &= ~SPI_CR1_BIDIMODE;
    }
    else
    {
        reg.CR1 &= ~(SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE | SPI_CR1_RXONLY);
    }
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

void HardwareInterface::raise_if_error() const
{
    uint32_t sr = reg.SR;
    if (not (sr & (SPI_SR_CRCERR | SPI_SR_OVR | SPI_SR_MODF | SPI_SR_UDR | SPI_SR_FRE)))
        return;

    [[maybe_unused]]volatile uint32_t dummy;

    if (sr & SPI_SR_CRCERR)
    {
        reg.SR &= ~SPI_SR_CRCERR;
        throw CRCError();
    }
    if (sr & SPI_SR_OVR)
    {
        dummy = reg.DR;
        dummy = reg.SR;  // clear OVR flag
        throw Overrun();
    }
    if (sr & SPI_SR_MODF)
    {
        dummy = reg.SR;
        reg.CR1 = reg.CR1;  // clear MODF flag
        throw SPIException("SPI Master Mode fault");
    }
    if (sr & SPI_SR_UDR)
    {
        dummy = reg.SR;  // clear UDR flag
        throw Underrun();
    }
    if (sr & SPI_SR_FRE)
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

uint32_t test()
{
    SPI_HandleTypeDef hspi{
        .Instance = SPI1,
        .Init{
            .Direction = SPI_DIRECTION_2LINES,
            .CLKPolarity = SPI_POLARITY_LOW,
            .CLKPhase = SPI_PHASE_1EDGE,
            .NSS = SPI_NSS_SOFT,
            .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
            .FirstBit = SPI_FIRSTBIT_MSB,
            .TIMode = SPI_TIMODE_DISABLE,
            .CRCCalculation = SPI_CRCCALCULATION_DISABLE,
        }
    };
    HAL_SPI_Init(&hspi);

    [[maybe_unused]]uint8_t tx[10], rx[10];
    tx[0] = 0x9f;
    HAL_SPI_TransmitReceive(&hspi, tx, rx, 1, 1000);
    return rx[0];
}

}
}
}
