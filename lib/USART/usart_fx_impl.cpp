#include "usart.hpp"
#include "_config.hpp"
#include <cmath>

#if defined(_VERMIL_STM32FX) && !_VERMIL_STM32_USE_GENERIC

namespace vms
{
namespace stm32
{
namespace usart
{
namespace detail
{
    #ifdef USART1
    Register &reg1 = *reinterpret_cast<Register *>(USART1);
    #endif
    #ifdef USART2
    Register &reg2 = *reinterpret_cast<Register *>(USART2);
    #endif
    #ifdef USART3
    Register &reg3 = *reinterpret_cast<Register *>(USART3);
    #endif
    #ifdef USART6
    Register &reg6 = *reinterpret_cast<Register *>(USART6);
    #endif
}

uint32_t get_usart_pclk(uint8_t order)
{
    uint32_t pclk;
    #if defined(USART6) && defined(UART9) && defined(UART10)
    if (order == 0 || order == 5 || order == 8 || order == 9)
    {
        pclk = clock::rcc::get_pclk2();
    }
    #elif defined(USART6)
    if (order == 0 || order == 5)
    {
        pclk = clock::rcc::get_pclk2();
    }
    #else
    if (order == 0)
    {
        pclk = clock::rcc::get_pclk2();
    }
    #endif // USART6 || UART9 || UART10 
    else
    {
        pclk = clock::rcc::get_pclk1();
    }
    return pclk;
}

float HardUsart::_BaudRate::getter() const noexcept
{
    uint32_t pclk=get_usart_pclk(owner.order);
    auto brr = owner.reg.BRR;
    
    float ratio = owner.is_oversample_8x()?8.0f:16.0f;
    float usart_div = (brr>>4) + (brr&0x7U)/ratio;

    if (auto mode = owner.get_extended_mode();
        mode == HardUsart::ExtendedMode::SmartCard or
        mode == HardUsart::ExtendedMode::LIN or
        mode == HardUsart::ExtendedMode::IrDA)
        return pclk / 16.0f / usart_div;
    return pclk / ratio / usart_div;
}

void HardUsart::_BaudRate::setter(float baudrate) const noexcept
{
    uint32_t pclk=get_usart_pclk(owner.order);
    float usart_div, ratio = owner.is_oversample_8x()?8.0f:16.0f;

    if (auto mode = owner.get_extended_mode();
        mode == HardUsart::ExtendedMode::SmartCard or
        mode == HardUsart::ExtendedMode::LIN or
        mode == HardUsart::ExtendedMode::IrDA)
        usart_div = pclk / 16.0f / baudrate;
    else
        usart_div = pclk / ratio / baudrate;

    uint32_t integer_p = usart_div;
    owner.reg.BRR = (integer_p << 4) + std::roundf((usart_div - integer_p) * ratio);
}

void HardUsart::init() noexcept
{
    clock::rcc::enable_clock(*this);
    set_extended_mode(ExtendedMode::None);
    set_sync_clock(false);
    set_word_length(WordLength::Bits8);
    set_parity(Parity::None);
    baudrate = 9600;
    reg.CR1 |= USART_CR1_UE;
}
void HardUsart::deinit() noexcept
{
    reg.CR1 &= ~USART_CR1_UE;
    clock::rcc::disable_clock(*this);
}
void HardUsart::set_parity(Parity parity) noexcept
{
    if (parity == Parity::None)
        reg.CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
    else
    {
        reg.CR1 |= USART_CR1_PCE;
        if (parity == Parity::Even)
            reg.CR1 &= ~USART_CR1_PS;
        else
            reg.CR1 |= USART_CR1_PS;
    }
}
Parity HardUsart::get_parity() const noexcept
{
    if (reg.CR1 & USART_CR1_PCE)
    {
        if (reg.CR1 & USART_CR1_PS)
            return Parity::Odd;
        else
            return Parity::Even;
    }
    return Parity::None;
}
void HardUsart::set_stop_bits(StopBits stop_bits) noexcept
{
    reg.CR2 = (reg.CR2 & ~USART_CR2_STOP) | static_cast<uint32_t>(stop_bits);
}
StopBits HardUsart::get_stop_bits() const noexcept
{
    return static_cast<StopBits>(reg.CR2 & USART_CR2_STOP);
}
void HardUsart::set_word_length(WordLength word_length) noexcept
{
    reg.CR1 = (reg.CR1 & ~USART_CR1_M) | static_cast<uint32_t>(word_length);
}
WordLength HardUsart::get_word_length() const noexcept
{
    return static_cast<WordLength>(reg.CR1 & USART_CR1_M);
}
void HardUsart::break_transmission() noexcept
{
    reg.CR1 |= USART_CR1_SBK;
}
size_t HardUsart::buffered_exchange_bytes(const void *send, size_t send_size, void *recv, size_t recv_size)
{
    if (not _buffer_enabled)
        throw std::logic_error("Buffer is not enabled");
    auto *recv_ptr = static_cast<uint8_t *>(recv);
    auto *send_ptr = static_cast<const uint8_t *>(send);
    if (send == nullptr)
        send_size = 0;
    if (recv == nullptr)
        recv_size = 0;
    auto checker = clock::make_timeout(timeout_us);
    size_t last_tx_size = tx_buffer.size();
    const uint64_t equiv_ticks = timeout_us * SystemCoreClock / 1000000ULL;
    while (send_size or recv_size)
    {
        raise_if_error();
        if ((send_size and tx_buffer.size() != last_tx_size)
            or (recv_size and rx_buffer.size()))
        {
            checker.target = clock::get_systick() + equiv_ticks;
        }
        if (timeout_us)
            checker.raise_if_timedout();
        while (send_size and tx_buffer.size() < _tx_buffer_max_size)
        {
            tx_buffer.push_back(*send_ptr++);
            --send_size;
        }
        last_tx_size = tx_buffer.size();
        if (tx_buffer.size())
            enable_interrupt_transmit_ready();
        while (recv_size and rx_buffer.size())
        {
            *recv_ptr++ = rx_buffer.front();
            rx_buffer.pop_front();
            --recv_size;
        }
    }
    return std::max(send_size, recv_size);
}
size_t HardUsart::sync_exchange_bytes(const void *send, size_t send_size, void *recv, size_t recv_size)
{
    size_t sent_count = 0;
    size_t recv_count = 0;
    const uint8_t *send_ptr = static_cast<const uint8_t *>(send);
    uint8_t *recv_ptr = static_cast<uint8_t *>(recv);

    if (send == nullptr)
        send_size = 0;
    if (recv == nullptr)
        recv_size = 0;

    reg.CR1 |= USART_CR1_TE | USART_CR1_RE;
    if ((get_word_length() == WordLength::Bits9) ^  // 8 bit mode
        (get_parity() == Parity::None))
    {
        auto checker = clock::make_timeout(timeout_us);
        const uint64_t equiv_ticks = timeout_us * SystemCoreClock / 1000000ULL;
        while (sent_count + recv_count < send_size + recv_size)
        {
            bool send_ready, recv_ready;
            do
            {
                send_ready = reg.SR & USART_SR_TXE;
                recv_ready = reg.SR & USART_SR_RXNE;
                raise_if_error();
                if (timeout_us)
                    checker.raise_if_timedout();
            }
            while(not (send_ready or recv_ready));

            if (send_ready and sent_count < send_size)
            {
                reg.DR = *send_ptr++;
                ++sent_count;
                if (timeout_us)
                    checker.target = clock::get_systick() + equiv_ticks;
            }
            if (recv_ready and recv_count < recv_size)
            {
                *recv_ptr++ = reg.DR;
                ++recv_count;
                if (timeout_us)
                    checker.target = clock::get_systick() + equiv_ticks;
            }
        }
    }
    else if (get_word_length() == WordLength::Bits9) // 9 bit mode
    {
        unsigned overtook = 0, overget = 0;
        uint16_t send_tmp, recv_tmp;
        while (sent_count + recv_count < send_size + recv_size)
        {
            bool send_ready, recv_ready;
            do
            {
                send_ready = reg.SR & USART_SR_TXE;
                recv_ready = reg.SR & USART_SR_RXNE;
                raise_if_error();
            }
            while(not send_ready and not recv_ready);

            if (send_ready and sent_count < send_size)
            {
                send_tmp = *send_ptr++ >> overtook;
                if (sent_count != send_size - 1)
                    send_tmp |= *send_ptr << (8-overtook);
                reg.DR = send_tmp & 0x1FFU;
                if (++overtook == 8)
                {
                    overtook = 0;
                    ++send_ptr;
                    ++sent_count;
                    continue;
                }
            }
            if (recv_ready and recv_count < recv_size)
            {
                recv_tmp = reg.DR & 0x1FFU;
                *recv_ptr++ |= recv_tmp << overget;
                if (recv_count != recv_size - 1)
                    *recv_ptr = recv_tmp >> (8-overget);
                if (++overget == 8)
                {
                    overget = 0;
                    ++recv_ptr;
                    ++recv_count;
                    continue;
                }
            }
        }
    }
    else // 7 bit mode
    {
        unsigned sent_bits=0, recv_bits=0;
        uint16_t send_tmp, recv_tmp;  // use 16bit to avoid undefined behavior
        while (sent_count + recv_count < send_size + recv_size)
        {
            bool send_ready, recv_ready;
            do
            {
                send_ready = reg.SR & USART_SR_TXE;
                recv_ready = reg.SR & USART_SR_RXNE;
                raise_if_error();
            }
            while(not send_ready and not recv_ready);

            if (send_ready and sent_count < send_size)
            {
                send_tmp = *send_ptr >> sent_bits;
                if (sent_count != send_size - 1)
                    send_tmp |= *(send_ptr+1) << (8-sent_bits);
                reg.DR = send_tmp & 0x7FU;
                sent_bits += 7;
                if (sent_bits >= 8)
                {
                    sent_bits -= 8;
                    ++send_ptr;
                    ++sent_count;
                    continue;
                }
            }
            if (recv_ready and recv_count < recv_size)
            {
                recv_tmp = reg.DR & 0x7FU;
                *recv_ptr |= recv_tmp << recv_bits;
                recv_bits += 7;
                if (recv_bits >= 8)
                {
                    recv_bits -= 8;
                    ++recv_ptr;
                    if (++recv_count != recv_size)
                        *recv_ptr = recv_tmp >> (8-recv_bits);
                    continue;
                }
            }
        }
    }

    while (not (reg.SR & USART_SR_TC));
    raise_if_error();
    return sent_count + recv_count;
}
// size_t HardUsart::write(const void *data_v, size_t size)
// {
//     size_t count = 0;
//     const uint8_t *data = static_cast<const uint8_t *>(data_v);
//     reg.CR1 |= USART_CR1_TE;
//     if ((get_word_length() == WordLength::Bits9) ^  // 8 bit mode
//         (get_parity() == Parity::None))
//     {
//         while (count < size)
//         {
//             while(not (reg.SR & USART_SR_TXE))
//                 raise_if_error();
//             reg.DR = *data++;
//             ++count;
//         }
//     }
//     else if (get_word_length() == WordLength::Bits9) // 9 bit mode
//     {
//         unsigned overtook = 0;
//         uint16_t tmp;
//         while(count < size)
//         {
//             while(not (reg.SR & USART_SR_TXE))
//                 raise_if_error();

//             tmp = *data++ >> overtook;
//             if (count != size - 1)
//                 tmp |= *data << (8-overtook);
//             reg.DR = tmp & 0x1FFU;
//             if (++overtook == 8)
//             {
//                 overtook = 0;
//                 ++data;
//                 ++count;
//                 continue;
//             }
//         }
//     }
//     else // 7 bit mode
//     {
//         unsigned sent_bits=0;
//         uint16_t tmp;  // use 16bit to avoid undefined behavior
//         while(count < size)
//         {
//             while(not (reg.SR & USART_SR_TXE))
//                 raise_if_error();

//             tmp = *data >> sent_bits;
//             if (count != size)
//                 tmp |= *(data+1) << (8-sent_bits);
//             reg.DR = tmp & 0x7FU;
//             sent_bits += 7;
//             if (sent_bits >= 8)
//             {
//                 sent_bits -= 8;
//                 ++data;
//                 ++count;
//                 continue;
//             }
//         }
//     }

//     while (not (reg.SR & USART_SR_TC));
//     raise_if_error();
//     return count;
// }
// size_t HardUsart::read(void *data, size_t size)
// {
//     reg.CR1 |= USART_CR1_RE;
//     uint8_t *ptr = static_cast<uint8_t *>(data);
//     size_t count = 0;
//     if ((get_word_length() == WordLength::Bits9) ^  // 8 bit mode
//         (get_parity() == Parity::None))
//     {
//         while (count < size)
//         {
//             while(not (reg.SR & USART_SR_RXNE))
//                 raise_if_error();
//             *ptr++ = reg.DR;
//             ++count;
//         }
//     }
//     else if (get_word_length() == WordLength::Bits9) // 9 bit mode
//     {
//         unsigned overtook = 0;
//         uint16_t tmp;
//         *ptr = 0;
//         while(count < size)
//         {
//             while(not (reg.SR & USART_SR_RXNE))
//                 raise_if_error();

//             tmp = reg.DR & 0x1FFU;
//             *ptr++ |= tmp << overtook;
//             if (count != size - 1)
//                 *ptr = tmp >> (8-overtook);
//             if (++overtook == 8)
//             {
//                 overtook = 0;
//                 ++ptr;
//                 ++count;
//                 continue;
//             }
//         }
//     }
//     else // 7 bit mode
//     {
//         unsigned filled_bits=0;
//         uint16_t tmp;  // use 16bit to avoid undefined behavior
//         while(count < size)
//         {
//             while(not (reg.SR & USART_SR_RXNE))
//                 raise_if_error();

//             tmp = reg.DR & 0x7FU;
//             *ptr |= tmp << filled_bits;
//             filled_bits += 7;
//             if (filled_bits >= 8)
//             {
//                 filled_bits -= 8;
//                 ++ptr;
//                 if (++count != size)
//                     *ptr = tmp >> (8-filled_bits);
//                 continue;
//             }
//         }
//     }

//     return count;
// }

/**
 * @brief Trade clock deviation tolerance for higher baudrate. Allow clock to reach FCLK/8 instead of FCLK/16.
 * 
 */
void HardUsart::set_oversample_8x(bool on) const noexcept
{
    if (on)
        reg.CR1 |= USART_CR1_OVER8;
    else
        reg.CR1 &= ~USART_CR1_OVER8;
}
bool HardUsart::is_oversample_8x() const noexcept
{
    return reg.CR1 & USART_CR1_OVER8;
}
/**
 * @brief Set the baudrate
 * 
 * @param baudrate
 * @param keep_oversampling If false, oversampling could be changed to achieve the baudrate.
 * @throw std::invalid_argument If the baudrate is not achievable.
 */
void HardUsart::set_baudrate(float baudrate_f, bool keep_oversampling) const
{
    uint32_t pclk=get_usart_pclk(order);

    // check if the baudrate is achievable
    if (baudrate_f > pclk / 16)
    {
        if (keep_oversampling || is_oversample_8x())
        {
            if (baudrate_f > pclk / 8)
                throw std::invalid_argument("Baudrate is too high");
            set_oversample_8x(true);
        }
        else
            throw std::invalid_argument("Baudrate is too high");
    }
    baudrate = baudrate_f;
}
void HardUsart::set_extended_mode(ExtendedMode mode) noexcept
{
    reg.CR2 &= ~USART_CR2_LINEN;
    reg.CR3 &= ~(USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN);
    switch (mode)
    {
    case ExtendedMode::None:
        break;
    case ExtendedMode::SmartCard:
        reg.CR3 |= USART_CR3_SCEN;
        break;
    case ExtendedMode::HalfDuplex:
        reg.CR3 |= USART_CR3_HDSEL;
        break;
    case ExtendedMode::LIN:
        reg.CR2 |= USART_CR2_LINEN;
        break;
    case ExtendedMode::IrDA:
        reg.CR3 |= USART_CR3_IREN;
        break;
    }
}

HardUsart::ExtendedMode HardUsart::get_extended_mode() const noexcept
{
    if (reg.CR2 & USART_CR2_LINEN)
        return ExtendedMode::LIN;
    if (reg.CR3 & USART_CR3_IREN)
        return ExtendedMode::IrDA;
    if (reg.CR3 & USART_CR3_HDSEL)
        return ExtendedMode::HalfDuplex;
    if (reg.CR3 & USART_CR3_SCEN)
        return ExtendedMode::SmartCard;
    return ExtendedMode::None;
}

bool HardUsart::has_noise() const noexcept
{
    return reg.SR & USART_SR_NE;
}
bool HardUsart::has_parity_error() const noexcept
{
    return reg.SR & USART_SR_PE;
}
void HardUsart::raise_if_error() const
{
    if (_rx_buffer_overrun)
    {
        _rx_buffer_overrun = false;
        throw OverrunError();
    }
    if (!(reg.SR & (USART_SR_NE | USART_SR_PE | USART_SR_FE | USART_SR_ORE)))
        return;
    if (reg.SR & USART_SR_NE)
    {
        reg.SR &= ~USART_SR_NE;
        if (!suppress_noise_error)
            throw NoiseError();
    }
    if (reg.SR & USART_SR_PE)
    {
        reg.SR &= ~USART_SR_PE;
        if (!suppress_parity_error)
            throw ParityError();
    }
    if (reg.SR & USART_SR_FE)
    {
        reg.SR &= ~USART_SR_FE;
        throw FramingError();
    }
    if (reg.SR & USART_SR_ORE)
    {
        reg.SR &= ~USART_SR_ORE;
        if (!suppress_overrun_error)
            throw OverrunError();
    }
}
void HardUsart::set_transmitter(bool on) const noexcept
{
    if (on)
        reg.CR1 |= USART_CR1_TE;
    else
        reg.CR1 &= ~USART_CR1_TE;
}
void HardUsart::set_receiver(bool on) const noexcept
{
    if (on)
        reg.CR1 |= USART_CR1_RE;
    else
        reg.CR1 &= ~USART_CR1_RE;
}
void HardUsart::set_sync_clock(bool enable, bool polarity, bool phase, bool last_bit) const noexcept
{
    reg.CR1 &= ~(USART_CR1_TE | USART_CR1_RE);

    reg.CR2 &= ~(USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN | USART_CR2_LBCL);

    reg.CR2 |= (polarity ? USART_CR2_CPOL : 0) | (phase ? USART_CR2_CPHA : 0) | (enable ? USART_CR2_CLKEN : 0) | (last_bit ? USART_CR2_LBCL : 0);

    reg.CR1 |= (USART_CR1_TE | USART_CR1_RE);
}
void HardUsart::set_flow_control(bool clear_to_send_on, bool request_to_send_on) const noexcept
{
    reg.CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
    reg.CR3 |= (clear_to_send_on ? USART_CR3_CTSE : 0) | (request_to_send_on ? USART_CR3_RTSE : 0);
}



void HardUsart::on_transmit_complete_handler() const noexcept
try{
    if (reg.CR1 & USART_CR1_TCIE
        && reg.SR & USART_SR_TC)
    {
        // won't clear TC flag, either by reading SR then write to DR or writing 0 manually (in multi-buffer mode)
        if (on_transmit_complete)
            on_transmit_complete();
    }
}
catch(...) {}
void HardUsart::on_transmit_ready_handler() const noexcept
try{
    if (reg.CR1 & USART_CR1_TXEIE
        && reg.SR & USART_SR_TXE)
    {
        // won't clear TXE flag, should be cleared by writing DR
        if (on_transmit_ready)
            on_transmit_ready();
    }
}
catch(...) {}
void HardUsart::on_receive_related_handler() const noexcept
try{
    if (reg.CR1 & USART_CR1_RXNEIE)
    {
        // won't clear RXNE flag, either by reading DR or writing 0 manually (in multi-buffer mode)
        if (reg.SR & USART_SR_RXNE && on_receive_ready)
            on_receive_ready();
    }
}
catch(...) {}
void HardUsart::on_clear_to_send_handler() const noexcept
try{
    if (reg.CR3 & USART_CR3_CTSIE
        && reg.SR & USART_SR_CTS)
    {
        // won't clear CTS flag, should writing 0 manually
        if (on_clear_to_send)
            on_clear_to_send();
    }
}
catch(...) {}
void HardUsart::on_idle_line_handler() const noexcept
try{
    if (reg.CR1 & USART_CR1_IDLEIE
        && reg.SR & USART_SR_IDLE)
    {
        // cleared by reading SR then reading DR
        if (on_idle_line)
            on_idle_line();
    }
}
catch(...) {}
void HardUsart::on_parity_error_handler() const noexcept
try{
    if (reg.CR1 & USART_CR1_PEIE
        && reg.SR & USART_SR_PE)
    {
        // cleared by calling raise_if_error()
        if (on_parity_error)
            on_parity_error();
    }
}
catch(...) {}
void HardUsart::on_break_detected_handler() const noexcept
try{
    if (reg.CR2 & USART_CR2_LBDIE
        && reg.SR & USART_SR_LBD)
    {
        // cleared by writing 0 manually
        if (on_break_detected && reg.CR2 & USART_CR2_LBDIE)
            on_break_detected();
    }
}
catch(...) {}
void HardUsart::on_multi_buffer_error_handler() const noexcept
try{
    if (reg.CR3 & USART_CR3_EIE)
    {
        if (reg.SR & USART_SR_NE && on_noise)
            on_noise();
        if (reg.SR & USART_SR_FE && on_framing_error)
            on_framing_error();
    }
}
catch(...) {}
void HardUsart::on_overrun_handler() const noexcept
try{
    if ((reg.CR3 & USART_CR3_EIE || reg.CR1 & USART_CR1_RXNEIE)
        && reg.SR & USART_SR_ORE)
    {
        // cleared by calling raise_if_error()
        if (on_overrun)
            on_overrun();
    }
}
catch(...) {}
void HardUsart::enable_interrupt_transmit_complete() const noexcept
{
    reg.CR1 |= USART_CR1_TCIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_transmit_complete() const noexcept
{
    reg.CR1 &= ~USART_CR1_TCIE;
}
void HardUsart::enable_interrupt_transmit_ready() const noexcept
{
    reg.CR1 |= USART_CR1_TXEIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_transmit_ready() const noexcept
{
    reg.CR1 &= ~USART_CR1_TXEIE;
}
void HardUsart::enable_interrupt_receive_related() const noexcept
{
    reg.CR1 |= USART_CR1_RXNEIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_receive_related() const noexcept
{
    reg.CR1 &= ~USART_CR1_RXNEIE;
}
void HardUsart::enable_interrupt_clear_to_send() const noexcept
{
    reg.CR3 |= USART_CR3_CTSIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_clear_to_send() const noexcept
{
    reg.CR3 &= ~USART_CR3_CTSIE;
}
void HardUsart::enable_interrupt_idle_line() const noexcept
{
    reg.CR1 |= USART_CR1_IDLEIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_idle_line() const noexcept
{
    reg.CR1 &= ~USART_CR1_IDLEIE;
}
void HardUsart::enable_interrupt_parity_error() const noexcept
{
    reg.CR1 |= USART_CR1_PEIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_parity_error() const noexcept
{
    reg.CR1 &= ~USART_CR1_PEIE;
}
void HardUsart::enable_interrupt_break_detected() const noexcept
{
    reg.CR2 |= USART_CR2_LBDIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_break_detected() const noexcept
{
    reg.CR2 &= ~USART_CR2_LBDIE;
}
void HardUsart::enable_interrupt_multi_buffer_error() const noexcept
{
    reg.CR3 |= USART_CR3_EIE;
    nvic::enable_irq(irqn);
}
void HardUsart::disable_interrupt_multi_buffer_error() const noexcept
{
    reg.CR3 &= ~USART_CR3_EIE;
}

#ifdef USART1
HardUsart Usart1(detail::reg1, 0, nvic::USART1_IRQn);
#endif
#ifdef USART2
HardUsart Usart2(detail::reg2, 1, nvic::USART2_IRQn);
#endif
#ifdef USART3
HardUsart Usart3(detail::reg3, 2, nvic::USART3_IRQn);
#endif
#ifdef USART6
HardUsart Usart6(detail::reg6, 5, nvic::USART6_IRQn);
#endif


extern "C"
{
    #ifdef USART1
    void USART1_IRQHandler()
    {
        Usart1.global_interrupt_handler();
    }
    #endif
    #ifdef USART2
    void USART2_IRQHandler()
    {
        Usart2.global_interrupt_handler();
    }
    #endif
    #ifdef USART3
    void USART3_IRQHandler()
    {
        Usart3.global_interrupt_handler();
    }
    #endif
    #ifdef USART6
    void USART6_IRQHandler()
    {
        Usart6.global_interrupt_handler();
    }
    #endif
}


}
}
}

#endif
