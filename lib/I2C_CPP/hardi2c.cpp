#include "i2c.hpp"
#include "rcc.hpp"
#include "units.hpp"
#include "_config.hpp"

namespace vermils
{
namespace stm32
{
namespace i2c
{
namespace detail
{
    #ifdef I2C1_BASE
    Register &reg1 = *reinterpret_cast<Register *>(I2C1_BASE);
    #endif
    #ifdef I2C2_BASE
    Register &reg2 = *reinterpret_cast<Register *>(I2C2_BASE);
    #endif
    #ifdef I2C3_BASE
    Register &reg3 = *reinterpret_cast<Register *>(I2C3_BASE);
    #endif
    #ifdef I2C4_BASE
    Register &reg4 = *reinterpret_cast<Register *>(I2C4_BASE);
    #endif
}


uint32_t HardMaster::_ClockSpeed::getter() const noexcept
{
    uint32_t ccr = owner.reg.CCR & I2C_CCR_CCR;
    if (owner.get_speed() == Speed::Standard)
    {
        return clock::rcc::get_pclk1() / (ccr * 2U);
    }
    else if (owner.get_fast_mode_duty_cycle() == FMDutyCycle::_16_9)
    {
        return clock::rcc::get_pclk1() / (ccr * 25ULL);
    }
    else
    {
        return clock::rcc::get_pclk1() / (ccr * 3U);
    }
}
void HardMaster::_ClockSpeed::setter(uint32_t value) const noexcept
{
    const bool original_state = owner.reg.CR1 & I2C_CR1_PE;
    owner.reg.CR1 &= ~I2C_CR1_PE;
    const uint32_t pclk = clock::rcc::get_pclk1();
    const uint32_t freq = pclk / 1_MHz;
    uint32_t ccr;
    const bool duty = !(freq % 10U);  // duty is true if clock is multiple of 10MHz, otherwise assume clock is multiple of 1.2MHz(duty is false)
    owner.reg.CR2 = (owner.reg.CR2 & ~I2C_CR2_FREQ) | (freq & I2C_CR2_FREQ);
    if (owner.get_speed() == Speed::Standard)
    {
        ccr = pclk / (value * 2U);
        ccr = ccr < 4U ? 4U : ccr;
        owner.max_rise_time = 1000ns;
        owner.reg.CCR &= ~I2C_CCR_DUTY;
    }
    else // fast modes 
    {
        owner.max_rise_time = 300ns;
        if (duty)  // fast modes duty
        {
            owner.reg.CCR |= I2C_CCR_DUTY;
            ccr = pclk / (value * 25ULL);
            ccr = ccr < 1U ? 1U : ccr;
        }
        else
        {
            owner.reg.CCR &= ~I2C_CCR_DUTY;
            ccr = pclk / (value * 3U);
            ccr = ccr < 4U ? 4U : ccr;
        }
    }
    ccr = ccr > I2C_CCR_CCR ? I2C_CCR_CCR : ccr;
    owner.reg.CCR = (owner.reg.CCR & ~I2C_CCR_CCR) | ccr;

    owner.reg.CR1 |= original_state ? I2C_CR1_PE : 0;
}
std::chrono::nanoseconds HardMaster::_MaxRiseTime::getter() const noexcept
{
    return 1ns * ((owner.reg.TRISE & I2C_TRISE_TRISE)-1U) * 1000U / (owner.reg.CR2 & I2C_CR2_FREQ);
}
void HardMaster::_MaxRiseTime::setter(std::chrono::nanoseconds value) const noexcept
{
    bool original_state = owner.reg.CR1 & I2C_CR1_PE;
    owner.reg.CR1 &= ~I2C_CR1_PE;
    uint32_t trise = value.count() * (owner.reg.CR2 & I2C_CR2_FREQ) / 1000U + 1;
    trise = trise > I2C_TRISE_TRISE ? I2C_TRISE_TRISE: trise;
    owner.reg.TRISE = (owner.reg.TRISE & ~I2C_TRISE_TRISE) | trise;
    owner.reg.CR1 |= original_state ? I2C_CR1_PE : 0;
}
bool HardMaster::is_arbitration_lost() const noexcept
{
    return reg.SR1 | I2C_SR1_ARLO;
}
void HardMaster::clear_arbitration_lost() const noexcept
{
    reg.SR1 &= ~I2C_SR1_ARLO;
}


void HardMaster::init()
{
    clock::rcc::enable_clock(*this);
    reg.CR1 &= ~I2C_CR1_PE;  // disable peripheral
    reg.CR1 |= I2C_CR1_SWRST; // reset peripheral
    reg.CR1 &= ~I2C_CR1_SWRST;
    set_speed(_speed);
    reg.SR2 |= I2C_SR2_MSL; // master mode
    reg.CR1 |= I2C_CR1_PE;  // enable peripheral
}
void HardMaster::deinit() noexcept
{
    reg.CR1 &= ~I2C_CR1_PE; // disable peripheral
    clock::rcc::disable_clock(*this);
}
void HardMaster::set_speed(Speed speed)
{
    switch (speed)
    {
        case Speed::Standard:
            _speed = Speed::Standard;
            reg.CCR &= ~I2C_CCR_FS;
            clock_speed = 200_KHz;
            break;
        case Speed::Fast:
            _speed = Speed::Fast;
            reg.CCR |= I2C_CCR_FS;
            clock_speed = 400_KHz;
            break;
        case Speed::FastPlus:
            _speed = Speed::FastPlus;
            reg.CCR |= I2C_CCR_FS;
            clock_speed = 1_MHz;
            break;
        default:
            throw std::invalid_argument("Invalid speed");
    }
}
Speed HardMaster::get_speed() const noexcept
{
    return _speed;
}
bool HardMaster::select(addr_t address, const bool read) const
{
    reg.SR1 &= ~I2C_SR1_AF;
    reg.CR1 |= I2C_CR1_PE;

    for (int i=0; (reg.SR2 & I2C_SR2_BUSY); ++i)
    {
        if (i == 100)
            throw BusBusy();
    }

    reg.CR1 &= ~I2C_CR1_POS;

    reg.CR1 |= I2C_CR1_ACK;
    reg.CR1 |= I2C_CR1_START;
    for (int i=0; not (reg.SR1 & I2C_SR1_SB); ++i)
        {
            raise_if_error();
            if (i == 100)
                return false;
        }

    // Determine 7/10 bit address
    if ((address & 0xf800) == 0xf000)
    {
        // 10 bit address
        // 5 bits(1111 0) + first 2 bits of address + W bit(W mode required) + last 8 bits of address
        address = (address & 0xfe00) | ((address & 0x1fe) >> 1);

        reg.DR = (uint8_t)(address>>8);
        for (int i=0; not (reg.SR1 & I2C_SR1_ADD10); ++i)
        {
            raise_if_error();
            if (i == 100)
                return false;
        }

        reg.DR = (uint8_t)(address & 0xff);
        for (int i=0; not (reg.SR1 & I2C_SR1_ADDR); ++i)
        {
            raise_if_error();
            if (i == 100)
                return false;
        }
        
        [[maybe_unused]]volatile uint32_t tmp;
        tmp = reg.SR1;
        tmp = reg.SR2;  // Clear ADDR flag
        
        if (read)
        {
            reg.CR1 |= I2C_CR1_START;
            for (int i=0; not (reg.SR1 & I2C_SR1_SB); ++i)
            {
                raise_if_error();
                if (i == 100)
                    return false;
            }
            while (not (reg.SR1 & I2C_SR1_TXE))
                raise_if_error();
            reg.DR = (uint8_t)(address >> 8 | 1);
            while (not (reg.SR1 & I2C_SR1_BTF))
                raise_if_error();
            return not (reg.SR1 & I2C_SR1_AF);
        }
        return not (reg.SR1 & I2C_SR1_AF);
    }
    else
    {
        // 7 bit address
        if (!(address & 0xff))
            // In case address is left aligned to uint16_t
            address >>= 8;
        reg.DR = (uint8_t)(address | read);

        for (int i=0; not (reg.SR1 & I2C_SR1_ADDR); ++i)
        {
            raise_if_error();
            if (i == 100)
                return false;
        }
        
        [[maybe_unused]]volatile uint32_t tmp;
        tmp = reg.SR1;
        tmp = reg.SR2;  // Clear ADDR flag

        if (read)
        {

        }

        return not (reg.SR1 & I2C_SR1_AF);
    }
}

void HardMaster::end() const
{
    reg.CR1 |= I2C_CR1_STOP;
}
bool HardMaster::write_byte(uint8_t data) const
{
    reg.SR1 &= ~I2C_SR1_AF;
    while (not (reg.SR1 & I2C_SR1_TXE))
        raise_if_error();
    reg.DR = data;

    while (not (reg.SR1 & I2C_SR1_BTF))
        raise_if_error();
    return not (reg.SR1 & I2C_SR1_AF);
}

uint8_t HardMaster::read_byte(bool acknowledge) const
{
    reg.SR1 &= ~I2C_SR1_AF;
    if (acknowledge)
        reg.CR1 |= I2C_CR1_ACK;
    else
        reg.CR1 &= ~I2C_CR1_ACK;
    while (not (reg.SR1 & I2C_SR1_RXNE))
        raise_if_error();
    return reg.DR;
}


void HardMaster::raise_if_error() const
{
    if (reg.SR1 & (I2C_SR1_PECERR | I2C_SR1_TIMEOUT | I2C_SR1_OVR | I2C_SR1_ARLO | I2C_SR1_BERR))
    {
        end();
        if (reg.SR1 & I2C_SR1_PECERR)
        {
            reg.SR1 &= ~I2C_SR1_PECERR;
            throw ValidationError();
        }
        else if (reg.SR1 & I2C_SR1_TIMEOUT)
        {
            reg.SR1 &= ~I2C_SR1_TIMEOUT;
            throw TimeoutError();
        }
        else if (reg.SR1 & I2C_SR1_OVR)
        {
            reg.SR1 &= ~I2C_SR1_OVR;
            throw OverrunError();
        }
        else if (reg.SR1 & I2C_SR1_ARLO)
        {
            reg.SR1 &= ~I2C_SR1_ARLO;
            throw ArbitrationLost();
        }
        else if (reg.SR1 & I2C_SR1_BERR)
        {
            reg.SR1 &= ~I2C_SR1_BERR;
            throw BusError();
        }
    }
}


void HardMaster::set_clock_strech(bool enable) const noexcept
{
    if (enable)
    {
        reg.CR1 &= ~I2C_CR1_NOSTRETCH;
    }
    else
    {
        reg.CR1 |= I2C_CR1_NOSTRETCH;
    }
}
bool HardMaster::get_clock_strech() const noexcept
{
    return !(reg.CR1 & I2C_CR1_NOSTRETCH);
}
void HardMaster::set_analog_filter(bool enable) const noexcept
{
    reg.CR1 &= ~I2C_CR1_PE;
    if (enable)
    {
        reg.FLTR &= ~I2C_FLTR_ANOFF;
    }
    else
    {
        reg.FLTR |= I2C_FLTR_ANOFF;
    }
    reg.CR1 |= I2C_CR1_PE;
}
bool HardMaster::get_analog_filter() const noexcept
{
    return !(reg.FLTR & I2C_FLTR_ANOFF);
}
void HardMaster::set_digital_filter(uint8_t cycles) const noexcept
{
    reg.CR1 &= ~I2C_CR1_PE;
    reg.FLTR |= (cycles & 0xFU);
    reg.CR1 |= I2C_CR1_PE;
}
uint8_t HardMaster::get_digital_filter() const noexcept
{
    return reg.FLTR & 0xFU;
}
void HardMaster::set_fast_mode_duty_cycle(FMDutyCycle duty_cycle) const noexcept
{
    if (duty_cycle == FMDutyCycle::_16_9)
    {
        reg.CCR |= I2C_CCR_DUTY;
    }
    else
    {
        reg.CCR &= ~I2C_CCR_DUTY;
    }
}
HardMaster::FMDutyCycle HardMaster::get_fast_mode_duty_cycle() const noexcept
{
    return (reg.CCR & I2C_CCR_DUTY) ? FMDutyCycle::_16_9 : FMDutyCycle::_2_1;
}
void HardMaster::set_extended_mode(HardMaster::ExtendedMode mode) const noexcept
{
    reg.CR1 &= ~(I2C_CR1_PE | I2C_CR1_SMBTYPE | I2C_CR1_SMBUS);
    if (mode == ExtendedMode::SMBusDevice)
    {
        reg.CR1 |= I2C_CR1_SMBUS | I2C_CR1_SMBTYPE;
    }
    else if (mode == ExtendedMode::SMBusHost)
    {
        reg.CR1 |= I2C_CR1_SMBUS;
    }
    reg.CR1 |= I2C_CR1_PE;
}
HardMaster::ExtendedMode HardMaster::get_extended_mode() const noexcept
{
    if (reg.CR1 & I2C_CR1_SMBUS)
    {
        if (reg.CR1 & I2C_CR1_SMBTYPE)
        {
            return ExtendedMode::SMBusDevice;
        }
        else
        {
            return ExtendedMode::SMBusHost;
        }
    }
    else
    {
        return ExtendedMode::None;
    }
}
void HardMaster::set_packet_error_checking(bool enable) const noexcept
{
    if (enable)
    {
        reg.CR1 |= I2C_CR1_PEC;
    }
    else
    {
        reg.CR1 &= ~I2C_CR1_PEC;
    }
}
bool HardMaster::get_packet_error_checking() const noexcept
{
    return reg.CR1 & I2C_CR1_PEC;
}


void HardMaster::set_event_interrupt(bool enable) const noexcept
{
    if (enable)
    {
        reg.CR2 |= I2C_CR2_ITEVTEN;
    }
    else
    {
        reg.CR2 &= ~I2C_CR2_ITEVTEN;
    }
}
void HardMaster::set_event_buffer_interrupt(bool enable) const noexcept
{
    if (enable)
    {
        reg.CR2 |= I2C_CR2_ITBUFEN;
    }
    else
    {
        reg.CR2 &= ~I2C_CR2_ITBUFEN;
    }
}
void HardMaster::set_error_interrupt(bool enable) const noexcept
{
    if (enable)
    {
        reg.CR2 |= I2C_CR2_ITERREN;
    }
    else
    {
        reg.CR2 &= ~I2C_CR2_ITERREN;
    }
}
void HardMaster::on_event_handler() const noexcept
try{
    Event event = Event::None;
    if (reg.SR1 & I2C_SR1_SB)
        event = Event::StartBitSent;
    else if (reg.SR1 & I2C_SR1_ADDR)
        event = Event::AddressSentOrMatched;
    else if (reg.SR1 & I2C_SR1_ADD10)
        event = Event::Address10bitHeaderSent;
    else if (reg.SR1 & I2C_SR1_STOPF)
        event = Event::StopReceived;
    else if (reg.SR1 & I2C_SR1_BTF)
        event = Event::DataByteFinished;
    else if (reg.SR1 & I2C_SR1_RXNE)
        event = Event::ReceiverAvailable;
    else if (reg.SR1 & I2C_SR1_TXE)
        event = Event::TransmitterAvailable;
    
    if (on_event)
        on_event(event);
}
catch(...) {}
void HardMaster::on_error_handler() const noexcept
try{
    Error error = Error::None;
    if (reg.SR1 & I2C_SR1_BERR)
        error = Error::BusError;
    else if (reg.SR1 & I2C_SR1_ARLO)
        error = Error::ArbitrationLost;
    else if (reg.SR1 & I2C_SR1_AF)
        error = Error::AcknowledgeFailure;
    else if (reg.SR1 & I2C_SR1_OVR)
        error = Error::OverrunUnderrun;
    else if (reg.SR1 & I2C_SR1_PECERR)
        error = Error::PECError;
    else if (reg.SR1 & I2C_SR1_TIMEOUT)
        error = Error::Timeout;
    else if (reg.SR1 & I2C_SR1_SMBALERT)
        error = Error::SMBusAlert;
    
    if (on_error)
        on_error(error);
}
catch(...) {}

#ifdef I2C1
HardMaster I2c1(detail::reg1, 0, nvic::IRQn_Type::I2C1_EV_IRQn, nvic::IRQn_Type::I2C1_ER_IRQn);
#endif
#ifdef I2C2
HardMaster I2c2(detail::reg2, 1, nvic::IRQn_Type::I2C2_EV_IRQn, nvic::IRQn_Type::I2C2_ER_IRQn);
#endif
#ifdef I2C3
HardMaster I2c3(detail::reg3, 2, nvic::IRQn_Type::I2C3_EV_IRQn, nvic::IRQn_Type::I2C3_ER_IRQn);
#endif
#ifdef I2C4
HardMaster I2c4(detail::reg4, 3, nvic::IRQn_Type::I2C4_EV_IRQn, nvic::IRQn_Type::I2C4_ER_IRQn);
#endif

extern "C"
{
    #ifdef I2C1
    void I2C1_EV_IRQHandler()
    {
        I2c1.on_event_handler();
    }
    void I2C1_ER_IRQHandler()
    {
        I2c1.on_error_handler();
    }
    #endif
    #ifdef I2C2
    void I2C2_EV_IRQHandler()
    {
        I2c2.on_event_handler();
    }
    void I2C2_ER_IRQHandler()
    {
        I2c2.on_error_handler();
    }
    #endif
    #ifdef I2C3
    void I2C3_EV_IRQHandler()
    {
        I2c3.on_event_handler();
    }
    void I2C3_ER_IRQHandler()
    {
        I2c3.on_error_handler();
    }
    #endif
    #ifdef I2C4
    void I2C4_EV_IRQHandler()
    {
        I2c4.on_event_handler();
    }
    void I2C4_ER_IRQHandler()
    {
        I2c4.on_error_handler();
    }
    #endif
}

}
}
}