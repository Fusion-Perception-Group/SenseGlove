#pragma once
#include <cstdint>
#include <functional>
#include "userconfig.hpp"
#include "nvic.hpp"
#include "property.hpp"
#include "gpio.hpp"

namespace vermils
{
namespace stm32
{
namespace clock
{
using CallbackType = std::function<void()>;
namespace detail
{
    struct Register
    {
        volatile uint32_t CR1;   /*!< TIM control register 1,                   Address offset: 0x00 */
        volatile uint32_t CR2;   /*!< TIM control register 2,                   Address offset: 0x04 */
        volatile uint32_t SMCR;  /*!< TIM slave mode control register,          Address offset: 0x08 */
        volatile uint32_t DIER;  /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
        volatile uint32_t SR;    /*!< TIM status register,                      Address offset: 0x10 */
        volatile uint32_t EGR;   /*!< TIM event generation register,            Address offset: 0x14 */
        volatile uint32_t CCMR1; /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
        volatile uint32_t CCMR2; /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
        volatile uint32_t CCER;  /*!< TIM capture/compare enable register,      Address offset: 0x20 */
        volatile uint32_t CNT;   /*!< TIM counter register,                     Address offset: 0x24 */
        volatile uint32_t PSC;   /*!< TIM prescaler,                            Address offset: 0x28 */
        volatile uint32_t ARR;   /*!< TIM auto-reload register,                 Address offset: 0x2C */
        volatile uint32_t RCR;   /*!< TIM repetition counter register,          Address offset: 0x30 */
        volatile uint32_t CCR1;  /*!< TIM capture/compare register 1,           Address offset: 0x34 */
        volatile uint32_t CCR2;  /*!< TIM capture/compare register 2,           Address offset: 0x38 */
        volatile uint32_t CCR3;  /*!< TIM capture/compare register 3,           Address offset: 0x3C */
        volatile uint32_t CCR4;  /*!< TIM capture/compare register 4,           Address offset: 0x40 */
        volatile uint32_t BDTR;  /*!< TIM break and dead-time register,         Address offset: 0x44 */
        volatile uint32_t DCR;   /*!< TIM DMA control register,                 Address offset: 0x48 */
        volatile uint32_t DMAR;  /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
    #if defined(__VERMIL_STM32HX)
        uint32_t RESERVED1;      /*!< Reserved, 0x50                                                 */
        volatile uint32_t CCMR3; /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
        volatile uint32_t CCR5;  /*!< TIM capture/compare register5,            Address offset: 0x58 */
        volatile uint32_t CCR6;  /*!< TIM capture/compare register6,            Address offset: 0x5C */
        volatile uint32_t AF1;   /*!< TIM alternate function option register 1, Address offset: 0x60 */
        volatile uint32_t AF2;   /*!< TIM alternate function option register 2, Address offset: 0x64 */
        volatile uint32_t TISEL; /*!< TIM Input Selection register,             Address offset: 0x68 */
    #elif defined(__VERMIL_STM32FX)
        volatile uint32_t OR;    /*!< TIM option register,                      Address offset: 0x50 */
    #endif
    };

    #if __VERMIL_STM32_USE_GENERIC
        #define __Register_Type Register
    #else
        #define __Register_Type Register &
    #endif

    extern __Register_Type TIM1_Reg;
    extern __Register_Type TIM2_Reg;
    extern __Register_Type TIM3_Reg;
    extern __Register_Type TIM4_Reg;
    extern __Register_Type TIM5_Reg;
    extern __Register_Type TIM6_Reg;
    extern __Register_Type TIM7_Reg;
    extern __Register_Type TIM8_Reg;
    extern __Register_Type TIM9_Reg;
    extern __Register_Type TIM10_Reg;
    extern __Register_Type TIM11_Reg;
    extern __Register_Type TIM12_Reg;
    extern __Register_Type TIM13_Reg;
    extern __Register_Type TIM14_Reg;
    extern __Register_Type TIM15_Reg;
    extern __Register_Type TIM16_Reg;
    extern __Register_Type TIM17_Reg;
}

enum class ClockSource : uint32_t
{
    ExternalMode2,  /*!< External clock source mode 2                          */
    Internal,       /*!< Internal clock source                                 */
    ITR0,           /*!< External clock source mode 1 (ITR0)                   */
    ITR1,           /*!< External clock source mode 1 (ITR1)                   */
    ITR2,           /*!< External clock source mode 1 (ITR2)                   */
    ITR3,           /*!< External clock source mode 1 (ITR3)                   */
    TTI1FP1_ED,     /*!< External clock source mode 1 (TTI1FP1 + edge detect.) */
    TTI1FP1,        /*!< External clock source mode 1 (TTI1FP1)                */
    TTI2FP2,        /*!< External clock source mode 1 (TTI2FP2)                */
    ExternalMode1   /*!< External clock source mode 1 (ETRF)                   */
};

enum class ClockPolarity : uint32_t
{
    Inverted,     /*!< Polarity for ETRx clock sources */
    NonInverted,  /*!< Polarity for ETRx clock sources */
    Rising,       /*!< Polarity for TIx clock sources  */
    Falling,      /*!< Polarity for TIx clock sources  */
    BothEdge      /*!< Polarity for TIx clock sources  */
};

enum class ClockPrescaler : uint32_t
{
    Div1 = 0U,  /*!< No prescaler is used */
    Div2 = 1U,  /*!< Prescaler = 2        */
    Div4 = 2U,  /*!< Prescaler = 4        */
    Div8 = 3U   /*!< Prescaler = 8        */
};

enum class CountMode : uint32_t
{
    Up,      /*!< Counter used as up-counter   */
    Down,    /*!< Counter used as down-counter */
    CenterAligned1,  /*!< Center-aligned mode 1, OC interrupt flag is set when
                          counter is counting down */
    CenterAligned2,  /*!< Center-aligned mode 2, OC interrupt flag is set when
                          counter is counting up */
    CenterAligned3   /*!< Center-aligned mode 3, OC interrupt flag is set when
                                    counter is counting up or down */
};

enum class ClockDivision : uint32_t
{
    Div1,  /*!< No division */
    Div2,  /*!< Division by 2 */
    Div4   /*!< Division by 4 */
};

enum class MasterTriggerMode : uint32_t
{
    Reset,  // EGR.UG bit is used as trigger output
    Enable, // CR1.CEN bit is used as trigger output
    Update, // Update event is used as trigger output
    ComparePulse, // Capture or a compare match 1 is used as trigger output
    OutputCompare1, // OC1REF signal is used as trigger output
    OutputCompare2, // OC2REF signal is used as trigger output
    OutputCompare3, // OC3REF signal is used as trigger output
    OutputCompare4  // OC4REF signal is used as trigger output
};

enum class OutputCompareMode : uint32_t
{
    Frozen = 0x0000U,           /*!< Output compare frozen, do nothing */
    ActiveOnMatch = 0x0010U,    /*!< Output compare active on match    */
    InactiveOnMatch = 0x0020U,  /*!< Output compare inactive on match  */
    Toggle = 0x0030U,           /*!< Output compare toggle             */
    ForceInactive = 0x0040U,    /*!< Output compare force inactive     */
    ForceActive = 0x0050U,      /*!< Output compare force active       */
    PWM2 = 0x0070U,             /*!< Output compare PWM2: upper valid ->  __/\\__ */
    PWM1 = 0x0060U,             /*!< Output compare PWM1: lower valid -> ^^/^^\\^^  */
};

enum class EdgeTrigger : uint32_t
{
    Rising = 0x0U,
    Falling = 0x2U,
    Both = 0xAU
};

enum class IOSelection : uint32_t
{
    Output = 0U,    /*!< No external input selected, use output mode */
    Direct = 1U,  /*!< TIM Input 1, 2, 3 or 4 is selected to be connected to IC1, IC2, IC3 or IC4, respectively */
    Indirect = 2U,/*!< TIM Input 1, 2, 3 or 4 is selected to be connected to IC2, IC1, IC4 or IC3, respectively */
    TRC = 3U      /*!< TIM Input 1, 2, 3 or 4 is selected to be connected to TRC, This mode is working only if an internal trigger input is selected through TS bit (TIMx_SMCR register)*/
};

struct ClockSourceConfig
{
    ClockSource source = ClockSource::Internal;
    ClockPolarity polarity = ClockPolarity::NonInverted;
    ClockPrescaler prescaler = ClockPrescaler::Div1;
    uint8_t filter = 0;  // filter for the input clock, consecutive samples must be equal
};

struct TimeBaseConfig
{
    CountMode count_mode = CountMode::Up;
    ClockDivision clock_division = ClockDivision::Div1;  // used for filter sampling
    uint32_t period = 0;  // determines the duration of a single cycle
    uint16_t prescaler = 0;  // in/decrements the counter every prescaler + 1 cycles
    uint16_t repetition_counter = 0; // UEV is generated every repetition_counter + 1 cycles
    bool auto_reload_preload = false;  // whether to preload the auto-reload register, equivalent to 0x80U
};

struct MasterConfig
{
    MasterTriggerMode mode = MasterTriggerMode::Reset;
    bool enable = false;  // equivalent to TIM_MASTERSLAVEMODE_ENABLE == 0x80U
};

struct CaptureCompareConfig
{
    OutputCompareMode output_mode = OutputCompareMode::Frozen;
    uint32_t pulse = 0;  // value to be loaded into the capture/compare register
    bool inverse_polarity = false;  // whether to use low as the active output for capture/compare
    bool output_n_low = false;  // whether to use low as the active output for capture/compare complementary
    bool idle_state = false;  // true for high, false for low
    bool idle_n_state = false;  // true for high, false for low
    bool fast_mode = false;  // whether to use fast mode for pwm
    EdgeTrigger input_trigger = EdgeTrigger::Rising;  // trigger for capture/compare
    IOSelection io_selection = IOSelection::Output;  // input/output selection for capture/compare
    ClockPrescaler input_prescaler = ClockPrescaler::Div1;  // input prescaler for capture/compare
    uint8_t input_filter = 0;  // input filter for capture/compare, max 15
};

class BasicTimer
{
public:
    detail::Register &reg;
    volatile uint32_t & counter;
    volatile uint32_t & auto_reload;
    volatile uint32_t & prescaler;
    const nvic::IRQn_Type reload_irqn;
    const uint32_t MAX_PERIOD;
    const uint32_t MAX_PRESCALER;
    const uint32_t MAX_REP_COUNTER;
    tricks::Property<bool> auto_reload_preload{
        [this]() -> bool { return reg.CR1 & 0x80U; },
        [this](bool value) -> void { reg.CR1 = value ? (reg.CR1|0x80U) : (reg.CR1&~0x80U); }
    };
    tricks::Property<bool> enabled{  /* time base enabled */
        [this]() -> bool { return reg.CR1 & 0x01U; },
        [this](bool value) -> void { if (value) this->start(); else this->stop(); }
    };
    tricks::Property<bool> direction{ /* true for down, false for up */
        [this]() -> bool { return reg.CR1 & 0x10U; },
        [this](bool value) -> void { reg.CR1 = value ? (reg.CR1|0x10U) : (reg.CR1&~0x10U); }
    };
    CallbackType on_reload = nullptr;  /* on overflow and underflow */
    
    BasicTimer(detail::Register &reg, nvic::IRQn_Type r_irqn)
        : BasicTimer(reg, r_irqn, ClockSourceConfig(), TimeBaseConfig())
    {}
    BasicTimer(detail::Register &reg, nvic::IRQn_Type r_irqn,
        const ClockSourceConfig &clock_config, const TimeBaseConfig &time_base_config,
        uint32_t max_peroid=0xFFFFU, uint32_t max_prescaler=0xFFFFU, uint32_t max_repetition=0xFFU)
        : reg(reg), counter(reg.CNT), auto_reload(reg.ARR), prescaler(reg.PSC), reload_irqn(r_irqn),
          MAX_PERIOD(max_peroid), MAX_PRESCALER(max_prescaler), MAX_REP_COUNTER(max_repetition)
    {
        set_time_base(time_base_config);
        set_clock_source(clock_config);
    }
    BasicTimer & operator=(const BasicTimer &) = delete;
    virtual ~BasicTimer() = default;

    void set_time_base(const TimeBaseConfig &config) noexcept;
    void set_clock_source(const ClockSourceConfig &config) noexcept;
    void set_master(const MasterConfig &config) noexcept;

    void start() noexcept
    {
        reg.CR1 |= 0x01U;
    }
    void stop() noexcept
    {
        reg.CR1 &= ~0x01U;
    }

    /**
     * @brief Set the counter
     * 
     * @param value 
     * @throw std::invalid_argument if value exceeds maximum period
     */
    void set_counter(uint32_t value)
    {
        if (value > MAX_PERIOD)
            throw std::invalid_argument("value exceeds maximum period");
        counter = value;
    }

    /**
     * @brief Set the auto reload
     * 
     * @param value 
     * @throw std::invalid_argument if value exceeds maximum period
     */
    void set_auto_reload(uint32_t value)
    {
        if (value > MAX_PERIOD)
            throw std::invalid_argument("value exceeds maximum period");
        auto_reload = value;
    }

    void set_prescaler(uint16_t value) noexcept
    {
        prescaler = value;
    }

    void enable_reload_irq() const noexcept;

    void set_reload_irq_priority(const uint8_t priority) const
    {
        nvic::set_priority(reload_irqn, priority);
    }

    void disable_reload_irq() const noexcept;

    void on_reload_handler() const noexcept
    try{
        const uint32_t UPDATE_MASK = 0x01U;
        if (reg.SR & UPDATE_MASK &&  // check if flag is set
            reg.DIER & UPDATE_MASK &&  // check if source is enabled
            on_reload)  // check if callback is set
        {
            on_reload();
        }

        reg.SR = ~UPDATE_MASK; // clear pending interrupt
    }
    catch(...)
    {
        // do nothing
    }

    void global_irq_handler() const noexcept
    {
        on_reload_handler();
    }

    bool operator==(const BasicTimer &rhs) const
    {
        return &reg == &rhs.reg;
    }
};

class BaseGeneralPurposeTimer : public BasicTimer
{
public:
    class Channel
    {
        using T = BaseGeneralPurposeTimer;
        T &_timer;
        constexpr volatile uint32_t & _get_ccr(const T& timer, const uint8_t order)
        {
            switch(order)
            {
            case 0: return timer.reg.CCR1;
            case 1: return timer.reg.CCR2;
            case 2: return timer.reg.CCR3;
            case 3: return timer.reg.CCR4;
            default: throw std::invalid_argument("order must be in [0, 3]");
            }
        }
        constexpr volatile uint32_t & _get_ccmr(const T& timer, const uint8_t order)
        {
            return (order < 2) ? timer.reg.CCMR1 : timer.reg.CCMR2;
        }
    public:
        const gpio::Pin output_pin;
        const gpio::Pin input_pin;
        const uint8_t order;
        volatile uint32_t &CapComRegister; // not that Capcom, no games here
        volatile uint32_t &CCMRx;
        CallbackType on_capture = nullptr;
        CallbackType on_compare = nullptr;
        tricks::Property<bool> enabled{
            [this]() -> bool { return _timer.reg.CCER & (1U << (order * 4)); },
            [this](bool value) -> void {
                if (value) enable(); else disable();
                }
        };
        tricks::Property<bool> inverse_polarity{
            [this]() -> bool { return _timer.reg.CCER & (0x2U << (order * 4)); },
            [this](bool value) -> void {
                const uint32_t mask = 0x2U << (order * 4);
                _timer.reg.CCER = (_timer.reg.CCER & ~mask) | (value ? mask : 0);
                }
        };
        tricks::Property<bool> output_preload_enabled{
            [this]() -> bool { return CCMRx & (1U << ((order%2) * 8 + 3)); },
            [this](bool value) -> void {
                const uint32_t mask = 1U << ((order%2) * 8 + 3);
                CCMRx = (CCMRx & ~mask) | (value ? mask : 0);
                }
        };
        tricks::Property<bool> pwm_fast_mode{
            [this]() -> bool { return CCMRx & (1U << ((order%2) * 8 + 2)); },
            [this](bool value) -> void {
                const uint32_t mask = 1U << ((order%2) * 8 + 2);
                CCMRx = (CCMRx & ~mask) | (value ? mask : 0);
                }
        };
        tricks::Property<OutputCompareMode> output_mode{
            [this]() -> OutputCompareMode {
                uint32_t tmp;
                const bool mod1 = (order % 2);
                tmp = CCMRx;
                tmp &= (mod1 ? 0x7300U : 0x73U);
                tmp >>= (mod1 ? 8U : 0U);
                return static_cast<OutputCompareMode>(tmp);
            },
            [this](OutputCompareMode value) -> void {
                uint32_t tmp;
                const uint32_t shift = (order%2) * 8;
                tmp = CCMRx;
                tmp &= ~(0x73U << shift);
                tmp |= static_cast<uint32_t>(value) << shift;
                CCMRx = tmp;
            }
        };
        tricks::Property<uint8_t> input_filter{
            [this]() -> uint8_t {
                return (CCMRx >> 4) & 0x0FU;
                },
            [this](uint8_t value) -> void {
                const unsigned shift = (order%2) * 8 + 4;
                const uint32_t mask = 0x0FU << shift;
                value &= 0x0FU;
                CCMRx = (CCMRx & ~mask) | (static_cast<uint32_t>(value) << shift);
                }
        };
        tricks::Property<ClockPrescaler> input_prescaler{
            [this]() -> ClockPrescaler {
                return static_cast<ClockPrescaler>((CCMRx >> 2) & 0x03U);
                },
            [this](ClockPrescaler value) -> void {
                const unsigned shift = (order%2) * 8 + 2;
                const uint32_t mask = 0x03U << shift;
                CCMRx = (CCMRx & ~mask) | (static_cast<uint32_t>(value) << shift);
                }
        };
        tricks::Property<EdgeTrigger> input_trigger{
            [this]() -> EdgeTrigger {
                return static_cast<EdgeTrigger>((_timer.reg.CCER >> (order*4)) & 0xAU);
            },
            [this](EdgeTrigger value) -> void {
                const unsigned shift = order * 4;
                const uint32_t mask = 0xAU << shift;
                _timer.reg.CCER = (_timer.reg.CCER & ~mask) | (static_cast<uint32_t>(value) << shift);
            }
        };
        tricks::Property<IOSelection> io_selection{  // only writable when channel is disabled
            [this]() -> IOSelection {
                return static_cast<IOSelection>((CCMRx >> ((order%2)*8) ) & 0x03U);
                },
            [this](IOSelection value) -> void {
                const unsigned shift = (order%2) * 8;
                const uint32_t mask = 0x03U << shift;
                CCMRx = (CCMRx & ~mask) | (static_cast<uint32_t>(value) << shift);
                }
        };

        Channel(T &timer, const gpio::Pin output, const gpio::Pin input, const uint8_t order)
            : _timer(timer), output_pin(output), input_pin(input), order(order),
                CapComRegister(_get_ccr(timer, order)), CCMRx(_get_ccmr(timer, order))
        {}
        virtual ~Channel() = default;
        Channel & operator=(const Channel &) = delete;

        void enable() const noexcept
        {
            _timer.reg.CCER |= 1U << (order * 4);
        }

        void disable() const noexcept
        {
            _timer.reg.CCER &= ~(1U << (order * 4));
        }

        virtual void load(const CaptureCompareConfig &config)
        {
            disable();
            io_selection = config.io_selection;
            
            if (config.io_selection == IOSelection::Output)
            {
                output_mode = config.output_mode;
                CapComRegister = config.pulse;
                inverse_polarity = config.inverse_polarity;
                output_preload_enabled = true;
                pwm_fast_mode = config.fast_mode;
            }
            else
            {
                if (config.input_filter > 15U)
                    throw std::invalid_argument("input filter must be in [0, 15]");
                input_filter = config.input_filter;
                input_trigger = config.input_trigger;
                input_prescaler = config.input_prescaler;
            }
        }

        void irq_handler() const noexcept
        try{
            const uint32_t FLAG_MASK = 2U << order;
            if (_timer.reg.SR & FLAG_MASK &&  // check if flag is set
                _timer.reg.DIER & FLAG_MASK)  // check if source is enabled
            {
                _timer.reg.SR = ~FLAG_MASK; // clear pending interrupt
                const uint32_t CAPTURE_MASK = (order % 2) ? 0x300U : 0x3U;
                if ((CCMRx & CAPTURE_MASK) && on_capture)
                {
                    on_capture();
                }
                else if (on_compare)
                {
                    on_compare();
                }
            }
        }
        catch(...)
        {
            // do nothing
        }
    };
};

inline BasicTimer Timer2(detail::TIM2_Reg, nvic::TIM2_IRQn);

}
}
}