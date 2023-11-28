#pragma once
#include <cstdint>
#include <functional>
#include <chrono>
#include "userconfig.hpp"
#include "clock_shared.hpp"
#include "nvic.hpp"
#include "property.hpp"

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace tim
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
        volatile uint32_t OR;    /*!< TIM option register,                      Address offset: 0x50 */
    #if defined(__VERMIL_STM32HX)
        volatile uint32_t CCMR3; /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
        volatile uint32_t CCR5;  /*!< TIM capture/compare register5,            Address offset: 0x58 */
        volatile uint32_t CCR6;  /*!< TIM capture/compare register6,            Address offset: 0x5C */
        volatile uint32_t AF1;   /*!< TIM alternate function option register 1, Address offset: 0x60 */
        volatile uint32_t AF2;   /*!< TIM alternate function option register 2, Address offset: 0x64 */
        volatile uint32_t TISEL; /*!< TIM Input Selection register,             Address offset: 0x68 */
    #endif
    };

    extern Register &TIM1_Reg;
    extern Register &TIM2_Reg;
    extern Register &TIM3_Reg;
    extern Register &TIM4_Reg;
    extern Register &TIM5_Reg;
    extern Register &TIM6_Reg;
    extern Register &TIM7_Reg;
    extern Register &TIM8_Reg;
    extern Register &TIM9_Reg;
    extern Register &TIM10_Reg;
    extern Register &TIM11_Reg;
    extern Register &TIM12_Reg;
    extern Register &TIM13_Reg;
    extern Register &TIM14_Reg;
    extern Register &TIM15_Reg;
    extern Register &TIM16_Reg;
    extern Register &TIM17_Reg;

    inline constexpr uint16_t int_sqrt32(uint32_t x)
    {
        uint16_t res=0;
        uint16_t add= 0x8000;   
        int i;
        for(i=0;i<16;i++)
        {
            uint16_t temp=res | add;
            uint32_t g2=temp*temp;      
            if (x>=g2)
            {
                res=temp;           
            }
            add>>=1;
        }
        return res;
    }

    inline constexpr uint32_t int_sqrt64(uint64_t x)
    {
        uint32_t res=0;
        uint32_t add= 0x80000000;   
        int i;
        for(i=0;i<32;i++)
        {
            uint32_t temp=res | add;
            uint64_t g2=temp*temp;      
            if (x>=g2)
            {
                res=temp;           
            }
            add>>=1;
        }
        return res;
    }
}

enum class ClockSource : uint32_t
{
    ExternalMode2 = 0x2000U,  /*!< External clock source mode 2                          */
    Internal = 0x1000U,       /*!< Internal clock source                                 */
    ITR0 = 0x0000U,           /*!< External clock source mode 1 (ITR0)                   */
    ITR1 = 0x0010U,           /*!< External clock source mode 1 (ITR1)                   */
    ITR2 = 0x0020U,           /*!< External clock source mode 1 (ITR2)                   */
    ITR3 = 0x0030U,           /*!< External clock source mode 1 (ITR3)                   */
    TTI1FP1_ED = 0x0040U,     /*!< External clock source mode 1 (TTI1FP1 + edge detect.) */
    TTI1FP1 = 0x0050U,        /*!< External clock source mode 1 (TTI1FP1)                */
    TTI2FP2 = 0x0060U,        /*!< External clock source mode 1 (TTI2FP2)                */
    ExternalMode1 = 0x0070U,   /*!< External clock source mode 1 (ETRF)                   */
    None = 0xFFFFU
};

enum class ClockPolarity : uint32_t
{
    Inverted = 0x8000U,     /*!< Polarity for ETRx clock sources */
    NonInverted = 0x0000U,  /*!< Polarity for ETRx clock sources */
    Rising = 0x0000U,       /*!< Polarity for TIx clock sources  */
    Falling = 0x0002U,      /*!< Polarity for TIx clock sources  */
    BothEdge = 0x000AU      /*!< Polarity for TIx clock sources  */
};

enum class ClockPrescaler : uint32_t
{
    Div1 = 0x0U,  /*!< No prescaler is used */
    Div2 = 0x1U,  /*!< Prescaler = 2        */
    Div4 = 0x2U,  /*!< Prescaler = 4        */
    Div8 = 0x3U   /*!< Prescaler = 8        */
};

enum class CountMode : uint32_t
{
    Up = 0x00U,      /*!< Counter used as up-counter   */
    Down = 0x10U,    /*!< Counter used as down-counter */
    CenterAligned1 = 0x20U,  /*!< Center-aligned mode 1, OC interrupt flag is set when
                          counter is counting down */
    CenterAligned2 = 0x40U,  /*!< Center-aligned mode 2, OC interrupt flag is set when
                          counter is counting up */
    CenterAligned3 = 0x60U   /*!< Center-aligned mode 3, OC interrupt flag is set when
                                    counter is counting up or down */
};

enum class ClockDivision : uint32_t  // used for input sampling clock
{
    Div1 = 0x000U,  /*!< No division */
    Div2 = 0x100U,  /*!< Division by 2 */
    Div4 = 0x200U   /*!< Division by 4 */
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
    uint32_t extern_freq = 0U;  // external clock frequency
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
    bool inverse_n_polarity = false;  // whether to use low as the active output for capture/compare complementary
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
protected:
    template <typename T>
    struct _Property : tricks::StaticProperty<T, BasicTimer &>
    {
        using tricks::StaticProperty<T, BasicTimer &>::StaticProperty;
        using tricks::StaticProperty<T, BasicTimer &>::operator=;
    };
    struct _AutoReloadPreload : public _Property<bool>
    {
        using _Property<bool>::_Property;
        using _Property<bool>::operator=;
        bool getter() const override { return owner.reg.CR1 & 0x80U; }
        void setter(const bool value) const override {
            owner.reg.CR1 = value ? (owner.reg.CR1|0x80U) : (owner.reg.CR1&~0x80U);
        }
    };
    struct _Enabled : public _Property<bool>
    {
        using _Property<bool>::_Property;
        using _Property<bool>::operator=;
        bool getter() const override { return owner.reg.CR1 & 0x01U; }
        void setter(const bool value) const override {
            if (value) owner.start(); else owner.stop();
        }
    };
    struct _Direction : public _Property<bool>
    {
        using _Property<bool>::_Property;
        using _Property<bool>::operator=;
        bool getter() const override { return owner.reg.CR1 & 0x10U; }
        void setter(const bool value) const override {
            owner.reg.CR1 = value ? (owner.reg.CR1|0x10U) : (owner.reg.CR1&~0x10U);
        }
    };
    struct _OnePulse : public _Property<bool>
    {
        using _Property<bool>::_Property;
        using _Property<bool>::operator=;
        bool getter() const override { return owner.reg.CR1 & 0x08U; }
        void setter(const bool value) const override {
            owner.reg.CR1 = value ? (owner.reg.CR1|0x08U) : (owner.reg.CR1&~0x08U);
        }
    };
public:
    using ClockSource = clock::tim::ClockSource;
    using ClockPolarity = clock::tim::ClockPolarity;
    using ClockPrescaler = clock::tim::ClockPrescaler;
    using CountMode = clock::tim::CountMode;
    using ClockDivision = clock::tim::ClockDivision;
    using MasterTriggerMode = clock::tim::MasterTriggerMode;
    using OutputCompareMode = clock::tim::OutputCompareMode;
    using EdgeTrigger = clock::tim::EdgeTrigger;
    using IOSelection = clock::tim::IOSelection;
    using CallbackType = clock::tim::CallbackType;
    using TimeBaseConfig = clock::tim::TimeBaseConfig;
    using ClockSourceConfig = clock::tim::ClockSourceConfig;
    using MasterConfig = clock::tim::MasterConfig;
    using CaptureCompareConfig = clock::tim::CaptureCompareConfig;
    using Register = clock::tim::detail::Register;

    detail::Register &reg;
    mutable uint32_t extern_freq = 0U;  // external clock frequency, user has the responsibility to make sure it is correct, 0 for unknown
    volatile uint32_t & counter;
    volatile uint32_t & auto_reload;
    volatile uint32_t & prescaler;
    const nvic::IRQn_Type reload_irqn;
    const uint32_t MAX_PERIOD;
    const uint32_t MAX_PRESCALER;
    const uint32_t MAX_REP_COUNTER;
    _AutoReloadPreload auto_reload_preload{*this};
    _Enabled enabled{*this};
    _Direction direction{*this};
    _OnePulse one_pulse{*this};
    mutable CallbackType on_reload{};  /* on overflow and underflow */
    mutable CallbackType on_trigger{};  /* on trigger event from ITRx, TI1...etc */

    BasicTimer(detail::Register &reg, nvic::IRQn_Type r_irqn,
        uint32_t max_peroid=0xFFFFU, uint32_t max_prescaler=0xFFFFU, uint32_t max_repetition=0xFFU)
        : reg(reg), counter(reg.CNT), auto_reload(reg.ARR), prescaler(reg.PSC), reload_irqn(r_irqn),
          MAX_PERIOD(max_peroid), MAX_PRESCALER(max_prescaler), MAX_REP_COUNTER(max_repetition)
    {}
    BasicTimer & operator=(const BasicTimer &) = delete;
    virtual ~BasicTimer() = default;

    void set_time_base(const TimeBaseConfig &config) const noexcept;
    void set_clock_source(const ClockSourceConfig &config) const noexcept;
    void set_master(const MasterConfig &config) const noexcept;
    ClockSourceConfig get_clock_source() const noexcept;

    void init() const noexcept;
    void deinit() const noexcept;

    void start() const noexcept
    {
        reg.CR1 |= 0x01U;
    }
    void stop() const noexcept
    {
        reg.CR1 &= ~0x01U;
    }

    /**
     * @brief Set the counter
     * 
     * @param value 
     * @throw std::invalid_argument if value exceeds maximum period
     */
    void set_counter(uint32_t value) const
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
    void set_auto_reload(uint32_t value) const
    {
        if (value > MAX_PERIOD)
            throw std::invalid_argument("value exceeds maximum period");
        auto_reload = value;
    }

    void set_prescaler(uint16_t value) const noexcept
    {
        prescaler = value;
    }

    /**
     * @brief Get the base clock, returns SystemCoreClock if internal clock is used, otherwise returns extern_freq
     * 
     * @return uint32_t 
     */
    uint32_t get_base_clock() const noexcept
    {
        return get_base_clock(get_clock_source());
    }

    uint32_t get_base_clock(const ClockSourceConfig &cfg) const noexcept
    {
        if (cfg.source == ClockSource::Internal)
            return SystemCoreClock;
        return extern_freq;
    }

    uint32_t get_base_ticks_per_sec() const noexcept
    {
        return get_base_ticks_per_sec(get_clock_source());
    }

    uint32_t get_base_ticks_per_sec(const ClockSourceConfig &cfg) const noexcept
    {
        switch (cfg.source)
        {
            case ClockSource::Internal:
                return SystemCoreClock;
            case ClockSource::ExternalMode1:
            case ClockSource::ExternalMode2:
                return cfg.extern_freq >> static_cast<uint32_t>(cfg.prescaler);
            default:
                return cfg.extern_freq;
        }
    }

    void set_period_time(const std::chrono::nanoseconds period) const
    {
        const uint64_t ticks_ps = get_base_ticks_per_sec();
        const uint64_t ticks = static_cast<uint64_t>(period.count()) * ticks_ps / 1000000000UL;
        set_period_ticks(ticks);
    }

    void set_period_ticks(const uint64_t ticks) const
    {
        if (ticks > static_cast<uint64_t>(MAX_PERIOD)+1)
        {
            uint64_t pre_ticks = ticks / (static_cast<uint64_t>(MAX_PERIOD)+1);
            if (pre_ticks > static_cast<uint64_t>(MAX_PRESCALER)+1)
                throw std::invalid_argument("ticks too large");
            set_prescaler(pre_ticks ? (pre_ticks-1) : 0);
            set_auto_reload(MAX_PERIOD);
        }
        else if (ticks)
        {
            set_prescaler(0);
            set_auto_reload(ticks-1);
        }
        else
        {
            throw std::invalid_argument("ticks too small");
        }
    }

    void set_frequency(const uint32_t frequency) const
    {
        if (frequency == 0)
            throw std::invalid_argument("frequency cannot be zero");
        const uint64_t ticks_ps = get_base_ticks_per_sec();
        const uint64_t ticks = ticks_ps / frequency;
        set_period_ticks(ticks);
    }

    void enable_irq() const noexcept;

    void set_irq_priority(const uint8_t priority) const
    {
        nvic::set_priority(reload_irqn, priority);
    }

    void disable_irq() const noexcept;

    void on_reload_handler() const noexcept
    try{
        const uint32_t UPDATE_MASK = 0x01U;
        if (reg.SR & UPDATE_MASK)  // check if flag is set
        {
            reg.SR = ~UPDATE_MASK; // clear pending interrupt
            if (reg.DIER & UPDATE_MASK &&  // check if source is enabled
                on_reload)  // check if callback is set
            {
                on_reload();
            }
        }
    }
    catch(...)
    {
        // do nothing
    }

    void on_trigger_handler() const noexcept
    try{
        const uint32_t FLAG_MASK = 0x40U;
        if (reg.SR & FLAG_MASK)  // check if flag is set
        {
            reg.SR = ~FLAG_MASK; // clear pending interrupt
            if (reg.DIER & FLAG_MASK &&  // check if source is enabled
                on_trigger)  // check if callback is set
            {
                on_trigger();
            }
        }
    }
    catch(...)
    {
    }

    void global_irq_handler() const noexcept
    {
        on_reload_handler();
        on_trigger_handler();
    }

    bool operator==(const BasicTimer &rhs) const
    {
        return &reg == &rhs.reg;
    }
};

class BaseGeneralPurposeTimer : public BasicTimer
{
public:
    using BasicTimer::BasicTimer;
    class Channel
    {
    protected:
        using T = BaseGeneralPurposeTimer;
        T &_timer;
        template <typename T>
        struct _Property : tricks::StaticProperty<T, Channel &>
        {
            using tricks::StaticProperty<T, Channel &>::StaticProperty;
            using tricks::StaticProperty<T, Channel &>::operator=;
        };
        struct _Enabled : public _Property<bool>
        {
            using _Property<bool>::_Property;
            using _Property<bool>::operator=;
            bool getter() const override { return owner._timer.reg.CCER & (1U << (owner.order * 4)); }
            void setter(const bool value) const override {
                if (value) owner.enable(); else owner.disable();
            }
        };
        struct _OutputPreloadEnabled : public _Property<bool>
        {
            using _Property<bool>::_Property;
            using _Property<bool>::operator=;
            bool getter() const override { return owner.CCMRx & (1U << ((owner.order%2) * 8 + 3)); }
            void setter(const bool value) const override {
                const uint32_t mask = 1U << ((owner.order%2) * 8 + 3);
                owner.CCMRx = (owner.CCMRx & ~mask) | (value ? mask : 0);
            }
        };
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
        const uint8_t order;
        volatile uint32_t &CapComRegister; // not that Capcom, no games here
        volatile uint32_t &CCMRx;
        mutable CallbackType on_capture{};
        mutable CallbackType on_compare{};
        _Enabled enabled{*this};
        _OutputPreloadEnabled output_preload_enabled{*this};

        Channel(T &timer, const uint8_t order) : _timer(timer), order(order),
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

        void set_inverse_polarity(const bool value) const noexcept
        {
            const uint32_t mask = 0x2U << (order * 4);
            _timer.reg.CCER = (_timer.reg.CCER & ~mask) | (value ? mask : 0);
        }

        bool get_inverse_polarity() const noexcept
        {
            return _timer.reg.CCER & (0x2U << (order * 4));
        }

        void set_fast_mode(const bool value) const noexcept
        {
            const uint32_t mask = 1U << ((order%2) * 8 + 2);
            CCMRx = (CCMRx & ~mask) | (value ? mask : 0);
        }

        bool get_fast_mode() const noexcept
        {
            return CCMRx & (1U << ((order%2) * 8 + 2));
        }

        void set_output_mode(const OutputCompareMode value) const noexcept
        {
            const uint32_t shift = (order%2) * 8;
            const uint32_t mask = 0x07U << shift;
            CCMRx = (CCMRx & ~mask) | (static_cast<uint32_t>(value) << shift);
        }

        OutputCompareMode get_output_mode() const noexcept
        {
            return static_cast<OutputCompareMode>((CCMRx >> ((order%2)*8)) & 0x07U);
        }

        void set_input_filter(const uint8_t value) const noexcept
        {
            const unsigned shift = (order%2) * 8 + 4;
            const uint32_t mask = 0x0FU << shift;
            CCMRx = (CCMRx & ~mask) | (static_cast<uint32_t>(value) << shift);
        }

        uint8_t get_input_filter() const noexcept
        {
            return (CCMRx >> ((order%2)*8+4)) & 0x0FU;
        }

        void set_input_prescaler(const ClockPrescaler value) const noexcept
        {
            const unsigned shift = (order%2) * 8 + 2;
            const uint32_t mask = 0x03U << shift;
            CCMRx = (CCMRx & ~mask) | (static_cast<uint32_t>(value) << shift);
        }

        ClockPrescaler get_input_prescaler() const noexcept
        {
            return static_cast<ClockPrescaler>((CCMRx >> ((order%2)*8+2)) & 0x03U);
        }

        void set_input_trigger(const EdgeTrigger value) const noexcept
        {
            const unsigned shift = order * 4;
            const uint32_t mask = 0xAU << shift;
            _timer.reg.CCER = (_timer.reg.CCER & ~mask) | (static_cast<uint32_t>(value) << shift);
        }

        EdgeTrigger get_input_trigger() const noexcept
        {
            return static_cast<EdgeTrigger>((_timer.reg.CCER >> (order*4)) & 0xAU);
        }

        void set_io_selection(const IOSelection value) const noexcept
        {
            const unsigned shift = (order%2) * 8;
            const uint32_t mask = 0x03U << shift;
            CCMRx = (CCMRx & ~mask) | (static_cast<uint32_t>(value) << shift);
        }

        IOSelection get_io_selection() const noexcept
        {
            return static_cast<IOSelection>((CCMRx >> ((order%2)*8)) & 0x03U);
        }

        /**
         * @brief 
         * 
         * @param config 
         * @throw std::invalid_argument if order is not in [0, 3]
         */
        virtual void load(const CaptureCompareConfig &config)
        {
            disable();
            set_io_selection(config.io_selection);
            
            if (config.io_selection == IOSelection::Output)
            {
                set_output_mode(config.output_mode);
                CapComRegister = config.pulse;
                set_inverse_polarity(config.inverse_polarity);
                output_preload_enabled = true;
                set_fast_mode(config.fast_mode);
            }
            else
            {
                if (config.input_filter > 15U)
                    throw std::invalid_argument("input filter must be in [0, 15]");
                set_input_filter(config.input_filter);
                set_input_trigger(config.input_trigger);
                set_input_prescaler(config.input_prescaler);
            }
        }

        void irq_handler() const noexcept
        try{
            const uint32_t FLAG_MASK = 2U << order;
            if (_timer.reg.SR & FLAG_MASK)  // check if flag is set
            {
                _timer.reg.SR = ~FLAG_MASK; // clear pending interrupt
                if (_timer.reg.DIER & FLAG_MASK)  // check if source is enabled
                {
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
        }
        catch(...)
        {
            // do nothing
        }
    };

    /**
     * @brief Channel with breaking and dead time function
     * 
     */
    class Channel_Break : public Channel
    {
    public:
        enum class LockLevel : uint8_t
        {
            Off = 0U,  /*!< Locking disabled   */
            Level1 = 1U,  /*!< Locking level 1    */
            Level2 = 2U,  /*!< Locking level 2    */
            Level3 = 3U   /*!< Locking level 3    */
        };
    protected:
        template <typename T>
        struct _Property : tricks::StaticProperty<T, Channel_Break &>
        {
            using tricks::StaticProperty<T, Channel_Break &>::StaticProperty;
            using tricks::StaticProperty<T, Channel_Break &>::operator=;
        };
        struct _MainOutputEnabled : public _Property<bool>
        {
            using _Property<bool>::_Property;
            using _Property<bool>::operator=;
            bool getter() const override { return owner._timer.reg.BDTR & 0x8000U; }
            void setter(const bool value) const override {
                owner._timer.reg.BDTR = value ? (owner._timer.reg.BDTR|0x8000U) : (owner._timer.reg.BDTR&~0x8000U);
            }
        };
        struct _AutomaticOutputEnabled : public _Property<bool>
        {
            using _Property<bool>::_Property;
            using _Property<bool>::operator=;
            bool getter() const override { return owner._timer.reg.BDTR & 0x4000U; }
            void setter(const bool value) const override {
                owner._timer.reg.BDTR = value ? (owner._timer.reg.BDTR|0x4000U) : (owner._timer.reg.BDTR&~0x4000U);
            }
        };
        struct _BreakPolarity : public _Property<bool>
        {
            using _Property<bool>::_Property;
            using _Property<bool>::operator=;
            bool getter() const override { return owner._timer.reg.BDTR & 0x2000U; }
            void setter(const bool value) const override {
                owner._timer.reg.BDTR = value ? (owner._timer.reg.BDTR|0x2000U) : (owner._timer.reg.BDTR&~0x2000U);
            }
        };
        struct _BreakEnabled : public _Property<bool>
        {
            using _Property<bool>::_Property;
            using _Property<bool>::operator=;
            bool getter() const override { return owner._timer.reg.BDTR & 0x1000U; }
            void setter(const bool value) const override {
                owner._timer.reg.BDTR = value ? (owner._timer.reg.BDTR|0x1000U) : (owner._timer.reg.BDTR&~0x1000U);
            }
        };
    public:
        Channel_Break(T &timer, const uint8_t order) : Channel(timer, order) {}

        _MainOutputEnabled main_output_enabled{*this};
        _AutomaticOutputEnabled automatic_output_enabled{*this};  // whether to enable main output in the next update event
        _BreakPolarity break_polarity{*this};
        _BreakEnabled break_enabled{*this};

        void set_lock_level(const LockLevel value) const noexcept
        {
            _timer.reg.BDTR = (_timer.reg.BDTR & ~0x300U) | (static_cast<uint32_t>(value) << 8);
        }

        LockLevel get_lock_level() const noexcept
        {
            return static_cast<LockLevel>((_timer.reg.BDTR >> 8) & 0x3U);
        }

        void set_idle_state(const bool value) const noexcept
        {
            const uint32_t mask = 0x100U << (order*2);
            _timer.reg.CR2 = (_timer.reg.CR2 & ~mask) | (value ? mask : 0);
        }

        bool get_idle_state() const noexcept
        {
            return _timer.reg.CR2 & (0x100U << (order*2));
        }

        /**
         * @brief Set the dead time to the closest possible value
         * 
         * @param value 
         */
        void set_dead_time(const uint32_t value) const noexcept
        {
            if (value > 1008U)
                _timer.reg.BDTR |= 0xFFU;
            else if (value > 504)
                _timer.reg.BDTR |= 0xE0U | (value / 16 - 32);
            else if (value > 254)
                _timer.reg.BDTR |= 0xC0U | (value / 8 - 32);
            else if (value > 127)
                _timer.reg.BDTR |= 0x80U | (value / 2 - 64);
            else
                _timer.reg.BDTR |= value;
        }

        /**
         * @brief return clock source period count of DT
         * 
         * @return uint32_t 
         */
        uint32_t get_dead_time() const noexcept
        {
            uint8_t dtg = _timer.reg.BDTR & 0xFFU;
            if (dtg & 0x80U)
            {
                if (dtg & 0x40U)
                {
                    if (dtg & 0x20U)
                        return (32 + (dtg & 0x1FU)) * 16;
                    else
                        return (32 + (dtg & 0x1FU)) * 8;
                }
                else
                {
                    return (64 + (dtg & 0x3FU)) * 2;
                }
            }
            else
            {
                return dtg;
            }
        }

        void load(const CaptureCompareConfig &config) override
        {
            Channel::load(config);
            set_idle_state(config.idle_state);
        }
    };

    /**
     * @brief Channel with breaking function and complementary output
     * 
     */
    class Channel_N_Break : public Channel_Break
    {
    protected:
        template <typename T>
        struct _Property : tricks::StaticProperty<T, Channel_N_Break &>
        {
            using tricks::StaticProperty<T, Channel_N_Break &>::StaticProperty;
            using tricks::StaticProperty<T, Channel_N_Break &>::operator=;
        };
    public:
        Channel_N_Break(T &timer, const uint8_t order) : Channel_Break(timer, order) {}

        void set_inverse_n_polarity(const bool value) const noexcept
        {
            const uint32_t mask = 0x8U << (order * 4);
            _timer.reg.CCER = (_timer.reg.CCER & ~mask) | (value ? mask : 0);
        }

        bool get_inverse_n_polarity() const noexcept
        {
            return _timer.reg.CCER & (0x8U << (order * 4));
        }

        void set_idle_n_state(const bool value) const noexcept
        {
            const uint32_t mask = 0x200U << (order*2);
            _timer.reg.CR2 = (_timer.reg.CR2 & ~mask) | (value ? mask : 0);
        }

        bool get_idle_n_state() const noexcept
        {
            return _timer.reg.CR2 & (0x200U << (order*2));
        }

        void load(const CaptureCompareConfig &config) override
        {
            Channel_Break::load(config);
            set_idle_n_state(config.idle_n_state);
            set_inverse_n_polarity(config.inverse_n_polarity);
        }
    };
};

class GeneralPurposeTimer_2CH : public BaseGeneralPurposeTimer
{
public:
    //using BaseGeneralPurposeTimer::BaseGeneralPurposeTimer;
    GeneralPurposeTimer_2CH(detail::Register &reg, nvic::IRQn_Type r_irqn,
        uint32_t max_peroid=0xFFFFU, uint32_t max_prescaler=0xFFFFU, uint32_t max_repetition=0xFFU)
        : BaseGeneralPurposeTimer(reg, r_irqn, max_peroid, max_prescaler, max_repetition)
    {}
    Channel channel1{*this, 0};
    Channel channel2{*this, 1};

    void global_irq_handler() const noexcept
    {
        BaseGeneralPurposeTimer::global_irq_handler();
        channel1.irq_handler();
        channel2.irq_handler();
    }
};

class GeneralPurposeTimer : public GeneralPurposeTimer_2CH
{
public:
    //using GeneralPurposeTimer_2CH::GeneralPurposeTimer_2CH;
    GeneralPurposeTimer(detail::Register &reg, nvic::IRQn_Type r_irqn,
        uint32_t max_peroid=0xFFFFU, uint32_t max_prescaler=0xFFFFU, uint32_t max_repetition=0xFFU)
        : GeneralPurposeTimer_2CH(reg, r_irqn, max_peroid, max_prescaler, max_repetition)
    {}
    Channel channel3{*this, 2};
    Channel channel4{*this, 3};

    void global_irq_handler() const noexcept
    {
        GeneralPurposeTimer_2CH::global_irq_handler();
        channel3.irq_handler();
        channel4.irq_handler();
    }
};

class AdvancedTimer : public BaseGeneralPurposeTimer
{
public:
    Channel_N_Break channel1{*this, 0};
    Channel_N_Break channel2{*this, 1};
    Channel_N_Break channel3{*this, 2};
    Channel_Break channel4{*this, 3};
    mutable CallbackType on_break{};
    mutable CallbackType on_communication{};
    const nvic::IRQn_Type break_irqn;
    const nvic::IRQn_Type trigger_com_irqn;  // trigger/comunication interrupt
    const nvic::IRQn_Type capcom_irqn;  // capture/compare interrupt

    AdvancedTimer(detail::Register &reg, nvic::IRQn_Type r_irqn,
        nvic::IRQn_Type brk_iqn, nvic::IRQn_Type tr_com_iqn, nvic::IRQn_Type cc_iqn,
        uint32_t max_peroid=0xFFFFU, uint32_t max_prescaler=0xFFFFU, uint32_t max_repetition=0xFFU)
        : BaseGeneralPurposeTimer(reg, r_irqn, max_peroid, max_prescaler, max_repetition),
            break_irqn(brk_iqn), trigger_com_irqn(tr_com_iqn), capcom_irqn(cc_iqn)
        {}
    
    /**
     * @brief Set the repetition counter
     * 
     * @param value 
     * @throw std::invalid_argument if value exceeds maximum repetition counter
     */
    void set_repetition(const uint32_t value) const
    {
        if (value > MAX_REP_COUNTER)
            throw std::invalid_argument("value exceeds maximum repetition counter");
        reg.RCR = value;
    }
    
    void enable_break_irq() const noexcept
    {
        nvic::enable_irq(break_irqn);
    }

    void disable_break_irq() const noexcept
    {
        nvic::disable_irq(break_irqn);
    }

    void enable_trigger_com_irq() const noexcept
    {
        nvic::enable_irq(trigger_com_irqn);
    }

    void disable_trigger_com_irq() const noexcept
    {
        nvic::disable_irq(trigger_com_irqn);
    }

    void enable_capcom_irq() const noexcept
    {
        nvic::enable_irq(capcom_irqn);
    }

    void disable_capcom_irq() const noexcept
    {
        nvic::disable_irq(capcom_irqn);
    }

    void cc_irq_handler() const noexcept
    {
        channel1.irq_handler();
        channel2.irq_handler();
        channel3.irq_handler();
        channel4.irq_handler();
    }

    void break_irq_handler() const noexcept
    try{
        const uint32_t FLAG_MASK = 0x80U;
        if (reg.SR & FLAG_MASK)  // check if flag is set
        {
            reg.SR = ~FLAG_MASK; // clear pending interrupt
            if (reg.DIER & FLAG_MASK &&  // check if source is enabled
                on_break)  // check if callback is set
            {
                on_break();
            }
        }
    }
    catch(...)
    {
    }

    void trigger_com_irq_handler() const noexcept
    try{
        const uint32_t FLAG_MASK = 0x20U;
        if (reg.SR & FLAG_MASK)  // check if flag is set
        {
            reg.SR = ~FLAG_MASK; // clear pending interrupt
            if (reg.DIER & FLAG_MASK)  // check if source is enabled
            {
                if (on_communication)  // check if callback is set
                {
                    on_communication();
                }
                if (on_trigger)  // check if callback is set
                {
                    on_trigger();
                }
            }
        }
    }
    catch(...)
    {
    }

    void global_irq_handler() const noexcept
    {
        on_reload_handler();
        cc_irq_handler();
        break_irq_handler();
        trigger_com_irq_handler();
    }
};

extern const AdvancedTimer Tim1;
extern const GeneralPurposeTimer Tim2;
extern const GeneralPurposeTimer Tim3;
extern const GeneralPurposeTimer Tim4;
extern const GeneralPurposeTimer Tim5;
extern const BasicTimer Tim6;
extern const BasicTimer Tim7;
extern const AdvancedTimer Tim8;
extern const GeneralPurposeTimer_2CH Tim9;
extern const GeneralPurposeTimer_2CH Tim10;
extern const GeneralPurposeTimer_2CH Tim11;
extern const GeneralPurposeTimer_2CH Tim12;
extern const GeneralPurposeTimer_2CH Tim13;
extern const GeneralPurposeTimer_2CH Tim14;
extern const GeneralPurposeTimer_2CH Tim15;
extern const GeneralPurposeTimer_2CH Tim16;
extern const GeneralPurposeTimer_2CH Tim17;

}

using tim::Tim1;
using tim::Tim2;
using tim::Tim3;
using tim::Tim4;
using tim::Tim5;
using tim::Tim6;
using tim::Tim7;
using tim::Tim8;
using tim::Tim9;
using tim::Tim10;
using tim::Tim11;
using tim::Tim12;
using tim::Tim13;
using tim::Tim14;
using tim::Tim15;
using tim::Tim16;
using tim::Tim17;

}
}
}