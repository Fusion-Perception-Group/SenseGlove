#pragma once
#include <cstdint>
#include "userconfig.hpp"
#include "property.hpp"

namespace vermils
{
namespace stm32
{
namespace clock
{
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
    Div1,  /*!< No prescaler is used */
    Div2,  /*!< Prescaler = 2        */
    Div4,  /*!< Prescaler = 4        */
    Div8   /*!< Prescaler = 8        */
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
    Frozen,  /*!< Output compare frozen, do nothing */
    ActiveOnMatch,  /*!< Output compare active on match */
    InactiveOnMatch,  /*!< Output compare inactive on match */
    Toggle,  /*!< Output compare toggle */
    ForceInactive,  /*!< Output compare force inactive */
    ForceActive,  /*!< Output compare force active */
    PWM2,  /*!< Output compare PWM2: upper valid ->  __/\\__ */
    PWM1,  /*!< Output compare PWM1: lower valid -> ^^/^^\\^^  */
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
    ClockDivision clock_division = ClockDivision::Div1;
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

struct OCConfig
{
    OutputCompareMode mode = OutputCompareMode::Frozen;
    uint32_t pulse = 0;  // value to be loaded into the capture/compare register
    bool output_low = false;  // whether to use low as the output for capture/compare
    bool output_n_low = false;  // whether to use low as the output for capture/compare complementary
    bool idle_state = false;  // true for high, false for low
    bool idle_n_state = false;  // true for high, false for low
};

class BasicTimer
{
public:
    static const uint32_t MAX_PERIOD = 0xFFFFU;
    static const uint32_t MAX_REP_COUNTER = 0xFFU;
    detail::Register &reg;
    volatile uint32_t & counter;
    volatile uint32_t & auto_reload;
    volatile uint32_t & prescaler;

    BasicTimer(detail::Register &reg) : BasicTimer(reg, ClockSourceConfig(), TimeBaseConfig()) {}
    BasicTimer(detail::Register &reg, const ClockSourceConfig &clock_config, const TimeBaseConfig &time_base_config)
        : reg(reg), counter(reg.CNT), auto_reload(reg.ARR), prescaler(reg.PSC)
    {
        set_time_base(time_base_config);
        set_clock_source(clock_config);
    }
    virtual ~BasicTimer() = default;

    virtual void set_clock_source(const ClockSourceConfig &config);
    virtual void set_time_base(const TimeBaseConfig &config);
};

}
}
}