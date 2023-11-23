#include "tim.hpp"
#include "_config.hpp"
#include <algorithm>
#include <stdexcept>

namespace vermils
{
namespace stm32
{
namespace clock
{
/**
 * @brief enable clock and return handle
 * 
 * @param t 
 * @return TIM_HandleTypeDef& 
 */
static inline TIM_HandleTypeDef & enable_clock(const BasicTimer & t)
{
    const auto ptr = reinterpret_cast<TIM_TypeDef *>(&t.reg);

    if (false) {}
    #if defined(TIM1)
    else if (ptr == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
        static TIM_HandleTypeDef TIM1_handle={
            .Instance = TIM1
        };
        return TIM1_handle;
    }
    #endif
    #if defined(TIM2)
    else if (ptr == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        static TIM_HandleTypeDef TIM2_handle={
            .Instance = TIM2
        };
        return TIM2_handle;
    }
    #endif
    #if defined(TIM3)
    else if (ptr == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        static TIM_HandleTypeDef TIM3_handle={
            .Instance = TIM3
        };
        return TIM3_handle;
    }
    #endif
    #if defined(TIM4)
    else if (ptr == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        static TIM_HandleTypeDef TIM4_handle={
            .Instance = TIM4
        };
        return TIM4_handle;
    }
    #endif
    #if defined(TIM5)
    else if (ptr == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        static TIM_HandleTypeDef TIM5_handle={
            .Instance = TIM5
        };
        return TIM5_handle;
    }
    #endif
    #if defined(TIM6)
    else if (ptr == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
        static TIM_HandleTypeDef TIM6_handle={
            .Instance = TIM6
        };
        return TIM6_handle;
    }
    #endif
    #if defined(TIM7)
    else if (ptr == TIM7)
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
        static TIM_HandleTypeDef TIM7_handle={
            .Instance = TIM7
        };
        return TIM7_handle;
    }
    #endif
    #if defined(TIM8)
    else if (ptr == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
        static TIM_HandleTypeDef TIM8_handle={
            .Instance = TIM8
        };
        return TIM8_handle;
    }
    #endif
    #if defined(TIM9)
    else if (ptr == TIM9)
    {
        __HAL_RCC_TIM9_CLK_ENABLE();
        static TIM_HandleTypeDef TIM9_handle={
            .Instance = TIM9
        };
        return TIM9_handle;
    }
    #endif
    #if defined(TIM10)
    else if (ptr == TIM10)
    {
        __HAL_RCC_TIM10_CLK_ENABLE();
        static TIM_HandleTypeDef TIM10_handle={
            .Instance = TIM10
        };
        return TIM10_handle;
    }
    #endif
    #if defined(TIM11)
    else if (ptr == TIM11)
    {
        __HAL_RCC_TIM11_CLK_ENABLE();
        static TIM_HandleTypeDef TIM11_handle={
            .Instance = TIM11
        };
        return TIM11_handle;
    }
    #endif
    #if defined(TIM12)
    else if (ptr == TIM12)
    {
        __HAL_RCC_TIM12_CLK_ENABLE();
        static TIM_HandleTypeDef TIM12_handle={
            .Instance = TIM12
        };
        return TIM12_handle;
    }
    #endif
    #if defined(TIM13)
    else if (ptr == TIM13)
    {
        __HAL_RCC_TIM13_CLK_ENABLE();
        static TIM_HandleTypeDef TIM13_handle={
            .Instance = TIM13
        };
        return TIM13_handle;
    }
    #endif
    #if defined(TIM14)
    else if (ptr == TIM14)
    {
        __HAL_RCC_TIM14_CLK_ENABLE();
        static TIM_HandleTypeDef TIM14_handle={
            .Instance = TIM14
        };
        return TIM14_handle;
    }
    #endif
    #if defined(TIM15)
    else if (ptr == TIM15)
    {
        __HAL_RCC_TIM15_CLK_ENABLE();
        static TIM_HandleTypeDef TIM15_handle={
            .Instance = TIM15
        };
        return TIM15_handle;
    }
    #endif
    #if defined(TIM16)
    else if (ptr == TIM16)
    {
        __HAL_RCC_TIM16_CLK_ENABLE();
        static TIM_HandleTypeDef TIM16_handle={
            .Instance = TIM16
        };
        return TIM16_handle;
    }
    #endif
    #if defined(TIM17)
    else if (ptr == TIM17)
    {
        __HAL_RCC_TIM17_CLK_ENABLE();
        static TIM_HandleTypeDef TIM17_handle={
            .Instance = TIM17
        };
        return TIM17_handle;
    }
    #endif

    throw std::invalid_argument("Invalid timer");
}

static inline TIM_Base_InitTypeDef TimeBaseConfigConv(const TimeBaseConfig &config)
{
    TIM_Base_InitTypeDef init;

    switch(config.count_mode)
    {
        case CountMode::Up:
            init.CounterMode = TIM_COUNTERMODE_UP;
            break;
        case CountMode::Down:
            init.CounterMode = TIM_COUNTERMODE_DOWN;
            break;
        case CountMode::CenterAligned1:
            init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
            break;
        case CountMode::CenterAligned2:
            init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED2;
            break;
        case CountMode::CenterAligned3:
            init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
            break;    
    }

    switch(config.clock_division)
    {
        case ClockDivision::Div1:
            init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
            break;
        case ClockDivision::Div2:
            init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
            break;
        case ClockDivision::Div4:
            init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
            break;
    }

    init.Prescaler = config.prescaler;
    init.Period = config.period;
    init.RepetitionCounter = config.repetition_counter;
    init.AutoReloadPreload = config.auto_reload_preload ? TIM_AUTORELOAD_PRELOAD_ENABLE : TIM_AUTORELOAD_PRELOAD_DISABLE;

    return init;
}

static inline TIM_ClockConfigTypeDef ClockConfigConv(const ClockSourceConfig & config)
{
    TIM_ClockConfigTypeDef clock_config;

    switch(config.source)
    {
        case ClockSource::Internal:
            clock_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
            break;
        case ClockSource::ExternalMode1:
            clock_config.ClockSource = TIM_CLOCKSOURCE_ETRMODE1;
            break;
        case ClockSource::ExternalMode2:
            clock_config.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
            break;
        case ClockSource::ITR0:
            clock_config.ClockSource = TIM_CLOCKSOURCE_ITR0;
            break;
        case ClockSource::ITR1:
            clock_config.ClockSource = TIM_CLOCKSOURCE_ITR1;
            break;
        case ClockSource::ITR2:
            clock_config.ClockSource = TIM_CLOCKSOURCE_ITR2;
            break;
        case ClockSource::ITR3:
            clock_config.ClockSource = TIM_CLOCKSOURCE_ITR3;
            break;
        case ClockSource::TTI1FP1:
            clock_config.ClockSource = TIM_CLOCKSOURCE_TI1;
            break;
        case ClockSource::TTI1FP1_ED:
            clock_config.ClockSource = TIM_CLOCKSOURCE_TI1ED;
            break;
        case ClockSource::TTI2FP2:
            clock_config.ClockSource = TIM_CLOCKSOURCE_TI2;
            break;
    }

    switch(config.polarity)
    {
        case ClockPolarity::NonInverted:
            clock_config.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
            break;
        case ClockPolarity::Inverted:
            clock_config.ClockPolarity = TIM_CLOCKPOLARITY_INVERTED;
            break;
        case ClockPolarity::Rising:
            clock_config.ClockPolarity = TIM_CLOCKPOLARITY_RISING;
            break;
        case ClockPolarity::Falling:
            clock_config.ClockPolarity = TIM_CLOCKPOLARITY_FALLING;
            break;
        case ClockPolarity::BothEdge:
            clock_config.ClockPolarity = TIM_CLOCKPOLARITY_BOTHEDGE;
            break;
    }

    switch(config.prescaler)
    {
        case ClockPrescaler::Div1:
            clock_config.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
            break;
        case ClockPrescaler::Div2:
            clock_config.ClockPrescaler = TIM_CLOCKPRESCALER_DIV2;
            break;
        case ClockPrescaler::Div4:
            clock_config.ClockPrescaler = TIM_CLOCKPRESCALER_DIV4;
            break;
        case ClockPrescaler::Div8:
            clock_config.ClockPrescaler = TIM_CLOCKPRESCALER_DIV8;
            break;
    }

    clock_config.ClockFilter = config.filter;

    return clock_config;
}

void BasicTimer::set_time_base(const TimeBaseConfig &config)
{
    auto htim = enable_clock(*this);

    htim.Init = TimeBaseConfigConv(config);
    htim.Init.Period = std::min(htim.Init.Period, MAX_PERIOD);
    htim.Init.RepetitionCounter = std::min(htim.Init.RepetitionCounter, MAX_REP_COUNTER);

    HAL_TIM_Base_Init(&htim);
}

void BasicTimer::set_clock_source(const ClockSourceConfig &config)
{
    auto htim = enable_clock(*this);

    auto clock_config = ClockConfigConv(config);
    clock_config.ClockFilter = std::min<uint8_t>(config.filter, 0xFU);

    HAL_TIM_ConfigClockSource(&htim, &clock_config);
}

}
}
}