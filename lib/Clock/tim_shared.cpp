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
static inline void enable_clock(const BasicTimer & t)
{
    const auto ptr = reinterpret_cast<TIM_TypeDef *>(&t.reg);

    if (false) {}
    #if defined(TIM1)
    else if (ptr == TIM1)
    {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
    #endif
    #if defined(TIM2)
    else if (ptr == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }
    #endif
    #if defined(TIM3)
    else if (ptr == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
    #endif
    #if defined(TIM4)
    else if (ptr == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
    }
    #endif
    #if defined(TIM5)
    else if (ptr == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
    }
    #endif
    #if defined(TIM6)
    else if (ptr == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();
    }
    #endif
    #if defined(TIM7)
    else if (ptr == TIM7)
    {
        __HAL_RCC_TIM7_CLK_ENABLE();
    }
    #endif
    #if defined(TIM8)
    else if (ptr == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
    #endif
    #if defined(TIM9)
    else if (ptr == TIM9)
    {
        __HAL_RCC_TIM9_CLK_ENABLE();
    }
    #endif
    #if defined(TIM10)
    else if (ptr == TIM10)
    {
        __HAL_RCC_TIM10_CLK_ENABLE();
    }
    #endif
    #if defined(TIM11)
    else if (ptr == TIM11)
    {
        __HAL_RCC_TIM11_CLK_ENABLE();
    }
    #endif
    #if defined(TIM12)
    else if (ptr == TIM12)
    {
        __HAL_RCC_TIM12_CLK_ENABLE();
    }
    #endif
    #if defined(TIM13)
    else if (ptr == TIM13)
    {
        __HAL_RCC_TIM13_CLK_ENABLE();
    }
    #endif
    #if defined(TIM14)
    else if (ptr == TIM14)
    {
        __HAL_RCC_TIM14_CLK_ENABLE();
    }
    #endif
    #if defined(TIM15)
    else if (ptr == TIM15)
    {
        __HAL_RCC_TIM15_CLK_ENABLE();
    }
    #endif
    #if defined(TIM16)
    else if (ptr == TIM16)
    {
        __HAL_RCC_TIM16_CLK_ENABLE();
    }
    #endif
    #if defined(TIM17)
    else if (ptr == TIM17)
    {
        __HAL_RCC_TIM17_CLK_ENABLE();
    }
    #endif
}

static inline void disable_clock(const BasicTimer & t)
{
    const auto ptr = reinterpret_cast<TIM_TypeDef *>(&t.reg);

    if (false) {}
    #if defined(TIM1)
    else if (ptr == TIM1)
    {
        __HAL_RCC_TIM1_CLK_DISABLE();
    }
    #endif
    #if defined(TIM2)
    else if (ptr == TIM2)
    {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }
    #endif
    #if defined(TIM3)
    else if (ptr == TIM3)
    {
        __HAL_RCC_TIM3_CLK_DISABLE();
    }
    #endif
    #if defined(TIM4)
    else if (ptr == TIM4)
    {
        __HAL_RCC_TIM4_CLK_DISABLE();
    }
    #endif
    #if defined(TIM5)
    else if (ptr == TIM5)
    {
        __HAL_RCC_TIM5_CLK_DISABLE();
    }
    #endif
    #if defined(TIM6)
    else if (ptr == TIM6)
    {
        __HAL_RCC_TIM6_CLK_DISABLE();
    }
    #endif
    #if defined(TIM7)
    else if (ptr == TIM7)
    {
        __HAL_RCC_TIM7_CLK_DISABLE();
    }
    #endif
    #if defined(TIM8)
    else if (ptr == TIM8)
    {
        __HAL_RCC_TIM8_CLK_DISABLE();
    }
    #endif
    #if defined(TIM9)
    else if (ptr == TIM9)
    {
        __HAL_RCC_TIM9_CLK_DISABLE();
    }
    #endif
    #if defined(TIM10)
    else if (ptr == TIM10)
    {
        __HAL_RCC_TIM10_CLK_DISABLE();
    }
    #endif
    #if defined(TIM11)
    else if (ptr == TIM11)
    {
        __HAL_RCC_TIM11_CLK_DISABLE();
    }
    #endif
    #if defined(TIM12)
    else if (ptr == TIM12)
    {
        __HAL_RCC_TIM12_CLK_DISABLE();
    }
    #endif
    #if defined(TIM13)
    else if (ptr == TIM13)
    {
        __HAL_RCC_TIM13_CLK_DISABLE();
    }
    #endif
    #if defined(TIM14)
    else if (ptr == TIM14)
    {
        __HAL_RCC_TIM14_CLK_DISABLE();
    }
    #endif
    #if defined(TIM15)
    else if
    {
        __HAL_RCC_TIM15_CLK_DISABLE();
    }
    #endif
    #if defined(TIM16)
    else if (ptr == TIM16)
    {
        __HAL_RCC_TIM16_CLK_DISABLE();
    }
    #endif
    #if defined(TIM17)
    else if (ptr == TIM17)
    {
        __HAL_RCC_TIM17_CLK_DISABLE();
    }
    #endif
}



void BasicTimer::set_time_base(const TimeBaseConfig &config) const noexcept
{
    enable_clock(*this);
    uint32_t period = std::min<uint32_t>(config.period, MAX_PERIOD);
    uint32_t prescaler = std::min<uint32_t>(config.prescaler, MAX_PRESCALER);
    uint32_t repetition = std::min<uint32_t>(config.repetition_counter, 0xFFU);
    uint32_t tmpcr1 = reg.CR1;

    tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
    tmpcr1 |= static_cast<uint32_t>(config.count_mode);

    tmpcr1 &= ~TIM_CR1_CKD;
    tmpcr1 |= static_cast<uint32_t>(config.clock_division);

    reg.CR1 = tmpcr1;

    auto_reload_preload = config.auto_reload_preload;

    set_auto_reload(period);
    set_prescaler(prescaler);
    reg.RCR = repetition;

    reg.EGR = TIM_EGR_UG;  // generate update event

    reg.SR = ~0x1U; // clear update flag
}

void BasicTimer::set_clock_source(const ClockSourceConfig &config) const noexcept
{
    enable_clock(*this);
    uint32_t tmp = reg.SMCR;
    tmp &= 0x80U;  // erase all bits except MSM

    if (config.source == ClockSource::Internal)
    {
    }
    else if (config.source == ClockSource::ExternalMode1 or
            config.source == ClockSource::ExternalMode2)
    {
        tmp |= static_cast<uint32_t>(config.prescaler) << 12;
        tmp |= static_cast<uint32_t>(config.polarity) & TIM_SMCR_ETP;
        tmp |= (static_cast<uint32_t>(config.filter) & 0xFU) << 8;

        if (config.source == ClockSource::ExternalMode1)
            tmp |= (TIM_SLAVEMODE_EXTERNAL1 | TIM_CLOCKSOURCE_ETRMODE1);
        else
            tmp |= TIM_SMCR_ECE;
    }
    else if (config.source == ClockSource::TTI1FP1 or
            config.source == ClockSource::TTI1FP1_ED)
    {
        reg.CCER = (
            (reg.CCER & ~(TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC1E))
            | (static_cast<uint32_t>(config.polarity) & 0xAU)
            );
        reg.CCMR1 &= ~TIM_CCMR1_IC1F;
        reg.CCMR1 |= (static_cast<uint32_t>(config.filter) & 0xFU) << 4;
        tmp &= ~TIM_SMCR_TS;
        tmp |= (static_cast<uint32_t>(config.source) | TIM_SLAVEMODE_EXTERNAL1);
    }
    else if (config.source == ClockSource::TTI2FP2)
    {
        reg.CCER = (
            (reg.CCER & ~(TIM_CCER_CC2P | TIM_CCER_CC2NP | TIM_CCER_CC2E))
            | ((static_cast<uint32_t>(config.polarity) & 0xAU) << 4U)
            );
        reg.CCMR1 &= ~TIM_CCMR1_IC2F;
        reg.CCMR1 |= (static_cast<uint32_t>(config.filter) & 0xFU) << 12;
        tmp &= ~TIM_SMCR_TS;
        tmp |= (static_cast<uint32_t>(config.source) | TIM_SLAVEMODE_EXTERNAL1);
    }
    else // ITRx
    {
        tmp &= ~TIM_SMCR_TS;
        tmp |= (static_cast<uint32_t>(config.source) | TIM_SLAVEMODE_EXTERNAL1);
    }

    reg.SMCR = tmp;
    extern_freq = config.extern_freq;
}

ClockSourceConfig BasicTimer::get_clock_source() const noexcept
{
    ClockSourceConfig config;
    config.extern_freq = extern_freq;

   uint32_t tmpsmcr = reg.SMCR;

   if (tmpsmcr & TIM_SMCR_ECE)
    {
        config.source = ClockSource::ExternalMode2;
        config.prescaler = static_cast<ClockPrescaler>((tmpsmcr >> 12) & 0x7U);
        config.polarity = static_cast<ClockPolarity>(tmpsmcr & TIM_SMCR_ETP);
        config.filter = static_cast<uint8_t>((tmpsmcr >> 8) & 0xFU);
    }
    else if (tmpsmcr & TIM_SMCR_TS)
    {
        switch (tmpsmcr & TIM_SMCR_TS)
        {
            case TIM_TS_ITR0:
                config.source = ClockSource::ITR0;
                break;
            case TIM_TS_ITR1:
                config.source = ClockSource::ITR1;
                break;
            case TIM_TS_ITR2:
                config.source = ClockSource::ITR2;
                break;
            case TIM_TS_ITR3:
                config.source = ClockSource::ITR3;
                break;
            case TIM_TS_TI1F_ED:
                config.source = ClockSource::TTI1FP1_ED;
                config.polarity = static_cast<ClockPolarity>(reg.CCER & 0xAU);
                config.filter = static_cast<uint8_t>((reg.CCMR1 >> 4) & 0xFU);
                break;
            case TIM_TS_TI1FP1:
                config.source = ClockSource::TTI1FP1;
                config.polarity = static_cast<ClockPolarity>(reg.CCER & 0xAU);
                config.filter = static_cast<uint8_t>((reg.CCMR1 >> 4) & 0xFU);
                break;
            case TIM_TS_TI2FP2:
                config.source = ClockSource::TTI2FP2;
                config.polarity = static_cast<ClockPolarity>(reg.CCER & 0xAU);
                config.filter = static_cast<uint8_t>((reg.CCMR1 >> 12) & 0xFU);
                break;
            case TIM_TS_ETRF:
                config.source = ClockSource::ExternalMode1;
                config.prescaler = static_cast<ClockPrescaler>((tmpsmcr >> 12) & 0x7U);
                config.polarity = static_cast<ClockPolarity>(tmpsmcr & TIM_SMCR_ETP);
                config.filter = static_cast<uint8_t>((tmpsmcr >> 8) & 0xFU);
                break;
        }
    }
    else
    {
        config.source = ClockSource::Internal;
    }

    return config;
}

void BasicTimer::set_master(const MasterConfig &config) const noexcept
{
    enable_clock(*this);

    uint32_t master_output_trigger;

    switch(config.mode)
    {
        case MasterTriggerMode::Reset:
            master_output_trigger = TIM_TRGO_RESET;
            break;
        case MasterTriggerMode::Enable:
            master_output_trigger = TIM_TRGO_ENABLE;
            break;
        case MasterTriggerMode::Update:
            master_output_trigger = TIM_TRGO_UPDATE;
            break;
        case MasterTriggerMode::ComparePulse:
            master_output_trigger = TIM_TRGO_OC1;
            break;
        case MasterTriggerMode::OutputCompare1:
            master_output_trigger = TIM_TRGO_OC1REF;
            break;
        case MasterTriggerMode::OutputCompare2:
            master_output_trigger = TIM_TRGO_OC2REF;
            break;
        case MasterTriggerMode::OutputCompare3:
            master_output_trigger = TIM_TRGO_OC3REF;
            break;
        case MasterTriggerMode::OutputCompare4:
            master_output_trigger = TIM_TRGO_OC4REF;
            break;
    }
    reg.CR2 = (reg.CR2 & ~TIM_CR2_MMS) | master_output_trigger;

    if (IS_TIM_SLAVE_INSTANCE((TIM_TypeDef *)(&reg)))
        reg.SMCR = (reg.SMCR & ~TIM_SMCR_MSM) | master_output_trigger;
}

void BasicTimer::init() const noexcept
{
    enable_clock(*this);
    set_time_base(TimeBaseConfig());
    set_clock_source(ClockSourceConfig());
    set_master(MasterConfig());
}

void BasicTimer::deinit() const noexcept
{
    disable_clock(*this);
}

void BasicTimer::enable_irq() const noexcept
{
    enable_clock(*this);
    reg.DIER |= TIM_IT_UPDATE; // enable interrupt
    reg.SR = ~TIM_IT_UPDATE; // clear pending interrupt
    nvic::enable_irq(reload_irqn);
}

void BasicTimer::disable_irq() const noexcept
{
    nvic::disable_irq(reload_irqn);
    reg.DIER &= ~TIM_IT_UPDATE; // disable interrupt
}

#ifdef TIM1_BASE
const AdvancedTimer Timer1(detail::TIM1_Reg, nvic::TIM1_UP_TIM10_IRQn,
    nvic::TIM1_BRK_TIM9_IRQn, nvic::TIM1_TRG_COM_TIM11_IRQn, nvic::TIM1_CC_IRQn);
#endif

#ifdef TIM2_BASE
const GeneralPurposeTimer Timer2(detail::TIM2_Reg, nvic::TIM2_IRQn, 0xFFFFFFFFU);
#endif

#ifdef TIM3_BASE
const GeneralPurposeTimer Timer3(detail::TIM3_Reg, nvic::TIM3_IRQn);
#endif

#ifdef TIM4_BASE
const GeneralPurposeTimer Timer4(detail::TIM4_Reg, nvic::TIM4_IRQn);
#endif

#ifdef TIM5_BASE
const GeneralPurposeTimer Timer5(detail::TIM2_Reg, nvic::TIM5_IRQn, 0xFFFFFFFFU);
#endif

#ifdef TIM6_BASE
const BasicTimer Timer6(detail::TIM6_Reg, nvic::TIM6_IRQn);
#endif

#ifdef TIM7_BASE
const BasicTimer Timer7(detail::TIM7_Reg, nvic::TIM7_IRQn);
#endif

#ifdef TIM8_BASE
const AdvancedTimer Timer8(detail::TIM8_Reg, nvic::TIM8_UP_TIM13_IRQn,
    nvic::TIM8_BRK_TIM12_IRQn, nvic::TIM8_TRG_COM_TIM14_IRQn, nvic::TIM8_CC_IRQn);
#endif

#ifdef TIM9_BASE
const GeneralPurposeTimer_2CH Timer9(detail::TIM9_Reg, nvic::TIM1_BRK_TIM9_IRQn);
#endif

#ifdef TIM10_BASE
const GeneralPurposeTimer_2CH Timer10(detail::TIM10_Reg, nvic::TIM1_UP_TIM10_IRQn);
#endif

#ifdef TIM11_BASE
const GeneralPurposeTimer_2CH Timer11(detail::TIM11_Reg, nvic::TIM1_TRG_COM_TIM11_IRQn);
#endif

#ifdef TIM12_BASE
const GeneralPurposeTimer_2CH Timer12(detail::TIM12_Reg, nvic::TIM8_BRK_TIM12_IRQn);
#endif

#ifdef TIM13_BASE
const GeneralPurposeTimer_2CH Timer13(detail::TIM13_Reg, nvic::TIM8_UP_TIM13_IRQn);
#endif

#ifdef TIM14_BASE
const GeneralPurposeTimer_2CH Timer14(detail::TIM14_Reg, nvic::TIM8_TRG_COM_TIM14_IRQn);
#endif

#ifdef TIM15_BASE
const GeneralPurposeTimer_2CH Timer15(detail::TIM15_Reg, nvic::TIM15_IRQn);
#endif

#ifdef TIM16_BASE
const GeneralPurposeTimer_2CH Timer16(detail::TIM16_Reg, nvic::TIM16_IRQn);
#endif

#ifdef TIM17_BASE
const GeneralPurposeTimer_2CH Timer17(detail::TIM17_Reg, nvic::TIM17_IRQn);
#endif

extern "C"
{
    #if defined(TIM1_BASE) || defined(TIM10_BASE)
    void TIM1_UP_TIM10_IRQHandler()
    {
        #ifdef TIM1_BASE
        clock::Timer1.on_reload_handler();
        #endif
        #ifdef TIM10_BASE
        clock::Timer10.global_irq_handler();
        #endif
    }
    #endif

    #if defined(TIM1_BASE) || defined(TIM9_BASE)
    void TIM1_BRK_TIM9_IRQHandler()
    {
        #ifdef TIM1_BASE
        clock::Timer1.break_irq_handler();
        #endif
        #ifdef TIM9_BASE
        clock::Timer9.global_irq_handler();
        #endif
    }
    #endif

    #if defined(TIM1_BASE) || defined(TIM11_BASE)
    void TIM1_TRG_COM_TIM11_IRQHandler()
    {
        #ifdef TIM1_BASE
        clock::Timer1.trigger_com_irq_handler();
        #endif
        #ifdef TIM11_BASE
        clock::Timer11.global_irq_handler();
        #endif
    }
    #endif

    #if defined(TIM1_BASE)
    void TIM1_CC_IRQHandler()
    {
        clock::Timer1.cc_irq_handler();
    }
    #endif

    #ifdef TIM2_BASE
    void TIM2_IRQHandler()
    {
        clock::Timer2.global_irq_handler();
    }
    #endif

    #ifdef TIM3_BASE
    void TIM3_IRQHandler()
    {
        clock::Timer3.global_irq_handler();
    }
    #endif

    #ifdef TIM4_BASE
    void TIM4_IRQHandler()
    {
        clock::Timer4.global_irq_handler();
    }
    #endif
    
    #ifdef TIM5_BASE
    void TIM5_IRQHandler()
    {
        clock::Timer5.global_irq_handler();
    }
    #endif

    #ifdef TIM6_BASE
    void TIM6_DAC_IRQHandler()
    {
        clock::Timer6.global_irq_handler();
    }
    #endif

    #ifdef TIM7_BASE
    void TIM7_IRQHandler()
    {
        clock::Timer7.global_irq_handler();
    }
    #endif

    #if defined(TIM8_BASE) || defined(TIM12_BASE)
    void TIM8_BRK_TIM12_IRQHandler()
    {
        #ifdef TIM8_BASE
        clock::Timer8.break_irq_handler();
        #endif
        #ifdef TIM12_BASE
        clock::Timer12.global_irq_handler();
        #endif
    }
    #endif

    #if defined(TIM8_BASE) || defined(TIM13_BASE)
    void TIM8_UP_TIM13_IRQHandler()
    {
        #ifdef TIM8_BASE
        clock::Timer8.on_reload_handler();
        #endif
        #ifdef TIM13_BASE
        clock::Timer13.global_irq_handler();
        #endif
    }
    #endif

    #if defined(TIM8_BASE) || defined(TIM14_BASE)
    void TIM8_TRG_COM_TIM14_IRQHandler()
    {
        #ifdef TIM8_BASE
        clock::Timer8.trigger_com_irq_handler();
        #endif
        #ifdef TIM14_BASE
        clock::Timer14.global_irq_handler();
        #endif
    }
    #endif

    #if defined(TIM8_BASE) || defined(TIM8_BASE)
    void TIM8_CC_IRQHandler()
    {
        clock::Timer8.cc_irq_handler();
    }
    #endif

    #ifdef TIM15_BASE
    void TIM1_BRK_TIM15_IRQHandler()
    {
        clock::Timer15.break_irq_handler();
    }
    #endif

    #ifdef TIM16_BASE
    void TIM1_UP_TIM16_IRQHandler()
    {
        clock::Timer16.on_reload_handler();
    }
    #endif

    #ifdef TIM17_BASE
    void TIM1_TRG_COM_TIM17_IRQHandler()
    {
        clock::Timer17.trigger_com_irq_handler();
    }
    #endif
}

}
}
}