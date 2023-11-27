#include "_config.hpp"
#include "block_future.hpp"  // resolve missing file dut to pio circular dependency
#include "rcc.hpp"
#include "gpio.hpp"
#include "adc.hpp"
#include "dma.hpp"
#include "tim.hpp"

#define weak __attribute__((weak))

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace rcc
{

weak [[noreturn]] void reset_system() noexcept
{
    NVIC_SystemReset();
}

weak void reset_backup_domain() noexcept
{
    #ifdef __HAL_RCC_BACKUPRESET_FORCE
    __HAL_RCC_BACKUPRESET_FORCE();
    __HAL_RCC_BACKUPRESET_RELEASE();
    #endif
}

weak void enable_clock(const gpio::hidden::_Port& port) noexcept
{
    port.enable_clock();
}
weak void enable_clock(const adc::RegularADC& adc) noexcept
{
    switch (adc.order)
    {
    case 0:
        #ifdef __HAL_RCC_ADC1_CLK_ENABLE
        __HAL_RCC_ADC1_CLK_ENABLE();
        #endif
        break;
    case 1:
        #ifdef __HAL_RCC_ADC2_CLK_ENABLE
        __HAL_RCC_ADC2_CLK_ENABLE();
        #endif
        break;
    case 2:
        #ifdef __HAL_RCC_ADC3_CLK_ENABLE
        __HAL_RCC_ADC3_CLK_ENABLE();
        #endif
        break;
    }
}
weak void enable_clock(const dma::BaseDMA& dma) noexcept
{
    switch (dma.order)
    {
    case 0:
        #ifdef __HAL_RCC_DMA1_CLK_ENABLE
        __HAL_RCC_DMA1_CLK_ENABLE();
        #endif
        break;
    case 1:
        #ifdef __HAL_RCC_DMA2_CLK_ENABLE
        __HAL_RCC_DMA2_CLK_ENABLE();
        #endif
        break;
    }
}
weak void enable_clock(const tim::BasicTimer& tim) noexcept
{
    const auto ptr = reinterpret_cast<TIM_TypeDef *>(&tim.reg);

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
weak void disable_clock(const gpio::hidden::_Port& port) noexcept
{
    port.disable_clock();
}
weak void disable_clock(const adc::RegularADC& adc) noexcept
{
    switch (adc.order)
    {
    case 0:
        #ifdef __HAL_RCC_ADC1_CLK_DISABLE
        __HAL_RCC_ADC1_CLK_DISABLE();
        #endif
        break;
    case 1:
        #ifdef __HAL_RCC_ADC2_CLK_DISABLE
        __HAL_RCC_ADC2_CLK_DISABLE();
        #endif
        break;
    case 2:
        #ifdef __HAL_RCC_ADC3_CLK_DISABLE
        __HAL_RCC_ADC3_CLK_DISABLE();
        #endif
        break;
    }
}
weak void disable_clock(const dma::BaseDMA& dma) noexcept
{
    switch (dma.order)
    {
    case 0:
        #ifdef __HAL_RCC_DMA1_CLK_DISABLE
        __HAL_RCC_DMA1_CLK_DISABLE();
        #endif
        break;
    case 1:
        #ifdef __HAL_RCC_DMA2_CLK_DISABLE
        __HAL_RCC_DMA2_CLK_DISABLE();
        #endif
        break;
    }
}
weak void disable_clock(const tim::BasicTimer& tim) noexcept
{
    const auto ptr = reinterpret_cast<TIM_TypeDef *>(&tim.reg);

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

void reset(const gpio::hidden::_Port& port) noexcept
{
    switch(reinterpret_cast<uintptr_t>(&port))
    {
        #ifdef GPIOA_BASE
        case GPIOA_BASE:
            __HAL_RCC_GPIOA_FORCE_RESET();
            __HAL_RCC_GPIOA_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOB_BASE
        case GPIOB_BASE:
            __HAL_RCC_GPIOB_FORCE_RESET();
            __HAL_RCC_GPIOB_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOC_BASE
        case GPIOC_BASE:
            __HAL_RCC_GPIOC_FORCE_RESET();
            __HAL_RCC_GPIOC_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOD_BASE
        case GPIOD_BASE:
            __HAL_RCC_GPIOD_FORCE_RESET();
            __HAL_RCC_GPIOD_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOE_BASE
        case GPIOE_BASE:
            __HAL_RCC_GPIOE_FORCE_RESET();
            __HAL_RCC_GPIOE_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOF_BASE
        case GPIOF_BASE:
            __HAL_RCC_GPIOF_FORCE_RESET();
            __HAL_RCC_GPIOF_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOG_BASE
        case GPIOG_BASE:
            __HAL_RCC_GPIOG_FORCE_RESET();
            __HAL_RCC_GPIOG_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOH_BASE
        case GPIOH_BASE:
            __HAL_RCC_GPIOH_FORCE_RESET();
            __HAL_RCC_GPIOH_RELEASE_RESET();
            break;
        #endif
        #ifdef GPIOI_BASE
        case GPIOI_BASE:
            __HAL_RCC_GPIOI_FORCE_RESET();
            __HAL_RCC_GPIOI_RELEASE_RESET();
            break;
        #endif
    }
}
void reset(const adc::RegularADC& adc) noexcept
{
    #ifdef __HAL_RCC_ADC_FORCE_RESET
    __HAL_RCC_ADC_FORCE_RESET();
    __HAL_RCC_ADC_RELEASE_RESET();
    #endif
}
void reset(const dma::BaseDMA& dma) noexcept
{
    switch (dma.order)
    {
    case 0:
        #ifdef __HAL_RCC_DMA1_FORCE_RESET
        __HAL_RCC_DMA1_FORCE_RESET();
        __HAL_RCC_DMA1_RELEASE_RESET();
        #endif
        break;
    case 1:
        #ifdef __HAL_RCC_DMA2_FORCE_RESET
        __HAL_RCC_DMA2_FORCE_RESET();
        __HAL_RCC_DMA2_RELEASE_RESET();
        #endif
        break;
    }
}
void reset(const tim::BasicTimer& tim) noexcept
{
    const auto ptr = reinterpret_cast<TIM_TypeDef *>(&tim.reg);

    if (false) {}
    #if defined(TIM1)
    else if (ptr == TIM1)
    {
        __HAL_RCC_TIM1_FORCE_RESET();
        __HAL_RCC_TIM1_RELEASE_RESET();
    }
    #endif
    #if defined(TIM2)
    else if (ptr == TIM2)
    {
        __HAL_RCC_TIM2_FORCE_RESET();
        __HAL_RCC_TIM2_RELEASE_RESET();
    }
    #endif
    #if defined(TIM3)
    else if (ptr == TIM3)
    {
        __HAL_RCC_TIM3_FORCE_RESET();
        __HAL_RCC_TIM3_RELEASE_RESET();
    }
    #endif
    #if defined(TIM4)
    else if (ptr == TIM4)
    {
        __HAL_RCC_TIM4_FORCE_RESET();
        __HAL_RCC_TIM4_RELEASE_RESET();
    }
    #endif
    #if defined(TIM5)
    else if (ptr == TIM5)
    {
        __HAL_RCC_TIM5_FORCE_RESET();
        __HAL_RCC_TIM5_RELEASE_RESET();
    }
    #endif
    #if defined(TIM6)
    else if (ptr == TIM6)
    {
        __HAL_RCC_TIM6_FORCE_RESET();
        __HAL_RCC_TIM6_RELEASE_RESET();
    }
    #endif
    #if defined(TIM7)
    else if (ptr == TIM7)
    {
        __HAL_RCC_TIM7_FORCE_RESET();
        __HAL_RCC_TIM7_RELEASE_RESET();
    }
    #endif
    #if defined(TIM8)
    else if (ptr == TIM8)
    {
        __HAL_RCC_TIM8_FORCE_RESET();
        __HAL_RCC_TIM8_RELEASE_RESET();
    }
    #endif
    #if defined(TIM9)
    else if (ptr == TIM9)
    {
        __HAL_RCC_TIM9_FORCE_RESET();
        __HAL_RCC_TIM9_RELEASE_RESET();
    }
    #endif
    #if defined(TIM10)
    else if (ptr == TIM10)
    {
        __HAL_RCC_TIM10_FORCE_RESET();
        __HAL_RCC_TIM10_RELEASE_RESET();
    }
    #endif
    #if defined(TIM11)
    else if (ptr == TIM11)
    {
        __HAL_RCC_TIM11_FORCE_RESET();
        __HAL_RCC_TIM11_RELEASE_RESET();
    }
    #endif
    #if defined(TIM12)
    else if (ptr == TIM12)
    {
        __HAL_RCC_TIM12_FORCE_RESET();
        __HAL_RCC_TIM12_RELEASE_RESET();
    }
    #endif
    #if defined(TIM13)
    else if (ptr == TIM13)
    {
        __HAL_RCC_TIM13_FORCE_RESET();
        __HAL_RCC_TIM13_RELEASE_RESET();
    }
    #endif
    #if defined(TIM14)
    else if (ptr == TIM14)
    {
        __HAL_RCC_TIM14_FORCE_RESET();
        __HAL_RCC_TIM14_RELEASE_RESET();
    }
    #endif
    #if defined(TIM15)
    else if (ptr == TIM15)
    {
        __HAL_RCC_TIM15_FORCE_RESET();
        __HAL_RCC_TIM15_RELEASE_RESET();
    }
    #endif
    #if defined(TIM16)
    else if (ptr == TIM16)
    {
        __HAL_RCC_TIM16_FORCE_RESET();
        __HAL_RCC_TIM16_RELEASE_RESET();
    }
    #endif
    #if defined(TIM17)
    else if (ptr == TIM17)
    {
        __HAL_RCC_TIM17_FORCE_RESET();
        __HAL_RCC_TIM17_RELEASE_RESET();
    }
    #endif
}
void reset(const flash::EmbeddedFlash& flash) noexcept
{
    #ifdef __HAL_RCC_FLASH_FORCE_RESET
    __HAL_RCC_FLASH_FORCE_RESET();
    __HAL_RCC_FLASH_RELEASE_RESET();
    #endif
}

}
}
}
}
