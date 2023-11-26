#include "_config.hpp"
#include "adc.hpp"

namespace vermils
{
namespace stm32
{
namespace adc
{
namespace detail
{
    #ifdef ADC1_BASE
    _ADCReg &ADC1Reg = *reinterpret_cast<_ADCReg *>(ADC1_BASE);
    #endif
    #ifdef ADC2_BASE
    _ADCReg &ADC2Reg = *reinterpret_cast<_ADCReg *>(ADC2_BASE);
    #endif
    #ifdef ADC3_BASE
    _ADCReg &ADC3Reg = *reinterpret_cast<_ADCReg *>(ADC3_BASE);
    #endif

    #ifdef ADC1_COMMON_BASE
    _ADCCommonReg &ADC1CommonReg = *reinterpret_cast<_ADCCommonReg *>(ADC1_COMMON_BASE);
    #endif
    #ifdef ADC2_COMMON_BASE
    _ADCCommonReg &ADC2CommonReg = *reinterpret_cast<_ADCCommonReg *>(ADC2_COMMON_BASE);
    #endif
    #ifdef ADC3_COMMON_BASE
    _ADCCommonReg &ADC3CommonReg = *reinterpret_cast<_ADCCommonReg *>(ADC3_COMMON_BASE);
    #endif
}

void RegularADC::init() const noexcept
{
    switch (order)
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
    reg.CR2 |= 1U;
}

void RegularADC::deinit() const noexcept
{
    switch (order)
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
    reg.CR2 &= ~1U;
}
#ifdef ADC1_BASE
const InjectedADC Adc1(0, detail::ADC1CommonReg, detail::ADC1Reg);
#endif
#ifdef ADC2_BASE
const InjectedADC Adc2(1, detail::ADC2CommonReg, detail::ADC2Reg);
#endif
#ifdef ADC3_BASE
const InjectedADC Adc3(2, detail::ADC3CommonReg, detail::ADC3Reg);
#endif

extern "C"
{
    void ADC_IRQHandler()
    {
        #ifdef ADC1_BASE
        Adc1.global_irq_handler();
        #endif
        #ifdef ADC2_BASE
        Adc2.global_irq_handler();
        #endif
        #ifdef ADC3_BASE
        Adc3.global_irq_handler();
        #endif
    }
}

}
}
}