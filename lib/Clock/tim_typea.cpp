#include "tim.hpp"
#include "_config.hpp"

#if (defined(__VERMIL_STM32FX) || defined(__VERMIL_STM32HX)) && !__VERMIL_STM32_USE_GENERIC

namespace vermils
{
namespace stm32
{
namespace clock
{
    namespace detail
    {
        #if defined(TIM1)
        Register & TIM1_Reg = *reinterpret_cast<Register *>(TIM1_BASE);
        #endif
        #if defined(TIM2)
        Register & TIM2_Reg = *reinterpret_cast<Register *>(TIM2_BASE);
        #endif
        #if defined(TIM3)
        Register & TIM3_Reg = *reinterpret_cast<Register *>(TIM3_BASE);
        #endif
        #if defined(TIM4)
        Register & TIM4_Reg = *reinterpret_cast<Register *>(TIM4_BASE);
        #endif
        #if defined(TIM5)
        Register & TIM5_Reg = *reinterpret_cast<Register *>(TIM5_BASE);
        #endif
        #if defined(TIM6)
        Register & TIM6_Reg = *reinterpret_cast<Register *>(TIM6_BASE);
        #endif
        #if defined(TIM7)
        Register & TIM7_Reg = *reinterpret_cast<Register *>(TIM7_BASE);
        #endif
        #if defined(TIM8)
        Register & TIM8_Reg = *reinterpret_cast<Register *>(TIM8_BASE);
        #endif
        #if defined(TIM9)
        Register & TIM9_Reg = *reinterpret_cast<Register *>(TIM9_BASE);
        #endif
        #if defined(TIM10)
        Register & TIM10_Reg = *reinterpret_cast<Register *>(TIM10_BASE);
        #endif
        #if defined(TIM11)
        Register & TIM11_Reg = *reinterpret_cast<Register *>(TIM11_BASE);
        #endif
        #if defined(TIM12)
        Register & TIM12_Reg = *reinterpret_cast<Register *>(TIM12_BASE);
        #endif
        #if defined(TIM13)
        Register & TIM13_Reg = *reinterpret_cast<Register *>(TIM13_BASE);
        #endif
        #if defined(TIM14)
        Register & TIM14_Reg = *reinterpret_cast<Register *>(TIM14_BASE);
        #endif
        #if defined(TIM15)
        Register & TIM15_Reg = *reinterpret_cast<Register *>(TIM15_BASE);
        #endif
        #if defined(TIM16)
        Register & TIM16_Reg = *reinterpret_cast<Register *>(TIM16_BASE);
        #endif
        #if defined(TIM17)
        Register & TIM17_Reg = *reinterpret_cast<Register *>(TIM17_BASE);
        #endif
    }
}
}
}
#endif