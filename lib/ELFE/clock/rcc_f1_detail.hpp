#pragma once

#include "_hpp_config.hpp"
#include <cstdint>

#if defined(_ELFE_STM32F1) && !_ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace clock {
        namespace rcc {

            namespace detail {
                struct RCCRegister {
                    volatile uint32_t CR;
                    volatile uint32_t CFGR;
                    volatile uint32_t CIR;
                    volatile uint32_t APB2RSTR;
                    volatile uint32_t APB1RSTR;
                    volatile uint32_t AHBENR;
                    volatile uint32_t APB2ENR;
                    volatile uint32_t APB1ENR;
                    volatile uint32_t BDCR;
                    volatile uint32_t CSR;
                };

                extern RCCRegister& RCCReg;
            }

        }
    }
}
}

#endif
