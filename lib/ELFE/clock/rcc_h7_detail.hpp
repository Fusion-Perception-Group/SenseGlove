#pragma once

#include "_hpp_config.hpp"
#include <cstdint>

#if defined(_ELFE_STM32H7) && !_ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace clock {
        namespace rcc {

            namespace detail {
                struct RCCRegister {
                    volatile uint32_t CR; /*!< RCC clock control register,                                              Address offset: 0x00  */
                    volatile uint32_t HSICFGR; /*!< HSI Clock Calibration Register,                                          Address offset: 0x04  */
                    volatile uint32_t CRRCR; /*!< Clock Recovery RC  Register,                                             Address offset: 0x08  */
                    volatile uint32_t CSICFGR; /*!< CSI Clock Calibration Register,                                          Address offset: 0x0C  */
                    volatile uint32_t CFGR; /*!< RCC clock configuration register,                                        Address offset: 0x10  */
                    uint32_t RESERVED1; /*!< Reserved,                                                                Address offset: 0x14  */
                    volatile uint32_t D1CFGR; /*!< RCC Domain 1 configuration register,                                     Address offset: 0x18  */
                    volatile uint32_t D2CFGR; /*!< RCC Domain 2 configuration register,                                     Address offset: 0x1C  */
                    volatile uint32_t D3CFGR; /*!< RCC Domain 3 configuration register,                                     Address offset: 0x20  */
                    uint32_t RESERVED2; /*!< Reserved,                                                                Address offset: 0x24  */
                    volatile uint32_t PLLCKSELR; /*!< RCC PLLs Clock Source Selection Register,                                Address offset: 0x28  */
                    volatile uint32_t PLLCFGR; /*!< RCC PLLs  Configuration Register,                                        Address offset: 0x2C  */
                    volatile uint32_t PLL1DIVR; /*!< RCC PLL1 Dividers Configuration Register,                                Address offset: 0x30  */
                    volatile uint32_t PLL1FRACR; /*!< RCC PLL1 Fractional Divider Configuration Register,                      Address offset: 0x34  */
                    volatile uint32_t PLL2DIVR; /*!< RCC PLL2 Dividers Configuration Register,                                Address offset: 0x38  */
                    volatile uint32_t PLL2FRACR; /*!< RCC PLL2 Fractional Divider Configuration Register,                      Address offset: 0x3C  */
                    volatile uint32_t PLL3DIVR; /*!< RCC PLL3 Dividers Configuration Register,                                Address offset: 0x40  */
                    volatile uint32_t PLL3FRACR; /*!< RCC PLL3 Fractional Divider Configuration Register,                      Address offset: 0x44  */
                    uint32_t RESERVED3; /*!< Reserved,                                                                Address offset: 0x48  */
                    volatile uint32_t D1CCIPR; /*!< RCC Domain 1 Kernel Clock Configuration Register                         Address offset: 0x4C  */
                    volatile uint32_t D2CCIP1R; /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x50  */
                    volatile uint32_t D2CCIP2R; /*!< RCC Domain 2 Kernel Clock Configuration Register                         Address offset: 0x54  */
                    volatile uint32_t D3CCIPR; /*!< RCC Domain 3 Kernel Clock Configuration Register                         Address offset: 0x58  */
                    uint32_t RESERVED4; /*!< Reserved,                                                                Address offset: 0x5C  */
                    volatile uint32_t CIER; /*!< RCC Clock Source Interrupt Enable Register                               Address offset: 0x60  */
                    volatile uint32_t CIFR; /*!< RCC Clock Source Interrupt Flag Register                                 Address offset: 0x64  */
                    volatile uint32_t CICR; /*!< RCC Clock Source Interrupt Clear Register                                Address offset: 0x68  */
                    uint32_t RESERVED5; /*!< Reserved,                                                                Address offset: 0x6C  */
                    volatile uint32_t BDCR; /*!< RCC Vswitch Backup Domain Control Register,                              Address offset: 0x70  */
                    volatile uint32_t CSR; /*!< RCC clock control & status register,                                     Address offset: 0x74  */
                    uint32_t RESERVED6; /*!< Reserved,                                                                Address offset: 0x78  */
                    volatile uint32_t AHB3RSTR; /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x7C  */
                    volatile uint32_t AHB1RSTR; /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x80  */
                    volatile uint32_t AHB2RSTR; /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x84  */
                    volatile uint32_t AHB4RSTR; /*!< RCC AHB4 peripheral reset register,                                      Address offset: 0x88  */
                    volatile uint32_t APB3RSTR; /*!< RCC APB3 peripheral reset register,                                      Address offset: 0x8C  */
                    volatile uint32_t APB1LRSTR; /*!< RCC APB1 peripheral reset Low Word register,                             Address offset: 0x90  */
                    volatile uint32_t APB1HRSTR; /*!< RCC APB1 peripheral reset High Word register,                            Address offset: 0x94  */
                    volatile uint32_t APB2RSTR; /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x98  */
                    volatile uint32_t APB4RSTR; /*!< RCC APB4 peripheral reset register,                                      Address offset: 0x9C  */
                    volatile uint32_t GCR; /*!< RCC RCC Global Control  Register,                                        Address offset: 0xA0  */
                    uint32_t RESERVED8; /*!< Reserved,                                                                Address offset: 0xA4  */
                    volatile uint32_t D3AMR; /*!< RCC Domain 3 Autonomous Mode Register,                                   Address offset: 0xA8  */
                    uint32_t RESERVED11[9]; /*!< Reserved, 0xAC-0xCC                                                      Address offset: 0xAC  */
                    volatile uint32_t RSR; /*!< RCC Reset status register,                                               Address offset: 0xD0  */
                    volatile uint32_t AHB3ENR; /*!< RCC AHB3 peripheral clock  register,                                     Address offset: 0xD4  */
                    volatile uint32_t AHB1ENR; /*!< RCC AHB1 peripheral clock  register,                                     Address offset: 0xD8  */
                    volatile uint32_t AHB2ENR; /*!< RCC AHB2 peripheral clock  register,                                     Address offset: 0xDC  */
                    volatile uint32_t AHB4ENR; /*!< RCC AHB4 peripheral clock  register,                                     Address offset: 0xE0  */
                    volatile uint32_t APB3ENR; /*!< RCC APB3 peripheral clock  register,                                     Address offset: 0xE4  */
                    volatile uint32_t APB1LENR; /*!< RCC APB1 peripheral clock  Low Word register,                            Address offset: 0xE8  */
                    volatile uint32_t APB1HENR; /*!< RCC APB1 peripheral clock  High Word register,                           Address offset: 0xEC  */
                    volatile uint32_t APB2ENR; /*!< RCC APB2 peripheral clock  register,                                     Address offset: 0xF0  */
                    volatile uint32_t APB4ENR; /*!< RCC APB4 peripheral clock  register,                                     Address offset: 0xF4  */
                    uint32_t RESERVED12; /*!< Reserved,                                                                Address offset: 0xF8  */
                    volatile uint32_t AHB3LPENR; /*!< RCC AHB3 peripheral sleep clock  register,                               Address offset: 0xFC  */
                    volatile uint32_t AHB1LPENR; /*!< RCC AHB1 peripheral sleep clock  register,                               Address offset: 0x100 */
                    volatile uint32_t AHB2LPENR; /*!< RCC AHB2 peripheral sleep clock  register,                               Address offset: 0x104 */
                    volatile uint32_t AHB4LPENR; /*!< RCC AHB4 peripheral sleep clock  register,                               Address offset: 0x108 */
                    volatile uint32_t APB3LPENR; /*!< RCC APB3 peripheral sleep clock  register,                               Address offset: 0x10C */
                    volatile uint32_t APB1LLPENR; /*!< RCC APB1 peripheral sleep clock  Low Word register,                      Address offset: 0x110 */
                    volatile uint32_t APB1HLPENR; /*!< RCC APB1 peripheral sleep clock  High Word register,                     Address offset: 0x114 */
                    volatile uint32_t APB2LPENR; /*!< RCC APB2 peripheral sleep clock  register,                               Address offset: 0x118 */
                    volatile uint32_t APB4LPENR; /*!< RCC APB4 peripheral sleep clock  register,                               Address offset: 0x11C */
                    uint32_t RESERVED13[4]; /*!< Reserved, 0x120-0x12C                                                    Address offset: 0x120 */
                };

                extern RCCRegister& RCCReg;
            }

        }
    }
}
}

#endif
