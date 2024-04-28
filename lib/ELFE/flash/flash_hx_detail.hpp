#pragma once

#include "_hpp_config.hpp"
#include <cstdint>

#if defined(_ELFE_STM32H7) && !_ELFE_STM32_USE_GENERIC

namespace elfe {
namespace stm32 {
    namespace flash {

        namespace detail {
            struct FlashRegister {
                volatile uint32_t ACR; /*!< FLASH access control register,                            Address offset: 0x00  */
                volatile uint32_t KEYR1; /*!< Flash Key Register for bank1,                             Address offset: 0x04  */
                volatile uint32_t OPTKEYR; /*!< Flash Option Key Register,                                Address offset: 0x08  */
                volatile uint32_t CR1; /*!< Flash Control Register for bank1,                         Address offset: 0x0C  */
                volatile uint32_t SR1; /*!< Flash Status Register for bank1,                          Address offset: 0x10  */
                volatile uint32_t CCR1; /*!< Flash Control Register for bank1,                         Address offset: 0x14  */
                volatile uint32_t OPTCR; /*!< Flash Option Control Register,                            Address offset: 0x18  */
                volatile uint32_t OPTSR_CUR; /*!< Flash Option Status Current Register,                     Address offset: 0x1C  */
                volatile uint32_t OPTSR_PRG; /*!< Flash Option Status to Program Register,                  Address offset: 0x20  */
                volatile uint32_t OPTCCR; /*!< Flash Option Clear Control Register,                      Address offset: 0x24  */
                volatile uint32_t PRAR_CUR1; /*!< Flash Current Protection Address Register for bank1,      Address offset: 0x28  */
                volatile uint32_t PRAR_PRG1; /*!< Flash Protection Address to Program Register for bank1,   Address offset: 0x2C  */
                volatile uint32_t SCAR_CUR1; /*!< Flash Current Secure Address Register for bank1,          Address offset: 0x30  */
                volatile uint32_t SCAR_PRG1; /*!< Flash Secure Address to Program Register for bank1,       Address offset: 0x34  */
                volatile uint32_t WPSN_CUR1; /*!< Flash Current Write Protection Register on bank1,         Address offset: 0x38  */
                volatile uint32_t WPSN_PRG1; /*!< Flash Write Protection to Program Register on bank1,      Address offset: 0x3C  */
                volatile uint32_t BOOT_CUR; /*!< Flash Current Boot Address for Pelican Core Register,     Address offset: 0x40  */
                volatile uint32_t BOOT_PRG; /*!< Flash Boot Address to Program for Pelican Core Register,  Address offset: 0x44  */
                uint32_t RESERVED0[2]; /*!< Reserved, 0x48 to 0x4C                                                          */
                volatile uint32_t CRCCR1; /*!< Flash CRC Control register For Bank1 Register ,           Address offset: 0x50  */
                volatile uint32_t CRCSADD1; /*!< Flash CRC Start Address Register for Bank1 ,              Address offset: 0x54  */
                volatile uint32_t CRCEADD1; /*!< Flash CRC End Address Register for Bank1 ,                Address offset: 0x58  */
                volatile uint32_t CRCDATA; /*!< Flash CRC Data Register for Bank1 ,                       Address offset: 0x5C  */
                volatile uint32_t ECC_FA1; /*!< Flash ECC Fail Address For Bank1 Register ,               Address offset: 0x60  */
                uint32_t RESERVED1[40]; /*!< Reserved, 0x64 to 0x100                                                         */
                volatile uint32_t KEYR2; /*!< Flash Key Register for bank2,                             Address offset: 0x104 */
                uint32_t RESERVED2; /*!< Reserved, 0x108                                                                 */
                volatile uint32_t CR2; /*!< Flash Control Register for bank2,                         Address offset: 0x10C */
                volatile uint32_t SR2; /*!< Flash Status Register for bank2,                          Address offset: 0x110 */
                volatile uint32_t CCR2; /*!< Flash Status Register for bank2,                          Address offset: 0x114 */
                uint32_t RESERVED3[4]; /*!< Reserved, 0x118 to 0x124                                                        */
                volatile uint32_t PRAR_CUR2; /*!< Flash Current Protection Address Register for bank2,      Address offset: 0x128 */
                volatile uint32_t PRAR_PRG2; /*!< Flash Protection Address to Program Register for bank2,   Address offset: 0x12C */
                volatile uint32_t SCAR_CUR2; /*!< Flash Current Secure Address Register for bank2,          Address offset: 0x130 */
                volatile uint32_t SCAR_PRG2; /*!< Flash Secure Address Register for bank2,                  Address offset: 0x134 */
                volatile uint32_t WPSN_CUR2; /*!< Flash Current Write Protection Register on bank2,         Address offset: 0x138 */
                volatile uint32_t WPSN_PRG2; /*!< Flash Write Protection to Program Register on bank2,      Address offset: 0x13C */
                uint32_t RESERVED4[4]; /*!< Reserved, 0x140 to 0x14C                                                        */
                volatile uint32_t CRCCR2; /*!< Flash CRC Control register For Bank2 Register ,           Address offset: 0x150 */
                volatile uint32_t CRCSADD2; /*!< Flash CRC Start Address Register for Bank2 ,              Address offset: 0x154 */
                volatile uint32_t CRCEADD2; /*!< Flash CRC End Address Register for Bank2 ,                Address offset: 0x158 */
                volatile uint32_t CRCDATA2; /*!< Flash CRC Data Register for Bank2 ,                       Address offset: 0x15C */
                volatile uint32_t ECC_FA2; /*!< Flash ECC Fail Address For Bank2 Register ,               Address offset: 0x160 */
            };

            extern FlashRegister& flash_reg;
        }

    }
}
}

#endif
