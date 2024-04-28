#pragma once

#include "utils/property.hpp"
#include <cstdint>

namespace elfe {
namespace stm32 {
    namespace dwt {
        void enable_trace();

        namespace hidden {
            struct Register {
                volatile uint32_t CTRL; /*!< Offset: 0x000 (R/W)  Control Register */
                volatile uint32_t CYCCNT; /*!< Offset: 0x004 (R/W)  Cycle Count Register */
                volatile uint32_t CPICNT; /*!< Offset: 0x008 (R/W)  CPI Count Register */
                volatile uint32_t EXCCNT; /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
                volatile uint32_t SLEEPCNT; /*!< Offset: 0x010 (R/W)  Sleep Count Register */
                volatile uint32_t LSUCNT; /*!< Offset: 0x014 (R/W)  LSU Count Register */
                volatile uint32_t FOLDCNT; /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
                volatile uint32_t PCSR; /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
                volatile uint32_t COMP0; /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
                volatile uint32_t MASK0; /*!< Offset: 0x024 (R/W)  Mask Register 0 */
                volatile uint32_t FUNCTION0; /*!< Offset: 0x028 (R/W)  Function Register 0 */
                volatile uint32_t RESERVED0[1U];
                volatile uint32_t COMP1; /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
                volatile uint32_t MASK1; /*!< Offset: 0x034 (R/W)  Mask Register 1 */
                volatile uint32_t FUNCTION1; /*!< Offset: 0x038 (R/W)  Function Register 1 */
                volatile uint32_t RESERVED1[1U];
                volatile uint32_t COMP2; /*!< Offset: 0x040 (R/W)  Comparator Register 2 */
                volatile uint32_t MASK2; /*!< Offset: 0x044 (R/W)  Mask Register 2 */
                volatile uint32_t FUNCTION2; /*!< Offset: 0x048 (R/W)  Function Register 2 */
                volatile uint32_t RESERVED2[1U];
                volatile uint32_t COMP3; /*!< Offset: 0x050 (R/W)  Comparator Register 3 */
                volatile uint32_t MASK3; /*!< Offset: 0x054 (R/W)  Mask Register 3 */
                volatile uint32_t FUNCTION3; /*!< Offset: 0x058 (R/W)  Function Register 3 */
            };
            extern Register& DWT_REG;
        }

        class DataWatchpointTrigger {
        public:
            hidden::Register& reg = hidden::DWT_REG;
            volatile uint32_t& CYCCNT = reg.CYCCNT;
            // tricks::Property<uint32_t> CTRL;
            // tricks::Property<bool> CYCCNTENA;
            // tricks::Property<bool> EXCTRCENA;
            // tricks::Property<bool> PCSAMPLENA;
            // tricks::Property<bool> CPIEVTENA;
            // tricks::Property<bool> EXCEVTENA;
            // tricks::Property<bool> SLEEPEVTENA;
            // tricks::Property<bool> LSUEVTENA;
            // tricks::Property<bool> FOLDEVTENA;
            // tricks::Property<bool> CYCEVTENA;
            // tricks::Property<bool> NOPRFCNT;
            // tricks::Property<bool> NOCYCCNT;
            // tricks::Property<bool> NOTRCPKT;
            // tricks::Property<bool> NOEXTTRIG;
            // tricks::Property<uint8_t> POSTPRESET;
            // tricks::Property<uint8_t> POSTINIT;
            // tricks::Property<bool> CYCTAP;
            // tricks::Property<uint8_t> SYNCTAP;
            // tricks::Property<uint8_t> NUMCOMP;
            // const tricks::Property<uint32_t> PCSR;
            DataWatchpointTrigger();

            void enable()
            {
                reg.CTRL |= 0x1U;
            }

            void disable()
            {
                reg.CTRL &= ~0x1U;
            }
        };
    }
}
}
