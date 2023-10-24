#ifndef _p_trace_hpp_
#define _p_trace_hpp_

#include "MCU_Conf.hpp"
#include "Property.hpp"

namespace vermils
{
namespace stm32
{
    static inline void enable_trace()
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    class DataWatchpointTrigger
    {
    public:
        mutable tricks::Property<uint32_t> CTRL = {
            []() {
                return DWT->CTRL;
            },
            [](uint32_t value) {
                DWT->CTRL = value;
            }
        };
        mutable tricks::Property<bool> CYCCNTENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> EXCTRCENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_EXCTRCENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_EXCTRCENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_EXCTRCENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> PCSAMPLENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_PCSAMPLENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_PCSAMPLENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_PCSAMPLENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> CPIEVTENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_CPIEVTENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_CPIEVTENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_CPIEVTENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> EXCEVTENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_EXCEVTENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_EXCEVTENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_EXCEVTENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> SLEEPEVTENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_SLEEPEVTENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_SLEEPEVTENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_SLEEPEVTENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> LSUEVTENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_LSUEVTENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_LSUEVTENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_LSUEVTENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> FOLDEVTENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_FOLDEVTENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_FOLDEVTENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_FOLDEVTENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> CYCEVTENA = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_CYCEVTENA_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_CYCEVTENA_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_CYCEVTENA_Msk;
                }
            }
        };
        mutable tricks::Property<bool> NOPRFCNT = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_NOPRFCNT_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_NOPRFCNT_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_NOPRFCNT_Msk;
                }
            }
        };
        mutable tricks::Property<bool> NOCYCCNT = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_NOCYCCNT_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_NOCYCCNT_Msk;
                }
            }
        };
        mutable tricks::Property<bool> NOTRCPKT = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_NOTRCPKT_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_NOTRCPKT_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_NOTRCPKT_Msk;
                }
            }
        };
        mutable tricks::Property<bool> NOEXTTRIG = {
            []() {
                return bool(DWT->CTRL & DWT_CTRL_NOEXTTRIG_Msk);
            },
            [](const bool value) {
                if (value)
                {
                    DWT->CTRL |= DWT_CTRL_NOEXTTRIG_Msk;
                }
                else
                {
                    DWT->CTRL &= ~DWT_CTRL_NOEXTTRIG_Msk;
                }
            }
        };
        mutable tricks::Property<uint8_t> POSTPRESET = {
            []() {
                return uint8_t((DWT->CTRL & DWT_CTRL_POSTPRESET_Msk) >> DWT_CTRL_POSTPRESET_Pos);
            },
            [](const uint8_t value) {
                DWT->CTRL &= ~DWT_CTRL_POSTPRESET_Msk;
                DWT->CTRL |= (value << DWT_CTRL_POSTPRESET_Pos) & DWT_CTRL_POSTPRESET_Msk;
            }
        };
        mutable tricks::Property<uint8_t> POSTINIT = {
            []() {
                return uint8_t((DWT->CTRL & DWT_CTRL_POSTINIT_Msk) >> DWT_CTRL_POSTINIT_Pos);
            },
            [](const uint8_t value) {
                DWT->CTRL &= ~DWT_CTRL_POSTINIT_Msk;
                DWT->CTRL |= (value << DWT_CTRL_POSTINIT_Pos) & DWT_CTRL_POSTINIT_Msk;
            }
        };
        mutable tricks::Property<bool> CYCTAP = {
            []() {
                return bool((DWT->CTRL & DWT_CTRL_CYCTAP_Msk) >> DWT_CTRL_CYCTAP_Pos);
            },
            [](const bool value) {
                DWT->CTRL &= ~DWT_CTRL_CYCTAP_Msk;
                DWT->CTRL |= (value << DWT_CTRL_CYCTAP_Pos) & DWT_CTRL_CYCTAP_Msk;
            }
        };
        mutable tricks::Property<uint8_t> SYNCTAP = {
            []() {
                return uint8_t((DWT->CTRL & DWT_CTRL_SYNCTAP_Msk) >> DWT_CTRL_SYNCTAP_Pos);
            },
            [](const uint8_t value) {
                DWT->CTRL &= ~DWT_CTRL_SYNCTAP_Msk;
                DWT->CTRL |= (value << DWT_CTRL_SYNCTAP_Pos) & DWT_CTRL_SYNCTAP_Msk;
            }
        };
        mutable tricks::Property<uint8_t> NUMCOMP = {
            []() {
                return uint8_t((DWT->CTRL & DWT_CTRL_NUMCOMP_Msk) >> DWT_CTRL_NUMCOMP_Pos);
            },
            [](const uint8_t value) {
                DWT->CTRL &= ~DWT_CTRL_NUMCOMP_Msk;
                DWT->CTRL |= (value << DWT_CTRL_NUMCOMP_Pos) & DWT_CTRL_NUMCOMP_Msk;
            }
        };
        mutable tricks::Property<uint32_t> CYCCNT = {
            []() {
                return DWT->CYCCNT;
            },
            [](uint32_t value) {
                DWT->CYCCNT = value;
            }
        };
        mutable tricks::Property<uint32_t> CPICNT = {
            []() {
                return DWT->CPICNT;
            },
            [](uint32_t value) {
                DWT->CPICNT = value;
            }
        };
        mutable tricks::Property<uint32_t> EXCCNT = {
            []() {
                return DWT->EXCCNT;
            },
            [](uint32_t value) {
                DWT->EXCCNT = value;
            }
        };
        mutable tricks::Property<uint32_t> SLEEPCNT = {
            []() {
                return DWT->SLEEPCNT;
            },
            [](uint32_t value) {
                DWT->SLEEPCNT = value;
            }
        };
        mutable tricks::Property<uint32_t> LSUCNT = {
            []() {
                return DWT->LSUCNT;
            },
            [](uint32_t value) {
                DWT->LSUCNT = value;
            }
        };
        mutable tricks::Property<uint32_t> FOLDCNT = {
            []() {
                return DWT->FOLDCNT;
            },
            [](uint32_t value) {
                DWT->FOLDCNT = value;
            }
        };
        const tricks::Property<uint32_t> PCSR = {
            []() {
                return DWT->PCSR;
            }
        };
        mutable tricks::Property<uint32_t> COMP0 = {
            []() {
                return DWT->COMP0;
            },
            [](uint32_t value) {
                DWT->COMP0 = value;
            }
        };
        mutable tricks::Property<uint32_t> MASK0 = {
            []() {
                return DWT->MASK0;
            },
            [](uint32_t value) {
                DWT->MASK0 = value;
            }
        };
        mutable tricks::Property<uint32_t> FUNCTION0 = {
            []() {
                return DWT->FUNCTION0;
            },
            [](uint32_t value) {
                DWT->FUNCTION0 = value;
            }
        };
        mutable tricks::Property<uint32_t> COMP1 = {
            []() {
                return DWT->COMP1;
            },
            [](uint32_t value) {
                DWT->COMP1 = value;
            }
        };
        mutable tricks::Property<uint32_t> MASK1 = {
            []() {
                return DWT->MASK1;
            },
            [](uint32_t value) {
                DWT->MASK1 = value;
            }
        };
        mutable tricks::Property<uint32_t> FUNCTION1 = {
            []() {
                return DWT->FUNCTION1;
            },
            [](uint32_t value) {
                DWT->FUNCTION1 = value;
            }
        };
        mutable tricks::Property<uint32_t> COMP2 = {
            []() {
                return DWT->COMP2;
            },
            [](uint32_t value) {
                DWT->COMP2 = value;
            }
        };
        mutable tricks::Property<uint32_t> MASK2 = {
            []() {
                return DWT->MASK2;
            },
            [](uint32_t value) {
                DWT->MASK2 = value;
            }
        };
        mutable tricks::Property<uint32_t> FUNCTION2 = {
            []() {
                return DWT->FUNCTION2;
            },
            [](uint32_t value) {
                DWT->FUNCTION2 = value;
            }
        };
        mutable tricks::Property<uint32_t> COMP3 = {
            []() {
                return DWT->COMP3;
            },
            [](uint32_t value) {
                DWT->COMP3 = value;
            }
        };
        mutable tricks::Property<uint32_t> MASK3 = {
            []() {
                return DWT->MASK3;
            },
            [](uint32_t value) {
                DWT->MASK3 = value;
            }
        };
        mutable tricks::Property<uint32_t> FUNCTION3 = {
            []() {
                return DWT->FUNCTION3;
            },
            [](uint32_t value) {
                DWT->FUNCTION3 = value;
            }
        };
    };
}
}
#endif
