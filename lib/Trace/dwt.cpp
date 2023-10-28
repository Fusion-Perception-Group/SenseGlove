#include "trace.hpp"
#include "_config.hpp"

#define CTRL_BIT_PROP(X) X({\
            []() {\
                return bool(DWT->CTRL & DWT_CTRL_##X##_Msk);\
            },\
            [](const bool value) {\
                if (value)\
                {\
                    DWT->CTRL |= DWT_CTRL_##X##_Msk;\
                }\
                else\
                {\
                    DWT->CTRL &= ~DWT_CTRL_##X##_Msk;\
                }\
            }\
        })

#define DWT_FWD(X) X({\
            []() {\
                return DWT->X;\
            },\
            [](uint32_t value) {\
                DWT->X = value;\
            }\
        })

namespace vermils
{
namespace stm32
{
    namespace hidden
    {
        volatile uint32_t & CYCCNT = DWT->CYCCNT;
    }

    void enable_trace()
    {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }

    DataWatchpointTrigger::DataWatchpointTrigger():
        CTRL({
            []() {
                return DWT->CTRL;
            },
            [](uint32_t value) {
                DWT->CTRL = value;
            }
        }),
        CTRL_BIT_PROP(CYCCNTENA),
        CTRL_BIT_PROP(EXCTRCENA),
        CTRL_BIT_PROP(PCSAMPLENA),
        CTRL_BIT_PROP(CPIEVTENA),
        CTRL_BIT_PROP(EXCEVTENA),
        CTRL_BIT_PROP(SLEEPEVTENA),
        CTRL_BIT_PROP(LSUEVTENA),
        CTRL_BIT_PROP(FOLDEVTENA),
        CTRL_BIT_PROP(CYCEVTENA),
        CTRL_BIT_PROP(NOPRFCNT),
        CTRL_BIT_PROP(NOCYCCNT),
        CTRL_BIT_PROP(NOTRCPKT),
        CTRL_BIT_PROP(NOEXTTRIG),
        CTRL_BIT_PROP(POSTPRESET),
        CTRL_BIT_PROP(POSTINIT),
        CTRL_BIT_PROP(CYCTAP),
        CTRL_BIT_PROP(SYNCTAP),
        CTRL_BIT_PROP(NUMCOMP),
        //DWT_FWD(CYCCNT),
        DWT_FWD(CPICNT),
        DWT_FWD(EXCCNT),
        DWT_FWD(SLEEPCNT),
        DWT_FWD(LSUCNT),
        DWT_FWD(FOLDCNT),
        PCSR([](){ return DWT->PCSR; }),
        DWT_FWD(COMP0),
        DWT_FWD(MASK0),
        DWT_FWD(FUNCTION0),
        DWT_FWD(COMP1),
        DWT_FWD(MASK1),
        DWT_FWD(FUNCTION1),
        DWT_FWD(COMP2),
        DWT_FWD(MASK2),
        DWT_FWD(FUNCTION2),
        DWT_FWD(COMP3),
        DWT_FWD(MASK3),
        DWT_FWD(FUNCTION3)
    {}
}
}
