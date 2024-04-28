#include "./_cpp_config.hpp"
#include "trace/trace.hpp"

#define CTRL_BIT_PROP(X) X({ []() {                                          \
                                return bool(DWT->CTRL & DWT_CTRL_##X##_Msk); \
                            },                                               \
    [](const bool value) {                                                   \
        if (value) {                                                         \
            DWT->CTRL |= DWT_CTRL_##X##_Msk;                                 \
        } else {                                                             \
            DWT->CTRL &= ~DWT_CTRL_##X##_Msk;                                \
        }                                                                    \
    } })

namespace elfe {
namespace stm32 {
    namespace dwt {
        namespace hidden {
            Register& DWT_REG = *reinterpret_cast<Register*>(DWT_BASE);
        }

        void enable_trace()
        {
            CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        }

        DataWatchpointTrigger::DataWatchpointTrigger()
        //:
        // CTRL({
        //     []() {
        //         return DWT->CTRL;
        //     },
        //     [](uint32_t value) {
        //         DWT->CTRL = value;
        //     }
        // }),
        // CTRL_BIT_PROP(CYCCNTENA),
        // CTRL_BIT_PROP(EXCTRCENA),
        // CTRL_BIT_PROP(PCSAMPLENA),
        // CTRL_BIT_PROP(CPIEVTENA),
        // CTRL_BIT_PROP(EXCEVTENA),
        // CTRL_BIT_PROP(SLEEPEVTENA),
        // CTRL_BIT_PROP(LSUEVTENA),
        // CTRL_BIT_PROP(FOLDEVTENA),
        // CTRL_BIT_PROP(CYCEVTENA),
        // CTRL_BIT_PROP(NOPRFCNT),
        // CTRL_BIT_PROP(NOCYCCNT),
        // CTRL_BIT_PROP(NOTRCPKT),
        // CTRL_BIT_PROP(NOEXTTRIG),
        // CTRL_BIT_PROP(POSTPRESET),
        // CTRL_BIT_PROP(POSTINIT),
        // CTRL_BIT_PROP(CYCTAP),
        // CTRL_BIT_PROP(SYNCTAP),
        // CTRL_BIT_PROP(NUMCOMP),
        // PCSR([](){ return DWT->PCSR; })
        {
        }
    }
}
}
