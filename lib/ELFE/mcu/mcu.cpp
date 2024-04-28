#include "./_cpp_config.hpp"
#include "mcu/mcu_basic.hpp"

#define weak __attribute__((weak))

namespace elfe {
namespace stm32 {
    namespace mcu {

        VoidResult<> default_init()
        {
            flash::enable_prefetch();
            nvic::set_priority_group(nvic::Pre2_Sub2);
            return clock::init_systick();
        }

        weak VoidResult<> init()
        {
            return default_init();
        }

    }
}
}