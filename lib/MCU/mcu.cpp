#include "mcu_basic.hpp"
#include "_config.hpp"

#define weak __attribute__((weak))

namespace vermils
{
namespace stm32
{
namespace mcu
{

void default_init()
{
    flash::enable_prefetch();
    nvic::set_priority_group(nvic::Pre2_Sub2);
    clock::init_systick();
}

weak void init()
{
    default_init();
}

}
}
}