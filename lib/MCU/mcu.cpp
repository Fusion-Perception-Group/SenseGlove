#include "mcu.hpp"
#include "_config.hpp"

#define weak __attribute__((weak))

namespace vermils
{
namespace stm32
{
namespace mcu
{

weak void init()
{
    flash::enable_prefetch();
    nvic::set_priority_group(nvic::Pre2_Sub2);
}

}
}
}