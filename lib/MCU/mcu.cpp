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
    if(HAL_Init())
        throw std::runtime_error("HAL init failed");
}

}
}
}