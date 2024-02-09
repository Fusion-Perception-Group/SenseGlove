#include "flash.hpp"
#include "_config.hpp"

namespace vms
{
namespace stm32
{
namespace flash
{

bool enable_instruction_cache() // enable instruction cache if available
{
    #ifdef __HAL_FLASH_INSTRUCTION_CACHE_ENABLE
    __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
    return true;
    #else
    return false;
    #endif
}

bool enable_data_cache() // enable data cache if available
{
    #ifdef __HAL_FLASH_DATA_CACHE_ENABLE
    __HAL_FLASH_DATA_CACHE_ENABLE();
    return true;
    #else
    return false;
    #endif
}

}
}
}
