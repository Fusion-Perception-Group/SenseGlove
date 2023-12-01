#include "_config.hpp"
#include "rtc.hpp"

namespace vermils
{
namespace stm32
{
namespace clock
{
namespace rtc
{
    namespace detail
    {
        Register &reg = *reinterpret_cast<Register *>(RTC_BASE);
    }
}
}
}
}