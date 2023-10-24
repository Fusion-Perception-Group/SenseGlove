#ifndef _p_mcu_hpp_
#define _p_mcu_hpp_

#include <string>
#include <cstdint>
#include "MCU_Conf.hpp"
#include "GPIO.hpp"
#include "Property.hpp"

namespace vermils
{
namespace stm32
{
class MCU
{
    class GPIOCls
    {

    };
public:
    const std::string model = ModelName;
    std::string name;


    #ifdef GPIOA
    GPIOCls gpioa[GPIO_PINS_N];
    #endif
    #ifdef GPIOB
    GPIOCls gpiob[GPIO_PINS_N];
    #endif

    int init()
    {
        return HAL_Init();
    }

};

}
}

#endif