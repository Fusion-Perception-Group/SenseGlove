#pragma once

#include <string>
#include <cstdint>
#include "userconfig.hpp"
#include "gpio.hpp"
#include "property.hpp"

namespace vermils
{
namespace stm32
{
class MCU
{
public:
    const std::string model = ModelName;
    std::string name;

    MCU(const std::string &name) : name(name) {}

    /**
     * @brief You don't have to explicitly call this function, it is called by the constructor.
     * 
     * @return int 0 if success, otherwise error code
     */
    int init()
    {
        
    }

};

}
}
