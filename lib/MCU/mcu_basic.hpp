#pragma once
#include <string>
#include <cstdint>
#include <stdexcept>
#include "userconfig.hpp"
#include "property.hpp"
#include "clock.hpp"
#include "nvic.hpp"
#include "flash.hpp"

namespace vermils
{
namespace stm32
{
namespace mcu
{
constexpr const std::string_view modelname = ModelName;

/**
 * @brief init mcu
 * 
 * @throw std::runtime_error if init failed
 */
void init();
void default_init();

}
}
}
