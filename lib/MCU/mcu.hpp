#pragma once

#include <string>
#include <cstdint>
#include <stdexcept>
#include "userconfig.hpp"
#include "property.hpp"
#include "clock.hpp"
#include "nvic.hpp"
#include "wdg.hpp"
#include "trace.hpp"
#include "flash.hpp"
#include "power.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "spi.hpp"
#include "usart.hpp"
#include "adc.hpp"
#include "dma.hpp"

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
