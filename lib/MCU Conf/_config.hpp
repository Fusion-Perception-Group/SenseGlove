#pragma once

#include "userconfig.hpp"

#if VERMIL_STM32_USE_CMSIS
#error "CMSIS is not supported yet"

#else // Use HAL lib

#ifdef VERMIL_STM32_CUSTOM_FIRMWARE_LIB
#include VERMIL_STM32_CUSTOM_FIRMWARE_LIB

#elif defined(__VERMIL_STM32F1)
#include "stm32f1xx_hal.h"

#elif defined(__VERMIL_STM32F4)
#include "stm32f4xx_hal.h"

#elif defined(__VERMIL_STM32H7)
#include "stm32h7xx_hal.h"

#else
#error "No MCU HAL lib available"
#endif
#endif