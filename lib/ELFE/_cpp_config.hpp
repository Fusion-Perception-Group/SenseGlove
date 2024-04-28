#pragma once

#include "_hpp_config.hpp"

#if ELFE_STM32_USE_CMSIS
#error "CMSIS is not supported yet"

#else // Use HAL lib

#ifdef ELFE_STM32_CUSTOM_HAL_LIB
#if __has_include(ELFE_STM32_CUSTOM_HAL_LIB)
#include ELFE_STM32_CUSTOM_HAL_LIB
#else
#error "Custom firmware lib not found"
#endif

#elif defined(_ELFE_STM32F1)
#include "stm32f1xx_hal.h"

#elif defined(_ELFE_STM32F4)
#include "stm32f4xx_hal.h"

#elif defined(_ELFE_STM32H7)
#include "stm32h7xx_hal.h"

#else
#error "No MCU HAL lib available, must define ELFE_STM32_CUSTOM_HAL_LIB"
#endif
#endif
