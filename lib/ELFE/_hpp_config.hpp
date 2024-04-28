#pragma once

#include "userconfig.hpp"

#ifndef ELFE_USE_EXCEPTIONS
#ifdef __cpp_exceptions
#define ELFE_USE_EXCEPTIONS 1
#else
#define ELFE_USE_EXCEPTIONS 0
#endif
#endif

#if defined(ELFE_STM32F103)
#define _ELFE_STM32F1

#elif defined(ELFE_STM32F411)
#define _ELFE_STM32F4

#elif defined(ELFE_STM32H743) || defined(ELFE_STM32H750)
#define _ELFE_STM32H7

#else
#define _ELFE_STM32_USE_GENERIC 1

#endif

#ifndef _ELFE_STM32_USE_GENERIC
#define _ELFE_STM32_USE_GENERIC ELFE_STM32_FORCE_GENERIC
#endif

#if defined(_ELFE_STM32F1) || defined(_ELFE_STM32F4)
#define _ELFE_STM32FX

#elif defined(_ELFE_STM32H7)
#define _ELFE_STM32HX

#endif
