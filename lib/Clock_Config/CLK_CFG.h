#ifndef p_CLK_CFG_h
#define p_CLK_CFG_h

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

void SystemClock_Config(void);
void MX_RTC_Init(void);

#ifdef __cplusplus
}
#endif
#endif