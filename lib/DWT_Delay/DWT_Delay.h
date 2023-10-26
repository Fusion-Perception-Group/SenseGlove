#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"
  bool DWT_delay_init(void);

  /**
   * @brief  This function provides a delay (in microseconds)
   * @param  microseconds: delay in microseconds
   */
  static inline void DWT_delay_us(volatile uint32_t microseconds)
  {
    const uint32_t clk_cycle_start = DWT->CYCCNT;

    /* Go to number of cycles for system */
    microseconds *= (HAL_RCC_GetHCLKFreq() / 1000000);

    /* Delay till end */
    while ((DWT->CYCCNT - clk_cycle_start) < microseconds)
      ;
  }

#ifdef __cplusplus
}
#endif
