#ifndef _W25QXXCONFIG_H
#define _W25QXXCONFIG_H

#include "stm32f4xx_hal.h"

SPI_HandleTypeDef spi1=
{
    .Instance = SPI1,
    .Init.Mode = SPI_MODE_MASTER,
    .Init.Direction = SPI_DIRECTION_2LINES,
    .Init.DataSize = SPI_DATASIZE_8BIT,
    .Init.CLKPolarity = SPI_POLARITY_LOW,
    .Init.CLKPhase = SPI_PHASE_1EDGE,
    .Init.NSS = SPI_NSS_SOFT,
    .Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
    .Init.FirstBit = SPI_FIRSTBIT_MSB,
    .Init.TIMode = SPI_TIMODE_DISABLE,
};



#define _W25QXX_SPI                   spi1
#define _W25QXX_CS_GPIO               GPIOA
#define _W25QXX_CS_PIN                GPIO_PIN_4
#define _W25QXX_USE_FREERTOS          0
#define _W25QXX_DEBUG                 0

#endif
