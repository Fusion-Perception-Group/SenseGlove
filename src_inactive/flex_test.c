#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "GPIO.h"
#include "I2C.h"
#include "CLK_CFG.h"
#include "SSD1306.h"

#define OLED_SDA GPIO_PIN_7
#define OLED_SCL GPIO_PIN_6
#define OLED_PORT GPIOB
#define OLED_ADDR SSD1306_ADDR
#define FLEX_PORT GPIOA
#define FLEX_PIN_0 GPIO_PIN_1
#define FLEX_PIN_1 GPIO_PIN_2
#define FLEX_PIN_2 GPIO_PIN_3
#define CMD SSD1306_CMD
#define CMD_S SSD1306_CMD_STREAM
#define DATA SSD1306_DATA
#define DATA_S SSD1306_DATA_STREAM
#define Canvas SSD1306_Canvas_t

I2C_t i2c = {
    .GPIOx = OLED_PORT,
    .sda = OLED_SDA,
    .scl = OLED_SCL,
    .delay_us = 0,
    .arb_lost = false
};

SPI_HandleTypeDef spi1 = {
    .Instance = SPI1,
    .Init = {
        .Mode = SPI_MODE_MASTER,
        .Direction = SPI_DIRECTION_2LINES,
        .DataSize = SPI_DATASIZE_8BIT,
        .CLKPolarity = SPI_POLARITY_LOW,
        .CLKPhase = SPI_PHASE_1EDGE,
        .NSS = SPI_NSS_SOFT,
        .BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
        .FirstBit = SPI_FIRSTBIT_MSB,
        .TIMode = SPI_TIMODE_DISABLE,
        .CRCCalculation = SPI_CRCCALCULATION_ENABLE,
        .CRCPolynomial = 10,
    }
};

ADC_HandleTypeDef adc1 = {
    .Instance = ADC1,
    .Init = {
        .ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4,
        .Resolution = ADC_RESOLUTION_12B,
        .ScanConvMode = ENABLE,
        .ContinuousConvMode = DISABLE,
        .DiscontinuousConvMode = DISABLE,
        .ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE,
        .ExternalTrigConv = ADC_SOFTWARE_START,
        .DataAlign = ADC_DATAALIGN_RIGHT,
        .NbrOfConversion = 1,
        .DMAContinuousRequests = DISABLE,
        .EOCSelection = ADC_EOC_SEQ_CONV,
    }
};

TIM_HandleTypeDef tim = {
    .Instance = TIM3,
    .Init = {
        .Prescaler = 99,
        .CounterMode = TIM_COUNTERMODE_CENTERALIGNED1,
        .Period = 30000, // 170hz
        .ClockDivision = TIM_CLOCKDIVISION_DIV1,
        .RepetitionCounter = 0,
        .AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE,
    }
};

int main()
{
    bool ret = true;
    HAL_Init();
    SystemClock_Config();
    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    enable_GPIO_CLK(FLEX_PORT);
    enable_GPIO_CLK(GPIOB);
    init_GPIO(FLEX_PORT,
        FLEX_PIN_0 | FLEX_PIN_1 | FLEX_PIN_2,
        GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH);
    GPIO_InitTypeDef SPI1GPIO = {
        .Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = GPIO_AF5_SPI1,
    };
    HAL_GPIO_Init(GPIOA, &SPI1GPIO);

    HAL_ADC_Init(&adc1);
    ADC_InjectionConfTypeDef ConfigInjected = {
        .InjectedChannel = ADC_CHANNEL_1,
        .InjectedRank = 1,
        .InjectedNbrOfConversion = 3,
        .InjectedSamplingTime = ADC_SAMPLETIME_56CYCLES,
        .ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE,
        .ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START,
        .AutoInjectedConv = DISABLE,
        .InjectedDiscontinuousConvMode = DISABLE,
        .InjectedOffset = 0,
    };
    HAL_ADCEx_InjectedConfigChannel(&adc1, &ConfigInjected);
    ConfigInjected.InjectedChannel = ADC_CHANNEL_2;
    ConfigInjected.InjectedRank = 2;
    HAL_ADCEx_InjectedConfigChannel(&adc1, &ConfigInjected);
    ConfigInjected.InjectedChannel = ADC_CHANNEL_3;
    ConfigInjected.InjectedRank = 3;
    HAL_ADCEx_InjectedConfigChannel(&adc1, &ConfigInjected);

    enable_GPIO_CLK(GPIOC);
    init_GPIO(GPIOC, GPIO_PIN_13, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH);

    HAL_Delay(100);
    ret = init_display(&i2c);
    set_memory_addressing_mode(&i2c, 2);
    uint16_t result = 0;
    //uint8_t data=0;


    unsigned duty = 500;//1000;
    TIM_ClockConfigTypeDef ClockSourceConfig = {0};
    TIM_MasterConfigTypeDef MasterConfig = {0};
    TIM_OC_InitTypeDef ConfigOC = {0};
    HAL_TIM_Base_Init(&tim);
    HAL_TIM_PWM_Init(&tim);
    ClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&tim, &ClockSourceConfig);
    MasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    MasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&tim, &MasterConfig);
    ConfigOC.OCMode = TIM_OCMODE_PWM1;
    ConfigOC.Pulse = duty;
    ConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    ConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    ConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    ConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    ConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&tim, &ConfigOC, TIM_CHANNEL_3);
    ConfigOC.OCMode = TIM_OCMODE_PWM2;
    ConfigOC.Pulse = tim.Init.Period - duty;
    HAL_TIM_PWM_ConfigChannel(&tim, &ConfigOC, TIM_CHANNEL_4);
    GPIO_InitTypeDef PWMGPIO = {
        .Pin = GPIO_PIN_0 | GPIO_PIN_1,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF2_TIM3,
    };
    HAL_GPIO_Init(GPIOB, &PWMGPIO);
    HAL_TIM_Base_Start(&tim);
    HAL_TIM_PWM_Start(&tim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&tim, TIM_CHANNEL_4);

    
    Canvas background, text = {0};
    cvnprintf(text, 127, 12, 8, "ADC Output");
    //cvnprintf(text, 127, 0, 48, "w25q64 %d", w25qxx.CapacityInKiloByte);
    scale_canvas(background, text, 1.15, 0);
    while(true)
    {
        clear_canvas(text);

        HAL_ADCEx_InjectedStart(&adc1);
        HAL_ADCEx_InjectedPollForConversion(&adc1, 100);
        result = HAL_ADCEx_InjectedGetValue(&adc1, 1);
        cvnprintf(text, 127, 0, 16, "ADC1: %d", result);
        result = HAL_ADCEx_InjectedGetValue(&adc1, 2);
        cvnprintf(text, 127, 0, 24, "ADC2: %d", result);
        result = HAL_ADCEx_InjectedGetValue(&adc1, 3);
        cvnprintf(text, 127, 0, 32, "ADC3: %d", result);
        cvnprintf(text, 127, 0, 40, "tim: %d", tim.Instance->CNT);

        merge_canvas(text, background, background);
        flush_canvas(&i2c, text);
        HAL_Delay(5);
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (GPIO_PinState)!ret);
    while(true)
    {
        if (!ret)
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        HAL_Delay(500);
    }
}

void SysTick_Handler()
{
    HAL_IncTick();
}