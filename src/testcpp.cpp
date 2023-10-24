#include "CLK_CFG.h"
#include "DWT_Delay.h"
#include "MCU.hpp"
#include <string>

#define LED_PIN 13
#define LED_GPIO_PORT GPIOC

#define BUTTON_PIN GPIO_PIN_0
#define BUTTON_GPIO_PORT GPIOB

void LED_Init();
void BUTTON_Init();

using std::string;

int main()
{
    HAL_Init();
    SystemClock_Config();
    // MX_RTC_Init();
    //LED_Init();
    BUTTON_Init();
    DWT_delay_init();

    vermils::stm32::GPIOPinConfig config(
        vermils::stm32::GPIOPinConfig::IO::Output,
        vermils::stm32::GPIOPinConfig::Pull::NoPull,
        vermils::stm32::GPIOPinConfig::Speed::High,
        vermils::stm32::GPIOPinConfig::OutMode::PushPull,
        uint8_t(0),
        vermils::stm32::GPIOPinConfig::EXTIMode::NoEXTI,
        vermils::stm32::GPIOPinConfig::Trigger::NoTrigger
        );
    vermils::stm32::GPIOPin Led(LED_GPIO_PORT, LED_PIN, config);

    while (true)
    {
        Led.toggle();
        //HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
        // LED_GPIO_PORT->ODR ^= LED_PIN;
        HAL_Delay(1000);
    }
}

void LED_Init()
{
    //enable_GPIO_CLK(LED_GPIO_PORT);
    //init_GPIO(
    //    LED_GPIO_PORT,
    //    LED_PIN,
    //    GPIO_MODE_OUTPUT_PP,
    //    GPIO_PULLUP,
    //    GPIO_SPEED_HIGH);
    vermils::stm32::GPIOPinConfig config(
        vermils::stm32::GPIOPinConfig::IO::Output,
        vermils::stm32::GPIOPinConfig::Pull::NoPull,
        vermils::stm32::GPIOPinConfig::Speed::High,
        vermils::stm32::GPIOPinConfig::OutMode::PushPull,
        uint8_t(0),
        vermils::stm32::GPIOPinConfig::EXTIMode::NoEXTI,
        vermils::stm32::GPIOPinConfig::Trigger::NoTrigger
        );
    vermils::stm32::GPIOPin Led(LED_GPIO_PORT, LED_PIN, config);
}

void BUTTON_Init()
{
    //enable_GPIO_CLK(BUTTON_GPIO_PORT);
    //init_GPIO(
    //    BUTTON_GPIO_PORT,
    //    BUTTON_PIN,
    //    GPIO_MODE_INPUT,
    //    GPIO_PULLUP,
    //    GPIO_SPEED_HIGH);
}

extern "C" void SysTick_Handler()
{
    HAL_IncTick();
}