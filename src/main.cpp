#include <string>
#include <chrono>
#include "ffmt.hpp"
#include "mcu.hpp"

using namespace vermils;
using namespace stm32;

int main()
{
    mcu::init();
    auto wdg = wdg::IndependentWatchDog();

    gpio::PinConfig pwm_config(
        gpio::PinConfig::Output,
        gpio::PinConfig::VeryHigh,
        gpio::PinConfig::PushPull,
        2  // TIM3 Alternate Function
    );

    gpio::Pin pwm_pin1(gpio::PortB, 0, pwm_config);
    gpio::Pin pwm_pin2(gpio::PortB, 1, pwm_config);

    auto pwm_tim = clock::tim::Tim3;
    pwm_tim.init();

    while (true);

    return 0;
}