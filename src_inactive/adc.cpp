#include <string>
#include <chrono>
#include "CLK_CFG.h"
#include "time.hpp"
#include "nvic.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "adc.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    HAL_Init();
    SystemClock_Config();

    using std::chrono::operator ""s;
    using std::chrono::operator ""ms;
    using std::chrono::operator ""ns;
    using namespace vermils;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    HighResTimer timer;

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    timer.delay_ms(100);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    try
    {
        Pin adc_pin(PortA, 3, PinConfig(PinConfig::Analog, PinConfig::NoPull));
        int channel = 3;
        auto &adc = adc::Adc1;
        adc.init();
        timer.delay_us(3);  // Wait for ADC to stabilize
        adc.set_sample_cycle(adc::SampleCycle::Cycles_56, channel);
        adc.enable_interrupts();
        adc.enable_irq();
        // adc.on_injected_done = [&render, &adc, channel]()
        // {
        //     render.format_at(0, 0, "ADC: {}\n", adc.get_injected_data(channel));
        // };
        adc.set_scan_mode(true);
        adc.set_injected_sequence(0, channel);
        adc.set_injected_sequence(1, channel);
        adc.set_injected_sequence(2, channel);
        adc.set_injected_sequence(3, channel);
        adc.set_regular_sequence(0, channel);
        while (true)
        {
            adc.start_injected();
            adc.start_regular();
            while (adc.is_regular_running());
            //while (adc.is_injected_running());
            render.format_at(0, 0, "ADC: {}\n", adc.get_injected_data(0));
            render.format_at(1, 0, "ADC: {}\n", adc.get_injected_data(1));
            render.format_at(2, 0, "ADC: {}\n", adc.get_injected_data(2));
            render.format_at(3, 0, "ADC: {}\n", adc.get_injected_data(3));
            render.format_at(4, 0, "ADC: {}\n", adc.reg.DR);
        }
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
    }
}

extern "C" void SysTick_Handler()
{
    HAL_IncTick();
}