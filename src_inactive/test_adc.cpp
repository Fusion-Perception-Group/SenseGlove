#include <string>
#include <tuple>
#include <chrono>
#include "mcu.hpp"
#include "units.hpp"
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
    using std::chrono::operator ""s;
    using std::chrono::operator ""ms;
    using std::chrono::operator ""ns;
    using namespace vms;
    using namespace stm32;
    using time::HighResTimer;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();

    Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);

    nvic::set_priority_group(nvic::Pre2_Sub2);

    try
    {
        auto adc_config = PinConfig(PinConfig::Analog, PinConfig::NoPull);
        Pin adc1_pin(PortA, 1, adc_config);
        Pin adc2_pin(PortA, 2, adc_config);
        Pin adc3_pin(PortA, 3, adc_config);
        Pin adc4_pin(PortA, 4, adc_config);
        auto &adc = adc::Adc1;
        adc.init();
        uint16_t buffer[4]={0};
        clock::delay(3us);  // Wait for ADC to stabilize
        // adc.set_sample_cycle(adc::SampleCycle::Cycles_56, 1);
        // adc.set_sample_cycle(adc::SampleCycle::Cycles_56, 2);
        // adc.set_sample_cycle(adc::SampleCycle::Cycles_56, 3);
        // adc.set_sample_cycle(adc::SampleCycle::Cycles_56, 4);
        // adc.enable_interrupts();
        // adc.on_injected_done = [&render, &adc, channel]()
        // {
        //     render.format_at(0, 0, "ADC: {}\n", adc.get_injected_data(channel));
        // };
        adc.set_scan_mode(true);
        // adc.set_injected_sequence(0, 1);
        // adc.set_injected_sequence(1, 2);
        // adc.set_injected_sequence(2, 3);
        // adc.set_injected_sequence(3, 4);
        adc.config_injected_sequence(
            std::tuple{1, adc::SampleCycle::Cycles_56},
            std::tuple{2, adc::SampleCycle::Cycles_56},
            std::tuple{3, adc::SampleCycle::Cycles_56, 2048},
            std::tuple{4, adc::SampleCycle::Cycles_56}
            );
        adc.config_regular_sequence(1, 2, 3, 4);
        adc.config_dma(buffer, dma::UnitSize::HalfWord, true);
        adc.set_dma_mode(true);
        adc.set_continuous(true);
        adc.start_regular();
        while (true)
        {
            adc.start_injected();
            while (not adc.is_injected_done());
            adc.clear_injected_done();
            render.format_at(0, 0, "ADC: {:5}\n", adc.get_injected_data(0));
            render.format_at(1, 0, "ADC: {:5}\n", adc.get_injected_data(1));
            render.format_at(2, 0, "ADC: {:5}\n", adc.get_injected_data(2));
            render.format_at(3, 0, "ADC: {:5}\n", adc.get_injected_data(3));
            // render.format_at(4, 0, "ADC: {:5}\n", adc.get_regular_data());
            render.format_at(4, 0, "Regu 1: {:5}\n", buffer[0]);
            render.format_at(5, 0, "Regu 2: {:5}\n", buffer[1]);
            render.format_at(6, 0, "Regu 3: {:5}\n", buffer[2]);
            render.format_at(7, 0, "Regu 4: {:5}\n", buffer[3]);
            adc.raise_if_error();
        }
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
    }
}
