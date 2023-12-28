#include <string>
#include <chrono>
#include "ffmt.hpp"
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "w25qxx.hpp"
#include "wit.hpp"
#include "glaze/glaze.hpp"
#include "directy.hpp"

using namespace vermils;
using namespace stm32;

int main()
{
    mcu::init();
    auto wdg = wdg::IndependentWatchDog();
    wdg.set_max_hungry_time(1000ms);
    
    gpio::PinConfig usart_cfg(
        gpio::PinConfig::AF,
        gpio::PinConfig::VeryHigh,
        gpio::PinConfig::PushPull,
        7  // USART1 Alternate Function
    );
    gpio::Pin tx(gpio::PortA, 9, usart_cfg), rx(gpio::PortA, 10, usart_cfg);
    auto &usart = usart::Usart1;
    // usart.init();
    // usart.write("Start!\n");

    clock::delay(100ms);  // screen needs wait for some time before init
    auto i2c = i2c::SoftMaster(gpio::Pin(gpio::PortB, 7), gpio::Pin(gpio::PortB, 6));
    auto screen = ssd1306::I2CDisplay(i2c);
    screen.clear();
    auto render = ssd1306::TexRender(screen);
    
    try
    {
        // ############################ Setup ############################
        gpio::PinConfig pwm_config(
            gpio::PinConfig::AF,
            gpio::PinConfig::High,
            gpio::PinConfig::PushPull,
            2  // TIM3 Alternate Function
        );

        gpio::Pin pwm_pin1(gpio::PortB, 0, pwm_config);
        gpio::Pin pwm_pin2(gpio::PortB, 1, pwm_config);

        auto pwm_tim = clock::tim::Tim3;
        pwm_tim.init();

        const double duty_ratio = 0.3;
        const uint32_t pwm_freq = 170;

        pwm_tim.channel3.enable();
        pwm_tim.channel4.enable();

        pwm_tim.channel3.set_pwm(pwm_freq, duty_ratio, true);
        pwm_tim.channel4.set_pwm(pwm_freq, duty_ratio, false);

        pwm_tim.start();

        auto adc_config = gpio::PinConfig(gpio::PinConfig::Analog, gpio::PinConfig::NoPull);
        gpio::Pin adc1_pin(gpio::PortA, 1, adc_config);
        gpio::Pin adc2_pin(gpio::PortA, 2, adc_config);
        gpio::Pin adc3_pin(gpio::PortA, 3, adc_config);
        gpio::Pin adc4_pin(gpio::PortA, 4, adc_config);
        auto &adc = adc::Adc1;
        adc.init();
        uint16_t flex_buffer[4]={0};
        clock::delay(3us);  // Wait for ADC to stabilize
        adc.set_scan_mode(true);
        adc.config_regular_sequence(
            std::tuple{1, adc::SampleCycle::Cycles_56},
            std::tuple{2, adc::SampleCycle::Cycles_56},
            std::tuple{3, adc::SampleCycle::Cycles_56},
            std::tuple{4, adc::SampleCycle::Cycles_56}
            );
        adc.config_dma(flex_buffer, dma::UnitSize::HalfWord, true);
        adc.set_dma_mode(true);
        adc.set_continuous(true);
        adc.start_regular();

        gpio::PinConfig spi_cfg(
            gpio::PinConfig::AF,
            gpio::PinConfig::VeryHigh,
            gpio::PinConfig::PushPull,
            5  // SPI1 Alternate Function
        );
        spi_cfg.pull = gpio::PinConfig::NoPull;
        gpio::Pin sck(gpio::PortA, 5, spi_cfg),
            miso(gpio::PortA, 6, spi_cfg),
            mosi(gpio::PortA, 7, spi_cfg),
            cs(gpio::PortA, 4);
        auto &spi = spi::Spi1;
        spi.init();
        auto flash = w25qxx::Flash(spi, cs);

        auto posensor = wit::I2CSensor(i2c);
        posensor.set_report_rate(wit::ReportRate::None);

        // ############################ Main Loop ############################
        
        while (true)
        {
            render.format_at(0, 0, "counter {:06}", pwm_tim.counter);
            render.format_at(2, 0, "ADC: {:04} {:04} {:04} {:04}", flex_buffer[0], flex_buffer[1], flex_buffer[2], flex_buffer[3]);
            clock::delay(50ms);
        }
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
        usart.write(ffmt::format("Exception: {}\n", e.what()));
    }
    catch (...)
    {
        usart.write("Unknown exception\n");
    }

    clock::rcc::reset_system();

    return 0;
}