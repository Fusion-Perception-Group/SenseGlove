#include <stdexcept>
#include <string>
#include <tuple>
#include <array>
#include <vector>
#include <chrono>
#include "ffmt.hpp"
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1680.hpp"
#include "texrender.hpp"
#include "w25qxx.hpp"
#include "wit.hpp"
#include "glaze/glaze.hpp"
#include "directy.hpp"
#include "gesture.hpp"
#include "ecb02.hpp"

using namespace vermils;
using namespace stm32;
using namespace gpio::ports;
using directy::PX_MAX;
using directy::PX_MIN;
using directy::TextBoxNode;

auto & init_pwm()
{
    gpio::PinConfig pwm_config(
            gpio::PinConfig::AF,
            gpio::PinConfig::High,
            gpio::PinConfig::PushPull,
            2  // TIM3 Alternate Function
        );

    gpio::Pin pwm_pin1(gpio::PortB, 0, pwm_config);
    gpio::Pin pwm_pin2(gpio::PortB, 1, pwm_config);

    auto &pwm_tim = clock::tim::Tim3;
    pwm_tim.init();

    const double duty_ratio = 0.3;
    const uint32_t pwm_freq = 170;

    pwm_tim.channel3.enable();
    pwm_tim.channel4.enable();

    pwm_tim.channel3.set_pwm(pwm_freq, duty_ratio, true);
    pwm_tim.channel4.set_pwm(pwm_freq, duty_ratio, false);

    return pwm_tim;
}

auto & init_usart()
{
    gpio::PinConfig usart_cfg(
        gpio::PinConfig::AF,
        gpio::PinConfig::VeryHigh,
        gpio::PinConfig::PushPull,
        7  // USART1 Alternate Function
    );
    gpio::Pin tx(gpio::PortA, 9, usart_cfg), rx(gpio::PortA, 10, usart_cfg);
    auto &usart = usart::Usart1;
    usart.init();
    usart.enable_buffer(512, 512);
    return usart;
}

auto & init_spi()
{
    gpio::Pin sclk(gpio::PortA, 5), miso(gpio::PortA, 6), mosi(gpio::PortA, 7);
    gpio::PinConfig spicfg(
        gpio::PinConfig::AF,
        gpio::PinConfig::VeryHigh,
        gpio::PinConfig::PushPull,
        5);  // SPI1 Alternate Function
    spicfg.pull = gpio::PinConfig::NoPull;
    sclk.load(spicfg);
    miso.load(spicfg);
    mosi.load(spicfg);
    clock::delay(1000ms);
    auto &spi = spi::Spi1;
    spi.init();
    return spi;
}

auto & init_flex_adc(volatile int16_t flex_buffer[])
{
    auto adc_config = gpio::PinConfig(
        gpio::PinConfig::Analog, gpio::PinConfig::NoPull);
    gpio::Pin adc1_pin(PortA, 1, adc_config);
    gpio::Pin adc2_pin(PortA, 2, adc_config);
    gpio::Pin adc3_pin(PortA, 3, adc_config);
    auto &adc = adc::Adc1;
    adc.init();
    clock::delay(3us);  // Wait for ADC to stabilize
    adc.set_scan_mode(true);
    adc.config_regular_sequence(
        std::tuple{1, adc::SampleCycle::Cycles_56},
        std::tuple{2, adc::SampleCycle::Cycles_56},
        std::tuple{3, adc::SampleCycle::Cycles_56}
        );
    adc.config_dma(flex_buffer, dma::UnitSize::HalfWord, true);
    adc.set_dma_mode(true);
    adc.set_continuous(true);
    return adc;
}

struct Quaternion
{
    float w, x, y, z;
};

struct Report
{
    // uint8_t thumb_status;
    // uint8_t index_status;
    // uint8_t middle_status;
    int16_t thumb_curvature;
    int16_t index_curvature;
    int16_t middle_curvature;
    bool pinched;
    bool wearing;
    uint64_t timestamp;
    Quaternion quaternion;
};

int main()
{
    mcu::init();
    auto wdg = wdg::IndependentWatchDog();
    wdg.set_max_hungry_time(1000ms);
    // wdg.start();

    auto &usart = init_usart();
    auto &spi = init_spi();

    clock::delay(200ms);  // wait for peripheral to stabilize

    gpio::Pin cs_pin(PortB, 10), dc_pin(PortB, 2), busy_pin(PortB, 8);
    ssd1680::Display<250, 122> screen(spi, cs_pin, dc_pin, busy_pin);
    auto &canvas = screen.bw;
    auto &text = canvas.create_child<TextBoxNode>();

    try
    {
        // ############################ Setup ############################
        gpio::Switch led(PortC, 13, true);
        gpio::Switch motor0(PortB, 0), motor1(PortB, 1);
        gpio::Button cap_sensor(PortA, 0), pinch(PortB, 9, true);

        auto i2c = i2c::SoftMaster(
            gpio::Pin(PortB, 7), gpio::Pin(PortB, 6));

        // auto &pwm_tim = init_pwm();
        // // pwm_tim.start();

        volatile int16_t flex_buffer[3]={0};
        auto &adc = init_flex_adc(flex_buffer);
        gesture::Hand hand;
        hand.thumb().curvature = flex_buffer;
        hand.index().curvature = flex_buffer + 1;
        hand.middle().curvature = flex_buffer + 2;
        adc.start_regular();

        gpio::Pin at_pin(PortB, 13), sleep_pin(PortB, 12);
        auto ble = ecb::ECB02S(usart, at_pin, sleep_pin);
        // ble.set_echo(false);
        // ble.set_con_notify(false);
        // ble.set_name("SenseGlove");

        gpio::Pin cs(gpio::PortA, 4);
        auto flash = w25qxx::Flash(spi, cs);

        auto posensor = wit::I2CSensor(i2c);
        posensor.set_report_rate(wit::ReportRate::None);

        // ############################ Main Loop ############################
        

        // text.content = ffmt::format(
        //     "Bluetooth MAC: {}\n"
        //     "Bluetooth Name: {}\n"
        //     ,
        //     ble.get_mac(),
        //     ble.get_name()
        // );
        // canvas.draw();
        // screen.flush(true);

        uint64_t time_ms = clock::get_systick_ms();

        while (true)
        {
            auto checker = clock::make_timeout(100ms);
            wdg.feed();
            uint64_t delta_ms = clock::get_systick_ms() - time_ms;
            time_ms += delta_ms;

            auto quat = posensor.get_quaternion().normalize<float>();
            // auto thumb_status = static_cast<uint8_t>(hand.thumb().get_status());
            // auto index_status = static_cast<uint8_t>(hand.index().get_status());
            // auto middle_status = static_cast<uint8_t>(hand.middle().get_status());

            motor0.set(pinch.pressed());
            led.set(cap_sensor.pressed());

            Report report{
                // .thumb_status = thumb_status,
                // .index_status = index_status,
                // .middle_status = middle_status,
                .thumb_curvature = *hand.thumb().curvature,
                .index_curvature = *hand.index().curvature,
                .middle_curvature = *hand.middle().curvature,
                .pinched = pinch.pressed(),
                .wearing = cap_sensor.pressed(),
                .timestamp = time_ms,
                .quaternion = {
                    .w = quat.w,
                    .x = quat.x,
                    .y = quat.y,
                    .z = quat.z
                }
            };

            std::string report_str = glz::write_json(report);

            // if (ble.is_connected())
            // {
                ble.uart.write(report_str);
                ble.uart.write("\n");
            // }

            // power::sleep();
            while(not checker.has_timedout());
            // clock::delay(100ms);
        }
    }
    catch (const std::exception &e)
    {
        wdg.feed();
        text.content = ffmt::format("Exception: {}\n", e.what());
        usart.write(ffmt::format("Exception: {}\n", e.what()));
        canvas.clear();
        canvas.draw();
        screen.flush(true);
    }
    catch (...)
    {
        wdg.feed();
        usart.write("Unknown exception\n");
    }

    clock::rcc::reset_system();

    return 0;
}