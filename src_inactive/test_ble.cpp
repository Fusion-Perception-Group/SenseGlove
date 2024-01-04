#include <string>
#include <tuple>
#include <chrono>
#include "mcu.hpp"
#include "units.hpp"
#include "ffmt.hpp"
#include "ecb02.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ssd1306_canvas.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT PortC

using std::string;

int main()
{
    using namespace vermils;
    using namespace stm32;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();

    gpio::Switch led(LED_GPIO_PORT, LED_PIN, true);

     Pin scl(PortB, 6), sda(PortB, 7);
    i2c::SoftMaster i2c(sda, scl);
    // PinConfig hw_i2c_cfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::OpenDrain, 4);
    // hw_i2c_cfg.pull = PinConfig::PullUp;
    // Pin scl(PortB, 6, hw_i2c_cfg), sda(PortB, 7, hw_i2c_cfg);
    // auto &i2c = i2c::I2c1;
    // i2c.init();
    clock::delay(100ms);
    ssd1306::I2CDisplay display(i2c);
    display.clear();

    ssd1306::TexRender render(display);
    // render.render("START!");

    Pin tx(PortA, 9), rx(PortA, 10);
    PinConfig cfg(PinConfig::AF, PinConfig::VeryHigh, PinConfig::PushPull, 7);
    tx.load(cfg);
    rx.load(cfg);

    auto &usrt = usart::Usart1;
    usrt.init();
    usrt.enable_buffer();

    //usrt.baudrate = 230400;
    // usrt.set_word_length(usart::WordLength::Bits9);
    // usrt.set_stop_bits(usart::StopBits::Two);
    // usrt.set_parity(usart::Parity::Even);

    gpio::Pin at_pin(PortB, 13), sleep_pin(PortB, 12);
    auto ble = ecb::ECB02S(usrt, at_pin, sleep_pin);

    try
    {
        // at_pin.reset();
        ble.set_name("SenseGloveTest");
        ble.set_con_notify(true);
        ble.clear_password();
        // ble.uart.write("AT+MAC?\r\n");
        // at_pin.set();
        render.format("mac : {}, ", ble.get_mac());
        // render.format("name : {}, ", ble.uart.read_line());
        render.format("name : {}. ", ble.get_name());
        ble.uart.timeout_us = 0;
        while (true)
        {
            auto line = ble.uart.read_line();
            render.render(line);
            // ble.uart.write(line);
        }
        // clock::delay(100ms);
        // render.format_at(3, 0, "cnt: {}", cnt);
        // auto tmp = ble.uart.read_line();
        // ble.uart.write("AT+ROLEMODE?\r\n");
        // render.format("echo : {}, ", ble.get_echo());
        // render.format("role : {}, ", (int)ble.get_role());
        // render.render(ble.uart.read_line());
        // led.set(ble.test());

        render.render("END!");

        while (true)
        {
            led.toggle();
            clock::delay(500ms);
        }
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "EXC: {}", e.what());
    }
    catch (...)
    {
        render.format_at(0, 0, "UNKNOWN EXCEPTION");
    }
}
