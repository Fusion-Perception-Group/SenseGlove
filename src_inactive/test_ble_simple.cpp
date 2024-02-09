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
    using namespace vms;
    using namespace stm32;
    using gpio::Pin;
    using gpio::PinConfig;
    using namespace gpio::ports;

    mcu::init();

    gpio::Switch led(LED_GPIO_PORT, LED_PIN, true);

    clock::delay(100ms);

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
        if (not ble.test())
        {
            throw std::runtime_error("BLE test failed");
        }
        // ble.set_echo(false);
        ble.set_name("SenseGloveTest");
        ble.set_con_notify(true);
        ble.clear_password();

        ble.uart.write("WTF????\r\n");

        // clock::delay(100ms);
        // render.format_at(3, 0, "cnt: {}", cnt);
        // auto tmp = ble.uart.read_line();
        // ble.uart.write("AT+ROLEMODE?\r\n");
        // render.format("echo : {}, ", ble.get_echo());
        // render.format("role : {}, ", (int)ble.get_role());
        // render.render(ble.uart.read_line());
        // led.set(ble.test());


        while (true)
        {
            led.toggle();
            clock::delay(500ms);
        }
    }
    catch (const std::exception &e)
    {
        while (true)
        {
            led.toggle();
            clock::delay(50ms);
        }
    }
    catch (...)
    {
        while (true)
        {
            led.toggle();
            clock::delay(1000ms);
        }
    }
}
