#include <string>
#include <tuple>
#include <chrono>
#include "mcu.hpp"
#include "units.hpp"
#include "ssd1306.hpp"
#include "texrender.hpp"
#include "ffmt.hpp"
#include "wit.hpp"

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

    try
    {
        auto sensor = wit::I2CSensor(i2c);
        sensor.set_report_rate(wit::ReportRate::None);
        sensor.set_led(true);

        render.format_at(0, 0, "Test started");

        while (true)
        {
            // auto raw_acc = sensor.get_accel();
            // auto acc = raw_acc.rescale<double>();
            // render.format_at(1, 0, "Accel: \n{:>+16.6}\n{:>+16.6}\n{:>+16.6})", acc.x, acc.y, acc.z);

            // auto raw_gyro = sensor.get_gyro();
            // auto gyro = raw_gyro.rescale<double>();
            // render.format_at(1, 0, "Gyro: \n{:>+16.6}\n{:>+16.6}\n{:>+16.6})", gyro.x, gyro.y, gyro.z);

            // auto raw_mag = sensor.get_magnet();
            // auto mag = raw_mag.rescale<double>();
            // render.format_at(1, 0, "Magnet: \n{:+.6}\n{:+.6}\n{:+.6})", mag.x, mag.y, mag.z);

            auto raw_quat = sensor.get_quaternion();
            auto quat = raw_quat.rescale<double>();
            render.format_at(1, 0, "Quat: \n{:+3.6}\n{:+3.6}\n{:+3.6}\n{:+3.6})", quat.w, quat.x, quat.y, quat.z);

            // auto raw_euler = sensor.get_euler();
            // auto euler = raw_euler.rescale<double>();
            // render.format_at(1, 0, "Euler: \n{:+3.6}\n{:+3.6}\n{:+3.6})", euler.roll, euler.pitch, euler.yaw);
        }
    }
    catch (const std::exception &e)
    {
        render.format_at(0, 0, "Exception: {}\n", e.what());
    }
}
