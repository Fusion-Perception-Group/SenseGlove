#pragma once

#include <cstdint>
#include <stdexcept>
#include <chrono>
#include "units.hpp"
#include "gpio.hpp"
#include "spi.hpp"
#include "clock.hpp"

namespace vermils
{
namespace ssd1680
{
using namespace stm32;

enum class Command : uint8_t
{
    DriverOutputControl = 0x01, // A7-0[7:0], A8[0], B2-0[2:0], A: MUX lines = (A+1)
    GateDrivingVoltageControl = 0x03, // A4-0[4:0], VGH setting
    SourceDrivingVoltageControl = 0x04, // A7-0[7:0], B7-0[7:0], C7-0[7:0], VSH/VSL setting
    InitialCodeSettingOTPProgram = 0x08,
    WriteRegisterForInitialCodeSetting = 0x09, //A, B, C, D, reserved
    ReadRegisterForInitialCodeSetting = 0x0A,
    BoosterSoftStartControl = 0x0C, // A7-0[7:0], B7-0[7:0], C7-0[7:0], D7-0[7:0], Booster for phases 1-3
    DeepSleepMode = 0x10, // A1-0[1:0], normal=00, mode1=01, mode2=11
    DataEntryModeSetting = 0x11, // A2-0[2:0], A[1:0]: X/Y increment/decrement#, A[2]: addr counter increase direction y/x#
    SwReset = 0x12,
    HVReadyDetection = 0x14, // A6-4,2-0[6:0]
    VCIDetection = 0x15,
    TemperatureSensorControl = 0x18, // A
    WriteTemperatureSensorRegister = 0x1A, // A11-4[7:0], A3-0[7:4]
    ReadTemperatureSensorRegister = 0x1B, // A11-4[7:0], A3-0[7:4]
    WriteCMDTemperatureSensor = 0x1C, // A, B, C
    MasterActivation = 0x20,
    DisplayUpdateControl1 = 0x21, // A7-0[7:0], B7[7], (A[7-4]:Red RAM)/(A[3:0]:BW RAM):normal=0x00, by pass RAM=0x4, inverse RAM=0x8, B available source s8-167/s0-175#
    DisplayUpdateControl2 = 0x22, // A7-0[7:0], [7]:enable clock, [6]:enable analog, [5]: load temperature, [4]: load LUT, [3]:use display mode 2, [2]:disable OSC, [1]:disable analog , [0]: disable clock
    WriteRAMBW = 0x24, // ...
    WriteRAMRed = 0x26, // ...
    ReadRAM = 0x27,
    VCOMSense = 0x28,
    VCOMSenseDuration = 0x29,
    ProgramVCOMOTP = 0x2A,
    WriteVCOMControlRegister = 0x2B, // 0x04, 0x63, key
    WriteVCOMRegister = 0x2C, // A
    OTPRegisterRead = 0x2D, // A, B, C, D, E, F, G, H, I, J, K
    UserIDRead = 0x2E, // A, B, C, D, E, F, G, H, I, J
    StatusRegisterRead = 0x2F, // A5-4,1-0[5:0]
    ProgramWSOTP = 0x30,
    LoadWSOTP = 0x31,
    WriteLUTRegister = 0x32, // ...
    CRCCalculation = 0x34,
    CRCStatusRead = 0x35,
    ProgramOTPSelection = 0x36,
    WriteDisplayOption = 0x37, // A, B, C, D, E, F, G, H, I, J
    WriteUserID = 0x38, // A, B, C, D, E, F, G, H, I, J
    ProgramOTPMode = 0x39, // A1-0[1:0]
    BorderWaveformControl = 0x3C, // A7-4, 2-0[7:0]
    EndOption = 0x3F, // A
    ReadRAMOption = 0x41, // A0[0]
    SetRAMXStartEndPos = 0x44, // A5-0[5:0], B5-0[5:0]
    SetRAMYStartEndPos = 0x45, // A7-0[7:0], A8[0], B7-0[7:0], B8[0]
    AutoWriteRedRAMForRegularPattern = 0x46, // A7-4, 2-0[7:0]
    AutoWriteBWRAMForRegularPattern = 0x47, // A7-4, 2-0[7:0]
    SetRAMXAddressCounter = 0x4E, // A5-0[5:0]
    SetRAMYAddressCounter = 0x4F, // A7-0[7:0], A8[0]
    NOP = 0x7F

};

inline constexpr const uint8_t lut_partial[] =
{
  0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x40, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x22, 0x22, 0x22, 0x22, 0x22, 0x22, 0x00, 0x00, 0x00,
};

/**
 * @brief Driver for SSD1680
 * 
 * @param SPI
 * @param CS
 * @param DC
 * @param BUSY
 */
class Driver
{
public:
    spi::BaseInterface &spi;
    gpio::Pin cs_pin;
    gpio::Pin dc_pin;  // data/command#
    gpio::Pin busy_pin;

    Driver(spi::BaseInterface &spi, gpio::Pin cs_pin, gpio::Pin dc_pin, gpio::Pin busy_pin)
        : spi(spi), cs_pin(cs_pin), dc_pin(dc_pin), busy_pin(busy_pin)
    {
        gpio::PinConfig out_cfg(
            gpio::PinConfig::Output,
            gpio::PinConfig::Low,
            gpio::PinConfig::PushPull,
            0);
        cs_pin.load(out_cfg);
        dc_pin.load(out_cfg);
        busy_pin.load(gpio::PinConfig(
            gpio::PinConfig::Input,
            gpio::PinConfig::PullDown)
            );
        
        deselect();
        dc_pin.set();
    }

    void select() const
    {
        cs_pin.reset();
    }

    void deselect() const
    {
        cs_pin.set();
    }

    template <typename... ARGS>
    void send_command(Command cmd, ARGS... data) const
    {
        wait_busy();
        dc_pin.reset();
        select();
        spi.put(static_cast<uint8_t>(cmd));
        dc_pin.set();
        deselect();
        send_data(data...);
    }

    template <typename... ARGS>
    void send_data(ARGS... data) const
    {
        std::array<uint8_t, sizeof...(ARGS)> buf = {
            static_cast<uint8_t>(data)...}
            ;
        wait_busy();
        dc_pin.set();
        select();
        spi.write_bytes(buf.data(), buf.size());
        deselect();
    }

    void init() const
    {
        sw_reset();
        clock::delay(10ms);
        send_command(Command::DriverOutputControl, 0x27, 0x01, 0x00);
        send_command(Command::DataEntryModeSetting, 0x07);
        // send_command(Command::WriteVCOMRegister, 0x36);
        // send_command(Command::GateDrivingVoltageControl, 0x17);
        // send_command(Command::SourceDrivingVoltageControl, 0x41, 0x00, 0x32);
        send_command(Command::BoosterSoftStartControl, 0xff, 0xff, 0xff, 0x00);
        send_command(Command::SetRAMXStartEndPos, 0x00, 0x0f);
        send_command(Command::SetRAMYStartEndPos, 0x00, 0x00, 0x27, 0x01);
        send_command(Command::BorderWaveformControl, 0x05);
        send_command(Command::DisplayUpdateControl1, 0x00, 0x80);
        send_command(Command::TemperatureSensorControl, 0x80);
        set_cursor(0, 0);
        power_on();

        // send_command(Command::WriteLUTRegister);
        // select();
        // spi.write_bytes(lut_partial, sizeof(lut_partial));
        // deselect();
    }

    void send_red_data(uint8_t data) const
    {
        send_command(Command::WriteRAMRed, data);
    }

    void send_bw_data(uint8_t data) const
    {
        send_command(Command::WriteRAMBW, data);
    }

    bool is_busy() const
    {
        return busy_pin.read();
    }

    void wait_busy(std::chrono::milliseconds timeout = 50000ms) const
    {
        auto checker = clock::make_timeout(timeout);
        while (is_busy())
            checker.raise_if_timedout();
    }

    void sw_reset() const
    {
        send_command(Command::SwReset);
    }

    void set_cursor(uint16_t x, uint16_t y) const
    {
        send_command(Command::SetRAMXAddressCounter, x);
        send_command(Command::SetRAMYAddressCounter, y&0xff, y>>8);
    }

    void full_update() const
    {
        send_command(Command::DisplayUpdateControl2, 0xf7);
        send_command(Command::MasterActivation);
        wait_busy();
    }

    void update() const
    {
        send_command(Command::DisplayUpdateControl2, 0xcc);
        send_command(Command::MasterActivation);
        wait_busy();
    }

    void power_on() const
    {
        send_command(Command::DisplayUpdateControl2, 0xf8);
        send_command(Command::MasterActivation);
        wait_busy();
    }
};

}
}
