#pragma once

#include <cstdint>
#include <stdexcept>
#include <chrono>
#include <string>
#include <vector>
#include <tuple>
#include <array>
#include "units.hpp"
#include "gpio.hpp"
#include "usart.hpp"
#include "clock.hpp"
#include "ffmt.hpp"

namespace vermils
{
namespace ecb
{
using namespace stm32;

namespace helper
{
    inline constexpr uint8_t parse_hex_digit(char c)
    {
        if (c >= '0' && c <= '9')
            return c - '0';
        else if (c >= 'a' && c <= 'f')
            return c - 'a' + 0xa;
        else if (c >= 'A' && c <= 'F')
            return c - 'A' + 0xa;
        else
            throw std::invalid_argument(
                ffmt::format("invalid hex character '{}'(0x{:.x})", c, (int)c));
    }

    inline constexpr int parse_int10(const std::string_view str)
    {
        int val = 0;
        for (auto c : str)
        {
            if (c >= '0' && c <= '9')
                val = val * 10 + c - '0';
            else
                throw std::invalid_argument("invalid integer");
        }
        return val;
    }
}

enum class Baudrate
{
    _2400,
    _9600,
    _19200,
    _115200,
};

enum class Role
{
    Central,
    Peripheral,
};

enum class RoleMode
{
    PinDepend,
    AlwaysCentral,
    AlwaysPeripheral,
};

enum class ATMode
{
    PinDepend,
    AlwaysValid,
    ValidOnlyDisconnected,
};

enum class SleepMode
{
    Forbidden,
    PinDepend,
};

enum class Power
{
    _N20dBm = 0,
    _N10dBm = 2,
    _N5dBm = 4,
    _0dBm = 6,
    _4dBm = 8,
};

enum class BroadcastInterval
{
    _50ms = 0,
    _100ms = 1,
    _200ms = 2,
    _500ms = 3,
    _1s = 4,
    _2s = 5,
};

struct MAC
{
    std::array<uint8_t, 6> addr;
    MAC() = default;
    MAC(const std::string_view str)
    {
        if (str.size() != 12)
            throw std::invalid_argument("invalid MAC address length");
        for (int i=str.size()-1; i>0; i-=2)
        {
            addr[i/2] = helper::parse_hex_digit(str[i-1]) << 4
                | helper::parse_hex_digit(str[i]);
        }
    }

    operator std::string() const
    {
        std::string str;
        for (auto b : addr)
        {
            str += ffmt::format("{:02.x}", b);
        }
        return str;
    }
    std::string to_str() const
    {
        return std::string(*this);
    }
    bool operator==(const MAC &other) const
    {
        return addr == other.addr;
    }
};

struct Device
{
    uint8_t order;
    std::string name;
    MAC mac;
    int signal;
};

using DevContainer = std::vector<Device>;

struct KeyValue
{
    std::string key;
    std::string value;
};

class BluetoothError : public std::runtime_error
{
public:
    using std::runtime_error::runtime_error;
};

class ATError : public BluetoothError
{
public:
    using BluetoothError::BluetoothError;
};

class ECB02S
{
    KeyValue _parse_kv(std::string_view str) const
    try{
        if (str.starts_with("+"))
            str = str.substr(1);
        auto pos = str.find_first_of(':');
        if (pos == std::string_view::npos)
            throw ATError("invalid key-value pair");
        return KeyValue{
            std::string(str.substr(0, pos)),
            std::string(str.substr(pos+1))
            };
    }
    catch (const std::out_of_range &e)
    {
        throw ATError("invalid key-value pair (out of range)");
    }

    void _at_send(std::string_view str) const
    {
        _at_start();
        uart.write(str);
        _at_end();
    }

    void _at_set(const std::string_view cmd, const std::string_view value) const
    {
        _at_start();
        uart.write("AT+");
        uart.write(cmd);
        uart.write("=");
        uart.write(value);
        uart.write("\r\n");
        _at_end();
        uart.read_line();  // read OK
    }

    bool _update_con_status(std::string_view resp) const
    {
        if (resp.starts_with("CONNECT OK"))
        {
            return true;
        }
        else if (resp.starts_with("CONNECT FAIL"))
        {
            return true;
        }
        else if (resp.starts_with("DISCONNECT"))
        {
            return true;
        }
        else if (resp.starts_with("SCANNING"))
        {
            return true;
        }
        return false;
    }

    void _at_start() const
    {
        at_pin.reset();
        clock::delay(20ms);
    }

    void _at_end() const
    {
        clock::delay(200ms);
        at_pin.set();
        clock::delay(20ms);
    }

    std::string _common_parse(std::string_view key) const
    try{
        _at_start();
        uart.write("AT+");
        uart.write(key);
        uart.write("?\r\n");
        _at_end();
        std::string resp = uart.read_line();
        while (resp == "OK"
            or resp == "ERROR"
            or _update_con_status(resp)
            or resp.starts_with("AT+")
            )
        {
            resp = uart.read_line();
        }
        auto kv = _parse_kv(resp);
        uart.read_line();  // read OK
        if (kv.key != key)
            throw ATError("invalid response");
        return kv.value;
    }
    catch(const clock::TimeoutError &e)
    {
        throw ATError("AT read timeout");
    }

public:
    usart::BaseUart &uart;
    gpio::Pin at_pin;  // AT command, active low
    gpio::Pin sleep_pin;  // sleep mode, active high

    ECB02S(usart::BaseUart &uart, gpio::Pin at_pin, gpio::Pin sleep_pin)
        : uart(uart), at_pin(at_pin), sleep_pin(sleep_pin)
    {
        gpio::PinConfig out_cfg(
            gpio::PinConfig::Output,
            gpio::PinConfig::VeryHigh,
            gpio::PinConfig::PushPull,
            0);
        at_pin.load(out_cfg);
        sleep_pin.load(out_cfg);
        sleep_pin.reset();
        at_pin.set();
        uart.set_baudrate(115200);
        uart.set_parity(usart::Parity::None);
        uart.set_stop_bits(usart::StopBits::One);
        uart.set_word_length(usart::WordLength::Bits8);
        uart.timeout_us = 1000000;
    }

    void sleep() const
    {
        sleep_pin.set();
    }

    bool is_sleeping() const
    {
        return sleep_pin.read();
    }

    void wakeup() const
    {
        sleep_pin.reset();
    }

    bool test() const
    {
        _at_send("AT\r\n");
        std::string resp = uart.read_line();
        if (resp.starts_with("AT"))
            resp = uart.read_line();  // incase of echo
        return resp == "OK";
    }

    void set_echo(bool enable) const
    {
        _at_set("ECHO", enable ? "1" : "0");
    }

    bool get_echo() const
    {
        auto value = _common_parse("ECHO");
        return value == "1";
    }

    std::string get_model() const
    {
        auto value = _common_parse("MODEL");
        return value;
    }

    std::string get_version() const
    {
        auto value = _common_parse("VER");
        return value;
    }

    void reset() const
    {
        _at_send("AT+RST\r\n");
        uart.read_line();  // read OK
    }

    Role get_role() const
    {
        auto value = _common_parse("ROLE");
        return value == "Peripheral" ? Role::Peripheral : Role::Central;
    }

    void set_role_mode(RoleMode mode) const
    {
        switch (mode)
        {
        case RoleMode::PinDepend:
            _at_set("ROLEMODE", "0");
            break;
        case RoleMode::AlwaysCentral:
            _at_set("ROLEMODE", "1");
            break;
        case RoleMode::AlwaysPeripheral:
            _at_set("ROLEMODE", "2");
            break;
        }
    }

    RoleMode get_role_mode() const
    {
        auto value = _common_parse("ROLEMODE");
        switch (value[0])
        {
        case '0':
            return RoleMode::PinDepend;
        case '1':
            return RoleMode::AlwaysCentral;
        case '2':
            return RoleMode::AlwaysPeripheral;
        default:
            throw std::runtime_error("invalid role mode");
        }
    }

    void set_at_mode(ATMode mode) const
    {
        switch (mode)
        {
        case ATMode::PinDepend:
            _at_set("MODE", "0");
            break;
        case ATMode::AlwaysValid:
            _at_set("MODE", "1");
            break;
        case ATMode::ValidOnlyDisconnected:
            _at_set("MODE", "2");
            break;
        }
    }

    ATMode get_at_mode() const
    {
        auto value = _common_parse("MODE");
        switch (value[0])
        {
        case '0':
            return ATMode::PinDepend;
        case '1':
            return ATMode::AlwaysValid;
        case '2':
            return ATMode::ValidOnlyDisconnected;
        default:
            throw std::runtime_error("invalid AT mode");
        }
    }

    void set_sleep_mode(SleepMode mode) const
    {
        switch (mode)
        {
        case SleepMode::Forbidden:
            _at_set("SLEEP", "0");
            break;
        case SleepMode::PinDepend:
            _at_set("SLEEP", "1");
            break;
        }
    }

    SleepMode get_sleep_mode() const
    {
        auto value = _common_parse("SLEEP");
        switch (value[0])
        {
        case '0':
            return SleepMode::Forbidden;
        case '1':
            return SleepMode::PinDepend;
        default:
            throw std::runtime_error("invalid sleep mode");
        }
    }

    bool get_led() const
    {
        auto value = _common_parse("LED");
        return value == "1";
    }

    void set_led(bool enable) const
    {
        _at_set("LED", enable ? "1" : "0");
    }

    bool get_watchdog() const
    {
        auto value = _common_parse("WDG");
        return value == "1";
    }

    void set_watchdog(bool enable) const
    {
        _at_set("WDG", enable ? "1" : "0");
    }

    bool get_con_notify() const
    {
        auto value = _common_parse("CONNOTIFY");
        return value == "1";
    }

    void set_con_notify(bool enable) const
    {
        _at_set("CONNOTIFY", enable ? "1" : "0");
    }

    void poweroff() const
    {
        _at_send("AT+POWEROFF\r\n");
        uart.read_line();  // read OK
    }

    void factory_reset() const
    {
        _at_send("AT+FACTORY\r\n");
        uart.read_line();  // read OK
    }

    bool is_at_enabled() const
    {
        auto value = _common_parse("BTAT");
        return value == "1";
    }

    uint32_t get_baudrate() const
    {
        auto value = _common_parse("UART");
        auto baud = helper::parse_int10(value);
        switch (baud)
        {
            case 0:
                return 2400;
            case 1: 
                return 9600;
            case 2:
                return 19200;
            case 3:
                return 115200;
            default:
                if (baud < 110)
                    throw std::runtime_error("invalid baudrate");
        }
        return baud;
    }

    void set_baudrate(uint32_t baud) const
    {
        uint32_t baud_code;
        switch (baud)
        {
            case 2400:
                baud_code = 0;
                break;
            case 9600:
                baud_code = 1;
                break;
            case 19200:
                baud_code = 2;
                break;
            case 115200:
                baud_code = 3;
                break;
            default:
                if (baud < 110)
                    throw std::runtime_error("invalid baudrate");
                baud_code = baud;
        }
        _at_set("UART", ffmt::format("{}", baud_code));
        uart.set_baudrate(baud);
    }

    void set_baudrate(Baudrate baudrate)
    {
        set_baudrate(static_cast<uint32_t>(baudrate));
    }

    size_t get_mtu() const
    {
        auto value = _common_parse("MTU");
        return helper::parse_int10(value);
    }

    bool is_ble_notify_enabled() const
    {
        auto value = _common_parse("BLENOTIFY");
        return value == "1";
    }

    void set_ble_notify(bool enable) const
    {
        _at_set("BLENOTIFY", enable ? "1" : "0");
    }

    bool is_connected() const
    {
        auto value = _common_parse("LINK");
        return value == "1";
    }

    void disconnect() const
    {
        _at_send("AT+DISC\r\n");
        uart.read_line();  // read OK
    }

    Power get_power() const
    {
        auto value = _common_parse("POWER");
        if (value.ends_with("db"))
            value = value.substr(0, value.size()-2);
        return static_cast<Power>(helper::parse_int10(value));
    }

    void set_power(Power power) const
    {
        int power_code = static_cast<int>(power);
        _at_set("POWER", ffmt::format("{}", power_code));
    }

    std::string get_service_uuid() const
    {
        auto value = _common_parse("SUUID");
        return value;
    }

    void set_service_uuid(const std::string_view uuid) const
    {
        _at_set("SUUID", uuid);
    }

    std::string get_read_uuid() const
    {
        auto value = _common_parse("RUUID");
        return value;
    }

    void set_read_uuid(const std::string_view uuid) const
    {
        _at_set("RUUID", uuid);
    }

    std::string get_write_uuid() const
    {
        auto value = _common_parse("WUUID");
        return value;
    }

    void set_write_uuid(const std::string_view uuid) const
    {
        _at_set("WUUID", uuid);
    }

    std::string get_name() const
    {
        auto value = _common_parse("NAME");
        return value;
    }

    void set_name(const std::string_view name) const
    {
        _at_set("NAME", name);
    }

    MAC get_mac() const
    {
        auto value = _common_parse("MAC");
        return MAC(value);
    }

    void set_mac(const MAC &mac) const
    {
        _at_set("MAC", std::string(mac));
    }

    uint32_t get_broadcast_interval_us() const
    {
        auto value = _common_parse("ADVINT");
        int interval = helper::parse_int10(value);
        switch (interval)
        {
        case 0:
            return 50000;
        case 1:
            return 100000;
        case 2:
            return 200000;
        case 3:
            return 500000;
        case 4:
            return 1000000;
        case 5:
            return 2000000;
        default:
            if (interval < 32)
                throw std::runtime_error("invalid broadcast interval");
        }
        return interval*625;
    }

    void set_broadcast_interval_us(uint32_t interval_us) const
    {
        uint32_t interval_code;
        switch (interval_us)
        {
        case 50000:
            interval_code = 0;
            break;
        case 100000:
            interval_code = 1;
            break;
        case 200000:
            interval_code = 2;
            break;
        case 500000:
            interval_code = 3;
            break;
        case 1000000:
            interval_code = 4;
            break;
        case 2000000:
            interval_code = 5;
            break;
        default:
            if (interval_us < 32)
                throw std::runtime_error("invalid broadcast interval");
            interval_code = interval_us / 625;
        }
        if (interval_code > 16384)
            throw std::runtime_error("invalid broadcast interval");
        _at_set("ADVINT", ffmt::format("{}", interval_code));
    }

    std::string get_broadcast_data() const
    {
        auto value = _common_parse("RESE");
        return value;
    }

    void set_broadcast_data(const std::string_view data) const
    {
        if (data.size() > 22)
            throw std::invalid_argument("broadcast data too long");
        _at_set("RESE", data);
    }

    void turn_off_broadcast() const
    {
        _at_send("AT+RESEOFF\r\n");
        uart.read_line();  // read OK
    }

    std::string get_password() const
    {
        auto value = _common_parse("PASSWORD");
        return value;
    }

    void set_password(const std::string_view password) const
    {
        _at_set("PASSWORD", password);
    }

    void clear_password() const
    {
        _at_send("AT+PASSWORDC\r\n");
        uart.read_line();  // read OK
    }


    /**
     * @brief Load low power configuration preset
     * 
     * @param enable 
     */
    void set_low_power(const bool enable) const
    {
        _at_set("CONPARAM", enable ? "1" : "0");
    }

    float get_con_interval_min_ms() const
    {
        auto value = _common_parse("CONINTMIN");
        return helper::parse_int10(value) * 1.25f;
    }

    void set_con_interval_min_ms(const float interval_ms) const
    {
        int interval = interval_ms / 1.25f;
        if (interval < 6 || interval > 3200)
            throw std::invalid_argument("invalid connection interval");
        _at_set("CONINTMIN", ffmt::format("{}", interval));
    }

    float get_con_interval_max_ms() const
    {
        auto value = _common_parse("CONINTMAX");
        return helper::parse_int10(value) * 1.25f;
    }

    void set_con_interval_max_ms(const float interval_ms) const
    {
        int interval = interval_ms / 1.25f;
        if (interval < 6 || interval > 3200)
            throw std::invalid_argument("invalid connection interval");
        _at_set("CONINTMAX", ffmt::format("{}", interval));
    }

    uint32_t get_latency() const
    {
        auto value = _common_parse("LATENCY");
        return helper::parse_int10(value);
    }

    void set_latency(const uint32_t latency) const
    {
        if (latency > 499)
            throw std::invalid_argument("invalid latency");
        _at_set("LATENCY", ffmt::format("{}", latency));
    }

    uint32_t get_timeout_ms() const
    {
        auto value = _common_parse("CONTIMEOUT");
        return helper::parse_int10(value) * 10;
    }

    void set_timeout_ms(const uint32_t timeout_ms) const
    {
        int timeout = timeout_ms / 10;
        if (timeout > 3200)
            throw std::invalid_argument("invalid timeout");
        _at_set("CONTIMEOUT", ffmt::format("{}", timeout));
    }

    size_t get_scan_max_n() const
    {
        auto value = _common_parse("SCANMAX");
        return helper::parse_int10(value);
    }

    void set_scan_max_n(const size_t max_n) const
    {
        if (max_n < 10 or max_n > 30)
            throw std::invalid_argument("invalid scan max number");
        _at_set("SCANMAX", ffmt::format("{}", max_n));
    }

    uint32_t get_scan_time_ms() const
    {
        auto value = _common_parse("SCANTIME");
        return helper::parse_int10(value);
    }

    void set_scan_time_ms(const uint32_t time_ms) const
    {
        if (time_ms < 500 or time_ms > 10000)
            throw std::invalid_argument("invalid scan time");
        _at_set("SCANTIME", ffmt::format("{}", time_ms));
    }

    DevContainer scan() const
    {
        _at_send("AT+SCAN\r\n");
        auto resp = uart.read_line();
        while (resp == "OK" or _update_con_status(resp) or resp.starts_with("AT+"))
            resp = uart.read_line();
        
        DevContainer devs;
        auto kv = _parse_kv(resp);
        int n = helper::parse_int10(kv.value);  // number of devices
        for (int i=0; i<n; ++i)
        {
            // demo: 0,Vermils,001122334455,-50
            Device dev;
            resp = uart.read_line();
            std::string_view str(resp);
            auto pos = str.find_first_of(',');
            if (pos == std::string_view::npos)
                throw std::runtime_error("invalid scan response");
            dev.order = helper::parse_int10(str.substr(0, pos));
            str = str.substr(pos+1);
            pos = str.find_first_of(',');
            if (pos == std::string_view::npos)
                throw std::runtime_error("invalid scan response");
            dev.name = std::string(str.substr(0, pos));
            str = str.substr(pos+1);
            pos = str.find_first_of(',');
            if (pos == std::string_view::npos)
                throw std::runtime_error("invalid scan response");
            dev.mac = MAC(str.substr(0, pos));
            str = str.substr(pos+1);
            dev.signal = helper::parse_int10(str);
            devs.push_back(dev);
        }
        uart.read_line();  // read empty line
    }
};

}
}
