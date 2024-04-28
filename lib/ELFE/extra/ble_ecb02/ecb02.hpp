#pragma once

#include "clock/clock.hpp"
#include "gpio/gpio.hpp"
#include "result.hpp"
#include "units.hpp"
#include "usart/usart.hpp"
#include "utils/ffmt.hpp"
#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

namespace elfe {
namespace ecb {
    using namespace stm32;
    using namespace err::extra::ecb;
    using EC = err::ErrorCode;

    namespace helper {
        inline constexpr Result<uint8_t> parse_hex_digit(char c)
        {
            if (c >= '0' && c <= '9')
                return c - '0';
            else if (c >= 'a' && c <= 'f')
                return c - 'a' + 0xa;
            else if (c >= 'A' && c <= 'F')
                return c - 'A' + 0xa;
            else
                ELFE_ERROR(Result<uint8_t>(0, EC::InvalidArgument),
                    std::invalid_argument(
                        ffmt::format("invalid hex character '{}'(0x{:.x})", c, (int)c)));
        }

        inline constexpr Result<int> parse_int10(const std::string_view str)
        {
            int val = 0;
            for (auto c : str) {
                if (c >= '0' && c <= '9')
                    val = val * 10 + c - '0';
                else
                    ELFE_ERROR(Result<int>(0, EC::InvalidArgument),
                        std::invalid_argument("invalid integer '" + std::string(str) + "'"));
            }
            return val;
        }
    }

    enum class Baudrate {
        _2400,
        _9600,
        _19200,
        _115200,
    };

    enum class Role {
        Central,
        Peripheral,
    };

    enum class RoleMode {
        PinDepend,
        AlwaysCentral,
        AlwaysPeripheral,
    };

    enum class ATMode {
        PinDepend,
        AlwaysValid,
        ValidOnlyDisconnected,
    };

    enum class SleepMode {
        Forbidden,
        PinDepend,
    };

    enum class Power {
        _N20dBm = 0,
        _N10dBm = 2,
        _N5dBm = 4,
        _0dBm = 6,
        _4dBm = 8,
    };

    enum class BroadcastInterval {
        _50ms = 0,
        _100ms = 1,
        _200ms = 2,
        _500ms = 3,
        _1s = 4,
        _2s = 5,
    };

    struct MAC {
        std::array<uint8_t, 6> addr;
        MAC() = default;
        MAC(const std::string_view str)
        {
            ELFE_PANIC_IF(
                str.size() != 12, std::invalid_argument("invalid MAC address length"));
            Result<uint8_t> r1, r2;
            for (int i = str.size() - 1; i > 0; i -= 2) {
                r1 = helper::parse_hex_digit(str[i - 1]);
                r2 = helper::parse_hex_digit(str[i]);
                ELFE_PANIC_IF(!r1.ok() || !r2.ok(), std::invalid_argument("invalid MAC address"));
                addr[i / 2] = (r1.value << 4) | r2.value;
            }
        }

        operator std::string() const
        {
            std::string str;
            for (auto b : addr) {
                str += ffmt::format("{:02.x}", b);
            }
            return str;
        }
        std::string to_str() const
        {
            return std::string(*this);
        }
        bool operator==(const MAC& other) const
        {
            return addr == other.addr;
        }
    };

    struct Device {
        uint8_t order;
        std::string name;
        MAC mac;
        int signal;
    };

    using DevContainer = std::vector<Device>;

    struct KeyValue {
        std::string key;
        std::string value;
    };

    class ECB02S {
        Result<KeyValue> _parse_kv(std::string_view str) const
        {
            if (str.starts_with("+")) {
                ELFE_ERROR_IF(
                    str.size() < 3,
                    Result<KeyValue>(KeyValue {}, EC::ExECBATError),
                    ATError("invalid key-value pair incomplate key-value pair"));
                str = str.substr(1);
            }
            auto pos = str.find_first_of(':');
            ELFE_ERROR_IF(
                pos == std::string_view::npos,
                Result<KeyValue>(KeyValue {}, EC::ExECBATError),
                ATError("invalid key-value pair"));
            return KeyValue {
                std::string(str.substr(0, pos)),
                std::string(str.substr(pos + 1))
            };
        }

        VoidResult<> _at_send(std::string_view str) const
        {
            _at_start();
            auto r = uart.write(str);
            ELFE_PROP(r, r.error);
            _at_end();
        }

        VoidResult<> _at_set(const std::string_view cmd, const std::string_view value) const
        {
            _at_start();
            auto r = uart.write("AT+");
            ELFE_PROP(r, r.error);
            r = uart.write(cmd);
            ELFE_PROP(r, r.error);
            r = uart.write("=");
            ELFE_PROP(r, r.error);
            r = uart.write(value);
            ELFE_PROP(r, r.error);
            r = uart.write("\r\n");
            ELFE_PROP(r, r.error);
            _at_end();
            return uart.read_line().error; // read OK
        }

        bool _update_con_status(std::string_view resp) const
        {
            if (resp.starts_with("CONNECT OK")) {
                return true;
            } else if (resp.starts_with("CONNECT FAIL")) {
                return true;
            } else if (resp.starts_with("DISCONNECT")) {
                return true;
            } else if (resp.starts_with("SCANNING")) {
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

        Result<std::string> _common_parse(std::string_view key) const
        try {
            _at_start();
            auto rst = uart.write("AT+");
            ELFE_PROP(rst, Result<std::string>("", rst.error));
            rst = uart.write(key);
            ELFE_PROP(rst, Result<std::string>("", rst.error));
            rst = uart.write("?\r\n");
            ELFE_PROP(rst, Result<std::string>("", rst.error));
            _at_end();
            auto rs = uart.read_line();
            ELFE_PROP(rs, rs);
            std::string resp = rs.value;
            while (resp == "OK"
                or resp == "ERROR"
                or _update_con_status(resp)
                or resp.starts_with("AT+")) {
                rs = uart.read_line();
                ELFE_PROP(rs, rs);
                resp = rs.value;
            }
            auto rkv = _parse_kv(resp);
            ELFE_PROP(rkv, Result<std::string>("", rkv.error));
            KeyValue kv = rkv.value;
            rs = uart.read_line(); // read OK
            ELFE_PROP(rs, rs);
            ELFE_ERROR_IF(
                kv.key != key,
                Result<std::string>("", EC::ExECBATError),
                ATError("invalid response"));
            return kv.value;
        } catch (const clock::TimeoutError& e) {
            throw ATError("AT read timeout");
        }

    public:
        usart::BaseUart& uart;
        gpio::Pin at_pin; // AT command, active low
        gpio::Pin sleep_pin; // sleep mode, active high

        ECB02S(usart::BaseUart& uart, gpio::Pin at_pin, gpio::Pin sleep_pin)
            : uart(uart)
            , at_pin(at_pin)
            , sleep_pin(sleep_pin)
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
            auto r = uart.set_baudrate(115200);
            ELFE_PANIC_IF(!r.ok(), std::runtime_error("cannot set baudrate"));
            r = uart.set_parity(usart::Parity::None);
            ELFE_PANIC_IF(!r.ok(), std::runtime_error("cannot set parity"));
            r = uart.set_stop_bits(usart::StopBits::One);
            ELFE_PANIC_IF(!r.ok(), std::runtime_error("cannot set stop bits"));
            r = uart.set_word_length(usart::WordLength::Bits8);
            ELFE_PANIC_IF(!r.ok(), std::runtime_error("cannot set word length"));
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

        Result<bool> test() const
        {
            auto r = _at_send("AT\r\n");
            ELFE_PROP(r, Result<bool>(false, r.error));
            auto rs = uart.read_line();
            ELFE_PROP(rs, Result<bool>(false, rs.error));
            std::string resp = rs.value;
            if (resp.starts_with("AT")) {
                rs = uart.read_line(); // incase of echo
                ELFE_PROP(rs, Result<bool>(false, rs.error));
                resp = rs.value;
            }
            return resp == "OK";
        }

        VoidResult<> set_echo(bool enable) const
        {
            return _at_set("ECHO", enable ? "1" : "0");
        }

        Result<bool> get_echo() const
        {
            auto rs = _common_parse("ECHO");
            ELFE_PROP(rs, Result<bool>(false, rs.error));
            auto value = rs.value;
            return value == "1";
        }

        Result<std::string> get_model() const
        {
            auto rs = _common_parse("MODEL");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        Result<std::string> get_version() const
        {
            auto rs = _common_parse("VER");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        VoidResult<> reset() const
        {
            auto r = _at_send("AT+RST\r\n");
            ELFE_PROP(r, r);
            return uart.read_line().error; // read OK
        }

        Result<Role> get_role() const
        {
            auto rs = _common_parse("ROLE");
            ELFE_PROP(rs, Result<Role>(Role {}, rs.error));
            auto value = rs.value;
            return value == "Peripheral" ? Role::Peripheral : Role::Central;
        }

        VoidResult<> set_role_mode(RoleMode mode) const
        {
            switch (mode) {
            case RoleMode::PinDepend:
                return _at_set("ROLEMODE", "0");
            case RoleMode::AlwaysCentral:
                return _at_set("ROLEMODE", "1");
            case RoleMode::AlwaysPeripheral:
                return _at_set("ROLEMODE", "2");
            default:
                ELFE_ERROR(EC::InvalidArgument, std::invalid_argument("invalid role mode"));
            }
        }

        Result<RoleMode> get_role_mode() const
        {
            auto rs = _common_parse("ROLEMODE");
            ELFE_PROP(rs, Result<RoleMode>(RoleMode {}, rs.error));
            auto value = rs.value;
            switch (value[0]) {
            case '0':
                return RoleMode::PinDepend;
            case '1':
                return RoleMode::AlwaysCentral;
            case '2':
                return RoleMode::AlwaysPeripheral;
            default:
                ELFE_ERROR(Result<RoleMode>(RoleMode {},
                               EC::ExECBATError),
                    ATError("invalid role mode"));
            }
        }

        VoidResult<> set_at_mode(ATMode mode) const
        {
            VoidResult<> r;
            switch (mode) {
            case ATMode::PinDepend:
                return _at_set("MODE", "0");
            case ATMode::AlwaysValid:
                return _at_set("MODE", "1");
            case ATMode::ValidOnlyDisconnected:
                return _at_set("MODE", "2");
            default:
                ELFE_ERROR(EC::InvalidArgument, std::invalid_argument("invalid AT mode"));
            }
        }

        Result<ATMode> get_at_mode() const
        {
            auto rs = _common_parse("MODE");
            ELFE_PROP(rs, Result<ATMode>(ATMode {}, rs.error));
            auto value = rs.value;
            switch (value[0]) {
            case '0':
                return ATMode::PinDepend;
            case '1':
                return ATMode::AlwaysValid;
            case '2':
                return ATMode::ValidOnlyDisconnected;
            default:
                ELFE_ERROR(
                    Result<ATMode>(ATMode {},
                        EC::ExECBATError),
                    ATError("invalid AT mode"));
            }
        }

        VoidResult<> set_sleep_mode(SleepMode mode) const
        {
            switch (mode) {
            case SleepMode::Forbidden:
                return _at_set("SLEEP", "0");
            case SleepMode::PinDepend:
                return _at_set("SLEEP", "1");
            default:
                ELFE_ERROR(EC::ExECBATError, ATError("invalid sleep mode"));
            }
        }

        Result<SleepMode> get_sleep_mode() const
        {
            auto rs = _common_parse("SLEEP");
            ELFE_PROP(rs, Result<SleepMode>(SleepMode {}, rs.error));
            auto value = rs.value;
            switch (value[0]) {
            case '0':
                return SleepMode::Forbidden;
            case '1':
                return SleepMode::PinDepend;
            default:
                ELFE_ERROR(
                    Result<SleepMode>(SleepMode {},
                        EC::ExECBATError),
                    ATError("invalid sleep mode"));
            }
        }

        Result<bool> get_led() const
        {
            auto rs = _common_parse("LED");
            ELFE_PROP(rs, Result<bool>(0, rs.error));
            auto value = rs.value;
            return value == "1";
        }

        VoidResult<> set_led(bool enable) const
        {
            return _at_set("LED", enable ? "1" : "0");
        }

        Result<bool> get_watchdog() const
        {
            auto rs = _common_parse("WDG");
            ELFE_PROP(rs, Result<bool>(0, rs.error));
            auto value = rs.value;
            return value == "1";
        }

        VoidResult<> set_watchdog(bool enable) const
        {
            return _at_set("WDG", enable ? "1" : "0");
        }

        Result<bool> get_con_notify() const
        {
            auto rs = _common_parse("CONNOTIFY");
            ELFE_PROP(rs, Result<bool>(0, rs.error));
            auto value = rs.value;
            return value == "1";
        }

        VoidResult<> set_con_notify(bool enable) const
        {
            return _at_set("CONNOTIFY", enable ? "1" : "0");
        }

        VoidResult<> poweroff() const
        {
            auto r = _at_send("AT+POWEROFF\r\n");
            ELFE_PROP(r, r);
            return uart.read_line().error; // read OK
        }

        VoidResult<> factory_reset() const
        {
            auto r = _at_send("AT+FACTORY\r\n");
            ELFE_PROP(r, r);
            return uart.read_line().error; // read OK
        }

        Result<bool> is_at_enabled() const
        {
            auto rs = _common_parse("BTAT");
            ELFE_PROP(rs, Result<bool>(0, rs.error));
            auto value = rs.value;
            return value == "1";
        }

        Result<uint32_t> get_baudrate() const
        {
            auto rs = _common_parse("UART");
            ELFE_PROP(rs, Result<uint32_t>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<uint32_t>(0, ri.error));
            auto baud = ri.value;
            switch (baud) {
            case 0:
                return 2400;
            case 1:
                return 9600;
            case 2:
                return 19200;
            case 3:
                return 115200;
            default:
                ELFE_ERROR_IF(
                    baud < 110,
                    Result<uint32_t>(0, EC::ExECBATError),
                    ATError("invalid baudrate"));
            }
            return baud;
        }

        VoidResult<> set_baudrate(uint32_t baud) const
        {
            uint32_t baud_code;
            switch (baud) {
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
                ELFE_ERROR_IF(
                    baud < 110,
                    EC::ExECBATError,
                    ATError("invalid baudrate"));
                baud_code = baud;
            }
            auto r = _at_set("UART", ffmt::format("{}", baud_code));
            ELFE_PROP(r, r);
            return uart.set_baudrate(baud);
        }

        VoidResult<> set_baudrate(Baudrate baudrate)
        {
            return set_baudrate(static_cast<uint32_t>(baudrate));
        }

        Result<size_t> get_mtu() const
        {
            auto rs = _common_parse("MTU");
            ELFE_PROP(rs, Result<size_t>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            return { static_cast<size_t>(ri.value), ri.error };
        }

        Result<bool> is_ble_notify_enabled() const
        {
            auto rs = _common_parse("BLENOTIFY");
            ELFE_PROP(rs, Result<bool>(0, rs.error));
            auto value = rs.value;
            return value == "1";
        }

        VoidResult<> set_ble_notify(bool enable) const
        {
            return _at_set("BLENOTIFY", enable ? "1" : "0");
        }

        Result<bool> is_connected() const
        {
            auto rs = _common_parse("LINK");
            ELFE_PROP(rs, Result<bool>(0, rs.error));
            auto value = rs.value;
            return value == "1";
        }

        VoidResult<> disconnect() const
        {
            auto r = _at_send("AT+DISC\r\n");
            ELFE_PROP(r, r);
            return uart.read_line().error; // read OK
        }

        Result<Power> get_power() const
        {
            auto rs = _common_parse("POWER");
            ELFE_PROP(rs, Result<Power>(Power {}, rs.error));
            auto value = rs.value;
            if (value.ends_with("db"))
                value = value.substr(0, value.size() - 2);
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<Power>(Power {}, ri.error));
            return static_cast<Power>(ri.value);
        }

        VoidResult<> set_power(Power power) const
        {
            int power_code = static_cast<int>(power);
            return _at_set("POWER", ffmt::format("{}", power_code));
        }

        Result<std::string> get_service_uuid() const
        {
            auto rs = _common_parse("SUUID");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        VoidResult<> set_service_uuid(const std::string_view uuid) const
        {
            return _at_set("SUUID", uuid);
        }

        Result<std::string> get_read_uuid() const
        {
            auto rs = _common_parse("RUUID");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        VoidResult<> set_read_uuid(const std::string_view uuid) const
        {
            return _at_set("RUUID", uuid);
        }

        Result<std::string> get_write_uuid() const
        {
            auto rs = _common_parse("WUUID");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        VoidResult<> set_write_uuid(const std::string_view uuid) const
        {
            return _at_set("WUUID", uuid);
        }

        Result<std::string> get_name() const
        {
            auto rs = _common_parse("NAME");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        VoidResult<> set_name(const std::string_view name) const
        {
            return _at_set("NAME", name);
        }

        Result<MAC> get_mac() const
        {
            auto rs = _common_parse("MAC");
            ELFE_PROP(rs, Result<MAC>(MAC {}, rs.error));
            auto value = rs.value;
            return MAC(value);
        }

        VoidResult<> set_mac(const MAC& mac) const
        {
            return _at_set("MAC", std::string(mac));
        }

        Result<uint32_t> get_broadcast_interval_us() const
        {
            auto rs = _common_parse("ADVINT");
            ELFE_PROP(rs, Result<uint32_t>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<uint32_t>(0, ri.error));
            int interval = ri.value;
            switch (interval) {
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
                ELFE_ERROR_IF(
                    interval > 16384,
                    Result<uint32_t>(0, EC::ExECBATError),
                    ATError("invalid broadcast interval"));
            }
            return interval * 625;
        }

        VoidResult<> set_broadcast_interval_us(uint32_t interval_us) const
        {
            uint32_t interval_code;
            switch (interval_us) {
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
                ELFE_ERROR_IF(
                    interval_us < 32,
                    EC::ExECBATError,
                    ATError("invalid broadcast interval"));
                interval_code = interval_us / 625;
            }
            ELFE_ERROR_IF(
                interval_code > 16384,
                EC::ExECBATError,
                ATError("invalid broadcast interval"));
            return _at_set("ADVINT", ffmt::format("{}", interval_code));
        }

        Result<std::string> get_broadcast_data() const
        {
            auto rs = _common_parse("RESE");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        VoidResult<> set_broadcast_data(const std::string_view data) const
        {
            ELFE_ERROR_IF(
                data.size() > 22,
                EC::InvalidArgument,
                std::invalid_argument("broadcast data too long"));
            return _at_set("RESE", data);
        }

        VoidResult<> turn_off_broadcast() const
        {
            auto r = _at_send("AT+RESEOFF\r\n");
            ELFE_PROP(r, r);
            return uart.read_line().error; // read OK
        }

        Result<std::string> get_password() const
        {
            auto rs = _common_parse("PASSWORD");
            ELFE_PROP(rs, rs);
            auto value = rs.value;
            return value;
        }

        VoidResult<> set_password(const std::string_view password) const
        {
            return _at_set("PASSWORD", password);
        }

        VoidResult<> clear_password() const
        {
            auto r = _at_send("AT+PASSWORDC\r\n");
            ELFE_PROP(r, r);
            return uart.read_line().error; // read OK
        }

        /**
         * @brief Load low power configuration preset
         *
         * @param enable
         */
        VoidResult<> set_low_power(const bool enable) const
        {
            return _at_set("CONPARAM", enable ? "1" : "0");
        }

        Result<float> get_con_interval_min_ms() const
        {
            auto rs = _common_parse("CONINTMIN");
            ELFE_PROP(rs, Result<float>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<float>(0, ri.error));
            return ri.value * 1.25f;
        }

        VoidResult<> set_con_interval_min_ms(const float interval_ms) const
        {
            int interval = interval_ms / 1.25f;
            ELFE_ERROR_IF(
                interval < 6 || interval > 3200,
                EC::InvalidArgument,
                std::invalid_argument("invalid connection interval"));
            return _at_set("CONINTMIN", ffmt::format("{}", interval));
        }
        Result<float> get_con_interval_max_ms() const
        {
            auto rs = _common_parse("CONINTMAX");
            ELFE_PROP(rs, Result<float>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<float>(0, ri.error));
            return ri.value * 1.25f;
        }

        VoidResult<> set_con_interval_max_ms(const float interval_ms) const
        {
            int interval = interval_ms / 1.25f;
            ELFE_ERROR_IF(
                interval < 6 || interval > 3200,
                EC::InvalidArgument,
                std::invalid_argument("invalid connection interval"));
            return _at_set("CONINTMAX", ffmt::format("{}", interval));
        }

        Result<uint32_t> get_latency() const
        {
            auto rs = _common_parse("LATENCY");
            ELFE_PROP(rs, Result<uint32_t>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<uint32_t>(ri.value, ri.error));
            return ri.value;
        }

        VoidResult<> set_latency(const uint32_t latency) const
        {
            ELFE_ERROR_IF(
                latency > 499,
                EC::InvalidArgument,
                std::invalid_argument("invalid latency"));
            return _at_set("LATENCY", ffmt::format("{}", latency));
        }

        Result<uint32_t> get_timeout_ms() const
        {
            auto rs = _common_parse("CONTIMEOUT");
            ELFE_PROP(rs, Result<uint32_t>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<uint32_t>(ri.value, ri.error));
            return ri.value * 10;
        }

        VoidResult<> set_timeout_ms(const uint32_t timeout_ms) const
        {
            int timeout = timeout_ms / 10;
            ELFE_ERROR_IF(
                timeout > 3200,
                EC::InvalidArgument,
                std::invalid_argument("invalid timeout"));
            return _at_set("CONTIMEOUT", ffmt::format("{}", timeout));
        }

        Result<size_t> get_scan_max_n() const
        {
            auto rs = _common_parse("SCANMAX");
            ELFE_PROP(rs, Result<size_t>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<size_t>(ri.value, ri.error));
            return ri.value;
        }

        VoidResult<> set_scan_max_n(const size_t max_n) const
        {
            ELFE_ERROR_IF(
                max_n < 10 || max_n > 30,
                EC::InvalidArgument,
                std::invalid_argument("invalid scan max number"));
            return _at_set("SCANMAX", ffmt::format("{}", max_n));
        }

        Result<uint32_t> get_scan_time_ms() const
        {
            auto rs = _common_parse("SCANTIME");
            ELFE_PROP(rs, Result<uint32_t>(0, rs.error));
            auto value = rs.value;
            auto ri = helper::parse_int10(value);
            ELFE_PROP(ri, Result<uint32_t>(ri.value, ri.error));
            return ri.value;
        }

        VoidResult<> set_scan_time_ms(const uint32_t time_ms) const
        {
            ELFE_ERROR_IF(
                time_ms < 500 || time_ms > 10000,
                EC::InvalidArgument,
                std::invalid_argument("invalid scan time"));
            return _at_set("SCANTIME", ffmt::format("{}", time_ms));
        }

        Result<DevContainer> scan() const
        {
            auto r = _at_send("AT+SCAN\r\n");
            ELFE_PROP(r, Result<DevContainer>(DevContainer {}, r.error));
            auto rs = uart.read_line();
            ELFE_PROP(rs, Result<DevContainer>(DevContainer {}, rs.error));
            auto resp = rs.value;
            while (resp == "OK" or _update_con_status(resp) or resp.starts_with("AT+")) {
                rs = uart.read_line();
                ELFE_PROP(rs, Result<DevContainer>(DevContainer {}, rs.error));
                resp = rs.value;
            }

            DevContainer devs;
            auto kv = _parse_kv(resp);
            ELFE_PROP(kv, Result<DevContainer>(DevContainer {}, kv.error));
            auto ri = helper::parse_int10(kv.value.value); // number of devices
            ELFE_PROP(ri, Result<DevContainer>(DevContainer {}, ri.error));
            int n = ri.value;
            for (int i = 0; i < n; ++i) {
                // demo: 0,elfe,001122334455,-50
                Device dev;
                rs = uart.read_line();
                ELFE_PROP(rs, Result<DevContainer>(DevContainer {}, rs.error));
                resp = rs.value;
                std::string_view str(resp);
                auto pos = str.find_first_of(',');
                ELFE_ERROR_IF(
                    pos == std::string_view::npos,
                    Result<DevContainer>(DevContainer {}, EC::ExECBATError),
                    ATError("invalid scan response"));
                ri = helper::parse_int10(str.substr(0, pos));
                ELFE_PROP(ri, Result<DevContainer>(DevContainer {}, ri.error));
                dev.order = ri.value;
                str = str.substr(pos + 1);
                pos = str.find_first_of(',');
                ELFE_ERROR_IF(
                    pos == std::string_view::npos,
                    Result<DevContainer>(DevContainer {}, EC::ExECBATError),
                    ATError("invalid scan response"));
                dev.name = std::string(str.substr(0, pos));
                str = str.substr(pos + 1);
                pos = str.find_first_of(',');
                ELFE_ERROR_IF(
                    pos == std::string_view::npos,
                    Result<DevContainer>(DevContainer {}, EC::ExECBATError),
                    ATError("invalid scan response"));
                dev.mac = MAC(str.substr(0, pos));
                str = str.substr(pos + 1);
                ri = helper::parse_int10(str);
                ELFE_PROP(ri, Result<DevContainer>(DevContainer {}, ri.error));
                dev.signal = ri.value;
                devs.push_back(dev);
            }
            rs = uart.read_line(); // read empty line
            ELFE_PROP(rs, Result<DevContainer>(DevContainer {}, rs.error));
            return devs;
        }
    };

}
}
