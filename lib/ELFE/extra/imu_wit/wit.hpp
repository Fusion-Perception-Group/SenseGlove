#pragma once

#include "extra/imu_wit/wit_const.hpp"
#include "i2c/i2c.hpp"
#include "result.hpp"
#include "units.hpp"
#include "utils/property.hpp"
#include <array>
#include <cstdint>
#include <limits>

namespace elfe {
namespace wit {
    using namespace stm32;
    using EC = err::ErrorCode;
    using IDType = std::array<char, 12>;

    /**
     * @brief
     *
     * @tparam T
     * @param max_ref when x, y, z is at their numerical maximum, they represent `max_ref` unit
     */
    template <typename T>
    struct AxisTriple {
        T x, y, z, max_ref;
        template <typename U>
        AxisTriple<U> cast() const noexcept
        {
            return { static_cast<U>(x), static_cast<U>(y), static_cast<U>(z), static_cast<U>(max_ref) };
        }
        /**
         * @brief Rescale x, y, z so that numerical `1` represents `1` unit
         *
         * @tparam U
         * @return AxisTriple<U>
         */
        template <typename U = T>
        AxisTriple<U> rescale() const noexcept
        {
            U MAX = std::numeric_limits<T>::max();
            return {
                static_cast<U>(x) * static_cast<U>(max_ref) / MAX,
                static_cast<U>(y) * static_cast<U>(max_ref) / MAX,
                static_cast<U>(z) * static_cast<U>(max_ref) / MAX,
                std::numeric_limits<U>::max()
            };
        }
    };

    template <typename T>
    struct Quaternion {
        T w, x, y, z, max_ref;
        template <typename U>
        Quaternion<U> cast() const noexcept
        {
            return { static_cast<U>(w), static_cast<U>(x), static_cast<U>(y), static_cast<U>(z) };
        }
        template <typename U = T>
        Quaternion<U> rescale() const noexcept
        {
            U MAX = std::numeric_limits<T>::max();
            return {
                static_cast<U>(w) * static_cast<U>(max_ref) / MAX,
                static_cast<U>(x) * static_cast<U>(max_ref) / MAX,
                static_cast<U>(y) * static_cast<U>(max_ref) / MAX,
                static_cast<U>(z) * static_cast<U>(max_ref) / MAX,
                std::numeric_limits<U>::max()
            };
        }
        template <typename U = T>
        Quaternion<U> normalize() const noexcept
        {
            U norm = std::sqrt(
                static_cast<U>(w) * static_cast<U>(w) + static_cast<U>(x) * static_cast<U>(x) + static_cast<U>(y) * static_cast<U>(y) + static_cast<U>(z) * static_cast<U>(z));

            return {
                static_cast<U>(w) / norm,
                static_cast<U>(x) / norm,
                static_cast<U>(y) / norm,
                static_cast<U>(z) / norm
            };
        }
    };

    template <typename T>
    struct EulerAngle {
        T roll, pitch, yaw, max_ref;
        template <typename U>
        EulerAngle<U> cast() const noexcept
        {
            return { static_cast<U>(roll), static_cast<U>(pitch), static_cast<U>(yaw) };
        }
        /**
         * @brief rescale roll, pitch, yaw to degree
         *
         * @tparam U
         * @return EulerAngle<U>
         */
        template <typename U = T>
        EulerAngle<U> rescale() const noexcept
        {
            U MAX = std::numeric_limits<T>::max();
            return {
                static_cast<U>(roll) * static_cast<U>(max_ref) / MAX,
                static_cast<U>(pitch) * static_cast<U>(max_ref) / MAX,
                static_cast<U>(yaw) * static_cast<U>(max_ref) / MAX,
                std::numeric_limits<U>::max()
            };
        }
    };

    class BaseSensor {
    protected:
        virtual VoidResult<> _write_reg(uint8_t reg_addr, uint16_t data) = 0;
        virtual VoidResult<> _write_regs(uint8_t reg_addr, const uint8_t* buf, size_t len) = 0;

    public:
        virtual ~BaseSensor() = default;

        virtual Result<uint16_t> read_reg(uint8_t reg_addr) = 0;
        virtual VoidResult<> read_regs(uint8_t reg_addr, uint16_t* buf, size_t len) = 0;

        VoidResult<> unlock()
        {
            return _write_reg(reg::KEY, 0xB588);
        }

        VoidResult<> _save()
        {
            return _write_reg(reg::SAVE, 0x0);
        }

        VoidResult<> reset()
        {
            return write_reg(reg::SAVE, 0x1);
        }

        VoidResult<> reboot()
        {
            return write_reg(reg::SAVE, 0x00FF);
        }

        VoidResult<> write_reg(uint8_t reg_addr, uint16_t data)
        {
            auto r = unlock();
            ELFE_PROP(r, r);
            r = _write_reg(reg_addr, data);
            ELFE_PROP(r, r);
            return _save();
        }

        VoidResult<> write_regs(uint8_t reg_addr, const uint8_t* buf, size_t len)
        {
            auto r = unlock();
            ELFE_PROP(r, r);
            r = _write_regs(reg_addr, buf, len);
            ELFE_PROP(r, r);
            return _save();
        }

        VoidResult<> set_led(const bool on)
        {
            return write_reg(reg::LEDOFF, on ? 0 : 1);
        }

        Result<bool> is_led_off()
        {
            auto r = read_reg(reg::LEDOFF);
            ELFE_PROP(r, Result<bool>(false, r.error));
            return r.value;
        }

        VoidResult<> sleep()
        {
            return write_reg(reg::SLEEP, 1);
        }

        VoidResult<> wake()
        {
            return write_reg(reg::SLEEP, 0);
        }

        VoidResult<> set_report_rate(ReportRate rate)
        {
            return write_reg(reg::RRATE, static_cast<uint16_t>(rate));
        }

        Result<ReportRate> get_report_rate()
        {
            auto r = read_reg(reg::RRATE);
            ELFE_PROP(r, Result<ReportRate>(ReportRate {}, r.error));
            return static_cast<ReportRate>(r.value);
        }

        VoidResult<> set_report_baud(ReportBaud baud)
        {
            return write_reg(reg::BAUD, static_cast<uint16_t>(baud));
        }

        Result<ReportBaud> get_report_baud()
        {
            auto r = read_reg(reg::BAUD);
            ELFE_PROP(r, Result<ReportBaud>(ReportBaud {}, r.error));
            return static_cast<ReportBaud>(r.value);
        }

        /**
         * @brief Get the temp
         *
         * @return int16_t max 100 Celsius degree
         */
        Result<int16_t> get_temp()
        {
            auto r = read_reg(reg::TEMP);
            ELFE_PROP(r, Result<int16_t>(0, r.error));
            return static_cast<int16_t>(r.value);
        }

        /**
         * @brief Get the pressure
         *
         * @return uint32_t in Pa
         */
        Result<uint32_t> get_pressure()
        {
            uint16_t buf[2];
            auto r = read_regs(reg::PRESSUREL, buf, 2);
            ELFE_PROP(r, Result<uint32_t>(0, r.error));
            return (buf[1] << 8) | buf[0];
        }

        /**
         * @brief Get the acceleration
         *
         * @return AxisTriple<int16_t> max_ref 2g or 16g
         */
        Result<AxisTriple<int16_t>> get_accel()
        {
            int16_t buf[3], max_ref = 2;
            auto r = read_regs(reg::AX, reinterpret_cast<uint16_t*>(buf), 3);
            ELFE_PROP(r, Result<AxisTriple<int16_t>>({ 0, 0, 0, 0 }, r.error));
            auto ru16 = read_reg(reg::ACCRANGE);
            ELFE_PROP(ru16, Result<AxisTriple<int16_t>>({ 0, 0, 0, 0 }, ru16.error));
            if (ru16.value == 0x3)
                max_ref = 16;
            return AxisTriple<int16_t> { buf[0], buf[1], buf[2], max_ref };
        }

        /**
         * @brief Get the gyro
         *
         * @return AxisTriple<int16_t> max_ref 2000 degree/s
         */
        Result<AxisTriple<int16_t>> get_gyro()
        {
            int16_t buf[3];
            auto r = read_regs(reg::GX, reinterpret_cast<uint16_t*>(buf), 3);
            ELFE_PROP(r, Result<AxisTriple<int16_t>>({ 0, 0, 0, 0 }, r.error));
            return AxisTriple<int16_t> { buf[0], buf[1], buf[2], 2000 };
        }

        /**
         * @brief Get the magnetic field
         *
         * @return AxisTriple<int16_t> unit LSB
         */
        Result<AxisTriple<int16_t>> get_magnet()
        {
            int16_t buf[3];
            auto r = read_regs(reg::HX, reinterpret_cast<uint16_t*>(buf), 3);
            ELFE_PROP(r, Result<AxisTriple<int16_t>>({ 0, 0, 0, 0 }, r.error));
            return AxisTriple<int16_t> { buf[0], buf[1], buf[2], std::numeric_limits<int16_t>::max() };
        }

        Result<AxisTriple<int16_t>> get_magnet_offset()
        {
            int16_t buf[3];
            auto r = read_regs(reg::HXOFFSET, reinterpret_cast<uint16_t*>(buf), 3);
            ELFE_PROP(r, Result<AxisTriple<int16_t>>({ 0, 0, 0, 0 }, r.error));
            return AxisTriple<int16_t> { buf[0], buf[1], buf[2], std::numeric_limits<int16_t>::max() };
        }

        VoidResult<> set_magnet_offset(const AxisTriple<int16_t>& offset)
        {
            uint8_t buf[6];
            buf[0] = offset.x & 0xff;
            buf[1] = offset.x >> 8;
            buf[2] = offset.y & 0xff;
            buf[3] = offset.y >> 8;
            buf[4] = offset.z & 0xff;
            buf[5] = offset.z >> 8;
            return write_regs(reg::HXOFFSET, buf, 6);
        }

        /**
         * @brief Get the altitude
         *
         * @return int32_t in cm
         */
        Result<int32_t> get_alti()
        {
            int16_t buf[2];
            auto r = read_regs(reg::HEIGHTL, reinterpret_cast<uint16_t*>(buf), 2);
            ELFE_PROP(r, Result<int32_t>(0, r.error));
            return (buf[1] << 16) | buf[0];
        }

        Result<Quaternion<int16_t>> get_quaternion()
        {
            int16_t buf[4];
            auto r = read_regs(reg::Q0, reinterpret_cast<uint16_t*>(buf), 4);
            ELFE_PROP(r, Result<Quaternion<int16_t>>({ 0, 0, 0, 0, 0 }, r.error));
            return Quaternion<int16_t> { buf[0], buf[1], buf[2], buf[3], 1 };
        }

        Result<EulerAngle<int16_t>> get_euler()
        {
            int16_t buf[3];
            auto r = read_regs(reg::ROLL, reinterpret_cast<uint16_t*>(buf), 3);
            ELFE_PROP(r, Result<EulerAngle<int16_t>>({ 0, 0, 0, 0 }, r.error));
            return EulerAngle<int16_t> { buf[0], buf[1], buf[2], 180 };
        }
    };

    class I2CSensor : public BaseSensor {
    protected:
        VoidResult<> _write_reg(uint8_t reg_addr, uint16_t data)
        {
            auto rb = i2c.select(i2c_addr, false);
            ELFE_PROP(rb, rb.error);
            ELFE_ERROR_IF(!rb.value, EC::I2CNoSlaveAck, err::i2c::NoSlaveAck());
            rb = i2c.write_byte(reg_addr);
            ELFE_PROP(rb, rb.error);
            ELFE_ERROR_IF(!rb.value, EC::I2CNoSlaveAck, err::i2c::NoSlaveAck());
            rb = i2c.write_byte(data & 0xff);
            ELFE_PROP(rb, rb.error);
            ELFE_ERROR_IF(!rb.value, EC::I2CNoSlaveAck, err::i2c::NoSlaveAck());
            rb = i2c.write_byte(data >> 8);
            ELFE_PROP(rb, rb.error);
            i2c.end();
            return EC::None;
        }
        virtual VoidResult<> _write_regs(uint8_t reg_addr, const uint8_t* buf, size_t len)
        {
            auto rb = i2c.select(i2c_addr, false);
            ELFE_PROP(rb, rb.error);
            ELFE_ERROR_IF(!rb.value, EC::I2CNoSlaveAck, err::i2c::NoSlaveAck());
            rb = i2c.write_byte(reg_addr);
            ELFE_ERROR_IF(!rb.value, EC::I2CNoSlaveAck, err::i2c::NoSlaveAck());
            for (size_t i = 0; i < len; ++i) {
                rb = i2c.write_byte(buf[i]);
                ELFE_PROP(rb, rb.error);
            }
            i2c.end();
            return EC::None;
        }

    public:
        i2c::BaseMaster& i2c;
        uint16_t i2c_addr;
        I2CSensor(i2c::BaseMaster& i2c, uint16_t i2c_addr = 0xa0)
            : i2c(i2c)
            , i2c_addr(i2c_addr)
        {
        }
        Result<uint16_t> read_reg(uint8_t reg_addr)
        {
            auto rb = i2c.select(i2c_addr, false);
            ELFE_PROP(rb, Result<uint16_t>(0, rb.error));
            ELFE_ERROR_IF(!rb.value,
                Result<uint16_t>(0, EC::I2CNoSlaveAck),
                err::i2c::NoSlaveAck());
            rb = i2c.write_byte(reg_addr);
            ELFE_PROP(rb, Result<uint16_t>(0, rb.error));
            ELFE_ERROR_IF(!rb.value,
                Result<uint16_t>(0, EC::I2CNoSlaveAck),
                err::i2c::NoSlaveAck());
            rb = i2c.select(i2c_addr, true);
            ELFE_PROP(rb, Result<uint16_t>(0, rb.error));
            uint16_t tmp;
            auto rby = i2c.read_byte();
            ELFE_PROP(rby, Result<uint16_t>(0, rb.error));
            tmp = rby.value;
            rby = i2c.read_byte(false);
            ELFE_PROP(rby, Result<uint16_t>(0, rb.error));
            tmp |= rby.value << 8;
            i2c.end();
            return tmp;
        }
        VoidResult<> read_regs(uint8_t reg_addr, uint16_t* buf, size_t len)
        {
            auto rb = i2c.select(i2c_addr, false);
            ELFE_PROP(rb, rb.error);
            ELFE_ERROR_IF(!rb.value, EC::I2CNoSlaveAck, err::i2c::NoSlaveAck());
            rb = i2c.write_byte(reg_addr);
            ELFE_PROP(rb, rb.error);
            ELFE_ERROR_IF(!rb.value, EC::I2CNoSlaveAck, err::i2c::NoSlaveAck());
            rb = i2c.select(i2c_addr, true);
            ELFE_PROP(rb, rb.error);
            for (size_t i = 0; i < len; ++i) {
                auto rby = i2c.read_byte();
                ELFE_PROP(rby, rb.error);
                buf[i] = rby.value;
                rby = i2c.read_byte(i != len - 1);
                ELFE_PROP(rby, rb.error);
                buf[i] |= rby.value << 8;
            }
            i2c.end();
            return EC::None;
        }
    };

}
}
