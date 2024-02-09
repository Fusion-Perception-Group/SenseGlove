#pragma once

#include <cstdint>
#include <array>
#include <limits>
#include "property.hpp"
#include "wit_const.hpp"
#include "i2c.hpp"
#include "units.hpp"

namespace vms
{
namespace wit
{
    using namespace stm32;
    using IDType = std::array<char, 12>;

    /**
     * @brief 
     * 
     * @tparam T 
     * @param max_ref when x, y, z is at their numerical maximum, they represent `max_ref` unit
     */
    template <typename T>
    struct AxisTriple
    {
        T x, y, z, max_ref;
        template <typename U>
        AxisTriple<U> cast() const noexcept
        {
            return {static_cast<U>(x), static_cast<U>(y), static_cast<U>(z), static_cast<U>(max_ref)};
        }
        /**
         * @brief Rescale x, y, z so that numerical `1` represents `1` unit
         * 
         * @tparam U 
         * @return AxisTriple<U> 
         */
        template <typename U=T>
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
    struct Quaternion
    {
        T w, x, y, z, max_ref;
        template <typename U>
        Quaternion<U> cast() const noexcept
        {
            return {static_cast<U>(w), static_cast<U>(x), static_cast<U>(y), static_cast<U>(z)};
        }
        template <typename U=T>
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
        template <typename U=T>
        Quaternion<U> normalize() const noexcept
        {
            U norm = std::sqrt(
                static_cast<U>(w) * static_cast<U>(w) +
                static_cast<U>(x) * static_cast<U>(x) +
                static_cast<U>(y) * static_cast<U>(y) +
                static_cast<U>(z) * static_cast<U>(z));
            
            return {
                static_cast<U>(w) / norm,
                static_cast<U>(x) / norm,
                static_cast<U>(y) / norm,
                static_cast<U>(z) / norm
            };
        }
    };

    template <typename T>
    struct EulerAngle
    {
        T roll, pitch, yaw, max_ref;
        template <typename U>
        EulerAngle<U> cast() const noexcept
        {
            return {static_cast<U>(roll), static_cast<U>(pitch), static_cast<U>(yaw)};
        }
        /**
         * @brief rescale roll, pitch, yaw to degree
         * 
         * @tparam U 
         * @return EulerAngle<U> 
         */
        template <typename U=T>
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

    class BaseSensor
    {
    protected:
        virtual void _write_reg(uint8_t reg_addr, uint16_t data) = 0;
        virtual void _write_regs(uint8_t reg_addr, const uint8_t *buf, size_t len) = 0;
    public:
        virtual ~BaseSensor() = default;

        virtual uint16_t read_reg(uint8_t reg_addr) = 0;
        virtual void read_regs(uint8_t reg_addr, uint16_t *buf, size_t len) = 0;

        void unlock()
        {
            _write_reg(reg::KEY, 0xB588);
        }
        void save()
        {
            _write_reg(reg::SAVE, 0x0);
        }
        void reset()
        {
            _write_reg(reg::SAVE, 0xFFFF);
        }
        void write_reg(uint8_t reg_addr, uint16_t data)
        {
            unlock();
            _write_reg(reg_addr, data);
            save();
        }
        void write_regs(uint8_t reg_addr, const uint8_t *buf, size_t len)
        {
            unlock();
            _write_regs(reg_addr, buf, len);
            save();
        }

        void set_led(const bool on)
        {
            write_reg(reg::LEDOFF, on ? 0 : 1);
        }
        bool is_led_off()
        {
            return read_reg(reg::LEDOFF);
        }

        void sleep()
        {
            write_reg(reg::SLEEP, 1);
        }
        void wake()
        {
            write_reg(reg::SLEEP, 0);
        }

        void set_report_rate(ReportRate rate)
        {
            write_reg(reg::RRATE, static_cast<uint16_t>(rate));
        }
        ReportRate get_report_rate()
        {
            return static_cast<ReportRate>(read_reg(reg::RRATE));
        }

        void set_report_baud(ReportBaud baud)
        {
            write_reg(reg::BAUD, static_cast<uint16_t>(baud));
        }
        ReportBaud get_report_baud()
        {
            return static_cast<ReportBaud>(read_reg(reg::BAUD));
        }
        

        /**
         * @brief Get the temp
         * 
         * @return int16_t max 100 Celsius degree
         */
        int16_t get_temp()
        {
            return read_reg(reg::TEMP);
        }

        /**
         * @brief Get the pressure
         * 
         * @return uint32_t in Pa
         */
        uint32_t get_pressure()
        {
            uint16_t buf[2];
            read_regs(reg::PRESSUREL, buf, 2);
            return (buf[1] << 8) | buf[0];
        }

        /**
         * @brief Get the acceleration
         * 
         * @return AxisTriple<int16_t> max_ref 2g or 16g
         */
        AxisTriple<int16_t> get_accel()
        {
            int16_t buf[3], max_ref=2;
            read_regs(reg::AX, reinterpret_cast<uint16_t*>(buf), 3);
            if (read_reg(reg::ACCRANGE) == 0x3)
                max_ref = 16;
            return {buf[0], buf[1], buf[2], max_ref};
        }

        /**
         * @brief Get the gyro
         * 
         * @return AxisTriple<int16_t> max_ref 2000 degree/s
         */
        AxisTriple<int16_t> get_gyro()
        {
            int16_t buf[3];
            read_regs(reg::GX, reinterpret_cast<uint16_t*>(buf), 3);
            return {buf[0], buf[1], buf[2], 2000};
        }

        /**
         * @brief Get the magnetic field
         * 
         * @return AxisTriple<int16_t> unit LSB
         */
        AxisTriple<int16_t> get_magnet()
        {
            int16_t buf[3];
            read_regs(reg::HX, reinterpret_cast<uint16_t*>(buf), 3);
            return {buf[0], buf[1], buf[2], std::numeric_limits<int16_t>::max()};
        }

        /**
         * @brief Get the altitude
         * 
         * @return int32_t in cm
         */
        int32_t get_alti()
        {
            int16_t buf[2];
            read_regs(reg::HEIGHTL, reinterpret_cast<uint16_t*>(buf), 2);
            return (buf[1] << 16) | buf[0];
        }

        Quaternion<int16_t> get_quaternion()
        {
            int16_t buf[4];
            read_regs(reg::Q0, reinterpret_cast<uint16_t*>(buf), 4);
            return {buf[0], buf[1], buf[2], buf[3], 1};
        }

        EulerAngle<int16_t> get_euler()
        {
            int16_t buf[3];
            read_regs(reg::ROLL, reinterpret_cast<uint16_t*>(buf), 3);
            return {buf[0], buf[1], buf[2], 180};
        }
    };

    class I2CSensor : public BaseSensor
    {
    protected:
        void _write_reg(uint8_t reg_addr, uint16_t data)
        {
            i2c.select(i2c_addr, false);
            i2c.write_byte(reg_addr);
            i2c.write_byte(data & 0xff);
            i2c.write_byte(data >> 8);
            i2c.end();
        }
        virtual void _write_regs(uint8_t reg_addr, const uint8_t *buf, size_t len)
        {
            i2c.select(i2c_addr, false);
            i2c.write_byte(reg_addr);
            for (size_t i = 0; i < len; ++i)
            {
                i2c.write_byte(buf[i]);
            }
            i2c.end();
        }
    public:
        i2c::BaseMaster &i2c;
        uint16_t i2c_addr;
        I2CSensor(i2c::BaseMaster &i2c, uint16_t i2c_addr=0xa0) : i2c(i2c), i2c_addr(i2c_addr)
        {
        }
        uint16_t read_reg(uint8_t reg_addr)
        {
            i2c.select(i2c_addr, false);
            i2c.write_byte(reg_addr);
            i2c.select(i2c_addr, true);
            uint16_t tmp = i2c.read_byte() | (i2c.read_byte(false) << 8);
            i2c.end();
            return tmp;
        }
        void read_regs(uint8_t reg_addr, uint16_t *buf, size_t len)
        {
            i2c.select(i2c_addr, false);
            i2c.write_byte(reg_addr);
            i2c.select(i2c_addr, true);
            for (size_t i = 0; i < len; ++i)
            {
                buf[i] = i2c.read_byte() | (i2c.read_byte(i != len - 1) << 8);
            }
            i2c.end();
        }
    };

}
}
