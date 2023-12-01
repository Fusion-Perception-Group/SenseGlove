#pragma once

#include <cstdint>
#include <stdexcept>
#include <iterator>
#include <ranges>
#include <concepts>
#include <functional>
#include <utility>
#include "userconfig.hpp"
#include "clock.hpp"

namespace vermils
{
namespace stm32
{
namespace spi
{
    using std::size_t;

    enum class Mode : uint8_t
    {
        Mode0=0,
        Mode1=1,
        Mode2=2,
        Mode3=3,
    };

    class SPIException : public std::runtime_error
    {
    public:
        SPIException(const char *msg="SPI Exception") : std::runtime_error(msg) {}
    };

    class CRCError : public SPIException
    {
    public:
        CRCError() : SPIException("CRC Error in SPI transmission") {}
    };

    class Overrun : public SPIException
    {
    public:
        Overrun() : SPIException("Overrun in SPI transmission") {}
    };

    class TIFrameError : public SPIException
    {
    public:
        TIFrameError() : SPIException("TI Frame Error in SPI transmission") {}
    };

    class I2SException : public SPIException
    {
    public:
        I2SException(const char *msg="I2S Exception") : SPIException(msg) {}
    };

    class Underrun : public I2SException
    {
    public:
        Underrun() : I2SException("Overrun/Underrun in TI/I2S transmission") {}
    };

    class Timeout : public SPIException, public clock::TimeoutError
    {
    public:
        Timeout() : SPIException("Timeout in SPI transmission") {}
    };

    class BaseInterface
    {
    public:
        virtual void turn_slave(bool yes) = 0;
        virtual bool is_slave() const = 0;
        virtual void set_mode(Mode mode) = 0;
        virtual Mode get_mode() const = 0;
        virtual size_t exchange_bytes(const void *tx, void *rx, size_t size) = 0;
        virtual void raise_if_error() const = 0;
        virtual void select_as_slave(bool yes) = 0;
        virtual bool is_selected_as_slave() const noexcept = 0;
        size_t write_bytes(const void *tx, size_t size)
        {
            return exchange_bytes(tx, nullptr, size);
        }
        size_t read_bytes(void *rx, size_t size)
        {
            return exchange_bytes(nullptr, rx, size);
        }

        template<typename T>
        void put(const T &data) requires(std::is_trivially_copyable_v<T>)
        {
            write_bytes(&data, sizeof(T));
        }
        template<typename T>
        T get() requires(std::is_trivially_copyable_v<T>)
        {
            T data;
            read_bytes(&data, sizeof(T));
            return data;
        }
        template<typename T>
        T& get(T& data) requires(std::is_trivially_copyable_v<T>)
        {
            read_bytes(&data, sizeof(T));
            return data;
        }
        template<typename T>
        T& swap(T &data) requires(std::is_trivially_copyable_v<T>)
        {
            exchange_bytes(&data, &data, sizeof(T));
            return data;
        }
        template <std::input_iterator Iter_t, std::sentinel_for<Iter_t> Senti_t>
        size_t write(Iter_t begin, Senti_t end)
        {
            size_t count = 0;
            size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
            while (begin != end)
            {
                put(*begin++);
                count += unit_bytes;
            }
            return count;
        }
        template <std::ranges::input_range Range_t>
        size_t write(const Range_t & range)
        {
            return write(std::ranges::begin(range), std::ranges::end(range));
        }

        template <typename Iter_t, typename Senti_t>
        requires std::sentinel_for<Senti_t, Iter_t> &&
                std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
        size_t read(Iter_t begin, Senti_t end) const
        {
            size_t count=0;
            size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
            while (begin != end)
            {
                *begin = get<typename std::iterator_traits<Iter_t>::value_type>();
                ++begin;
                count += unit_bytes;
            }
            return count;
        }
        template <typename Range_t>
        requires std::ranges::output_range<Range_t, typename std::ranges::range_value_t<Range_t>>
        size_t read(Range_t && range) const
        {
            return read(std::ranges::begin(range), std::ranges::end(range));
        }
    };
}
}
}