#pragma once

#include "_hpp_config.hpp"
#include "clock/clock.hpp"
#include "errors.hpp"
#include "result.hpp"
#include <concepts>
#include <cstdint>
#include <functional>
#include <iterator>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace elfe {
namespace stm32 {
    namespace spi {
        using namespace err::spi;
        using std::size_t;
        using EC = err::ErrorCode;

        enum class Mode : uint8_t {
            Mode0 = 0,
            Mode1 = 1,
            Mode2 = 2,
            Mode3 = 3,
        };

        class BaseInterface {
        public:
            virtual VoidResult<> turn_slave(bool yes) = 0;
            virtual bool is_slave() const noexcept = 0;
            virtual VoidResult<> set_mode(Mode mode) = 0;
            virtual Mode get_mode() const noexcept = 0;
            virtual Result<size_t> exchange_bytes(const void* tx, void* rx, size_t size) = 0;
            virtual VoidResult<> raise_if_error() const = 0;
            virtual VoidResult<> select_as_slave(bool yes) = 0;
            virtual bool is_selected_as_slave() const noexcept = 0;
            Result<size_t> write_bytes(const void* tx, size_t size)
            {
                return exchange_bytes(tx, nullptr, size);
            }
            Result<size_t> read_bytes(void* rx, size_t size)
            {
                return exchange_bytes(nullptr, rx, size);
            }

            template <typename T>
            VoidResult<> put(const T& data)
                requires(std::is_trivially_copyable_v<T>)
            {
                return write_bytes(&data, sizeof(T)).error;
            }
            template <typename T>
            Result<T> get()
                requires(std::is_trivially_copyable_v<T>)
            {
                T data;
                auto r = read_bytes(&data, sizeof(T));
                ELFE_PROP(r, Result<T>(data, r.error));
                return data;
            }
            template <typename T>
            Result<T&> get(T& data)
                requires(std::is_trivially_copyable_v<T>)
            {
                read_bytes(&data, sizeof(T));
                return data;
            }
            template <typename T>
            Result<T&> swap(T& data)
                requires(std::is_trivially_copyable_v<T>)
            {
                exchange_bytes(&data, &data, sizeof(T));
                return data;
            }
            template <std::input_iterator Iter_t, std::sentinel_for<Iter_t> Senti_t>
            Result<size_t> write(Iter_t begin, Senti_t end)
            {
                size_t count = 0;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    put(*begin++);
                    count += unit_bytes;
                }
                return count;
            }
            template <std::ranges::input_range Range_t>
            Result<size_t> write(const Range_t& range)
            {
                return write(std::ranges::begin(range), std::ranges::end(range));
            }

            template <typename Iter_t, typename Senti_t>
            Result<size_t> read(Iter_t begin, Senti_t end) const
                requires std::sentinel_for<Senti_t, Iter_t> && std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
            {
                size_t count = 0;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    *begin = get<typename std::iterator_traits<Iter_t>::value_type>();
                    ++begin;
                    count += unit_bytes;
                }
                return count;
            }
            template <typename Range_t>
            Result<size_t> read(Range_t&& range) const
                requires std::ranges::output_range<Range_t, typename std::ranges::range_value_t<Range_t>>
            {
                return read(std::ranges::begin(range), std::ranges::end(range));
            }
        };
    }
}
}