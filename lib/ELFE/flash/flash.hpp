#pragma once

#include "_hpp_config.hpp"
#include "errors.hpp"
#include "flash/flash_f1_detail.hpp"
#include "flash/flash_f4_detail.hpp"
#include "flash/flash_generic_detail.hpp"
#include "flash/flash_hx_detail.hpp"
#include "nvic/nvic.hpp"
#include "result.hpp"
#include "units.hpp"
#include "utils/property.hpp"
#include <concepts>
#include <cstdint>
#include <functional>
#include <iterator>
#include <ranges>
#include <stdexcept>
#include <utility>

namespace elfe {
namespace stm32 {
    namespace flash {
        using namespace err::flash;
        using EC = err::ErrorCode;
        using std::size_t;
        using addr_t = std::uintptr_t;
        using CompleteCallbackType = std::function<void()>;
        using ErrorCallbackType = std::function<void(const FlashError&)>;

        bool enable_instruction_cache(); // enable instruction cache if available
        bool enable_data_cache(); // enable data cache if available
        inline bool enable_prefetch() // enable prefetch buffer if available
        {
            return enable_instruction_cache() && enable_data_cache();
        }

        struct Range {
            addr_t start;
            addr_t end;
        };

        class BaseFlash {
        protected:
        public:
            bool validate = false;
            virtual ~BaseFlash() = default;
            virtual bool is_valid_range(addr_t addr_start, size_t size) const noexcept = 0;
            /**
             * @brief returns affected range in start and end address, a unit is a smallest unit that can be erased at provided address
             *
             * @param addr
             * @return Range
             * @throw invalid_argument if addr is not valid
             */
            virtual Result<Range> get_affected_range(addr_t addr) const = 0;
            /**
             * @throw invalid_argument if addr is not valid
             * @throw FlashError (std::runtime_error)
             * @throw ValidationError (FlashError & std::runtime_error)
             */
            virtual VoidResult<> write_bytes(addr_t addr, const void* data, size_t bytes) = 0;
            /**
             * @throw FlashError
             * @throw invalid_argument if addr is not valid
             */
            virtual VoidResult<> read_bytes(addr_t addr, void* data, size_t bytes) const = 0;
            /**
             * @throw FlashError
             * @throw invalid_argument if addr is not valid
             */
            virtual VoidResult<> erase(addr_t addr, size_t bytes) = 0;
            /**
             * @throw FlashError
             */
            virtual VoidResult<> erase_all() = 0;
            virtual VoidResult<> raise_if_error(bool clear = true) const = 0;
            template <typename T>
            VoidResult<> put(addr_t addr, const T& item)
            {
                auto r = write_bytes(addr, &item, sizeof(T));
                ELFE_PROP(r, r);
                return EC::None;
            }
            template <typename T>
            Result<T> get(addr_t addr) const
                requires(std::is_trivially_copyable_v<T>)
            {
                T item;
                auto r = read_bytes(addr, &item, sizeof(T));
                ELFE_PROP(r, Result<T>(item, r.error));
                return item;
            }
            template <std::input_iterator Iter_t, std::sentinel_for<Iter_t> Senti_t>
            VoidResult<> write(addr_t addr, Iter_t begin, Senti_t end)
            {
                while (begin != end) {
                    auto r = put(addr, *begin++);
                    ELFE_PROP(r, r);
                }
                return EC::None;
            }

            template <std::ranges::input_range Range_t>
            VoidResult<> write(addr_t addr, const Range_t& range)
            {
                return write(addr, std::ranges::begin(range), std::ranges::end(range));
            }

            template <typename Iter_t, typename Senti_t>
            Result<size_t> read(addr_t addr, Iter_t begin, Senti_t end) const
                requires std::sentinel_for<Senti_t, Iter_t> && std::output_iterator<Iter_t, typename std::iterator_traits<Iter_t>::value_type>
            {
                const addr_t start_addr = addr;
                size_t unit_bytes = sizeof(typename std::iterator_traits<Iter_t>::value_type);
                while (begin != end) {
                    auto r = get<typename std::iterator_traits<Iter_t>::value_type>(addr);
                    ELFE_PROP(r, Result<size_t>(0, r.error));
                    *begin = r.value;
                    ++begin;
                    addr += unit_bytes;
                }
                return addr - start_addr;
            }
            template <typename Range_t>
            Result<size_t> read(addr_t addr, Range_t&& range) const
                requires std::ranges::output_range<Range_t, typename std::ranges::range_value_t<Range_t>>
            {
                return read(addr, std::ranges::begin(range), std::ranges::end(range));
            }
        };

        class EmbeddedFlash : public BaseFlash {
        protected:
            template <typename T>
            struct _Property : public tricks::StaticProperty<T, EmbeddedFlash&> {
                constexpr _Property(EmbeddedFlash& owner)
                    : tricks::StaticProperty<T, EmbeddedFlash&>(owner)
                {
                }
                using tricks::StaticProperty<T, EmbeddedFlash&>::operator=;
            };
            template <typename T>
            struct _ROProperty : public tricks::StaticReadOnlyProperty<T, EmbeddedFlash&> {
                constexpr _ROProperty(EmbeddedFlash& owner)
                    : tricks::StaticReadOnlyProperty<T, EmbeddedFlash&>(owner)
                {
                }
                using tricks::StaticReadOnlyProperty<T, EmbeddedFlash&>::operator=;
            };

        public:
            static constexpr const nvic::IRQn_Type irqn = nvic::IRQn_Type::FLASH_IRQn;
            detail::FlashRegister& reg = detail::flash_reg;
            ErrorCallbackType on_error;
            CompleteCallbackType on_complete;
            bool is_valid_range(addr_t addr_start, size_t size) const noexcept override;
            Result<Range> get_affected_range(addr_t addr) const override;
            VoidResult<> write_bytes(addr_t addr, const void* data, size_t bytes) override;
            VoidResult<> read_bytes(addr_t addr, void* data, size_t bytes) const override;
            VoidResult<> erase(addr_t addr, size_t bytes) override;
            VoidResult<> erase_all() override;

            void set_latency(const uint8_t latency) const noexcept
            {
                reg.ACR = (reg.ACR & ~0xFU) | (latency & 0xFU);
            }

            uint8_t get_latency() const noexcept
            {
                return reg.ACR & 0xFU;
            }

            VoidResult<> enable_interrupt_error() const;
            VoidResult<> disable_interrupt_error() const;
            VoidResult<> enable_interrupt_complete() const;
            VoidResult<> disable_interrupt_complete() const;
            VoidResult<> enable_interrupts() const noexcept
            {
                auto r = enable_interrupt_error();
                ELFE_PROP(r, r);
                return enable_interrupt_complete();
            }
            VoidResult<> disable_interrupts() const noexcept
            {
                nvic::disable_irq(irqn);
                auto r = disable_interrupt_error();
                ELFE_PROP(r, r);
                return disable_interrupt_complete();
            }
            VoidResult<> set_irq_priority(const uint8_t priority = 8) const
            {
                return nvic::set_priority(irqn, priority);
            }

            void clear_error() const noexcept;
            VoidResult<> raise_if_error(bool clear = true) const override final;
            void on_error_handler() const noexcept;
            void on_complete_handler() const noexcept;

            void global_interrupt_handler() const noexcept
            {
                on_error_handler();
                on_complete_handler();
            }
        };

        inline EmbeddedFlash Flash;

    }
}
}
