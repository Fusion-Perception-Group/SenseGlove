#include "./_cpp_config.hpp"
#include "clock/rcc.hpp"
#include "flash/flash.hpp"
#include <cstring>

namespace elfe {
namespace stm32 {
    namespace flash {

        namespace detail {
            FlashRegister& flash_reg = *reinterpret_cast<FlashRegister*>(FLASH);
        }

        enum class ParallelSize {
            x8 = 0, // 1.7 V-2.1V
            x16 = 1, // 2.1 - 2.7 V
            x32 = 2, // 2.7 - 3.6 V
            x64 = 3 // requires high voltage Vpp(8-9V), NOT recommended for long term usage
        };

        enum class SectorID {
            Sec0 = 0x0U,
            Sec1 = 0x1U,
            Sec2 = 0x2U,
            Sec3 = 0x3U,
            Sec4 = 0x4U,
            Sec5 = 0x5U,
            Sec6 = 0x6U,
            Sec7 = 0x7U,
            OTP = 0x8U, // One Time Programmable
            Options = 0xDU,
        };

        static constexpr const size_t size_map[] = {
            16_KiB,
            16_KiB,
            16_KiB,
            16_KiB,
            64_KiB,
            128_KiB,
            128_KiB,
            128_KiB,
            16_Bytes,
            0,
            0,
            0,
            0,
            512_Bytes,
        };

        constexpr const addr_t MAIN_MEM_START = 0x08000000U;
        constexpr const addr_t MAIN_MEM_END = 0x08080000U;
        constexpr const addr_t OTP_START = 0x1FFF7800U;
        constexpr const addr_t OTP_END = 0x1FFF7A10U;
        constexpr const addr_t OPT_START = 0x1FFFC000U;
        constexpr const addr_t OPT_END = 0x1FFFC010U;

        static inline void reset_ins_cache() noexcept
        {
            bool on = detail::flash_reg.ACR & (1U << 9U);
            detail::flash_reg.ACR &= ~(1U << 9U); // disable first
            detail::flash_reg.ACR |= (1U << 11U); // clear cache
            detail::flash_reg.ACR &= ~(1U << 11U);
            if (on)
                detail::flash_reg.ACR |= (1U << 9U); // enable
        }

        static inline bool is_busy() noexcept
        {
            return detail::flash_reg.SR & (1U << 16U);
        }

        static inline VoidResult<> unlock()
        {
            detail::flash_reg.KEYR = 0x45670123U;
            detail::flash_reg.KEYR = 0xCDEF89ABU;
            ELFE_ERROR_IF(
                detail::flash_reg.CR & FLASH_CR_LOCK,
                EC::FlashError,
                FlashError("Flash unlock failed"));
            return EC::None;
        }

        static inline void lock() noexcept
        {
            detail::flash_reg.CR |= (1U << 31U);
        }

        static inline void start_main() noexcept
        {
            detail::flash_reg.CR |= (1U << 16U);
        }

        static inline void stop_main() noexcept
        {
            detail::flash_reg.CR &= ~(1U << 16U);
        }

        static inline void set_erase_sector(const SectorID sector) noexcept
        {
            detail::flash_reg.CR = (detail::flash_reg.CR & ~0x78U) | (static_cast<uint32_t>(sector) << 3U) | 2U;
        }

        static inline void unset_erase_sector() noexcept
        {
            detail::flash_reg.CR &= ~2U;
        }

        static inline void set_program_sector() noexcept
        {
            detail::flash_reg.CR |= 1U;
        }

        static inline void unset_program_sector() noexcept
        {
            detail::flash_reg.CR &= ~1U;
        }

        static inline void set_erase_all() noexcept
        {
            detail::flash_reg.CR |= 4U;
        }

        static inline void unset_erase_all() noexcept
        {
            detail::flash_reg.CR &= ~4U;
        }

        static inline void start_option() noexcept
        {
            detail::flash_reg.OPTCR |= FLASH_OPTCR_OPTSTRT;
        }

        static inline void stop_option() noexcept
        {
            detail::flash_reg.OPTCR &= ~FLASH_OPTCR_OPTSTRT;
        }

        static inline void set_parallel_size(const ParallelSize size) noexcept
        {
            detail::flash_reg.CR = (detail::flash_reg.CR & ~(3U << 8U)) | (static_cast<uint32_t>(size) << 8U);
        }

        static inline VoidResult<> unlockopts()
        {
            detail::flash_reg.OPTKEYR = 0x08192A3BU;
            detail::flash_reg.OPTKEYR = 0x4C5D6E7FU;
            ELFE_ERROR_IF(
                detail::flash_reg.OPTCR & FLASH_OPTCR_OPTLOCK,
                EC::FlashError,
                FlashError("Flash option unlock failed"));
            return EC::None;
        }

        static inline void lockopts() noexcept
        {
            detail::flash_reg.OPTCR |= FLASH_OPTCR_OPTLOCK;
        }

        static inline void write_data(addr_t addr, const void* data, size_t bytes) noexcept
        {
            const unsigned align = 16;
            unsigned leading = align - (addr % align);
            uint8_t* data8 = (uint8_t*)data;
            set_program_sector();
            set_parallel_size(ParallelSize::x8);
            while (leading--) {
                if (!(bytes--))
                    return;
                *reinterpret_cast<volatile uint8_t*>(addr++) = *(data8++);
                while (is_busy())
                    ;
            }
            set_parallel_size(ParallelSize::x32);
            for (size_t i = 0; i < bytes / 4; ++i) {
                *reinterpret_cast<volatile uint32_t*>(addr + i * 4) = *reinterpret_cast<const uint32_t*>(data8 + i * 4);
                while (is_busy())
                    ;
            }
            set_parallel_size(ParallelSize::x8);
            switch (bytes % 4) {
            case 3:
                *reinterpret_cast<volatile uint8_t*>(addr + bytes - 3) = *(data8 + bytes - 3);
                while (is_busy())
                    ;
            case 2:
                *reinterpret_cast<volatile uint8_t*>(addr + bytes - 2) = *(data8 + bytes - 2);
                while (is_busy())
                    ;
            case 1:
                *reinterpret_cast<volatile uint8_t*>(addr + bytes - 1) = *(data8 + bytes - 1);
                while (is_busy())
                    ;
            }
            unset_program_sector();
        }

        static inline void erase_range_sector(addr_t addr, size_t bytes) noexcept
        {
            addr -= MAIN_MEM_START;
            size_t end_addr = addr + bytes;
            for (int i = 0; i < 8; ++i) {
                if (addr > size_map[i]) {
                    addr -= size_map[i];
                    end_addr -= size_map[i];
                } else {
                    set_erase_sector(static_cast<SectorID>(i));
                    start_main();
                    while (is_busy())
                        ;
                    stop_main();
                    unset_erase_sector();
                    if (end_addr <= size_map[i])
                        break;
                    end_addr -= size_map[i];
                }
            }
        }

        bool EmbeddedFlash::is_valid_range(addr_t addr_start, size_t size) const noexcept
        {
            return (addr_start >= MAIN_MEM_START && addr_start + size < MAIN_MEM_END) || (addr_start >= OTP_START && addr_start + size < OTP_END) || (addr_start >= OPT_START && addr_start + size < OPT_END);
        }

        Result<Range> EmbeddedFlash::get_affected_range(addr_t addr) const
        {
            // if (!is_valid_range(addr, 1))
            //     throw std::invalid_argument("Invalid address");
            ELFE_ERROR_IF(
                !is_valid_range(addr, 1),
                Result<Range>(Range {}, EC::FlashInvalidAddr),
                InvalidAddr());

            if (addr >= MAIN_MEM_START && addr < MAIN_MEM_END) {
                size_t start = MAIN_MEM_START;
                for (auto size : size_map) {
                    if (addr < start + size)
                        return Range(start, start + size);
                    start += size;
                }
            }

            return Range(addr, addr + 1_Bytes);
        }

        VoidResult<> EmbeddedFlash::write_bytes(addr_t addr, const void* data, size_t size)
        {
            VoidResult<> r;
            r = raise_if_error();
            ELFE_PROP(r, r);

            if (addr >= OPT_START && addr < OPT_END) {
                r = unlockopts();
                ELFE_PROP(r, r);
                write_data(addr, data, size);
                lockopts();
            } else if ((addr >= MAIN_MEM_START && addr < MAIN_MEM_END) || (addr >= OTP_START && addr < OTP_END)) {
                r = unlock();
                ELFE_PROP(r, r);
                write_data(addr, data, size);
                lock();
            } else {
                ELFE_ERROR(EC::InvalidArgument, std::invalid_argument("Invalid address"));
            }

            return raise_if_error();
        }
        VoidResult<> EmbeddedFlash::read_bytes(addr_t addr, void* data, size_t size) const
        {
            ELFE_ERROR_IF(
                !is_valid_range(addr, size),
                EC::FlashInvalidAddr,
                InvalidAddr());

            while (is_busy())
                ; // wait for flash to be ready

            std::memcpy(data, reinterpret_cast<const void*>(addr), size);

            return raise_if_error();
        }
        VoidResult<> EmbeddedFlash::erase(addr_t addr, size_t size)
        {
            VoidResult<> r;
            ELFE_ERROR_IF(
                !is_valid_range(addr, size),
                EC::FlashInvalidAddr,
                InvalidAddr());

            while (is_busy())
                ; // wait for flash to be ready

            if (addr >= MAIN_MEM_START && addr < MAIN_MEM_END) {
                r = unlock();
                ELFE_PROP(r, r);
                erase_range_sector(addr, size);
                lock();
            } else if (addr >= OPT_START && addr < OPT_END) {
                r = unlock();
                ELFE_PROP(r, r);
                r = unlockopts();
                ELFE_PROP(r, r);
                set_erase_sector(SectorID::Options);
                start_option();
                while (is_busy())
                    ;
                stop_option();
                unset_erase_sector();
                lockopts();
                lock();
            } else {
                ELFE_ERROR(EC::InvalidArgument, std::invalid_argument("Invalid address"));
            }

            return raise_if_error();
        }
        VoidResult<> EmbeddedFlash::erase_all()
        {
            VoidResult<> r;
            r = unlock();
            ELFE_PROP(r, r);
            while (is_busy())
                ; // wait for flash to be ready
            set_erase_all();
            start_main();
            while (is_busy())
                ; // wait for flash to be ready
            unset_erase_all();
            stop_main();
            lock();
            reset_ins_cache();

            return raise_if_error();
        }

        VoidResult<> EmbeddedFlash::enable_interrupt_error() const
        {
            auto r = unlock();
            ELFE_PROP(r, r);
            detail::flash_reg.CR |= (1U << 25U);
            lock();
            nvic::enable_irq(irqn);
            return EC::None;
        }
        VoidResult<> EmbeddedFlash::disable_interrupt_error() const
        {
            auto r = unlock();
            ELFE_PROP(r, r);
            detail::flash_reg.CR &= ~(1U << 25U);
            lock();
            return EC::None;
        }
        VoidResult<> EmbeddedFlash::enable_interrupt_complete() const
        {
            auto r = unlock();
            ELFE_PROP(r, r);
            detail::flash_reg.CR |= (1U << 24U);
            lock();
            nvic::enable_irq(irqn);
            return EC::None;
        }
        VoidResult<> EmbeddedFlash::disable_interrupt_complete() const
        {
            auto r = unlock();
            ELFE_PROP(r, r);
            detail::flash_reg.CR &= ~(1U << 24U);
            lock();
            return EC::None;
        }
        void EmbeddedFlash::clear_error() const noexcept
        {
            detail::flash_reg.SR = 0xF9U << 1; // clear error flags
        }
        VoidResult<> EmbeddedFlash::raise_if_error(bool clear) const
        {
            if (!(detail::flash_reg.SR & (0xF9 << 1)))
                return EC::None;

            if (detail::flash_reg.SR & (0xF0U << 1)) {
                if (detail::flash_reg.SR & (0xC0U << 1)) {
                    if (detail::flash_reg.SR & FLASH_SR_RDERR) {
                        if (clear)
                            detail::flash_reg.SR &= ~FLASH_SR_RDERR;
                        ELFE_ERROR(EC::FlashReadProtected, ReadProtected());
                    } else {
                        if (clear)
                            detail::flash_reg.SR &= ~FLASH_SR_PGSERR;
                        ELFE_ERROR(EC::FlashInvalidConfig, InvalidConfig());
                    }
                } else {
                    if (detail::flash_reg.SR & FLASH_SR_PGPERR) {
                        if (clear)
                            detail::flash_reg.SR &= ~FLASH_SR_PGPERR;
                        ELFE_ERROR(EC::FlashParallelSizeMismatch, ParallelSizeMismatch());
                    } else {
                        if (clear)
                            detail::flash_reg.SR &= ~FLASH_SR_PGAERR;
                        ELFE_ERROR(EC::FlashAlignmentError, AlignmentError());
                    }
                }
            } else {
                if (detail::flash_reg.SR & FLASH_SR_WRPERR) {
                    if (clear)
                        detail::flash_reg.SR &= ~FLASH_SR_WRPERR;
                    ELFE_ERROR(EC::FlashWriteProtected, WriteProtected());
                } else {
                    ELFE_ERROR(EC::FlashError, FlashError());
                }
            }
            return EC::None;
        }
        void EmbeddedFlash::on_error_handler() const noexcept
        try {
            if (not(detail::flash_reg.CR & (1U << 25U)))
                return;
            try {
                [[maybe_unused]] auto r = raise_if_error(false);
            } catch (const FlashError& err) {
                detail::flash_reg.SR = 0x2U; // clear interrupt flag
                if (on_error)
                    on_error(err);
            }
        } catch (...) {
        }

        void EmbeddedFlash::on_complete_handler() const noexcept
        try {
            if (detail::flash_reg.SR & 1U and detail::flash_reg.CR & (1U << 24U)) {
                detail::flash_reg.SR = 1U; // clear flag
                if (on_complete)
                    on_complete();
            }
        } catch (...) {
        }

        extern "C" {
        void FLASH_IRQHandler()
        {
            Flash.global_interrupt_handler();
        }
        }

    }
}
}
