#pragma once

#include "errors.hpp"
#include "extra/w25qxx/w25qxx_const.hpp"
#include "flash/flash.hpp"
#include "gpio/gpio.hpp"
#include "result.hpp"
#include "spi/spi.hpp"
#include "units.hpp"
#include "utils/property.hpp"
#include <cstdint>

namespace elfe {
namespace w25qxx {
    using namespace stm32;
    using EC = err::ErrorCode;
    using err::flash::FlashError;
    using flash::addr_t;
    using flash::Range;
    class UnknownModel : public FlashError {
    public:
        UnknownModel()
            : FlashError("Unknown flash model")
        {
        }
    };
    /**
     * @brief W25Qxx flash class
     *
     * @throw `FlashError[std::runtime_error]`
     * @throw `UnknownModel[FlashError]` for unknown model in constructor
     */
    class Flash : public flash::BaseFlash {
        void _select() const noexcept
        {
            cs.set();
            while (!cs)
                ;
            cs.reset();
        }
        void _deselect() const noexcept
        {
            cs.set();
            while (!cs)
                ;
        }
        bool _is_selected() const noexcept
        {
            return !cs;
        }
        VoidResult<> _send_byte(uint8_t byte) const
        {
            return spi.put<uint8_t>(byte);
        }
        Result<uint8_t> _recv_byte() const
        {
            return spi.get<uint8_t>();
        }
        VoidResult<> _send_addr(addr_t addr) const
        {
            auto r = _send_byte((addr >> 16) & 0xff);
            ELFE_PROP(r, r);
            r = _send_byte((addr >> 8) & 0xff);
            ELFE_PROP(r, r);
            return _send_byte(addr & 0xff);
        }
        VoidResult<> _send_ins(Instruction ins) const
        {
            return _send_byte(static_cast<uint8_t>(ins));
        }
        VoidResult<> _write_enable() const
        {
            _select();
            auto r = _send_ins(Instruction::WriteEnable);
            ELFE_PROP(r, r);
            _deselect();
            return EC::None;
        }
        VoidResult<> _write_disable() const
        {
            _select();
            auto r = _send_ins(Instruction::WriteDisable);
            ELFE_PROP(r, r);
            _deselect();
            return EC::None;
        }
        VoidResult<> _write_volatile_sr_enable() const
        {
            _select();
            auto r = _send_ins(Instruction::VolatileSRWriteEnable);
            ELFE_PROP(r, r);
            _deselect();
            return EC::None;
        }
        Result<uint8_t> _read_sr1() const
        {
            _select();
            auto r = _send_ins(Instruction::ReadStatusRegister1);
            ELFE_PROP(r, Result<uint8_t>(0, r.error));
            auto ru = _recv_byte();
            ELFE_PROP(ru, ru);
            uint8_t sr1 = ru.value;
            _deselect();
            return sr1;
        }
        VoidResult<> _write_sr1(uint8_t sr1) const
        {
            auto r = (_write_enable() && _write_volatile_sr_enable() && _write_enable());
            ELFE_PROP(r, r);
            _select();
            r = _send_ins(Instruction::WriteStatusRegister);
            ELFE_PROP(r, r);
            r = _send_byte(sr1);
            ELFE_PROP(r, r);
            _deselect();
        }
        Result<uint8_t> _read_sr2() const
        {
            _select();
            auto r = _send_ins(Instruction::ReadStatusRegister2);
            ELFE_PROP(r, Result<uint8_t>(0, r.error));
            auto ru = _recv_byte();
            ELFE_PROP(ru, ru);
            uint8_t sr2 = ru.value;
            _deselect();
            return sr2;
        }
        VoidResult<> _write_sr2(uint8_t sr2) const
        {
            auto r = wait_busy();
            ELFE_PROP(r, r);
            r = _write_volatile_sr_enable();
            ELFE_PROP(r, r);
            r = _write_enable();
            ELFE_PROP(r, r);
            _select();
            r = _send_ins(Instruction::WriteStatusRegister2);
            ELFE_PROP(r, r);
            r = _send_byte(sr2);
            ELFE_PROP(r, r);
            _deselect();
            return EC::None;
        }
        Result<uint8_t> _read_sr3() const
        {
            _select();
            auto r = _send_ins(Instruction::ReadStatusRegister3);
            ELFE_PROP(r, Result<uint8_t>(0, r.error));
            auto ru = _recv_byte();
            ELFE_PROP(ru, ru);
            uint8_t sr3 = ru.value;
            _deselect();
            return sr3;
        }
        VoidResult<> _write_sr3(uint8_t sr3) const
        {
            auto r = wait_busy();
            ELFE_PROP(r, r);
            r = _write_volatile_sr_enable();
            ELFE_PROP(r, r);
            r = _write_enable();
            ELFE_PROP(r, r);
            _select();
            r = _send_ins(Instruction::WriteStatusRegister3);
            ELFE_PROP(r, r);
            r = _send_byte(sr3);
            ELFE_PROP(r, r);
            _deselect();
            return EC::None;
        }

        void _update_max_block(const uint32_t model_id) const noexcept
        {
            switch (model_id & 0xFFU) {
            case 0x20U: // Q512
                _MAX_BLOCK = 1024;
                break;
            case 0x19U: // Q256
                _MAX_BLOCK = 512;
                break;
            case 0x18U: // Q128
                _MAX_BLOCK = 256;
                break;
            case 0x17U: // Q64
                _MAX_BLOCK = 128;
                break;
            case 0x16U: // Q32
                _MAX_BLOCK = 64;
                break;
            case 0x15U: // Q16
                _MAX_BLOCK = 32;
                break;
            case 0x14U: // Q80
                _MAX_BLOCK = 16;
                break;
            case 0x13U: // Q40
                _MAX_BLOCK = 8;
                break;
            case 0x12U: // Q20
                _MAX_BLOCK = 4;
                break;
            default:
                _MAX_BLOCK = 0;
            }
        }
        mutable uint32_t _MAX_BLOCK;

    public:
        static constexpr const unsigned PAGE_SIZE = 256;
        static constexpr const unsigned SECTOR_SIZE = PAGE_SIZE * 16;
        static constexpr const unsigned BLOCK_SIZE = SECTOR_SIZE * 16;
        static constexpr const addr_t SECURE_PAGE_ADDR1 = 0x1000;
        static constexpr const addr_t SECURE_PAGE_ADDR2 = 0x2000;
        static constexpr const addr_t SECURE_PAGE_ADDR3 = 0x3000;
        static constexpr const addr_t SFDP_PAGE_ADDR = 0x0;
        spi::BaseInterface& spi;
        gpio::Pin cs;
        Flash(spi::BaseInterface& spi, gpio::Pin cs)
            : spi(spi)
            , cs(cs)
        {
            cs.load(gpio::PinConfig(gpio::PinConfig::Output, gpio::PinConfig::High, gpio::PinConfig::PushPull));
            cs.set();
            auto r = spi.set_mode(spi::Mode::Mode0);
            ELFE_PANIC_IF(!r.ok(), FlashError("W25Qxx spi mode setting failed"));
            r = spi.turn_slave(false);
            ELFE_PANIC_IF(!r.ok(), FlashError("W25Qxx spi slave setting failed"));
            auto ru = get_model_id(); // update _MAX_BLOCK
            ELFE_PANIC_IF(!ru.ok(), FlashError("W25Qxx model id read failed"));
            ELFE_PANIC_IF(!_MAX_BLOCK, UnknownModel());
        }
        Result<bool> is_busy() const
        {
            auto r = _read_sr1();
            ELFE_PROP(r, Result<bool>(true, r.error));
            return r.value & status_mask::SR1_BUSY;
        }

        VoidResult<> wait_busy() const
        {
            // while (is_busy())
            //     ;
            auto r = Result<bool>(true);
            do{
                r = is_busy();
                ELFE_PROP(r, r.error);
            }
            while(r.value);
            return EC::None;
        }

        Result<uint32_t> get_status_register() const
        {
            auto r1 = _read_sr1();
            ELFE_PROP(r1, r1);
            auto r2 = _read_sr2();
            ELFE_PROP(r2, r2);
            auto r3 = _read_sr3();
            ELFE_PROP(r3, r3);
            return (r3.value << 16) | (r2.value << 8) | r1.value;
        }

        VoidResult<> set_status_register(uint32_t sr) const
        {
            auto r = _write_sr1(sr & 0xff);
            ELFE_PROP(r, r);
            r = _write_sr2((sr >> 8) & 0xff);
            ELFE_PROP(r, r);
            return _write_sr3((sr >> 16) & 0xff);
        }

        /**
         * @brief Get the model id (manufacturer id and device id)
         *
         * @return uint32_t manu_id[23:16] | device_id[15:0]
         */
        Result<uint32_t> get_model_id() const
        {
            _select();
            auto r = _send_ins(Instruction::JEDECID);
            ELFE_PROP(r, Result<uint32_t>(0, r.error));
            auto ru = _recv_byte();
            ELFE_PROP(ru, ru);
            uint32_t id = static_cast<uint32_t>(ru.value) << 16;
            ru = _recv_byte();
            ELFE_PROP(ru, ru);
            id |= static_cast<uint32_t>(ru.value) << 8;
            ru = _recv_byte();
            ELFE_PROP(ru, ru);
            id |= ru.value;
            _deselect();
            _update_max_block(id);
            return id;
        }

        Result<uint64_t> get_unique_id() const
        {
            _select();
            auto r = _send_ins(Instruction::ReadUniqueID);
            ELFE_PROP(r, Result<uint64_t>(0, r.error));
            for (int i = 4; i > 0; --i)
            {
                auto ru = _recv_byte(); // dummy
                ELFE_PROP(ru, ru);
            }
            uint64_t id = 0;
            for (int i = 0; i < 8; ++i) {
                id <<= 8;
                auto ru = _recv_byte();
                ELFE_PROP(ru, ru);
                id |= ru.value;
            }
            _deselect();
            return id;
        }

        bool is_valid_range(addr_t addr_start, size_t size) const noexcept override
        {
            return addr_start + size <= _MAX_BLOCK * BLOCK_SIZE;
        }
        /**
         * @brief returns affected range in start and end address, a unit is a smallest unit that can be erased at procided address
         *
         * @param addr
         * @return Range
         * @throw invalid_argument if addr is not valid
         */
        Result<Range> get_affected_range(addr_t addr) const override
        {
            addr_t start = addr & ~(SECTOR_SIZE - 1);
            addr_t end = start + SECTOR_SIZE;
            return Range(start, end);
        }
        /**
         * @throw invalid_argument if addr is not valid
         * @throw FlashError (std::runtime_error)
         * @throw ValidationError (FlashError & std::runtime_error)
         */
        VoidResult<> write_bytes(addr_t addr, const void* data, size_t bytes) override
        {
            if (not is_valid_range(addr, bytes) or addr < SFDP_PAGE_ADDR + PAGE_SIZE)
                throw std::invalid_argument("Invalid address");
            auto data_ptr = static_cast<const uint8_t*>(data);

            while (bytes > 0) {
                const addr_t PAGE_START = addr & ~(PAGE_SIZE - 1);
                const addr_t PAGE_END = PAGE_START + PAGE_SIZE;
                auto r = wait_busy();
                ELFE_PROP(r, r);
                r = _write_enable();
                ELFE_PROP(r, r);
                _select();

                if (PAGE_START >= SECURE_PAGE_ADDR1 and PAGE_START <= SECURE_PAGE_ADDR3)
                    r = _send_ins(Instruction::ProgramSecurityRegister);
                else
                    r = _send_ins(Instruction::PageProgram);
                ELFE_PROP(r, r);
                r = _send_addr(addr);
                ELFE_PROP(r, r);
                while (addr < PAGE_END and bytes) {
                    r = _send_byte(*data_ptr++);
                    ELFE_PROP(r, r);
                    ++addr;
                    --bytes;
                }
                _deselect();
            }
            return EC::None;
        }
        /**
         * @throw FlashError
         * @throw invalid_argument if addr is not valid
         */
        VoidResult<> read_bytes(addr_t addr, void* data, size_t bytes) const override
        {
            if (not is_valid_range(addr, bytes))
                throw std::invalid_argument("Invalid address");
            auto data_ptr = static_cast<uint8_t*>(data);
            auto r = wait_busy();
            ELFE_PROP(r, r);
            if (addr < SFDP_PAGE_ADDR + PAGE_SIZE) {
                _select();
                r = _send_ins(Instruction::ReadSFDPRegister);
                ELFE_PROP(r, r);
                r = _send_addr(addr);
                ELFE_PROP(r, r);
                auto ru = _recv_byte(); // dummy
                ELFE_PROP(ru, ru.error);
                while (addr++ < SFDP_PAGE_ADDR + PAGE_SIZE and bytes--) {
                    ru = _recv_byte();
                    ELFE_PROP(ru, ru.error);
                    *data_ptr++ = ru.value; 
                }
                _deselect();
            }
            _select();
            r = _send_ins(Instruction::ReadData);
            ELFE_PROP(r, r);
            r = _send_addr(addr);
            ELFE_PROP(r, r);
            while (bytes--) {
                if (addr >= SECURE_PAGE_ADDR1 and addr < SECURE_PAGE_ADDR3 + PAGE_SIZE) {
                    _deselect();
                    _select();
                    r = _send_ins(Instruction::ReadSecurityRegister);
                    ELFE_PROP(r, r);
                    r = _send_addr(addr);
                    ELFE_PROP(r, r);
                    auto ru = _recv_byte(); // dummy
                    ELFE_PROP(ru, ru.error);
                    while (addr++ < SECURE_PAGE_ADDR3 + PAGE_SIZE and bytes--) {
                        ru = _recv_byte();
                        ELFE_PROP(ru, ru.error);
                        *data_ptr++ = ru.value;
                    }
                    _deselect();
                    if (bytes == 0)
                        return EC::None;
                    _select();
                    r = _send_ins(Instruction::ReadData);
                    ELFE_PROP(r, r);
                    r = _send_addr(addr);
                    ELFE_PROP(r, r);
                }
                auto ru = _recv_byte();
                ELFE_PROP(ru, ru.error);
                *data_ptr++ = ru.value;
                ++addr;
            }
            _deselect();
            return EC::None;
        }
        /**
         * @throw FlashError
         * @throw invalid_argument if addr is not valid
         */
        VoidResult<> erase(addr_t addr, size_t bytes) override
        {
            if (not is_valid_range(addr, bytes))
                throw std::invalid_argument("Invalid address");
            while (bytes) {
                auto r = wait_busy();
                ELFE_PROP(r, r);
                r = _write_enable();
                ELFE_PROP(r, r);
                _select();
                const addr_t THIS_PAGE = addr & ~(PAGE_SIZE - 1);
                const addr_t THIS_SECTOR = addr & ~(SECTOR_SIZE - 1);
                const addr_t THIS_BLOCK = addr & ~(BLOCK_SIZE - 1);
                if (THIS_PAGE == SFDP_PAGE_ADDR) {
                    r = _send_ins(Instruction::EraseSecurityRegister);
                    ELFE_PROP(r, r);
                    r = _send_addr(THIS_PAGE);
                    ELFE_PROP(r, r);
                    addr += PAGE_SIZE;
                    bytes -= PAGE_SIZE;
                } else if (THIS_PAGE >= SECURE_PAGE_ADDR1 and THIS_PAGE <= SECURE_PAGE_ADDR3) {
                    r = _send_ins(Instruction::EraseSecurityRegister);
                    ELFE_PROP(r, r);
                    r = _send_addr(THIS_PAGE);
                    ELFE_PROP(r, r);
                    addr += PAGE_SIZE;
                    if (bytes < PAGE_SIZE)
                        bytes = 0;
                    else
                        bytes -= PAGE_SIZE;
                } else if (THIS_PAGE == THIS_BLOCK and bytes >= 32_KiB) {
                    if (bytes >= 64_KiB) {
                        r = _send_ins(Instruction::BlockErase64K);
                        ELFE_PROP(r, r);
                        r = _send_addr(THIS_BLOCK);
                        ELFE_PROP(r, r);
                        addr += 64_KiB;
                        bytes -= 64_KiB;
                    } else {
                        r = _send_ins(Instruction::BlockErase32K);
                        ELFE_PROP(r, r);
                        r = _send_addr(THIS_BLOCK);
                        ELFE_PROP(r, r);
                        addr += 32_KiB;
                        bytes -= 32_KiB;
                    }
                } else {
                    r = _send_ins(Instruction::SectorErase);
                    ELFE_PROP(r, r);
                    r = _send_addr(THIS_SECTOR);
                    ELFE_PROP(r, r);
                    addr += SECTOR_SIZE;
                    if (bytes < SECTOR_SIZE)
                        bytes = 0;
                    else
                        bytes -= SECTOR_SIZE;
                }

                _deselect();
            }
            return EC::None;
        }
        /**
         * @throw FlashError
         */
        VoidResult<> erase_all() override
        {
            auto r = wait_busy();
            ELFE_PROP(r, r);
            r = _write_enable();
            ELFE_PROP(r, r);
            _select();
            r = _send_ins(Instruction::ChipErase);
            ELFE_PROP(r, r);
            _deselect();
            return EC::None;
        }
        VoidResult<> raise_if_error(bool clear = true) const override
        {
            return spi.raise_if_error();
        }
    };
}
}