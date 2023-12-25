#pragma once

#include <cstdint>
#include "property.hpp"
#include "w25qxx_const.hpp"
#include "gpio.hpp"
#include "spi.hpp"
#include "flash.hpp"
#include "units.hpp"

namespace vermils
{
namespace w25qxx
{
    using namespace stm32;
    using flash::addr_t;
    using flash::UnitRange;
    using flash::FlashException;
    class UnknownModel : public FlashException
    {
    public:
        UnknownModel() : FlashException("Unknown flash model") {}
    };
    /**
     * @brief W25Qxx flash class
     * 
     * @throw `FlashError[std::runtime_error]`
     * @throw `UnknownModel[FlashError]` for unknown model in constructor
     */
    class Flash : public flash::BaseFlash
    {
        template <typename T>
        struct _Property : public tricks::StaticProperty<T, Flash&>
        {
            constexpr _Property(Flash &owner) : tricks::StaticProperty<T, Flash&>(owner) {}
            using tricks::StaticProperty<T, Flash&>::operator=;
        };
        template <typename T>
        struct _ROProperty : public tricks::StaticReadOnlyProperty<T, Flash&>
        {
            constexpr _ROProperty(Flash &owner) : tricks::StaticReadOnlyProperty<T, Flash&>(owner) {}
            using tricks::StaticReadOnlyProperty<T, Flash&>::operator=;
        };
        struct _StatusReg : public _Property<uint32_t>
        {
            constexpr _StatusReg(Flash &owner) : _Property<uint32_t>(owner) {}
            using _Property<uint32_t>::operator=;
            void setter(uint32_t value) const noexcept override
            {
                owner._write_sr(value);
            }
            uint32_t getter() const noexcept override
            {
                return owner._read_sr();
            }
        };
        void _select() const noexcept
        {
            cs.set();
            while (!cs);
            cs.reset();
        }
        void _deselect() const noexcept
        {
            cs.set();
            while (!cs);
        }
        bool _is_selected() const noexcept
        {
            return !cs;
        }
        void _send_byte(uint8_t byte) const noexcept
        {
            spi.put<uint8_t>(byte);
        }
        uint8_t _recv_byte() const noexcept
        {
            return spi.get<uint8_t>();
        }
        void _send_addr(addr_t addr) const noexcept
        {
            _send_byte((addr >> 16) & 0xff);
            _send_byte((addr >> 8) & 0xff);
            _send_byte(addr & 0xff);
        }
        void _send_ins(Instruction ins) const noexcept
        {
            _send_byte(static_cast<uint8_t>(ins));
        }
        void _write_enable() const noexcept
        {
            _select();
            _send_ins(Instruction::WriteEnable);
            _deselect();
        }
        void _write_disable() const noexcept
        {
            _select();
            _send_ins(Instruction::WriteDisable);
            _deselect();
        }
        void _write_volatile_sr_enable() const noexcept
        {
            _select();
            _send_ins(Instruction::VolatileSRWriteEnable);
            _deselect();
        }
        uint8_t _read_sr1() const noexcept
        {
            _select();
            _send_ins(Instruction::ReadStatusRegister1);
            uint8_t sr1 = _recv_byte();
            _deselect();
            return sr1;
        }
        void _write_sr1(uint8_t sr1) const noexcept
        {
            _write_enable();
            _write_volatile_sr_enable();
            _write_enable();
            _select();
            _send_ins(Instruction::WriteStatusRegister);
            _send_byte(sr1);
            _deselect();
        }
        uint8_t _read_sr2() const noexcept
        {
            _select();
            _send_ins(Instruction::ReadStatusRegister2);
            uint8_t sr2 = _recv_byte();
            _deselect();
            return sr2;
        }
        void _write_sr2(uint8_t sr2) const noexcept
        {
            wait_busy();
            _write_volatile_sr_enable();
            _write_enable();
            _select();
            _send_ins(Instruction::WriteStatusRegister2);
            _send_byte(sr2);
            _deselect();
        }
        uint8_t _read_sr3() const noexcept
        {
            _select();
            _send_ins(Instruction::ReadStatusRegister3);
            uint8_t sr3 = _recv_byte();
            _deselect();
            return sr3;
        }
        void _write_sr3(uint8_t sr3) const noexcept
        {
            wait_busy();
            _write_volatile_sr_enable();
            _write_enable();
            _select();
            _send_ins(Instruction::WriteStatusRegister3);
            _send_byte(sr3);
            _deselect();
        }
        uint32_t _read_sr() const noexcept
        {
            return (_read_sr3() << 16) | (_read_sr2() << 8) | _read_sr1();
        }
        void _write_sr(uint32_t sr) const noexcept
        {
            _write_sr1(sr & 0xff);
            _write_sr2((sr >> 8) & 0xff);
            _write_sr3((sr >> 16) & 0xff);
        }
        void _update_max_block(const uint32_t model_id) const noexcept
        {
            switch (model_id & 0xFFU)
            {
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
        spi::BaseInterface &spi;
        gpio::Pin cs;
        _StatusReg status_register{*this};
        Flash(spi::BaseInterface &spi, gpio::Pin cs) : spi(spi), cs(cs)
        {
            cs.load(gpio::PinConfig(gpio::PinConfig::Output, gpio::PinConfig::High, gpio::PinConfig::PushPull));
            cs.set();
            spi.set_mode(spi::Mode::Mode0);
            spi.turn_slave(false);
            get_model_id(); // update _MAX_BLOCK
            if (!_MAX_BLOCK)
            {
                throw UnknownModel();
            }
        }
        bool is_busy() const noexcept
        {
            return _read_sr1() & status_mask::SR1_BUSY;
        }

        void wait_busy() const noexcept
        {
            while (is_busy());
        }

        /**
         * @brief Get the model id (manufacturer id and device id)
         * 
         * @return uint32_t manu_id[23:16] | device_id[15:0]
         */
        uint32_t get_model_id() const noexcept
        {
            _select();
            _send_ins(Instruction::JEDECID);
            uint32_t id = _recv_byte() << 16;
            id |= _recv_byte() << 8;
            id |= _recv_byte();
            _deselect();
            _update_max_block(id);
            return id;
        }

        uint64_t get_unique_id() const noexcept
        {
            _select();
            _send_ins(Instruction::ReadUniqueID);
            for (int i=4; i>0; --i)
                _recv_byte(); // dummy
            uint64_t id = 0;
            for (int i = 0; i < 8; ++i)
            {
                id <<= 8;
                id |= _recv_byte();
            }
            _deselect();
            return id;
        }

        bool is_valid_range(addr_t addr_start, size_t size) const noexcept override
        {
            return addr_start + size <= _MAX_BLOCK * BLOCK_SIZE;
        }
        /**
         * @brief returns affected unit range in start and end address, a unit is a smallest unit that can be erased at procided address
         * 
         * @param addr 
         * @return UnitRange 
         * @throw invalid_argument if addr is not valid
         */
        UnitRange get_unit_range(addr_t addr) const override
        {
            addr_t start = addr & ~(SECTOR_SIZE - 1);
            addr_t end = start + SECTOR_SIZE;
            return UnitRange(start, end);
        }
        /**
         * @throw invalid_argument if addr is not valid
         * @throw FlashError (std::runtime_error)
         * @throw ValidationError (FlashError & std::runtime_error)
         */
        void write_bytes(addr_t addr, const void * data, size_t bytes) override
        {
            if (not is_valid_range(addr, bytes) or addr < SFDP_PAGE_ADDR + PAGE_SIZE)
                throw std::invalid_argument("Invalid address");
            auto data_ptr = static_cast<const uint8_t*>(data);

            while (bytes > 0)
            {
                const addr_t PAGE_START = addr & ~(PAGE_SIZE - 1);
                const addr_t PAGE_END = PAGE_START + PAGE_SIZE;
                wait_busy();
                _write_enable();
                _select();

                if (PAGE_START >= SECURE_PAGE_ADDR1 and PAGE_START <= SECURE_PAGE_ADDR3)
                    _send_ins(Instruction::ProgramSecurityRegister);
                else
                    _send_ins(Instruction::PageProgram);
                _send_addr(addr);
                while (addr < PAGE_END and bytes)
                {
                    _send_byte(*data_ptr++);
                    ++addr;
                    --bytes;
                }
                _deselect();
            }
        }
        /**
         * @throw FlashError
         * @throw invalid_argument if addr is not valid
         */
        void read_bytes(addr_t addr, void * data, size_t bytes) const override
        {
            if (not is_valid_range(addr, bytes))
                throw std::invalid_argument("Invalid address");
            auto data_ptr = static_cast<uint8_t*>(data);
            wait_busy();
            if (addr < SFDP_PAGE_ADDR + PAGE_SIZE)
            {
                _select();
                _send_ins(Instruction::ReadSFDPRegister);
                _send_addr(addr);
                _recv_byte(); // dummy
                while (addr++ < SFDP_PAGE_ADDR + PAGE_SIZE and bytes--)
                {
                    *data_ptr++ = _recv_byte();
                }
                _deselect();
            }
            _select();
            _send_ins(Instruction::ReadData);
            _send_addr(addr);
            while (bytes--)
            {
                if (addr >= SECURE_PAGE_ADDR1 and addr < SECURE_PAGE_ADDR3 + PAGE_SIZE)
                {
                    _deselect();
                    _select();
                    _send_ins(Instruction::ReadSecurityRegister);
                    _send_addr(addr);
                    _recv_byte(); // dummy
                    while (addr++ < SECURE_PAGE_ADDR3 + PAGE_SIZE and bytes--)
                    {
                        *data_ptr++ = _recv_byte();
                    }
                    _deselect();
                    if (bytes == 0)
                        return;
                    _select();
                    _send_ins(Instruction::ReadData);
                    _send_addr(addr);
                }
                *data_ptr++ = _recv_byte();
                ++addr;
            }
            _deselect();
        }
        /**
         * @throw FlashError
         * @throw invalid_argument if addr is not valid
         */
        void erase(addr_t addr, size_t bytes) override
        {
            if (not is_valid_range(addr, bytes))
                throw std::invalid_argument("Invalid address");
            while (bytes)
            {
                wait_busy();
                _write_enable();
                _select();
                const addr_t THIS_PAGE = addr & ~(PAGE_SIZE - 1);
                const addr_t THIS_SECTOR = addr & ~(SECTOR_SIZE - 1);
                const addr_t THIS_BLOCK = addr & ~(BLOCK_SIZE - 1);
                if (THIS_PAGE == SFDP_PAGE_ADDR)
                {
                    _send_ins(Instruction::EraseSecurityRegister);
                    _send_addr(THIS_PAGE);
                    addr += PAGE_SIZE;
                    bytes -= PAGE_SIZE;
                }
                else if (THIS_PAGE >= SECURE_PAGE_ADDR1 and THIS_PAGE <= SECURE_PAGE_ADDR3)
                {
                    _send_ins(Instruction::EraseSecurityRegister);
                    _send_addr(THIS_PAGE);
                    addr += PAGE_SIZE;
                    if (bytes < PAGE_SIZE)
                        bytes = 0;
                    else
                        bytes -= PAGE_SIZE;
                }
                else if (THIS_PAGE == THIS_BLOCK and bytes >= 32_KiB)
                {
                    if (bytes >= 64_KiB)
                    {
                        _send_ins(Instruction::BlockErase64K);
                        _send_addr(THIS_BLOCK);
                        addr += 64_KiB;
                        bytes -= 64_KiB;
                    }
                    else
                    {
                        _send_ins(Instruction::BlockErase32K);
                        _send_addr(THIS_BLOCK);
                        addr += 32_KiB;
                        bytes -= 32_KiB;
                    }
                }
                else
                {
                    _send_ins(Instruction::SectorErase);
                    _send_addr(THIS_SECTOR);
                    addr += SECTOR_SIZE;
                    if (bytes < SECTOR_SIZE)
                        bytes = 0;
                    else
                        bytes -= SECTOR_SIZE;
                }

                _deselect();
            }
        }
        /**
         * @throw FlashError
         */
        void erase_all() override
        {
            wait_busy();
            _write_enable();
            _select();
            _send_ins(Instruction::ChipErase);
            _deselect();
        }
        void raise_if_error(bool clear=true) const override
        {
            spi.raise_if_error();
        }
    };
}
}