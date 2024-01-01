#pragma once

#include "base_spi.hpp"
#include "nvic.hpp"
#include "rcc.hpp"

namespace vermils
{
namespace stm32
{
namespace spi
{
    namespace detail
    {
        struct Register
        {
            volatile uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
            volatile uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
            #ifdef _VERMIL_STM32FX
            volatile uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
            volatile uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
            volatile uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
            volatile uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
            volatile uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
            volatile uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
            volatile uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
            #elif defined(_VERMIL_STM32HX)
            volatile uint32_t CFG1;          /*!< SPI Configuration register 1,                    Address offset: 0x08 */
            volatile uint32_t CFG2;          /*!< SPI Configuration register 2,                    Address offset: 0x0C */
            volatile uint32_t IER;           /*!< SPI/I2S Interrupt Enable register,               Address offset: 0x10 */
            volatile uint32_t SR;            /*!< SPI/I2S Status register,                         Address offset: 0x14 */
            volatile uint32_t IFCR;          /*!< SPI/I2S Interrupt/Status flags clear register,   Address offset: 0x18 */
            uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                        */
            volatile uint32_t TXDR;          /*!< SPI/I2S Transmit data register,                  Address offset: 0x20 */
            uint32_t      RESERVED1[3];  /*!< Reserved, 0x24-0x2C                                                   */
            volatile uint32_t RXDR;          /*!< SPI/I2S Receive data register,                   Address offset: 0x30 */
            uint32_t      RESERVED2[3];  /*!< Reserved, 0x34-0x3C                                                   */
            volatile uint32_t CRCPOLY;       /*!< SPI CRC Polynomial register,                     Address offset: 0x40 */
            volatile uint32_t TXCRC;         /*!< SPI Transmitter CRC register,                    Address offset: 0x44 */
            volatile uint32_t RXCRC;         /*!< SPI Receiver CRC register,                       Address offset: 0x48 */
            volatile uint32_t UDRDR;         /*!< SPI Underrun data register,                      Address offset: 0x4C */
            volatile uint32_t I2SCFGR;       /*!< I2S Configuration register,                      Address offset: 0x50 */
            #endif
        };
    }

    class HardwareInterface : public BaseInterface
    {
        template<typename T>
        struct _Property : tricks::StaticProperty<T, HardwareInterface&>
        {
            constexpr _Property(HardwareInterface &spi) : tricks::StaticProperty<T, HardwareInterface&>(spi) {}
            using tricks::StaticProperty<T, HardwareInterface&>::operator=;
        };
        struct _BaudRate : _Property<uint32_t>
        {
            constexpr _BaudRate(HardwareInterface &spi) : _Property<uint32_t>(spi) {}
            using _Property<uint32_t>::operator=;
            uint32_t getter() const override;
            void setter(uint32_t) const override;
        };
    public:
        using CallbackType = std::function<void()>;
        detail::Register &reg;
        const uint8_t order;
        const nvic::IRQn_Type irqn;
        _BaudRate baudrate{*this};
        CallbackType on_error;
        CallbackType on_send_ready;
        CallbackType on_recv_ready;

        HardwareInterface(detail::Register &reg, uint8_t order, nvic::IRQn_Type irqn) : reg(reg), order(order), irqn(irqn) {}

        void turn_slave(bool yes) noexcept override final;
        bool is_slave() const noexcept override final;
        void set_mode(Mode mode) override final;
        Mode get_mode() const noexcept override final;
        size_t exchange_bytes(const void *tx, void *rx, size_t size) override final;
        void select_as_slave(bool yes) override;
        bool is_selected_as_slave() const noexcept override;
        void raise_if_error() const override final;

        void init();
        void deinit();
        void use_16bits(bool yes);
        bool is_using_16bits() const noexcept;
        void use_software_ss(bool yes);
        bool is_using_software_ss() const noexcept;
        void set_msb_first(bool yes);
        bool is_msb_first() const noexcept;

        void config_half_duplex(bool bidirectional=false, bool rxonly=false);
        bool is_half_duplex() const noexcept;
        bool is_bidirectional() const noexcept;
        bool is_rxonly() const noexcept;

        void enable_on_send_interrupt() const noexcept;
        void enable_on_recv_interrupt() const noexcept;
        void enable_on_error_interrupt() const noexcept;
        void disable_on_send_interrupt() const noexcept;
        void disable_on_recv_interrupt() const noexcept;
        void disable_on_error_interrupt() const noexcept;

        void enable_interrupts() const noexcept
        {
            enable_on_send_interrupt();
            enable_on_recv_interrupt();
            enable_on_error_interrupt();
        }
        void disable_interrupts() const noexcept
        {
            nvic::disable_irq(irqn);
            disable_on_send_interrupt();
            disable_on_recv_interrupt();
            disable_on_error_interrupt();
        }

        void set_irq_priority(const uint8_t priority=8) const
        {
            nvic::set_priority(irqn, priority);
        }

        void on_send_handler() const noexcept;
        void on_recv_handler() const noexcept;
        void on_error_handler() const noexcept;

        void global_interrupt_handler() const noexcept
        {
            on_send_handler();
            on_recv_handler();
            on_error_handler();
        }
    };


extern HardwareInterface Spi1;
extern HardwareInterface Spi2;
extern HardwareInterface Spi3;
extern HardwareInterface Spi4;
extern HardwareInterface Spi5;
extern HardwareInterface Spi6;

}
}
}