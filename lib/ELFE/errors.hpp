#pragma once

#include <exception>
#include <stdexcept>

namespace elfe {
namespace err {
    /**
     * @brief Error codes for the ELFE library
     *
     * @note Range (-10000, -1] is for non-ELFE errors
     * @note Code -1 is for unknown errors
     * @note Range [-1000, 10000) is reserved for ELFE library
     * @note For your own custom errors, you should pick a big enough number to avoid conflicts
     *
     */
    enum class ErrorCode : int {
        LogicError = -4,
        NotImplemented = -3,
        InvalidArgument = -2,
        UnknownError = -1,
        None = 0,
        ELFEError = 1,
        TimeoutError = 2,
        ADCError = 100,
        ADCOverrun = 101,
        AsyncError = 200,
        AsyncCancelled = 201,
        DMAError = 300,
        DMATransferError = 301,
        DMADirectModeError = 302,
        DMAFIFOError = 303,
        FlashError = 400,
        FlashReadProtected = 401,
        FlashWriteProtected = 402,
        FlashInvalidConfig = 403,
        FlashInvalidAddr = 404,
        FlashParallelSizeMismatch = 405,
        FlashAlignmentError = 406,
        FlashValidationError = 407,
        GPIOError = 500,
        I2CError = 600,
        I2CArbitrationLost = 601,
        I2CBusBusy = 602,
        I2CBusError = 603,
        I2CNoSlaveAck = 604,
        I2COverrun = 605,
        I2CTimeout = 606,
        I2CValidationError = 607,
        I2CSMBusAlert = 608,
        SPIError = 700,
        SPICRCError = 701,
        SPIOverrun = 702,
        SPITIFrameError = 703,
        SPIUnderrun = 704,
        SPITimeout = 705,
        SPIModeFault = 706,
        I2SError = 800,
        USARTError = 900,
        USARTOverrun = 901,
        USARTParityError = 902,
        USARTNoiseError = 903,
        USARTFramingError = 904,
        USARTBreakError = 905,

        ExECBBluetoothError = 5000,
        ExECBATError = 5001,
        ExW25QError = 5100,
        ExW25QUnknownModel = 5101,
        ExSSD1306Error = 5200,
        ExSSD1680Error = 5300
    };

    class ELFEError : public std::runtime_error {
    public:
        ELFEError(const char* msg = "ELFE Error")
            : std::runtime_error(msg)
        {
        }
        virtual ~ELFEError() = default;
    };

    class TimeoutError : public ELFEError {
    public:
        TimeoutError(const char* msg = "Timeout")
            : ELFEError(msg)
        {
        }
    };

    namespace adc {
        class ADCError : public ELFEError {
        public:
            ADCError(const char* msg = "ADC Error")
                : ELFEError(msg)
            {
            }
        };

        class Overrun : public ADCError {
        public:
            Overrun(const char* msg = "ADC Overrun")
                : ADCError(msg)
            {
            }
        };
    }

    namespace async {
        class AsyncError : public ELFEError {
        public:
            AsyncError(const char* msg = "Async Error")
                : ELFEError(msg)
            {
            }
        };

        class CancelledError : public AsyncError {
        public:
            CancelledError(const char* msg = "Async Cancelled")
                : AsyncError(msg)
            {
            }
        };
    }

    namespace dma {
        class DMAError : public ELFEError {
        public:
            DMAError(const char* msg = "DMA Error")
                : ELFEError(msg)
            {
            }
        };

        class TransferError : public DMAError {
        public:
            TransferError(const char* msg = "DMA Transfer Error")
                : DMAError(msg)
            {
            }
        };

        class DirectModeError : public DMAError {
        public:
            DirectModeError(const char* msg = "Direct Mode Error")
                : DMAError(msg)
            {
            }
        };

        class FIFOError : public DMAError {
        public:
            FIFOError(const char* msg = "FIFO Error")
                : DMAError(msg)
            {
            }
        };
    }

    namespace flash {
        class FlashError : public ELFEError {
        public:
            FlashError(const char* msg = "Flash Error")
                : ELFEError(msg)
            {
            }
        };

        class ReadProtected : public FlashError {
        public:
            ReadProtected()
                : FlashError("Attempt to write read protected flash address")
            {
            }
        };

        class WriteProtected : public FlashError {
        public:
            WriteProtected()
                : FlashError("Attempt to write write protected flash address")
            {
            }
        };

        class InvalidConfig : public FlashError {
        public:
            InvalidConfig()
                : FlashError("Flash registers are not configured properly")
            {
            }
        };

        class InvalidAddr : public FlashError {
        public:
            InvalidAddr()
                : FlashError("Invalid flash address")
            {
            }
        };

        class ParallelSizeMismatch : public FlashError {
        public:
            ParallelSizeMismatch()
                : FlashError("Parallel size mismatch")
            {
            }
        };

        class AlignmentError : public FlashError {
        public:
            AlignmentError()
                : FlashError("Alignment error")
            {
            }
        };

        class ValidationError : public FlashError {
        public:
            ValidationError(const char* msg = "Flash validation error")
                : FlashError(msg)
            {
            }
        };
    }

    namespace gpio {
        class GPIOError : public ELFEError {
        public:
            GPIOError(const char* msg = "GPIO Error")
                : ELFEError(msg)
            {
            }
        };
    }

    namespace i2c {
        class I2CError : public ELFEError {
        public:
            I2CError(const char* msg = "I2C Error")
                : ELFEError(msg)
            {
            }
        };
        class ArbitrationLost : public I2CError {
        public:
            ArbitrationLost(
                const char* msg = "Arbitration lost during I2C communication (Signal on the bus doesn't match the output value)")
                : I2CError(msg)
            {
            }
        };
        class BusBusy : public I2CError {
        public:
            BusBusy(const char* msg = "I2C bus busy")
                : I2CError(msg)
            {
            }
        };
        class BusError : public I2CError {
        public:
            BusError(const char* msg = "I2C bus misplaced start or stop condition")
                : I2CError(msg)
            {
            }
        };
        class NoSlaveAck : public I2CError {
        public:
            NoSlaveAck(const char* msg = "No I2C slave acknowledged the address")
                : I2CError(msg)
            {
            }
        };
        class Overrun : public I2CError {
        public:
            Overrun(const char* msg = "I2C Underrun when sending or overrun when receiving")
                : I2CError(msg)
            {
            }
        };
        class I2CTimeout : public I2CError, public TimeoutError {
        public:
            I2CTimeout(const char* msg = "I2CTimeout during I2C communication")
                : I2CError(msg)
            {
            }
        };
        class ValidationError : public I2CError {
        public:
            ValidationError(const char* msg = "I2C Invalid data received")
                : I2CError(msg)
            {
            }
        };
    }

    namespace spi {
        class SPIError : public ELFEError {
        public:
            SPIError(const char* msg = "SPI Error")
                : ELFEError(msg)
            {
            }
        };

        class CRCError : public SPIError {
        public:
            CRCError(const char* msg = "CRC Error in SPI transmission")
                : SPIError(msg)
            {
            }
        };

        class Overrun : public SPIError {
        public:
            Overrun(const char* msg = "Overrun in SPI transmission")
                : SPIError(msg)
            {
            }
        };

        class TIFrameError : public SPIError {
        public:
            TIFrameError(const char* msg = "TI Frame Error in SPI transmission")
                : SPIError(msg)
            {
            }
        };
        class Underrun : public SPIError {
        public:
            Underrun(const char* msg = "Underrun in SPI transmission")
                : SPIError(msg)
            {
            }
        };
        class SPITimeout : public SPIError, public TimeoutError {
        public:
            SPITimeout(const char* msg = "SPI Timeout")
                : SPIError(msg)
            {
            }
        };
        class ModeFault : public SPIError {
        public:
            ModeFault(const char* msg = "Mode fault in SPI transmission")
                : SPIError(msg)
            {
            }
        };
    }

    namespace i2s {
        class I2SError : public ELFEError {
        public:
            I2SError(const char* msg = "I2S Error")
                : ELFEError(msg)
            {
            }
        };
    }

    namespace usart {

        class USARTError : public std::runtime_error {
        public:
            USARTError(const char* msg)
                : std::runtime_error(msg)
            {
            }
        };
        class Overrun : public USARTError {
        public:
            Overrun()
                : USARTError("Usart overrun error")
            {
            }
        };
        class ParityError : public USARTError {
        public:
            ParityError()
                : USARTError("Usart parity error")
            {
            }
        };
        class NoiseError : public USARTError {
        public:
            NoiseError()
                : USARTError("Usart noise error")
            {
            }
        };
        class FramingError : public USARTError {
        public:
            FramingError()
                : USARTError("Usart framing error")
            {
            }
        };
        class BreakError : public USARTError {
        public:
            BreakError()
                : USARTError("Usart break error")
            {
            }
        };
    }

    namespace extra {
        namespace ecb {
            class BluetoothError : public err::ELFEError {
            public:
                BluetoothError(const char* msg = "ECB Bluetooth Error")
                    : err::ELFEError(msg)
                {
                }
            };

            class ATError : public BluetoothError {
            public:
                ATError(const char* msg = "ECB AT Command Error")
                    : BluetoothError(msg)
                {
                }
            };
        }

        namespace w25qxx {
            class W25QError : public err::ELFEError {
            public:
                W25QError(const char* msg = "W25QXX Error")
                    : err::ELFEError(msg)
                {
                }
            };

            class UnknownModel : public W25QError {
            public:
                UnknownModel(const char* msg = "Unknown W25QXX model")
                    : W25QError(msg)
                {
                }
            };
        }

        namespace ssd1306 {
            class SSD1306Error : public err::ELFEError {
            public:
                SSD1306Error(const char* msg = "SSD1306 Error")
                    : err::ELFEError(msg)
                {
                }
            };
        }

        namespace ssd1680 {
            class SSD1680Error : public err::ELFEError {
            public:
                SSD1680Error(const char* msg = "SSD1680 Error")
                    : err::ELFEError(msg)
                {
                }
            };
        }
    }
}
}
