#pragma once

#include <cstdint>

namespace vms
{
    namespace w25qxx
    {
        enum class Instruction : uint8_t
        {
            WriteEnable = 0x06,
            VolatileSRWriteEnable = 0x50,
            WriteDisable = 0x04,
            ReadStatusRegister1 = 0x05,  // (S7-S0)
            WriteStatusRegister = 0x01,  // (S7-S0), [(S15-S8)](for 64FV)
            ReadStatusRegister2 = 0x35,  // (S15-S8)
            WriteStatusRegister2 = 0x31, // (S15-S8)
            ReadStatusRegister3 = 0x15,  // (S23-S16)
            WriteStatusRegister3 = 0x11, // (S23-S16)
            ChipErase = 0xC7,            // or 0x60
            EraseProgramSuspend = 0x75,
            EraseProgramResume = 0x7A,
            PowerDown = 0xB9,
            ReleasePowerDown = 0xAB, // dummy, dummy, dummy, (ID7-ID0)
            ManuDeviceID = 0x90,     // dummy, dummy, dummy, dummy, ManuID(7-0), (ID7-ID0)
            JEDECID = 0x9F,          // ManuID(7-0), (ID15-ID8), (ID7-ID0)
            GlobalBlockLock = 0x7E,
            GlobalBlockUnlock = 0x98,
            QPIEnable = 0x38,
            ResetEnable = 0x66,
            Reset = 0x99,
            ReadUniqueID = 0x4B,            // dummy, dummy, dummy, dummy, UniqueID(63-0)
            PageProgram = 0x02,             // A23-A16, A15-A8, A7-A0, data...(up to 256 bytes)
            QuadPageProgram = 0x32,         // A23-A16, A15-A8, A7-A0, data...(up to 256 bytes)
            SectorErase = 0x20,             // A23-A16, A15-A8, A7-A0
            BlockErase32K = 0x52,           // A23-A16, A15-A8, A7-A0
            BlockErase64K = 0xD8,           // A23-A16, A15-A8, A7-A0
            ReadData = 0x03,                // A23-A16, A15-A8, A7-A0, data...
            FastRead = 0x0B,                // A23-A16, A15-A8, A7-A0, dummy, data...
            FastReadDualOutput = 0x3B,      // A23-A16, A15-A8, A7-A0, dummy, data...
            FastReadQuadOutput = 0x6B,      // A23-A16, A15-A8, A7-A0, dummy, data...
            ReadSFDPRegister = 0x5A,        // 0x00, 0x00, A7-A0, dummy, data...
            EraseSecurityRegister = 0x44,   // A23-A16, A15-A8, A7-A0
            ProgramSecurityRegister = 0x42, // A23-A16, A15-A8, A7-A0, data...(up to 256 bytes)
            ReadSecurityRegister = 0x48,    // A23-A16, A15-A8, A7-A0, dummy, data...
            IndividualBlockLock = 0x36,     // A23-A16, A15-A8, A7-A0
            IndividualBlockUnlock = 0x39,   // A23-A16, A15-A8, A7-A0
            ReadBlockLock = 0x3D,           // A23-A16, A15-A8, A7-A0, lock

            FastReadDualIO = 0xBB,     // A23-A16, A15-A8, A7-A0, dummy, data...
            ManuDeviceIDDualIO = 0x92, // A23-A16, A15-A8, A7-A0, dummy, ManuID(7-0), (ID7-ID0)

            SetBurstWithWrap = 0x77,    // dummy, dummy, dummy, (W8-W0)
            FastReadQuadIO = 0xEB,      // A23-A16, A15-A8, A7-A0, dummy, dummy, data...
            WordReadQuadIO = 0xE7,      // A23-A16, A15-A8, A7-A0, dummy, data...
            OctalWordReadQuadIO = 0xE3, // A23-A16, A15-A8, A7-A0, data...
            ManuDeviceQuadIO = 0x94,    // A23-A16, A15-A8, A7-A0, dummy, dummy, ManuID(7-0), (ID7-ID0)
        };

        namespace status_mask
        {
            inline constexpr const uint8_t SR1_BUSY = 0x01;  // Busy, volatile, Read Only
            inline constexpr const uint8_t SR1_WEL = 0x02;   // Write Enable Latch, volatile, Read Only
            inline constexpr const uint8_t SR1_BP0 = 0x04;   // Block Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR1_BP1 = 0x08;   // Block Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR1_BP2 = 0x10;   // Block Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR1_TB = 0x20;    // Top/Bottom Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR1_SEC = 0x40;   // SectorProtect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR1_SRP0 = 0x80;  // Status Register Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR2_SRP1 = 0x01;  // Status Register Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR2_QE = 0x02;    // Quad Enable, non-volatile, ReadWrite
            inline constexpr const uint8_t SR2_LB1 = 0x08;   // Security Register Lock Bit 1, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint8_t SR2_LB2 = 0x10;   // Security Register Lock Bit 2, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint8_t SR2_LB3 = 0x20;   // Security Register Lock Bit 3, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint8_t SR2_LBS = 0x38;   // Security Register Lock Bits, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint8_t SR2_CMP = 0x40;   // Complement Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR2_SUS = 0x80;   // Erase/Program Suspend, volatile, Read Only
            inline constexpr const uint8_t SR3_RSTE = 0x01;  // Reset Enable, non-volatile, ReadWrite
            inline constexpr const uint8_t SR3_PERR = 0x02;  // Program Erase Resume, volatile, Read Only
            inline constexpr const uint8_t SR3_EFAIL = 0x04; // Erase Fail Flag, volatile, Read Only
            inline constexpr const uint8_t SR3_PFAIL = 0x08; // Program Fail Flag, volatile, Read Only
            inline constexpr const uint8_t SR3_SPERR = 0x10; // Sector Protection Error, volatile, Read Only
            inline constexpr const uint8_t SR3_RES = 0x20;   // Reserved, volatile, Read Only
            inline constexpr const uint8_t SR3_CMP = 0x40;   // Complement Protect, non-volatile, ReadWrite
            inline constexpr const uint8_t SR3_SUS = 0x80;   // Erase/Program Suspend, volatile, Read Only

            inline constexpr const uint32_t BUSY = SR1_BUSY;         // Busy, volatile, Read Only
            inline constexpr const uint32_t WEL = SR1_WEL;           // Write Enable Latch, volatile, Read Only
            inline constexpr const uint32_t BP0 = SR1_BP0;           // Block Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t BP1 = SR1_BP1;           // Block Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t BP2 = SR1_BP2;           // Block Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t TB = SR1_TB;             // Top/Bottom Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t SEC = SR1_SEC;           // SectorProtect, non-volatile, ReadWrite
            inline constexpr const uint32_t SRP0 = SR1_SRP0;         // Status Register Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t SRP1 = SR2_SRP1 << 8;    // Status Register Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t QE = SR2_QE << 8;        // Quad Enable, non-volatile, ReadWrite
            inline constexpr const uint32_t LB1 = SR2_LB1 << 8;      // Security Register Lock Bit 1, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint32_t LB2 = SR2_LB2 << 8;      // Security Register Lock Bit 2, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint32_t LB3 = SR2_LB3 << 8;      // Security Register Lock Bit 3, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint32_t LBS = SR2_LBS << 8;      // Security Register Lock Bits, non-volatile, ReadWrite, OTP(from 0 to 1)
            inline constexpr const uint32_t CMP = SR2_CMP << 8;      // Complement Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t SUS = SR2_SUS << 8;      // Erase/Program Suspend, volatile, Read Only
            inline constexpr const uint32_t RSTE = SR3_RSTE << 16;   // Reset Enable, non-volatile, ReadWrite
            inline constexpr const uint32_t PERR = SR3_PERR << 16;   // Program Erase Resume, volatile, Read Only
            inline constexpr const uint32_t EFAIL = SR3_EFAIL << 16; // Erase Fail Flag, volatile, Read Only
            inline constexpr const uint32_t PFAIL = SR3_PFAIL << 16; // Program Fail Flag, volatile, Read Only
            inline constexpr const uint32_t SPERR = SR3_SPERR << 16; // Sector Protection Error, volatile, Read Only
            inline constexpr const uint32_t RES = SR3_RES << 16;     // Reserved, volatile, Read Only
            inline constexpr const uint32_t CMP2 = SR3_CMP << 16;    // Complement Protect, non-volatile, ReadWrite
            inline constexpr const uint32_t SUS2 = SR3_SUS << 16;    // Erase/Program Suspend, volatile, Read Only
        }
    }

}
