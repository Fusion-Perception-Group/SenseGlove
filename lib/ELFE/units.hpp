#pragma once

#include <chrono>
#include <cstdint>
using std::chrono::operator""ns;
using std::chrono::operator""us;
using std::chrono::operator""ms;
using std::chrono::operator""s;
using std::chrono::operator""min;
using std::chrono::operator""h;
using std::chrono::operator""d;
inline consteval uint64_t operator""_Hz(unsigned long long int value)
{
    return value;
}
inline consteval uint64_t operator""_KHz(unsigned long long int value)
{
    return value * 1000;
}
inline consteval uint64_t operator""_MHz(unsigned long long int value)
{
    return value * 1000 * 1000;
}
inline consteval uint64_t operator""_GHz(unsigned long long int value)
{
    return value * 1000 * 1000 * 1000;
}
inline consteval uint64_t operator""_Byte(unsigned long long int value)
{
    return value;
}
inline consteval uint64_t operator""_Bytes(unsigned long long int value)
{
    return value;
}
inline consteval uint64_t operator""_KiB(unsigned long long int value)
{
    return value * 1024;
}
inline consteval uint64_t operator""_MiB(unsigned long long int value)
{
    return value * 1024 * 1024;
}
inline consteval uint64_t operator""_GiB(unsigned long long int value)
{
    return value * 1024 * 1024 * 1024;
}

inline consteval uint64_t operator""_BytesPS(unsigned long long int value)
{
    return value;
}
inline consteval uint64_t operator""_KiBPS(unsigned long long int value)
{
    return value * 1024;
}
inline consteval uint64_t operator""_MiBPS(unsigned long long int value)
{
    return value * 1024 * 1024;
}
inline consteval uint64_t operator""_GiBPS(unsigned long long int value)
{
    return value * 1024 * 1024 * 1024;
}
inline consteval uint64_t operator""_KB(unsigned long long int value)
{
    return value * 1000;
}
inline consteval uint64_t operator""_MB(unsigned long long int value)
{
    return value * 1000 * 1000;
}
inline consteval uint64_t operator""_GB(unsigned long long int value)
{
    return value * 1000 * 1000 * 1000;
}
inline consteval uint64_t operator""_KBPS(unsigned long long int value)
{
    return value * 1000;
}
inline consteval uint64_t operator""_MBPS(unsigned long long int value)
{
    return value * 1000 * 1000;
}
inline consteval uint64_t operator""_GBPS(unsigned long long int value)
{
    return value * 1000 * 1000 * 1000;
}
