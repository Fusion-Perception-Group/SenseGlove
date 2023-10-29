#include <string>
namespace vermils
{
namespace ffmt
{
    // hopman_fun

template <typename T> 
inline T _reduce2(T v) {
    T k = ((v * 410) >> 12) & 0x000F000F000F000Full;
    return (((v - k * 10) << 8) + k);
}

template <typename T>
inline T _reduce4(T v) {
    T k = ((v * 10486) >> 20) & 0xFF000000FFull;
    return _reduce2(((v - k * 100) << 16) + (k));
}

typedef unsigned long long ull;
inline ull _reduce8(ull v) {
    ull k = ((v * 3518437209u) >> 45);
    return _reduce4(((v - k * 10000) << 32) + (k));
}

template <typename T>
std::string itostr(T o) {
    union {
        char str[16];
        unsigned short u2[8];
        unsigned u4[4];
        unsigned long long u8[2];
    };

    unsigned v = o < 0 ? ~o + 1 : o;

    u8[0] = (ull(v) * 3518437209u) >> 45;
    u8[0] = (u8[0] * 28147497672ull);
    u8[1] = v - u2[3] * 100000000;

    u8[1] = _reduce8(u8[1]);
    char* f;
    if (u2[3]) {
        u2[3] = _reduce2(u2[3]);
        f = str + 6;
    } else {
        unsigned short* k = u4[2] ? u2 + 4 : u2 + 6;
        f = *k ? (char*)k : (char*)(k + 1);
    }
    if (!*f) f++;

    u4[1] |= 0x30303030;
    u4[2] |= 0x30303030;
    u4[3] |= 0x30303030;
    if (o < 0) *--f = '-';
    return std::string(f, (str + 16) - f);
}

}
}
