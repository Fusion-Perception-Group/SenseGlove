#pragma once

#include <cstddef>      // std::size_t
#include <cstdint>      // std::uint_fast16_t
#include <string>       // std::string
#include <stdexcept>    // std::runtime_error
#include <cctype>      // isdigit
#include <vector>       // std::vector
#include "itostr.hpp"
#include "grisu.hpp"

namespace vermils
{
namespace ffmt
{
    template <typename... T>
    std::string format(const std::string & src, T && ...args);
    
    template <typename... T>
    std::string format(const std::size_t maxsize, const std::string & src, T && ...args);
}
}


namespace vermils
{
namespace ffmt
{
using std::string;
using std::size_t;

/**
 * @brief Configuration for each {} in the string
 * 
 */
struct Placeholder
{
    enum Format : uint_fast8_t {
        Decimal,
        Hex,
        Octal,
        Binary,
        Pointer,
        Float,
        Scientific,
    };
    enum Align : uint_fast8_t {
        Left,
        Center,
        Right,
    };

    size_t begin = 0;
    size_t end = 0;
    uint_fast16_t index = 0;
    uint_fast16_t padding=0;
    uint_fast16_t precision=6;
    Align align=Right;
    Format format=Decimal;
    char fill=' ';
    bool show_positive=false;
    bool escape=false; // "{{" escape
};

using HolderContainer = std::vector<Placeholder>;
using StrContainer = string[];

/**
 * @brief Function that find all {} and parse arguments in it into Placeholder
 * 
 * @param phs 
 * @param src 
 */
inline void parse(HolderContainer & phs, const string & src)
{
    const size_t length=src.length();
    size_t pos = 0; // position of current character

    while (pos < length)
    {
        Placeholder p;
        
        pos = src.find('{', pos);
        if (pos == string::npos) return; // no {} left
        
        p = Placeholder({pos, pos}); // init placeholder
        ++pos; // skip '{'

        try
        {
            p.index = phs.size(); // use default index (argument order)
            

            uint_fast16_t n;
            bool fill_set = false;

[[maybe_unused]] Escape:
                if (src.at(pos) == '}') goto Loopend;
                if (src[pos] == '{')  // "{{" escape
                {
                    p.escape = true;
                    goto Loopend;
                }
            
[[maybe_unused]] Index:
                n = 0;
                if (src[pos] == ':')
                {
                    ++pos; // skip ':'
                    goto Align;
                }
                else do
                {
                    if (!std::isdigit(src[pos]))
                    {
                        throw std::runtime_error("Invalid Format String");
                    }
                    n = 10 * n + src[pos] - '0';
                    ++pos;
                }
                while (src.at(pos) != ':' && src[pos] != '}');
                p.index = n;
                if (src[pos] == '}') goto Loopend;
                if (src[pos] != ':')
                    throw std::runtime_error("Invalid Format String");
                ++pos; // skip ':'
            Align:
                if (src.at(pos) == '.')
                {
                    ++pos; // skip '.'
                    goto Precision;
                }
                else if (src[pos] == '}')
                {
                    goto Loopend;
                }
                else if (std::isdigit(src[pos]) && src[pos] != '0')
                {
                    goto Padding;
                }
                else if (src[pos] == '+')
                {
                    p.show_positive = true;
                    ++pos;
                    goto Align;
                }
                else if (src[pos] == '-')
                {
                    p.show_positive = false;
                    ++pos;
                    goto Align;
                }
                else if (src[pos] == '<')
                {
                    p.align = Placeholder::Left;
                    ++pos;
                    goto Align;
                }
                else if (src[pos] == '^')
                {
                    p.align = Placeholder::Center;
                    ++pos;
                    goto Align;
                }
                else if (src[pos] == '>')
                {
                    p.align = Placeholder::Right;
                    ++pos;
                    goto Align;
                }
                else if (!fill_set)
                {
                    fill_set = true;
                    p.fill = src[pos++];
                    goto Align;
                }
                else
                {
                    throw std::runtime_error("Invalid Format String");
                }
            Padding:
                if (src.at(pos) == '}') goto Loopend;
                while (std::isdigit(src.at(pos)))
                {
                    p.padding = 10 * p.padding + src[pos++] - '0';
                }
                if (src[pos] == '}') goto Loopend;
                if (src[pos] != '.')
                    throw std::runtime_error("Invalid Format String");
                ++pos; // skip '.'
            Precision:
                if (src.at(pos) == '}') goto Loopend;
                n = 0;
                if (!std::isdigit(src[pos]))
                    goto Format;
                else do
                {
                    n = 10 * n + src[pos++] - '0';
                }
                while (std::isdigit(src.at(pos)));
                p.precision = n;
            Format:
                if (src.at(pos) == '}') goto Loopend;
                if (src[pos] == 'd')
                {
                    p.format = Placeholder::Decimal;
                }
                else if (src[pos] == 'x')
                {
                    p.format = Placeholder::Hex;
                }
                else if (src[pos] == 'p')
                {
                    p.format = Placeholder::Pointer;
                }
                else if (src[pos] == 'o')
                {
                    p.format = Placeholder::Octal;
                }
                else if (src[pos] == 'b')
                {
                    p.format = Placeholder::Binary;
                }
                else if (src[pos] == 'f')
                {
                    p.format = Placeholder::Float;
                }
                else if (src[pos] == 'e')
                {
                    p.format = Placeholder::Scientific;
                }
                else
                {
                    throw std::runtime_error("Invalid Format String");
                }
                if (src.at(++pos) != '}')
                    throw std::runtime_error("Invalid Format String");

        }
        catch (std::out_of_range & e)
        {
            throw std::runtime_error("Unclosed {} pair");
        }

        Loopend:
        p.end = ++pos;  // store next position of '}' and skip '}'
        phs.push_back(std::move(p));
    }
}

inline string & pad_str(string & s, const Placeholder & p,
    bool allow_trailing_0 = false, bool move_sign = false)
{
    if (p.padding <= s.length()) return s;
    uint_fast16_t left, right;
    size_t pos = 0;
    switch(p.align)
    {
    case Placeholder::Left:
        s.append(p.padding - s.length(), p.fill);
        break;
    case Placeholder::Center:
        left = (p.padding - s.length()) / 2;
        right = p.padding - s.length() - left;
        if (move_sign && s.length() && (s[0] == '-' || s[0] == '+'))
        {
            ++pos;
        }
        s.insert(pos, left, p.fill);
        if (p.fill != '0' || allow_trailing_0)
            s.append(right, p.fill);
        else
            s.append(right, ' ');
        break;
    case Placeholder::Right:
        if (move_sign && s.length() && (s[0] == '-' || s[0] == '+'))
        {
            ++pos;
        }
        s.insert(pos, p.padding - s.length(), p.fill);
        break;
    }
    return s;
}

inline const string & stringify(string & arg, const Placeholder & p) { return arg; }
inline const string & stringify(const string & arg, const Placeholder & p) { return arg; }
inline string && stringify(string && arg, const Placeholder & p) { return std::move(arg); }
inline string stringify(const char * str, const Placeholder & p) { return string(str); }

template <typename T>
inline string stringify(T && arg, const Placeholder & p)
requires(!std::floating_point<T> && !std::integral<T> && std::convertible_to<T, string>)
{
    return std::to_string(std::forward<T>(arg));
}

template <std::floating_point T>
inline string stringify(T arg, const Placeholder & p)
{
    if (std::isnan(arg)) return "nan";
    if (std::isinf(arg)) return "inf";

    string s;
    char buf[24];
    int length, K;
    bool negative = arg < 0;
    bool allow_trailing_0 = true;
    Placeholder::Format format = p.format;
    if (!arg)
    {
        buf[0] = '0';
        length = 1;
        K = -1;
    }
    else
    {
        if (negative) arg = -arg;
        grisu2::Grisu2(arg, buf, &length, &K);
    }
    
    int i_size = std::max<int>(1, length + K);  // size for integer part
    int guessed_l = p.precision + i_size + 2;
    int expo = 0;

    if (format != Placeholder::Float && p.format != Placeholder::Scientific)
    {
        const int THRE = (p.padding ? p.padding : 24);  // 24 is a personal choice
        format = (guessed_l > THRE) ? Placeholder::Scientific : Placeholder::Float;
    }

    if (format == Placeholder::Scientific)
    {
        guessed_l = p.precision + 9;
        expo = K + length - 1;
        i_size = 1;
        K = 1 - length;
        allow_trailing_0 = false;
    }
    
    s.reserve(guessed_l);
    if(negative)
        s.push_back('-');
    else if (p.show_positive)
        s.push_back('+');
    
    int dot_pos = length + K;

    if (dot_pos <= 0)
    {
        s.push_back('0');
        if (p.precision)
        {
            int prec = p.precision;
            s.push_back('.');
            s.append(std::min<int>(-dot_pos, prec), '0');
            if (prec > -dot_pos)
                s.append(buf, std::min<int>(length, prec + dot_pos));
        }
    }
    else if (dot_pos > length)
    {
        s.append(buf, length);
        s.append(dot_pos - length, '0');
        if (p.precision)
        {
            s.push_back('.');
            s.append(p.precision, '0');
        }
    }
    else
    {
        s.append(buf, dot_pos);
        if (p.precision)
        {
            s.push_back('.');
            s.append(buf + dot_pos, std::min<int>(length - dot_pos, p.precision));
            if (static_cast<int>(p.precision) > length - dot_pos)
                s.append(p.precision - length + dot_pos, '0');
        }
    }
    
    if (format == Placeholder::Scientific)
    {
        s.push_back('e');
        s.append(itostr(expo));
    }

    return pad_str(s, p, allow_trailing_0, true);
}

template <std::integral T>
inline string stringify(T && arg, const Placeholder & p)
{
    static const char digits[] = "0123456789abcdef";
    static const char b_lookup[][5] = 
    {
        "0000", "0001", "0010", "0011",
        "0100", "0101", "0110", "0111",
        "1000", "1001", "1010", "1011",
        "1100", "1101", "1110", "1111",
    };

    size_t size;
    bool nonneg = arg >= 0;
    string s;

    switch (p.format)
    {
    case Placeholder::Decimal: {
        if (p.show_positive && nonneg) s.push_back('+');
        s += itostr(arg);
        break;
    case Placeholder::Hex:
        size = std::max<size_t>(sizeof(T) * 2 + 1, p.padding);
        s.reserve(size);

        if (!nonneg)
        {
            s.push_back('-');
            arg = -arg;
        }
        else if (p.show_positive)
            s.push_back('+');

        if (!arg)
            s.push_back('0');
        else
        {
            size = sizeof(T) * 2;
            while (!(0x0f & (arg >> (--size * 4))));
            do
            {
                s.push_back(digits[(arg >> (size * 4)) & 0xF]);
            }
            while (size--);
        }
        break;
    }
    case Placeholder::Pointer:{
        size = std::max<size_t>(sizeof(T) * 2, p.padding);
        s.reserve(size);
        size = sizeof(T);
        while (size--)
        {
            s.push_back(digits[(arg >> (size * 8 + 4)) & 0xF]);
            s.push_back(digits[(arg >> (size * 8)) & 0xF]);
        }
        break;
    }
    case Placeholder::Octal: {
        bool allzero = true;
        size = std::max<size_t>(sizeof(T) * 3 + 1, p.padding);
        s.reserve(size);

        if (!nonneg)
        {
            s.push_back('-');
            arg = -arg;
        }
        else if (p.show_positive)
            s.push_back('+');
    
        if (!arg)
            s.push_back('0');
        else
        {
            size = sizeof(T) * 8;
            while (size >= 3)
            {
                auto digit = (arg >> (size -= 3)) & 0x7;
                if (!digit && allzero)
                    continue;
                allzero = false;
                s.push_back(digits[digit]);
            }
            if (size)
                s.push_back(digits[arg & ((1U << size) - 1)]);
        }
        break;
    }
    case Placeholder::Binary: {
        size = std::max<size_t>(sizeof(T) * 8 + 1, p.padding);
        s.reserve(size);
        if (!nonneg)
        {
            s.push_back('-');
            arg = -arg;
        }
        else if (p.show_positive)
            s.push_back('+');

        if (!arg)
            s.push_back('0');
        else
        {
            size = sizeof(T) * 2;
            while (!(0x0f & (arg >> (--size * 4))));
            do
            {
                s.append(b_lookup[(arg >> (size * 4)) & 0xF]);
            }
            while (size--);
        }
        break;
    }
    case Placeholder::Scientific:
    case Placeholder::Float:
        return stringify(double(arg), p);
    }

    return pad_str(s, p, false, true);
}

template <typename T>
void compose(const HolderContainer & phs, StrContainer strs, uint_fast16_t & arg_index, T && arg)
{
    for (size_t i = 0; i < phs.size(); ++i)
    {
        auto & p = phs[i];
        if (p.index != arg_index) continue; // skip if not current argument

        if (p.escape) continue;

        strs[i] = stringify(std::forward<T>(arg), p); // convert argument to string
    }

    ++arg_index;  // update argument index for other compose calls
}

template <typename... T>
std::string format(const string & src, T && ...args)
{
    HolderContainer phs;
    std::string ret;
    size_t maxsize = ret.max_size();
    size_t opos=0; // position of current character in original string
    uint_fast16_t arg_index=0; // index of current argument
    constexpr uint_fast16_t args_n=sizeof...(args);
    phs.reserve(args_n + 2); // reserve extra 2 for possible escapes

    parse(phs, src);

    string strs[phs.size()]; // make sure there is enough space for all placeholders
    ret.reserve(std::min<size_t>(maxsize, src.length() + std::min<size_t>(256, phs.size() * 8)));  // give a guess of the final size

    (compose(phs, strs, arg_index, std::forward<T>(args)), ...);

    for (size_t i = 0; i < phs.size(); ++i)
    {
        if (ret.length() >= maxsize) break;
        auto & p = phs[i];
        auto & s = strs[i];
        auto size = std::min(maxsize - ret.length(), p.begin - opos);

        ret.append(src, opos, size); // append string before placeholder
        opos = p.end;  // update position of current character in original string

        if (p.escape)
        {
            ret.push_back('{');
            continue;
        }

        size = std::min(maxsize - ret.length(), s.length());
        ret.append(s, 0, size); // append stringified argument
    }

    if (ret.length() < maxsize)
    {
        auto size = std::min(maxsize - ret.length(), src.length() - opos);
        ret.append(src, opos, size); // append string after last placeholder
    }

    return ret;
}

template <typename... T>
std::string format(const std::size_t maxsize, const std::string & src, T && ...args)
{
    HolderContainer phs;
    std::string ret;
    size_t opos=0; // position of current character in original string
    uint_fast16_t arg_index=0; // index of current argument
    constexpr uint_fast16_t args_n=sizeof...(args);
    phs.reserve(args_n + 2); // reserve extra 2 for possible escapes

    parse(phs, src);

    string strs[phs.size()]; // make sure there is enough space for all placeholders
    ret.reserve(std::min<size_t>(maxsize, src.length() + std::min<size_t>(256, phs.size() * 8)));  // give a guess of the final size

    (compose(phs, strs, arg_index, std::forward<T>(args)), ...);

    for (size_t i = 0; i < phs.size(); ++i)
    {
        if (ret.length() > maxsize) break;
        auto & p = phs[i];
        auto & s = strs[i];
        auto size = std::min(maxsize - ret.length(), p.begin - opos);

        ret.append(src, opos, size); // append string before placeholder
        opos = p.end;  // update position of current character in original string

        size = std::min(maxsize - ret.length(), s.length());
        ret.append(s, 0, size); // append stringified argument
    }

    if (ret.length() < maxsize)
    {
        auto size = std::min(maxsize - ret.length(), src.length() - opos);
        ret.append(src, opos, size); // append string after last placeholder
    }

    return ret;
}

}
}

