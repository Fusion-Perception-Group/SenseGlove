#include "ffmt.hpp"

namespace vermils
{
namespace ffmt
{
    using std::string;
    using std::size_t;

    struct Placeholder
    {
        enum Format {
            Decimal,
            Hex,
            Octal,
            Binary,
            Float,
            Scientific,
        };
        enum Align {
            Left,
            Center,
            Right,
        };
        size_t begin;
        size_t end;
        Align align=Right;
        Format format=Decimal;
        uint_fast16_t padding=0;
        uint_fast16_t precision=6;
        char fill='0';
        bool show_sign=false;
    };

    template <typename... T>
    std::string format(const string & src, const T & ...args)
    {
        std::vector<Placeholder> phs;
        std::string ret;
        phs.reserve(sizeof...(args));
    }

    template <typename... T>
    std::string format(std::size_t maxsize, const std::string & src, const T & ...args)
    {}
}
}