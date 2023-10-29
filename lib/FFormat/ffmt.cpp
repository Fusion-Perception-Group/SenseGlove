#define __VERMIL_FFMT_DEBUG 0


#if __VERMIL_FFMT_DEBUG
#include <iostream>
#include "ffmt.hpp"

int main()
{
    using namespace vermils::ffmt;
    int a = 123;
    HolderContainer phs;
    char fmt[] = "a = {0:^+020.d}, {1:.e}, {{}";
    parse(phs, fmt);
    auto p = phs[0];
    std::cout << "index " << p.index << std::endl;
    std::cout << "padding " << p.padding << std::endl;
    std::cout << "precision " << p.precision << std::endl;
    std::cout << "align " << (int)p.align << std::endl;
    std::cout << "format " << (int)p.format << std::endl;
    std::cout << "fill " << p.fill << std::endl;
    std::cout << "show_positive " << p.show_positive << std::endl;
    std::cout << "escape " << p.escape << std::endl;
    std::cout << "begin " << p.begin << std::endl;
    std::cout << "end " << p.end << std::endl;

    std::cout << string(fmt).find('{') << std::endl;

    std::cout << format(fmt, 114514, 0.12345678900987654321) << std::endl;

    //std::cout << format("{}", a) << std::endl;
}
#endif