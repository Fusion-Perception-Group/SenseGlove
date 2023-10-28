#pragma once

#include <cstddef>      // std::size_t
#include <cstdint>      // std::uint_fast16_t
#include <string>       // std::string
#include <stdexcept>    // std::runtime_error
#include <ctype.h>      // isdigit
#include <vector>       // std::vector
#include <algorithm>    // std::find

namespace vermils
{
namespace ffmt
{
    template <typename... T>
    std::string format(const std::string & src, const T & ...args);
    
    template <typename... T>
    std::string format(std::size_t maxsize, const std::string & src, const T & ...args);
}
}
