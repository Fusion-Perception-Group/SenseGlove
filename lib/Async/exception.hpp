#pragma once
#include <stdexcept>

namespace vermils
{
namespace aio
{

class CancelledException : public std::runtime_error
{
public:
    CancelledException() : std::runtime_error("Cancelled") {}
};

}
}