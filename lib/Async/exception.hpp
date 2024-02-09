#pragma once
#include <stdexcept>

namespace vms
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