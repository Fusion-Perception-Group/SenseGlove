#pragma once

#include <chrono>
#include "exception.hpp"

namespace vms
{
namespace aio
{
    template <typename T>
    class BaseBlockFuture
    {
    public:
        virtual ~BaseBlockFuture() = default;

        virtual bool ready() const noexcept = 0;

        virtual T &get() const = 0;
    };

        
}
}