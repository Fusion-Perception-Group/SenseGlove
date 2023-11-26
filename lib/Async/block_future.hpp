#pragma once

#include "exception.hpp"

namespace vermils
{
namespace aio
{
    template <typename T, typename EXC = void>
    class BlockFuture
    {
        volatile T _value;
        volatile EXC _exception;
        volatile bool _ready = false;
        volatile bool _has_exception = false;
        volatile bool _cancelled = false;
    public:
        BlockFuture() {};

        bool ready() const noexcept
        {
            return _ready;
        }

        bool has_exception() const noexcept
        {
            return _has_exception;
        }

        T &get() const
        {
            while (!_ready)
            {
                if (_cancelled)
                {
                    throw CancelledException();
                }
            }
            if (_has_exception)
            {
                throw _exception;
            }
            return _value;
        }

        void set_value(T &&value) noexcept
        {
            _value = std::forward(value);
            _ready = true;
        }

        void set_exception(EXC &&exception) noexcept
        {
            _exception = std::forward(exception);
            _has_exception = true;
            _ready = true;
        }

    };

        
}
}