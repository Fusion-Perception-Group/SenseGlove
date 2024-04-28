#pragma once

#include <functional>
#include <stdexcept>

namespace elfe {
namespace tricks {

    template <typename T, typename O>
    struct StaticReadOnlyProperty {
    protected:
        O&& owner;

    public:
        constexpr StaticReadOnlyProperty(O&& owner)
            : owner(owner)
        {
        }
        virtual ~StaticReadOnlyProperty() = default;
        virtual T getter() const = 0;
        operator T() const
        {
            return getter();
        }
        T operator()() const
        {
            return static_cast<T>(*this);
        }
    };

    template <typename T, typename O>
    struct StaticProperty : StaticReadOnlyProperty<T, O> {
    public:
        constexpr StaticProperty(O&& owner)
            : StaticReadOnlyProperty<T, O>(std::forward<O>(owner))
        {
        }
        virtual void setter(T value) const {};
        T operator=(T value) const
        {
            setter(value);
            return value;
        }
    };

    template <typename T, typename O>
    struct StaticReadOnlyProperty<T&, O> {
    protected:
        O&& owner;

    public:
        constexpr StaticReadOnlyProperty(O&& owner)
            : owner(owner)
        {
        }
        virtual ~StaticReadOnlyProperty() = default;
        virtual T& getter() const = 0;
        operator T&() const
        {
            return getter();
        }
        T& operator()() const
        {
            return static_cast<T&>(*this);
        }
    };

    template <typename T, typename O>
    struct StaticProperty<T&, O> : StaticReadOnlyProperty<T&, O> {
    public:
        constexpr StaticProperty(O&& owner)
            : StaticReadOnlyProperty<T&, O>(std::forward<O>(owner))
        {
        }
        virtual void setter(const T& value) const {};
        T& operator=(const T& value) const
        {
            setter(value);
            return value;
        }
    };

}
}

namespace vermils {
using namespace elfe::tricks; // backward compatibility
}
