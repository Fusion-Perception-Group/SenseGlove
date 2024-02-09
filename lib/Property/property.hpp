#pragma once

#include <functional>
#include <stdexcept>

namespace vms
{
namespace tricks
{

template <typename T, typename O>
struct StaticReadOnlyProperty
{
protected:
    O &&owner;
public:
    constexpr StaticReadOnlyProperty(O &&owner) : owner(owner) {}
    virtual ~StaticReadOnlyProperty() = default;
    virtual T getter() const = 0;
    operator T() const
    {
        return getter();
    }
    T operator ()() const
    {
        return static_cast<T>(*this);
    }
};

template <typename T, typename O>
struct StaticProperty : StaticReadOnlyProperty<T, O>
{
public:
    constexpr StaticProperty(O &&owner) : StaticReadOnlyProperty<T, O>(std::forward<O>(owner)) {}
    virtual void setter(T value) const {};
    T operator=(T value) const
    {
        setter(value);
        return value;
    }
};

template <typename T, typename O>
struct StaticReadOnlyProperty<T &, O>
{
protected:
    O &&owner;
public:
    constexpr StaticReadOnlyProperty(O &&owner) : owner(owner) {}
    virtual ~StaticReadOnlyProperty() = default;
    virtual T& getter() const = 0;
    operator T&() const
    {
        return getter();
    }
    T &operator ()() const
    {
        return static_cast<T&>(*this);
    }
};

template <typename T, typename O>
struct StaticProperty<T &, O> : StaticReadOnlyProperty<T &, O>
{
public:
    constexpr StaticProperty(O &&owner) : StaticReadOnlyProperty<T &, O>(std::forward<O>(owner)) {}
    virtual void setter(const T &value) const {};
    T &operator=(const T &value) const
    {
        setter(value);
        return value;
    }
};

/**
 * @brief Property Class
 *
 * @tparam T the type of the property
 * @param getter const std::function<T()> getter
 * @param setter const std::function<void(const T)> setter
 */
template <typename T>
class Property
{
protected:
    const std::function<T()> getter;
    const std::function<void(const T)> setter;

public:
    typedef std::function<T()> Getter;
    typedef std::function<void(const T)> Setter;
    Property() = delete;
    Property(std::function<T()> getter, std::function<void(const T)> setter = nullptr)
        : getter(getter), setter(setter)
    {
    }

    operator T() const
    {
        return getter();
    }

    T operator ()() const
    {
        return getter();
    }

    T operator=(const T value)
    {
        if (!setter)
            throw std::runtime_error("Property setter is not defined");
        setter(value);
        return value;
    }
};

template <typename T>
class Property<T &>
{
protected:
    const std::function<T()> getter;
    const std::function<void(T &&)> setter;

public:
    Property(std::function<T()> getter, std::function<void(T &&)> setter = nullptr)
        : getter(getter), setter(setter)
    {
    }

    operator T() const
    {
        return getter();
    }

    T& operator ()() const
    {
        return getter();
    }

    T &operator=(const T &value)
    {
        if (!setter)
            throw std::runtime_error("Property setter is not defined");
        setter(value);
        return value;
    }

    T &operator=(T &&value)
    {
        if (!setter)
            throw std::runtime_error("Property setter is not defined");
        setter(std::forward<T>(value));
        return getter();
    }
};

}
}

namespace vermils
{
    using namespace vms::tricks;  // backward compatibility
}
