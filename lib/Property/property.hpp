#pragma once

#include <functional>
#include <stdexcept>

namespace vermils
{
namespace tricks
{

template <typename T, typename O>
struct StaticProperty
{
protected:
    O &&owner;
public:
    StaticProperty(O &&owner) : owner(owner) {}
    virtual T getter() const = 0;
    virtual void setter(const T value) {};
    operator T() const
    {
        return getter();
    }
    T operator=(const T value)
    {
        setter(value);
        return value;
    }
    T operator ()() const
    {
        return static_cast<T>(*this);
    }
};

template <typename T, typename O>
struct StaticProperty<T &, O>
{
protected:
    O &&owner;
public:
    StaticProperty(O &&owner) : owner(owner) {}
    virtual T& getter() const = 0;
    virtual void setter(const T &value) {};
    operator T&() const
    {
        return getter();
    }
    T &operator=(const T &value)
    {
        setter(value);
        return value;
    }
    T &operator ()() const
    {
        return static_cast<T>(*this);
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
    const std::function<void(const T &)> setter;

public:
    Property(std::function<T()> getter, std::function<void(const T &)> setter = nullptr)
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

    const T &operator=(const T &value)
    {
        if (!setter)
            throw std::runtime_error("Property setter is not defined");
        setter(value);
        return value;
    }
};

}
}
