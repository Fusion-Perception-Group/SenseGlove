#ifndef _p_property_hpp_
#define _p_property_hpp_

#include <functional>

/**
 * @brief A template class for creating properties with getter and setter functions.
 * 
 * This class provides a way to create properties with getter and setter functions. It can be used to
 * create properties for any type, including references. The class overloads the assignment operator
 * and the cast operator to allow for easy assignment and retrieval of property values.
 * 
 * @tparam T The type of the property.
 */
template <typename T>
class Property
{
    // ...
};

/**
 * @brief A template class for creating properties with getter and setter functions for reference types.
 * 
 * This class provides a way to create properties with getter and setter functions for reference types.
 * It can be used to create properties for any type, including references. The class overloads the assignment
 * operator and the cast operator to allow for easy assignment and retrieval of property values.
 * 
 * @tparam T The type of the property.
 */
template <typename T>
class Property<T&>
{
    // ...
};
namespace vermils
{
namespace tricks
{

template <typename T>
class Property
{
    const std::function<T()> getter;
    const std::function<void(const T)> setter;
public:
    Property() = delete;
    /**
     * @brief Construct a new Property object.
     * 
     * @param getter 
     * @param setter 
     */
    Property(std::function<T()> getter, std::function<void(const T)> setter = nullptr)
        : getter(getter), setter(setter)
    {}

    operator T () const
    {
        return getter();
    }

    T operator = (const T value)
    {
        setter(value);
        return value;
    }
};

template <typename T>
class Property<T&>
{
    const std::function<T()> getter;
    const std::function<void(const T&)> setter;
public:
    Property(std::function<T()> getter, std::function<void(const T&)> setter = nullptr)
        : getter(getter), setter(setter)
    {}

    operator T () const
    {
        return getter();
    }

    const T& operator = (const T& value)
    {
        setter(value);
        return value;
    }
};

}
}

#endif