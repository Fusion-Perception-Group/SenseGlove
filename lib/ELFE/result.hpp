#pragma once

#ifndef ELFE_RESULT_HPP
#define ELFE_RESULT_HPP

#include "_hpp_config.hpp"
#include "errors.hpp"
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

#if ELFE_USE_EXCEPTIONS
#define ELFE_ERROR(res, exc) throw exc
#define ELFE_PANIC(exc) throw exc
#else
#define ELFE_ERROR(res, exc) return res
#define ELFE_PANIC(exc) std::abort()
#endif

#define ELFE_ERROR_IF(cond, res, exc) \
    do {                              \
        [[unlikely]] if (cond) {      \
            ELFE_ERROR(res, exc);     \
        }                             \
    } while (0)

#define ELFE_PANIC_IF(cond, exc) \
    do {                         \
        [[unlikely]] if (cond) { \
            ELFE_PANIC(exc);     \
        }                        \
    } while (0)

#define ELFE_PROP(prev_res, new_res)                               \
    do {                                                           \
        [[unlikely]] if (!ELFE_USE_EXCEPTIONS && !prev_res.ok()) { \
            return new_res;                                        \
        }                                                          \
    } while (0)

#if ELFE_USE_EXCEPTIONS || ELFE_FORCE_IGNORE_UNUSED_RESULT
#define ELFE_TMP_NODISCARD
#else
#define ELFE_TMP_NODISCARD [[nodiscard]]
#endif

#if ELFE_USE_EXCEPTIONS || ELFE_FORCE_RESULT_IMPLICIT_CONVERSION
#define ELFE_TMP_EXPLICIT
#else
#define ELFE_TMP_EXPLICIT explicit
#endif

/**
 * @namespace elfe
 * @brief Namespace for the ELFE library
 *
 * The `elfe` namespace contains classes and functions related to the ELFE library.
 * ELFE is a library that provides result types for error handling in C++.
 */
namespace elfe {
template <typename V>
struct worth_ref : std::conditional_t<
                       (sizeof(void*) > sizeof(V)) && !std::is_integral_v<std::decay_t<V>> && !std::is_floating_point_v<std::decay_t<V>> && std::is_trivially_copyable_v<V>,
                       std::true_type, std::false_type> {
};

template <typename T>
concept ResultLike = requires(T t) {
    {
        t.ok()
    }
    -> std::convertible_to<bool>;
};

template <typename T>
concept ResultFuncLike = requires(T t) {
    {
        t()
    }
    -> ResultLike;
};

template <ResultLike T>
constexpr T operator&&(T const& lhs, T const& rhs)
{
    [[unlikely]] if (!lhs.ok())
        return lhs;
    return rhs;
}

template <typename ERR_T = err::ErrorCode>
class ELFE_TMP_NODISCARD BaseResult {
public:
    using Self = BaseResult<ERR_T>;
    ERR_T error = ERR_T {};

    constexpr BaseResult(ERR_T error)
        : error(error)
    {
    }
    constexpr BaseResult() noexcept(std::is_nothrow_default_constructible_v<ERR_T>) = default;

    constexpr BaseResult(Self const&) = default;
    constexpr BaseResult(Self&&) = default;

    constexpr Self& operator=(Self const&) = default;
    constexpr Self& operator=(Self&&) = default;

    virtual constexpr bool ok() const noexcept = 0;

    /**
     * @brief Invoke a function if the result is ok
     *
     * @tparam F
     * @tparam Args
     * @param f
     * @param args
     * @return auto
     */
    template <ResultFuncLike F, typename... Args>
    auto then(F&& f, Args&&... args) const
    {
        if (ok())
            return f(std::forward<Args>(args)...);
        return *this;
    }
};

/**
 * @brief Wrapper for a result type
 *
 * @tparam T type to be wrapped, must be copy constructible
 * @tparam ERR_T enum class or any class that support comparison, default constructor and copy constructor
 * @note ERR_T constructor must be constexpr
 * @note ERR_T with a default constructor means no error
 * @param value value to be wrapped
 * @param error error, 0 by default
 *
 */
template <typename T, typename ERR_T = err::ErrorCode>
    requires std::equality_comparable<ERR_T> && std::default_initializable<ERR_T> && std::copy_constructible<ERR_T> && std::copy_constructible<T>
class ELFE_TMP_NODISCARD Result : public BaseResult<ERR_T> {

public:
    using Self = Result<T, ERR_T>;
    using Parent = BaseResult<ERR_T>;
    using Parent::error;
    T value;

    constexpr Result(T value, ERR_T error = ERR_T {})
        requires(!worth_ref<T>::value)
        : Parent(error)
        , value(value)
    {
    }

    constexpr Result() noexcept(std::is_nothrow_default_constructible_v<T>&& std::is_nothrow_constructible_v<ERR_T, int>) = default;
    constexpr Result(Self const&) = default;
    constexpr Result(Self&&) = default;

    template <std::convertible_to<T> U>
    constexpr Result(U&& value, ERR_T error = ERR_T {}) noexcept(std::is_nothrow_constructible_v<T, U&&>)
        requires(worth_ref<U>::value)
        : Parent(error)
        , value(std::forward<U>(value))
    {
    }

    virtual constexpr ~Result() = default;

    constexpr Self& operator=(Self const&) = default;
    constexpr Self& operator=(Self&&) = default;

    constexpr bool ok() const noexcept override { return error == ERR_T {}; }
    ELFE_TMP_EXPLICIT constexpr operator T() const noexcept(std::is_nothrow_copy_constructible_v<T>) { return value; }
    constexpr std::pair<T, ERR_T> as_pair() const noexcept { return { value, error }; }

    template <std::convertible_to<T> U>
    operator Result<U, ERR_T>() const noexcept(std::is_nothrow_constructible_v<U, T>)
    {
        return Result<U, ERR_T>(value, error);
    }

    template <typename U>
    operator Result<U, ERR_T>() && noexcept(std::is_nothrow_constructible_v<U, T>)
        requires std::convertible_to<T, U>
    {
        return Result<U, ERR_T>(std::move(value), error);
    }
};

template <typename ERR_T = err::ErrorCode>
class ELFE_TMP_NODISCARD VoidResult : public BaseResult<ERR_T> {
public:
    using Self = VoidResult<ERR_T>;
    using Parent = BaseResult<ERR_T>;
    using Parent::error;

    constexpr VoidResult(ERR_T error)
        : Parent(error)
    {
    }
    constexpr VoidResult() noexcept(std::is_nothrow_default_constructible_v<ERR_T>) = default;

    constexpr VoidResult(Self const&) = default;
    constexpr VoidResult(Self&&) = default;

    constexpr Self& operator=(Self const&) = default;
    constexpr Self& operator=(Self&&) = default;

    constexpr bool ok() const noexcept override { return error == ERR_T {}; }
    explicit operator bool() const { return ok(); }
};
}

#undef ELFE_TMP_NODISCARD
#undef ELFE_TMP_EXPLICIT

#endif
