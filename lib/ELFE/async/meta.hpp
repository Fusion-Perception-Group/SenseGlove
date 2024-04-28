#pragma once
#include <concepts>
#include <coroutine>
#include <type_traits>
#include <utility>

namespace elfe {
namespace aio {
    template <typename T, typename P = void>
    concept CoroutineHandle = std::derived_from<T, std::coroutine_handle<P>>;

    template <typename T>
    concept CoAwaitAsMethod = requires(T&& t) {
        std::forward<T>(t).operator co_await();
    };

    template <typename T>
    concept CoAwaitAsFriend = requires(T&& t) {
        operator co_await(std::forward<T>(t));
    };

    template <typename T>
    concept Awaitable = CoAwaitAsMethod<T> || CoAwaitAsFriend<T>;

    template <typename T> // valid return types of await_suspend
    concept AwaitSuspendReturnType = std::same_as<T, void> || std::same_as<T, bool> || CoroutineHandle<T>;

    template <typename T> // checks if T is a valid awaiter
    concept Awaiter = requires(T&& t) {
        {
            std::forward<T>(t).await_ready()
        } -> std::same_as<bool>;
        {
            std::forward<T>(t).await_suspend(std::declval<std::coroutine_handle<>>())
        } -> AwaitSuspendReturnType;
        {
            std::forward<T>(t).await_resume()
        } -> std::same_as<void>;
    };

    template <CoAwaitAsMethod T>
    decltype(auto) get_awaiter(T&& t) noexcept(noexcept(std::forward<T>(t).operator co_await()))
    {
        return std::forward<T>(t).operator co_await();
    }

    template <Awaiter T>
    decltype(auto) get_awaiter(T&& t) noexcept(noexcept(std::forward<T>(t)))
    {
        return std::forward<T>(t);
    }

    template <CoAwaitAsFriend T>
    decltype(auto) get_awaiter(T&& t) noexcept(noexcept(operator co_await(std::forward<T>(t))))
    {
        return operator co_await(std::forward<T>(t));
    }

    template <typename T>
    concept _BasePromise = requires(T&& t) {
        {
            std::forward<T>(t).get_return_object()
        } -> CoroutineHandle<T>;
        {
            std::forward<T>(t).initial_suspend()
        } -> Awaiter;
        {
            std::forward<T>(t).final_suspend()
        } -> Awaiter;
        {
            std::forward<T>(t).unhandled_exception()
        } -> std::same_as<void>;
        requires noexcept(std::forward<T>(t).final_suspend());
        requires noexcept(std::forward<T>(t).unhandled_exception());
    };

    template <typename T>
    concept _PromiseReturnVoid = requires(T&& t) {
        {
            std::forward<T>(t).return_void()
        } -> std::same_as<void>;
    };

    template <typename T>
    concept _PromiseReturn = requires(T&& t) {
        std::forward<T>(t).return_value;
    };

    template <typename T>
    concept _PromiseWithAwaitTransform = requires(T&& t) {
        std::forward<T>(t).await_transform;
    };

    template <typename T>
    concept Promise = _BasePromise<T> && (_PromiseReturnVoid<T> || _PromiseReturn<T>);

    template <typename T>
    concept PromiseWithAwaitTransform = Promise<T> && _PromiseWithAwaitTransform<T>;
}
}

// struct Coroutine
// {
//     struct promise_type;
//     using handle_type = std::coroutine_handle<promise_type>;
//     struct promise_type
//     {
//         handle_type get_return_object() noexcept
//         {
//             return handle_type::from_promise(*this);
//         }
//         auto initial_suspend() noexcept
//         {
//             return std::suspend_always{};
//         }
//         auto final_suspend() noexcept
//         {
//             return std::suspend_always{};
//         }
//         void return_void() noexcept
//         {
//         }
//         void unhandled_exception() noexcept
//         {
//         }
//     };
// };

// int main()
// {
//     using namespace elfe::aio;
//     static_assert(CoroutineHandle<Coroutine::handle_type, Coroutine::promise_type>);
//     static_assert(Promise<Coroutine::promise_type>);
//     return 0;
// }