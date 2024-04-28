# ELFE - ELFE Library For Embedded

## Introduction

ELFE is a library for STM32 embedded systems. It is designed to be as simple as Python, yet as performant as C++.

## NOTICE

Work in progress. Currently, only certain models of STM32 microcontrollers are supported.

Demos are made with F411CE Blackpill.

## Features

- **Modern** - ELFE is written in modern C++ (C++20).
- **Versatile** - ELFE exposes low-level APIs for hardware access, as well as high-level APIs for application development.
- **Macro-free (Almost)** - ELFE uses macros only when necessary.
- **Built-in Event System** - ELFE has a built-in event system for asynchronous programming based on coroutines.
- **Exceptions support** - ELFE supports both exceptions and return codes for error handling.

## TODO

- [ ] Add Renode integration.
- [ ] Add more tests
- [ ] Change approach for high-speed USART communication.
- [ ] Add USB support.
- [ ] Add SD card support.
- [ ] Add generic HAL support.
- [ ] Implement event system.
- [ ] Implement asynchronous APIs.
- [ ] CMSIS compatibility.
- [ ] Add more STM32 MCU support.
- [ ] Add documentation for codes.
- [ ] Add auto documentation generation.
- [ ] Add more demos
- [ ] Refine current APIs.
- [ ] Publish as a PlatformIO library.

## LED Example

Let there be light!

```cpp
#include "core.hpp"

#define LED_PIN 13
#define LED_GPIO_PORT gpio::ports::PortC

int main()
{
    using namespace elfe::stm32;
    using gpio::Pin;
    using gpio::Switch;

    mcu::init();

    Switch led(LED_GPIO_PORT, LED_PIN);

    while (true) {
        led.on();
        clock::delay(100ms);
        led.off();
        clock::delay(200ms);
    }
}
```

## About exceptions

Inside of `userconfig.hpp`, you can modify the `ELFE_USE_EXCEPTIONS` macro to enable or disable exceptions support.

When exceptions are enabled, ELFE will throw exceptions when an error occurs.

When exceptions are disabled, ELFE will return `Result` objects when an error occurs. If the error is impossible to return (e.g. inside constructors), the program will halt by calling `std::abort()`.

No matter which mode you choose, functions that might fail will always return a `Result` object.

When `ELFE_USE_EXCEPTIONS` is enabled and `ELFE_FORCE_IGNORE_UNUSED_RESULT` is disabled, you must check the return value of a function that might fail.

When `ELFE_USE_EXCEPTIONS` is enabled and `ELFE_FORCE_RESULT_IMPLICIT_CONVERSION` is disabled, implicit conversion from `Result` to its content type is disabled. You must use `.value` to get the content.

When `ELFE_USE_EXCEPTIONS` is disabled, `Result` is a simple wrapper around the content type. You can use `.value()` to get the content.

No matter whether you use exceptions or not, `-fexceptions` flag must be enabled in your compiler.
To reduce the binary size, simply use `--specs=nano.specs` to remove the exception handling code.
You may also want to add `--fno-exceptions` and `--specs=nosys.specs` to `build-unflags` in your configuration file to make sure the feature is enabled in the compiler but not in the runtime.

When the ROM size is not a concern, I suggest to use exceptions for error handling.

## `Result` class

There are two main `BaseResult` classes in ELFE: `Result` and `VoidResult`. Both are template classes. You can customize their error class, by default, they use `ErrorCode` enum class from `errors.hpp`.

Both `BaseResult` have an attribute `error` to store the error. You can use `Result::ok()` to check if the result is successful.
Additionally, `Result` has an attribute `value` to store the content. When exceptions are enabled or `ELFE_FORCE_RESULT_IMPLICIT_CONVERSION` is enabled, `Result` can be implicitly converted to its content type. so you can ignore the wrapper to some degree.
