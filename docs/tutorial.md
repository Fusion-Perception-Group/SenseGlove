# ELFE Tutorial

This tutorial covers the basics of using ELFE to control hardware in the embedded device. We will skip some of the cumbersome details and focus on the most important aspects of the library. Check the reference documentation for more details.

## Ideas Behind ELFE

`HAL` and `stdperiph` libraries are great for controlling hardware in the embedded device. However, they are not very user-friendly and require a lot of boilerplate code. ELFE is a library that simplifies the process of controlling hardware by providing a more user-friendly API.

ELFE offers both high level abstraction and low level control over hardware. One can write code that is easy to read and understand, while still having the ability to access the low level hardware registers.Just imaging _Python_ like codes running at _C++_ speed.

## Getting Started

### What is inside ELFE?

The folder structure of the library is mostly based on peripheral devices. `extra` folder contains additional folders that are not part of the STM32 device drivers but some third-party device drivers.

Each folder contains files that are related to the peripheral device. For example, the `gpio` folder contains files that define the classes for GPIO pin and GPIO port.

ELFE is also bundled with several useful functionalities such as an event loop system and a dual-mode error report mechanism.

### How to use ELFE?

- [Check the results and errors](./ttr/results.md)
