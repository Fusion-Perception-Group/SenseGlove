#pragma once
#include "_hpp_config.hpp"
#include "clock/clock.hpp"
#include "flash/flash.hpp"
#include "nvic/nvic.hpp"
#include "result.hpp"
#include "utils/property.hpp"
#include <cstdint>
#include <stdexcept>
#include <string>

namespace elfe {
namespace stm32 {
    namespace mcu {
        using EC = err::ErrorCode;
        constexpr const std::string_view modelname = ModelName;

        /**
         * @brief init mcu
         *
         * @throw std::runtime_error if init failed
         */
        VoidResult<> init();
        VoidResult<> default_init();

    }
}
}
