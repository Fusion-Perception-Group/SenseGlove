#include "./_cpp_config.hpp"
#include "gpio/gpio.hpp"

#if defined(DUAL_CORE) && defined(CORE_CM4)
#define GET_IT(MASK) __HAL_GPIO_EXTID2_GET_IT(MASK)
#define CLEAR_IT(MASK) __HAL_GPIO_EXTID2_CLEAR_IT(MASK)
#else
#define GET_IT(MASK) __HAL_GPIO_EXTI_GET_IT(MASK)
#define CLEAR_IT(MASK) __HAL_GPIO_EXTI_CLEAR_IT(MASK)
#endif

namespace elfe {
namespace stm32 {
    namespace gpio {
        namespace hidden {
            std::array<CallbackType, GPIO_PINS_N> interrupt_callbacks = { nullptr };

            static inline void cb_handler(const uint32_t pin_mask) noexcept
            try {
                const uint8_t pin_index = __builtin_ctz(pin_mask);

                if (pin_index >= GPIO_PINS_N)
                    return;

                if (interrupt_callbacks[pin_index] != nullptr)
                    interrupt_callbacks[pin_index]();
            } catch (...) {
            }

            static inline void exti_handler(const uint32_t mask) noexcept
            {
                uint32_t its_mask = GET_IT(mask);
                CLEAR_IT(its_mask);

                while (its_mask) {
                    const uint32_t pin_mask = its_mask & (-its_mask);
                    cb_handler(pin_mask);
                    its_mask &= ~pin_mask;
                }
            }

            extern "C" {
            void EXTI0_IRQHandler()
            {
                exti_handler(GPIO_PIN_0);
            }

            void EXTI1_IRQHandler()
            {
                exti_handler(GPIO_PIN_1);
            }

            void EXTI2_IRQHandler()
            {
                exti_handler(GPIO_PIN_2);
            }

            void EXTI3_IRQHandler()
            {
                exti_handler(GPIO_PIN_3);
            }

            void EXTI4_IRQHandler()
            {
                exti_handler(GPIO_PIN_4);
            }

            void EXTI9_5_IRQHandler()
            {
                exti_handler(GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);
            }

            void EXTI15_10_IRQHandler()
            {
                exti_handler(GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);
            }
            }

        } // namespace hidden
        void Pin::trigger_interrupt() const noexcept
        {
#ifdef _ELFE_STM32HX
            EXTI->SWIER1 |= mask;
#else
            EXTI->SWIER |= mask;
#endif
        }

    } // namespace gpio
} // namespace stm32
} // namespace elfe