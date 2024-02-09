#pragma once

#include <cstdint>
#include <stdexcept>
#include "property.hpp"
#include "_nvic_generic.hpp"
#include "_nvic_f1.hpp"
#include "_nvic_f4.hpp"
#include "_nvic_h7.hpp"

namespace vms
{
    namespace stm32
    {
        namespace nvic
        {
            enum PriorityGroupPolicy
            {
                Sub4 = 0x7U,      // 0 bits for preemption priority, 4 bits for subpriority
                Pre1_Sub3 = 0x6U, // 1 bits for preemption priority, 3 bits for subpriority
                Pre2_Sub2 = 0x5U, // 2 bits for preemption priority, 2 bits for subpriority
                Pre3_Sub1 = 0x4U, // 3 bits for preemption priority, 1 bits for subpriority
                Pre4_Sub0 = 0x3U, // 4 bits for preemption priority, 0 bits for subpriority
            };

            void set_priority_group(PriorityGroupPolicy policy) noexcept;
            PriorityGroupPolicy get_priority_group();

            void set_priority(IRQn_Type irq, uint8_t priority);
            uint8_t get_priority(IRQn_Type irq) noexcept;

            void enable_irq(IRQn_Type irq) noexcept;
            void disable_irq(IRQn_Type irq) noexcept;

        }
    }
}
