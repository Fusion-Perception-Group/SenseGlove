#include "nvic/nvic.hpp"
#include "./_cpp_config.hpp"

namespace elfe {
namespace stm32 {
    namespace nvic {
        /**
         * @brief Set the priority group object
         *
         * @param policy
         */
        void set_priority_group(PriorityGroupPolicy policy) noexcept
        {
            switch (policy) {
            case Sub4:
                NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
                break;
            case Pre1_Sub3:
                NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_1);
                break;
            case Pre2_Sub2:
                NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
                break;
            case Pre3_Sub1:
                NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_3);
                break;
            case Pre4_Sub0:
                NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
                break;
            }
        }

        Result<PriorityGroupPolicy> get_priority_group()
        {
            switch (NVIC_GetPriorityGrouping()) {
            case NVIC_PRIORITYGROUP_0:
                return Sub4;
            case NVIC_PRIORITYGROUP_1:
                return Pre1_Sub3;
            case NVIC_PRIORITYGROUP_2:
                return Pre2_Sub2;
            case NVIC_PRIORITYGROUP_3:
                return Pre3_Sub1;
            case NVIC_PRIORITYGROUP_4:
                return Pre4_Sub0;
            default:
                ELFE_ERROR(
                    Result<PriorityGroupPolicy>(PriorityGroupPolicy {}, EC::UnknownError),
                    std::runtime_error("nvic get: Unknown priority group"));
            }
        }

        VoidResult<> set_priority(IRQn_Type irq, uint8_t priority)
        {
            ELFE_ERROR_IF(
                priority > 0xFU,
                EC::InvalidArgument,
                std::invalid_argument("priority out of range"));
            NVIC_SetPriority(static_cast<::IRQn_Type>(irq), priority);
            return EC::None;
        }

        uint8_t get_priority(IRQn_Type irq) noexcept
        {
            return NVIC_GetPriority(static_cast<::IRQn_Type>(irq));
        }

        void enable_irq(IRQn_Type irq) noexcept
        {
            NVIC_EnableIRQ(static_cast<::IRQn_Type>(irq));
        }

        void disable_irq(IRQn_Type irq) noexcept
        {
            NVIC_DisableIRQ(static_cast<::IRQn_Type>(irq));
        }

    } // namespace nvic
} // namespace stm32
} // namespace elfe