#include "dma/dma.hpp"
#include "./_cpp_config.hpp"

namespace elfe {
namespace stm32 {
    namespace dma {

        namespace detail {
#ifdef DMA1_BASE
            _DMAReg& DMA1Reg = *reinterpret_cast<_DMAReg*>(DMA1_BASE);
            _DMAStreamReg& DMA1Stream0Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream0_BASE);
            _DMAStreamReg& DMA1Stream1Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream1_BASE);
            _DMAStreamReg& DMA1Stream2Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream2_BASE);
            _DMAStreamReg& DMA1Stream3Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream3_BASE);
            _DMAStreamReg& DMA1Stream4Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream4_BASE);
            _DMAStreamReg& DMA1Stream5Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream5_BASE);
            _DMAStreamReg& DMA1Stream6Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream6_BASE);
            _DMAStreamReg& DMA1Stream7Reg = *reinterpret_cast<_DMAStreamReg*>(DMA1_Stream7_BASE);
#endif

#ifdef DMA2_BASE
            _DMAReg& DMA2Reg = *reinterpret_cast<_DMAReg*>(DMA2_BASE);
            _DMAStreamReg& DMA2Stream0Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream0_BASE);
            _DMAStreamReg& DMA2Stream1Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream1_BASE);
            _DMAStreamReg& DMA2Stream2Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream2_BASE);
            _DMAStreamReg& DMA2Stream3Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream3_BASE);
            _DMAStreamReg& DMA2Stream4Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream4_BASE);
            _DMAStreamReg& DMA2Stream5Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream5_BASE);
            _DMAStreamReg& DMA2Stream6Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream6_BASE);
            _DMAStreamReg& DMA2Stream7Reg = *reinterpret_cast<_DMAStreamReg*>(DMA2_Stream7_BASE);
#endif
        }

        const DMA Dma1(0, detail::DMA1Reg);
        const DMA_M2M Dma2(1, detail::DMA2Reg);

        extern "C" {
#ifdef DMA1
        void DMA1_Stream0_IRQHandler()
        {
            Dma1.streams[0].global_irq_handler();
        }

        void DMA1_Stream1_IRQHandler()
        {
            Dma1.streams[1].global_irq_handler();
        }

        void DMA1_Stream2_IRQHandler()
        {
            Dma1.streams[2].global_irq_handler();
        }

        void DMA1_Stream3_IRQHandler()
        {
            Dma1.streams[3].global_irq_handler();
        }

        void DMA1_Stream4_IRQHandler()
        {
            Dma1.streams[4].global_irq_handler();
        }

        void DMA1_Stream5_IRQHandler()
        {
            Dma1.streams[5].global_irq_handler();
        }

        void DMA1_Stream6_IRQHandler()
        {
            Dma1.streams[6].global_irq_handler();
        }

        void DMA1_Stream7_IRQHandler()
        {
            Dma1.streams[7].global_irq_handler();
        }
#endif

#ifdef DMA2
        void DMA2_Stream0_IRQHandler()
        {
            Dma2.streams[0].global_irq_handler();
        }

        void DMA2_Stream1_IRQHandler()
        {
            Dma2.streams[1].global_irq_handler();
        }

        void DMA2_Stream2_IRQHandler()
        {
            Dma2.streams[2].global_irq_handler();
        }

        void DMA2_Stream3_IRQHandler()
        {
            Dma2.streams[3].global_irq_handler();
        }

        void DMA2_Stream4_IRQHandler()
        {
            Dma2.streams[4].global_irq_handler();
        }

        void DMA2_Stream5_IRQHandler()
        {
            Dma2.streams[5].global_irq_handler();
        }

        void DMA2_Stream6_IRQHandler()
        {
            Dma2.streams[6].global_irq_handler();
        }

        void DMA2_Stream7_IRQHandler()
        {
            Dma2.streams[7].global_irq_handler();
        }
#endif
        }

    }
}
}
