#pragma once

#include "property.hpp"

namespace vermils
{
namespace stm32
{
    void enable_trace();

    namespace hidden
    {
        extern volatile uint32_t & CYCCNT;
    }
    
    class DataWatchpointTrigger
    {
    public:
        volatile uint32_t & CYCCNT = hidden::CYCCNT;
        mutable tricks::Property<uint32_t> CTRL;
        mutable tricks::Property<bool> CYCCNTENA;
        mutable tricks::Property<bool> EXCTRCENA;
        mutable tricks::Property<bool> PCSAMPLENA;
        mutable tricks::Property<bool> CPIEVTENA;
        mutable tricks::Property<bool> EXCEVTENA;
        mutable tricks::Property<bool> SLEEPEVTENA;
        mutable tricks::Property<bool> LSUEVTENA;
        mutable tricks::Property<bool> FOLDEVTENA;
        mutable tricks::Property<bool> CYCEVTENA;
        mutable tricks::Property<bool> NOPRFCNT;
        mutable tricks::Property<bool> NOCYCCNT;
        mutable tricks::Property<bool> NOTRCPKT;
        mutable tricks::Property<bool> NOEXTTRIG;
        mutable tricks::Property<uint8_t> POSTPRESET;
        mutable tricks::Property<uint8_t> POSTINIT;
        mutable tricks::Property<bool> CYCTAP;
        mutable tricks::Property<uint8_t> SYNCTAP;
        mutable tricks::Property<uint8_t> NUMCOMP;
        //mutable tricks::Property<uint32_t> CYCCNT;
        mutable tricks::Property<uint32_t> CPICNT;
        mutable tricks::Property<uint32_t> EXCCNT;
        mutable tricks::Property<uint32_t> SLEEPCNT;
        mutable tricks::Property<uint32_t> LSUCNT;
        mutable tricks::Property<uint32_t> FOLDCNT;
        const tricks::Property<uint32_t> PCSR;
        mutable tricks::Property<uint32_t> COMP0;
        mutable tricks::Property<uint32_t> MASK0;
        mutable tricks::Property<uint32_t> FUNCTION0;
        mutable tricks::Property<uint32_t> COMP1;
        mutable tricks::Property<uint32_t> MASK1;
        mutable tricks::Property<uint32_t> FUNCTION1;
        mutable tricks::Property<uint32_t> COMP2;
        mutable tricks::Property<uint32_t> MASK2;
        mutable tricks::Property<uint32_t> FUNCTION2;
        mutable tricks::Property<uint32_t> COMP3;
        mutable tricks::Property<uint32_t> MASK3;
        mutable tricks::Property<uint32_t> FUNCTION3;
        DataWatchpointTrigger();
    };
}
}
