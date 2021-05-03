#include "EncoderInterruptManager/Utils/TriggeredInterruptStates.h"

struct InterruptMask {
    
    short mask;

    void operator+=(TriggeredInterruptStates state) {
        if(mask & state == 0) mask+=state;
    }

    bool is_complete() {
        return (mask & 15) == 15;
    }

    void operator~() {
        mask = 0;
    }
};