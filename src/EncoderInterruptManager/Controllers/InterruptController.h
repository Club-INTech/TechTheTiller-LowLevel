#ifndef INTERRUPT_CONTROLLER_H
#define INTERRUPT_CONTROLLER_H


#include "EncoderInterruptManager/Utils/EventMask.h"
#include "EncoderInterruptManager/Utils/InterruptMask.h"
#include <Arduino.h>

class InterruptController {
    
public:
    InterruptController();
    long get_delta();
    void inc_ticks();
    void dec_ticks();
    void reset_ticks();
    int get_ticks();
    
private:
    InterruptMask interruptMask;
    EventMask eventMask;
    volatile long time;
    volatile long prev_time;
    volatile int ticks;
};

#endif