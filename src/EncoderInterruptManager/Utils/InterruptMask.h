#ifndef INTERRUPT_MASK_H
#define INTERRUPT_MASK_H

enum TriggeredInterruptStates {LEFT_FORWARD = 1, LEFT_BACKWARD = 2, 
                                RIGHT_FORWARD = 4, RIGHT_BACKWARD = 8};

struct InterruptMask {
    
    short mask;

    InterruptMask() : mask(0) {}

    void operator+=(TriggeredInterruptStates state) {
        if(mask & state == 0) mask+=state;
    }

    bool is_complete() {
        return (mask & 15) == 15;
    }

    void reset() {
        mask = 0;
    }
};

#endif