#include "InterruptController.h"
#include <Arduino.h>

InterruptController::InterruptController() : ticks(0), time(0), prev_time(0) {}

void InterruptController::inc_ticks() {
    this->eventMask+=FORWARD;
    if(this->eventMask.forward_event_trigger) {
        if(this->ticks < 0) this->ticks = 0;
        this->ticks++;
        this->time = micros();
    } else {
        this->eventMask.reset();
    }
}

void InterruptController::dec_ticks() {
    this->eventMask+=BACKWARD;
    if(this->eventMask.backward_event_trigger) {
        if(this->ticks > 0) this->ticks = 0;
        this->ticks--;
        this->time = micros();
    } else {
        this->eventMask.reset();
    }
}

long InterruptController::get_delta() {
    long tmp = this->prev_time;
    this->prev_time = this->time;
    return this->time - tmp;
}

void InterruptController::reset_ticks() {
    this->ticks = 0;
}

int InterruptController::get_ticks() {
    return this->ticks;
}