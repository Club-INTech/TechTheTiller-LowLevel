#ifndef EVENT_MASK_H
#define EVENT_MASK_H

enum RotationDirection {FORWARD, BACKWARD};

struct EventMask {
    bool forward_event_trigger;
    bool backward_event_trigger;

    EventMask() : forward_event_trigger(false), backward_event_trigger(false) {}

    void operator+=(RotationDirection d) {
        if(d == FORWARD && !backward_event_trigger) forward_event_trigger = true;
        else if(d == BACKWARD && !forward_event_trigger) backward_event_trigger = true;
    }

    void reset() {
        forward_event_trigger = false;
        backward_event_trigger = false;
    }
    
};

#endif