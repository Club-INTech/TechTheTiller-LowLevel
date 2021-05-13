#ifndef EVENT_MASK_H
#define EVENT_MASK_H

enum RotationDirection {FORWARD, BACKWARD};

struct EventMask {
    bool forward_event_trigger;
    bool backward_event_trigger;

    EventMask() : forward_event_trigger(false), backward_event_trigger(false) {}

    void operator+=(RotationDirection d) {
        forward_event_trigger = (d == FORWARD && !backward_event_trigger || forward_event_trigger);
        backward_event_trigger = (d == BACKWARD && !forward_event_trigger || backward_event_trigger);
    }

    void reset() {
        forward_event_trigger = false;
        backward_event_trigger = false;
    }
    
};

#endif