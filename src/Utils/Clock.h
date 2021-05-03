#pragma once

#include <Arduino.h>

namespace clock {
    extern volatile long time_left;
    extern volatile long prev_time_left;
    extern volatile long time_right;
    extern volatile long prev_time_right;
    extern volatile int ticks_left;
    extern volatile int ticks_right;

    extern bool left_trigo;
    extern bool left_antitrigo;

    extern bool right_trigo;
    extern bool right_antitrigo;

    long get_delta_right();
    long get_delta_left();
    void inc_left_ticks();
    void dec_left_ticks();
    void inc_right_ticks();
    void dec_right_ticks();
    void reset_left_ticks();
    void reset_right_ticks();
    void init();
};