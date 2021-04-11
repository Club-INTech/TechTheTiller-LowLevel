
#include "Clock.h"

volatile long clock::time_left = 0;
volatile long clock :: prev_time_left = 0;
volatile long clock::time_right = 0;
volatile long clock :: prev_time_right = 0;
volatile int clock::ticks_left = 0;
volatile int clock::ticks_right = 0;


long clock::get_delta_left() {
        long tmp = clock::prev_time_left;
        clock::prev_time_left = clock::time_left;
        return clock::time_left - tmp;
    }

long clock::get_delta_right() {
        long tmp = clock::prev_time_right;
        clock::prev_time_right = clock::time_right;
        return clock::time_right - tmp;
    }

void clock::inc_left_ticks() {
        noInterrupts();
        clock::ticks_left++;
        clock::time_left = micros();
        interrupts();
    }

void clock::dec_left_ticks() {
        noInterrupts();
        clock::ticks_left--;
        clock::time_left = micros();
        interrupts();
    }

void clock::inc_right_ticks() {
        noInterrupts();
        clock::ticks_right++;
        clock::time_right = micros();
        interrupts();
    }

void clock::dec_right_ticks() {
        noInterrupts();
        clock::ticks_right--;
        clock::time_right = micros();
        interrupts();
    }

void clock::reset_left_ticks() {
        clock::ticks_left = 0;
    }

void clock::reset_right_ticks() {
        clock::ticks_right = 0;
    }


void clock::init() {
        clock::ticks_left = 0;
        clock::time_left = 0;
        clock::prev_time_left = 0;

        clock::ticks_right = 0;
        clock::time_right = 0;
        clock::prev_time_right = 0;
    }
