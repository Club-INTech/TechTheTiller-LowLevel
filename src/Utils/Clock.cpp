
#include "Clock.h"
#include "Config/PinMapping.h"

volatile long clock::time_left = 0;
volatile long clock :: prev_time_left = 0;
volatile long clock::time_right = 0;
volatile long clock :: prev_time_right = 0;
volatile int clock::ticks_left = 0;
volatile int clock::ticks_right = 0;

bool clock::left_trigo = false;
bool clock::left_antitrigo = false;
bool clock::right_trigo = false;
bool clock::right_antitrigo = false;

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
        if(!clock::left_antitrigo) {
            if(digitalRead(ENCODER_RIGHT_A) == HIGH) {
                clock::right_trigo = true;
            } else if(digitalRead(ENCODER_RIGHT_B) == HIGH) {
                clock::right_antitrigo = true;
            }
            clock::left_trigo = true;
            if(clock::ticks_left < 0) {
                clock::ticks_left = 0;
            }
            clock::ticks_left++;
            clock::time_left = micros();
        } else {
            clock::left_trigo = false;
            clock::left_antitrigo = false;
        }
        interrupts();
    }

void clock::dec_left_ticks() {
        noInterrupts();
        if(!clock::left_trigo) {
            clock::left_antitrigo = true;
            if(clock::ticks_left > 0) {
                clock::ticks_left = 0;
            }
            clock::ticks_left--;
            clock::time_left = micros();
        } else {
            clock::left_trigo = false;
            clock::left_antitrigo = false;
        }
        interrupts();
    }

void clock::inc_right_ticks() {
        noInterrupts();
        if(!clock::right_antitrigo) {
            if(digitalRead(ENCODER_LEFT_A) == HIGH) {
                clock::left_trigo = true;
            } else if(digitalRead(ENCODER_LEFT_B) == HIGH) {
                clock::left_antitrigo = true;
            }
            clock::right_trigo = true;
            if(clock::ticks_right < 0) {
                clock::ticks_right = 0;
            }
            clock::ticks_right++;
            clock::time_right = micros();
        } else {
            clock::right_trigo = false;
            clock::right_antitrigo = false;
        }
        interrupts();
    }

void clock::dec_right_ticks() {
        noInterrupts();
        if(!clock::right_trigo) {
            clock::right_antitrigo = true;
            if(clock::ticks_right > 0) {
                clock::ticks_right = 0;
            }
            clock::ticks_right--;
            clock::time_right = micros();
        } else {
            clock::right_trigo = false;
            clock::right_antitrigo = false;
        }
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
