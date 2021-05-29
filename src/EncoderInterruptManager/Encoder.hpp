#ifndef ENCODER_WITH_INTERRUPT
#define ENCODER_WITH_INTERRUPT

#include "Config/PinMapping.h"
#include <Arduino.h>

enum RobotSide{LEFT, RIGHT};

template<RobotSide S>
class Encoder {

public:

    Encoder() {
        if(S == LEFT) {
            pinA = ENCODER_LEFT_A;
            pinB = ENCODER_LEFT_B;
            ports[0] = GPIOA, ports[1] = GPIOA;
            pin_maskA = 0x0040;
            pin_maskB = 0x1000;
            shiftA = 6;
            shiftB = 11;

        } else if(S == RIGHT) {
            pinA = ENCODER_RIGHT_A;
            pinB = ENCODER_RIGHT_B;
            ports[0] = GPIOB, ports[1] = GPIOA;
            pin_maskA = 0x0001;
            pin_maskB = 0x0020;
            shiftA = 0;
            shiftB = 4;
        }
    }

    int get_ticks() {
        return ticks;
    }

    void reset_ticks() {
        ticks = 0;
    }

    void begin() {
        pinMode(pinA, INPUT);
        pinMode(pinB, INPUT);
    }

    void initInterrupt() {
        attachInterrupt(digitalPinToInterrupt(pinA), [=](){this->read_state();}, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pinB), [=](){this->read_state();}, CHANGE);
    }

    virtual void read_state() {
        
        state = (state << 2) | ((ports[0]->IDR & pin_maskA) >> shiftA) | ((ports[1]->IDR & pin_maskB) >> shiftB);
        ticks += quadrature_state[state];
        state &= 0b0011;
    }

private:
    volatile int ticks;

    volatile uint8_t state;

    volatile uint32_t pinA;
    volatile uint32_t pinB;

    volatile int pin_maskA;
    volatile int pin_maskB;
    volatile int shiftA;
    volatile int shiftB;

    GPIO_TypeDef* ports[2];
};

#endif ENCODER_WITH_INTERRUPT