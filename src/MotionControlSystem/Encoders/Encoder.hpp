/*
    Encoder lib for nucleo l432kc using IDR register
    @author sudo_gauss
*/


#ifndef ENCODER_WITH_INTERRUPT
#define ENCODER_WITH_INTERRUPT

#include "Config/PinMapping.h"
#include <Arduino.h>
#include "PinMaskDefines.h"


enum RobotSide{LEFT, RIGHT}; // defining robot side

/*
    ==========================================================
                    Reading pin state and
                    updating interrupt state

        --> IDR is a register which contains all input pins
        states. When we read IDR register we know exactly
        which pin of the port that we use is in HIGH state

        --> Nucleo l432kc has two ports GPIOA and GPIOB,
        each port has its own IDR register. Example of
        reading IDR : GPIOA->IDR

        --> Each bit of IDR represents a pin number. For
        example, in order to read PA_3 we can make bit 'and'
        on third bit : GPIOA-IDR & 0b01000 (0x0008 in hex)
        ___________________________________________________

        --> Interrupt state is a 4 bit number. First two
        bits represent precedent state and others the actual
        state. (from left to right)

        --> The order of bits in state is: prev_back, 
        prev_fwd, actual_back, actual_fwd (be careful A0 and
        D3 pins may be used only for forward, in order to
        make a right shift)

        --> Example 0b1001 means that backward pin was in
        HIGH state and forward pin was in LOW state, but now
        backward pin in LOW state and forward pin is in HIGH
        state
        _____________________________________________________

        send mail to: tsimafei.liashkevich@telecom-sudparis.eu
    ============================================================
*/

template<RobotSide S>
class Encoder {

public:

    Encoder(uint32_t fwd_pin, uint32_t bwd_pin) {
        pinA = fwd_pin;
        pinB = bwd_pin;
        ports[0] = S == LEFT ? GPIOA : GPIOB;
        ports[1] = GPIOA;
        pin_maskA = pin_mask(pinA);
        pin_maskB = pin_mask(pinB);
        shiftA = get_shift(pinA, FORWARD);
        shiftB = get_shift(pinB, BACKWARD);
        //Serial.printf("pA: %i, pB: %i, sA: %i, sB: %i\n", pin_maskA, pin_maskB, shiftA, shiftB);
        //Serial.println(ports[0] == ports[1]);
        ticks = 0;
        //init state
        state = ((ports[0]->IDR & pin_maskA) >> shiftA) | ((ports[1]->IDR & pin_maskB) >> shiftB);
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
        
        //reading new state : read the beginning of the file to understand
        state = (state << 2) | ((ports[0]->IDR & pin_maskA) >> shiftA) | ((ports[1]->IDR & pin_maskB) >> shiftB);
        ticks += quadrature_state[state]; // updating ticks
        state &= 0b0011; // reinitialising the state and saving the precedent state
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

    GPIO_TypeDef* ports[2]; // ports GPIO that used to read IDR for each pin
};

#endif ENCODER_WITH_INTERRUPT