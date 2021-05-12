#ifndef ENCODER_INTERRUPT_MANAGER
#define ENCODER_INTERRUPT_MANAGER

#include "Utils/Singleton.hpp"
#include "Controllers/InterruptController.h"
#include <Arduino.h>
#include "Config/PinMapping.h"

enum RobotSide {LEFT, RIGHT};

class EncoderInterruptManager : public Singleton<EncoderInterruptManager> {

public:

    EncoderInterruptManager() : leftController(), rightController() {
        attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), [=](){leftController.inc_ticks();}, FALLING);
        attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), [=](){leftController.dec_ticks();}, FALLING);
        attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), [=](){rightController.inc_ticks();}, FALLING);
        attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), [=](){rightController.dec_ticks();}, FALLING);
    }

    template<RobotSide S> int get_ticks() {
        if(S == LEFT)  return leftController.get_ticks();
        else if(S == RIGHT) return rightController.get_ticks();
    }

    template<RobotSide S> long get_delta() {
        if(S == LEFT) return leftController.get_delta();
        else if(S == RIGHT) return rightController.get_delta();
    }

    template<RobotSide S> void reset_ticks() {
        if(S == LEFT) leftController.reset_ticks();
        else if(S == RIGHT) rightController.reset_ticks();
    }

private:
    InterruptController leftController;
    InterruptController rightController;
};

#endif