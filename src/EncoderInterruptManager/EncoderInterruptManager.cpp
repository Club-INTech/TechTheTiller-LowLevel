#include "EncoderInterruptManager.h"
#include <Arduino.h>
#include "Config/PinMapping.h"

EncoderInterruptManager::EncoderInterruptManager() : leftController(), rightController() {
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), [=](){leftController.inc_ticks();}, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), [=](){leftController.dec_ticks();}, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), [=](){rightController.inc_ticks();}, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), [=](){rightController.dec_ticks();}, FALLING);
}

template<RobotSide S> int EncoderInterruptManager::get_ticks() {
    if(S == LEFT) return leftController.get_ticks();
    else if(S == RIGHT) return rightController.get_ticks();
}

template<RobotSide S> long EncoderInterruptManager::get_delta() {
    if(S == LEFT) return leftController.get_delta();
    else if(S == RIGHT) return rightController.get_delta();
}

template<RobotSide S> void EncoderInterruptManager::reset_ticks() {
    if(S == LEFT) leftController.reset_ticks();
    else if(S == RIGHT) rightController.reset_ticks();
}