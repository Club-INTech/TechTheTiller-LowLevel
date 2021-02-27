//
// Created by serandour on 03/10/2019.
//

#include <Config/PinMapping.h>
#include "Encoder.h"

Encoder::Encoder(int pin1, int pin2): leftPin(pin1), rightPin(pin2) {
    backingEncoder = new RotaryEncoder(pin1, pin2);
}

int32_t Encoder::read() {
    return backingEncoder->getPosition();
}

void Encoder::write(int val_reset) {
    backingEncoder->setPosition(val_reset);
}

void Encoder::tick() {
    backingEncoder->tick();
}
