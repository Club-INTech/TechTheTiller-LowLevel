//
// Created by serandour on 03/10/2019.
//

#ifndef TECHTHETACHYON_LOWLEVEL_ENCODER_H
#define TECHTHETACHYON_LOWLEVEL_ENCODER_H
#include <Arduino.h>
#include <RotaryEncoder.h>

class Encoder {
private:
    RotaryEncoder* backingEncoder;
    int leftPin;
    int rightPin;

public:
    Encoder(int pin1, int pin2);
    int32_t read();
    void write(int val_reset);

    /**
     * Reads the pins to poll the current state of the encoder
     */
    void tick();
    auto get_delta() const { return backingEncoder->getMillisBetweenRotations(); }

};


#endif //TECHTHETACHYON_LOWLEVEL_ENCODER_H
