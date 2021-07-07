//
// Created by jglrxavpok on 16/05/19.
//
#include "PinMapping.h"


void InitAllPins() {
    // Jumper
    pinMode(PIN_JMPR, INPUT_PULLUP);

    // Moteurs
    pinMode(FWD_LEFT, OUTPUT);
    pinMode(BKW_LEFT, OUTPUT);
    pinMode(FWD_RIGHT, OUTPUT);
    pinMode(BKW_RIGHT, OUTPUT);
    digitalWrite(FWD_LEFT, LOW);
    digitalWrite(BKW_LEFT, LOW);
    digitalWrite(FWD_RIGHT, LOW);
    digitalWrite(BKW_RIGHT, LOW);

    // Éclairages
    pinMode(LED1,OUTPUT);
    digitalWrite(LED1,LOW);
    pinMode(LED2,OUTPUT);
    digitalWrite(LED2,LOW);
}

