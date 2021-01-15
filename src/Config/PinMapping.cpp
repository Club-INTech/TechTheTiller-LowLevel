//
// Created by jglrxavpok on 16/05/19.
//
#include "PinMapping.h"


#if defined(MAIN)

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

#elif defined(SLAVE)

void InitAllPins()
{
    // Jumper
    pinMode(PIN_JMPR, INPUT);

    // Moteurs
    pinMode(PIN_PWM_LEFT, OUTPUT);
    digitalWrite(PIN_PWM_LEFT, LOW);
    pinMode(PIN_PWM_RIGHT, OUTPUT);
    digitalWrite(PIN_PWM_RIGHT, LOW);

    pinMode(FWD_LEFT, OUTPUT);
    digitalWrite(FWD_LEFT, LOW);
    pinMode(BKW_LEFT, OUTPUT);
    digitalWrite(BKW_LEFT, LOW);

    pinMode(FWD_RIGHT, OUTPUT);
    digitalWrite(FWD_RIGHT, LOW);
    pinMode(BKW_RIGHT, OUTPUT);
    digitalWrite(BKW_RIGHT, LOW);

    // Steppers
    pinMode(DIR_PIN_RIGHT, OUTPUT);
    digitalWrite(DIR_PIN_RIGHT, LOW);
    pinMode(STEP_PIN_RIGHT, OUTPUT);
    digitalWrite(STEP_PIN_RIGHT, LOW);


    pinMode(LED1_1, OUTPUT);
    pinMode(LED2_1, OUTPUT);
    pinMode(LED3_1, OUTPUT);
    pinMode(LED1_2, OUTPUT);
    pinMode(LED2_2, OUTPUT);
    pinMode(LED3_2, OUTPUT);
    pinMode(LED1_3, OUTPUT);
    pinMode(LED2_3, OUTPUT);
    pinMode(LED3_3, OUTPUT);

    digitalWrite(LED1_1, HIGH);
    digitalWrite(LED2_1, HIGH);
    digitalWrite(LED3_1, HIGH);
    digitalWrite(LED1_2, HIGH);
    digitalWrite(LED2_2, HIGH);
    digitalWrite(LED3_2, HIGH);
    digitalWrite(LED1_3, HIGH);
    digitalWrite(LED2_3, HIGH);
    digitalWrite(LED3_3, HIGH);
}

#endif
