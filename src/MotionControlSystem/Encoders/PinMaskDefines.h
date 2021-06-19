/* Definitions of pin masks used for mask interruption
    Made for Nucleo l432kc !!!!!!!!!!!!!!!!!!!!!!!!!*/

// @author sudo_gauss

//Compiling with -D TEST defines only pin out and basic functions

#ifndef TECH_THE_TILLER_PIN_MASK_DEFINES
#define TECH_THE_TILLER_PIN_MASK_DEFINES


#ifdef TEST

typedef unsigned char uint8_t;

enum {
  D0,   D1,   D2,   D3,   D4,   D5,   D6,   D7,   D8,   D9,
  D10,  D11,  D12,  D13
};

#define NUM_ANALOG_FIRST  14U
#define PIN_A0       NUM_ANALOG_FIRST
constexpr uint8_t A0 = PIN_A0;

#define PIN_A1       (PIN_A0 + 1)
constexpr uint8_t A1 = PIN_A1;


#define PIN_A2       (PIN_A1 + 1)
constexpr uint8_t A2 = PIN_A2;


#define PIN_A3       (PIN_A2 + 1)
constexpr uint8_t A3 = PIN_A3;


#define PIN_A4       (PIN_A3 + 1)
constexpr uint8_t A4 = PIN_A4;


#define PIN_A5       (PIN_A4 + 1)
constexpr uint8_t A5 = PIN_A5;


#define PIN_A6       (PIN_A5 + 1)
constexpr uint8_t A6 = PIN_A6;


#else

#include <Arduino.h>

#endif

enum ENCODER_ORIENTATION {BACKWARD, FORWARD};

// To know the mask of pin you can call pin_mask(your_pin),
// you can add your masks here.
// Mask is a bit representation of pin according to port and number (PA_12 for exemple).
// It has been made to know the state of masked pin using IDR register


inline volatile int pin_mask(uint8_t pin) {
    switch(pin) {
        case A0: case A1:
            return 1 << (pin - A0);
        case A2: case A3: case A4: case A5: case A6:
            return 1 << (pin - A0 + 1);
        case D9:
            return 1 << 8;
        case D10:
            return 1 << 11;
        case D1:
            return 1 << 9;
        case D0: case D2:
            return 1 << (pin + 10);
        case D3:
            return 1;
        case D4: case D5:
            return 1 << (11 -pin);
        case D6:
            return 2;
        case D11: case D12:
            return 1 << (16 - pin);
        case D13:
            return 1 << 3;
    }
}

// Each encoder has forward pin and backward pin
// After reading the encoder pin state we must apply a shift
// This is used to mask a quadrature state defined in "Defines.h".

inline volatile int get_shift(uint8_t pin, ENCODER_ORIENTATION orientation) {
    switch(pin) {
        case A0: case D3: // A0 and D3 can only be forward pins
            return 0;
        case A1:
            return (pin - A0) - 1 + orientation;
        case A2: case A3: case A4: case A5: case A6:
            return (pin - A0) + orientation;
        case D9:
            return (7 + orientation);
        case D10:
            return (10 + orientation);
        case D1:
            return (8 + orientation);
        case D0: case D2:
            return (pin + 9) + orientation;
        case D4: case D5:
            return (10 - pin) + orientation;
        case D6:
            return orientation;
        case D11: case D12:
            return (15 - pin) + orientation;
        case D13:
            return (2 + orientation);
    }
}


// Nucleo l432kc has ports A and B
// For each port there is a struct defining registers (GPIOA and GPIOB)

#ifdef TEST

#else

inline GPIO_TypeDef* get_pin_gpio(uint8_t pin) {
    if((pin >= D0 && pin <= D2) || (pin == D9) || (pin == D10) || (pin >= A0 && pin <= A6)) {
        return GPIOA;
    }
    else {
        return GPIOB;
    }
}

#endif

#endif TECH_THE_TILLER_PIN_MASK_DEFINES