#ifndef _PIN_MAPPING_h
#define _PIN_MAPPING_h

#include "Defines.h"

#if defined(MAIN)
/**
 * Pinmapping du principal : https://cadlab.io/project/23715/master/circuit/Q2FydGUtUHJpbmNpcGFsZS5icmQ%3D
 */

/* Moteurs */
constexpr uint8_t FWD_LEFT = D9;
constexpr uint8_t BKW_LEFT = D5;
constexpr uint8_t FWD_RIGHT = A2;
constexpr uint8_t BKW_RIGHT = D10;

/* Roues codeuses */
constexpr uint8_t ENCODER_LEFT_A = A5;
constexpr uint8_t ENCODER_LEFT_B = D2;
constexpr uint8_t ENCODER_RIGHT_A = D3;
constexpr uint8_t ENCODER_RIGHT_B = A4;


/* CAPTEURS */
constexpr uint8_t PIN_JMPR = D4;
constexpr uint8_t SICK_PINS[] = {A0, A1, A6, A3};

constexpr uint8_t XL1 = D6;
constexpr uint8_t XL2 = D9;

/* LEDs debug */
constexpr uint8_t LED1 = D8;
constexpr uint8_t LED2 = D7;

void InitAllPins();

#endif

#endif //_PIN_MAPPING_h
