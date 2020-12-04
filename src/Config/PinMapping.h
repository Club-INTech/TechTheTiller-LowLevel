#ifndef _PIN_MAPPING_h
#define _PIN_MAPPING_h

#include "Defines.h"

#if defined(MAIN)

/* Ethernet */
constexpr uint8_t CS = 9;
constexpr uint8_t RST = 29;
constexpr uint8_t INT = 30;

/* Moteurs */
constexpr uint8_t INA_LEFT = 2;
constexpr uint8_t INB_LEFT = 16;
constexpr uint8_t PIN_PWM_LEFT = 5;

constexpr uint8_t INA_RIGHT = 28;
constexpr uint8_t INB_RIGHT = 27;
constexpr uint8_t PIN_PWM_RIGHT = 8;

/* Roues codeuses */
constexpr uint8_t ENCODER_LEFT_A = 4;
constexpr uint8_t ENCODER_LEFT_B = 3;
constexpr uint8_t ENCODER_RIGHT_A = 25;
constexpr uint8_t ENCODER_RIGHT_B = 32;

/* CAPTEURS */
constexpr uint8_t PIN_JMPR = 10;
constexpr uint8_t SICK_PINS[NBR_OF_DISTANCE_SENSOR] = {
        A2, // 1er
        A3, // 2e
        A4, // 3e
        A5, // 4e
        A6, // 5e
        A1, // 6e
};

constexpr uint8_t PIN_TX_DYNAMIXEL = D9;
constexpr uint8_t PIN_RX_DYNAMIXEL = D9;



// ! Inversées par rapport au pin mapping !
constexpr uint8_t STEP_PIN_RIGHT = 22; //vitesse
constexpr uint8_t DIR_PIN_RIGHT = 23; //direction
constexpr uint8_t RST_PIN_RIGHT = 99; //reset

constexpr uint8_t STEP_PIN_LEFT = 17; //vitesse
constexpr uint8_t DIR_PIN_LEFT = 20; //direction
constexpr uint8_t RST_PIN_LEFT = 99; //reset



/* LEDs debug */
constexpr uint8_t LED1 = 35;
constexpr uint8_t LED2 = 14;
constexpr uint8_t LED3 = 15;
constexpr uint8_t LED4 = 31;


#elif defined(SLAVE)

/* Ethernet */
constexpr uint8_t CS = 10;
constexpr uint8_t RST = 24;


// Moteurs
constexpr uint8_t INA_LEFT = 28;
constexpr uint8_t INB_LEFT = 27;
constexpr uint8_t PIN_PWM_LEFT = 25;

constexpr uint8_t INA_RIGHT = 8;
constexpr uint8_t INB_RIGHT = 9;
constexpr uint8_t PIN_PWM_RIGHT = 7;

//Roues codeuses
constexpr uint8_t ENCODER_LEFT_B = 3;
constexpr uint8_t ENCODER_LEFT_A = 4;
constexpr uint8_t ENCODER_RIGHT_B = 30;
constexpr uint8_t ENCODER_RIGHT_A = 29;


/* CAPTEURS */
constexpr uint8_t PIN_JMPR = 2;
constexpr uint8_t SICK_PINS[NBR_OF_DISTANCE_SENSOR] = {
        17, // 1er
        15, // 2e
        16, // 3e
};


constexpr uint8_t PIN_TX_DYNAMIXEL = 0;
constexpr uint8_t PIN_RX_DYNAMIXEL = 0;


// ! Inversées par rapport au pin mapping !
constexpr uint8_t STEP_PIN_RIGHT =5; //vitesse
constexpr uint8_t DIR_PIN_RIGHT = 6; //direction



/* LEDs RGB debug ATTENTION logique inversée */
constexpr uint8_t LED1_1 = 35;
constexpr uint8_t LED1_2 = 36;
constexpr uint8_t LED1_3 = 37;
constexpr uint8_t LED2_1 = 38;
constexpr uint8_t LED2_2 = 14;
constexpr uint8_t LED2_3 = 20;
constexpr uint8_t LED3_1 = 21;
constexpr uint8_t LED3_2 = 22;
constexpr uint8_t LED3_3 = 23;

#endif

void InitAllPins();

#endif //_PIN_MAPPING_h
