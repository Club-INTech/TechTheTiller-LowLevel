// defines.h

#ifndef _DEFINES_h
#define _DEFINES_h

#include <Arduino.h>
#include "ComOptions.h"

/**
*COMMUNICATION
*/

#define SIGN(var) ABS(var) / var


constexpr uint32_t MIN_TIME_BETWEEN_GOTO_TR_ROT = 100; // en ticks d'asserv'
// Nombre d'octets acceptables depuis le HL
constexpr uint8_t RX_BUFFER_SIZE = 64; // Taille max des messages
constexpr uint8_t RX_WORD_COUNT = 10; // Nombre de mots max par ordre

constexpr uint8_t INTERRUPT_PRINT_STACK_MAX_SIZE = 200;

#if defined(MAIN)
constexpr uint8_t NBR_OF_US_SENSOR = 0;
constexpr uint8_t NBR_OF_DISTANCE_SENSOR = 6;
constexpr uint8_t NBR_SICK_MEASUREMENTS = 100;
#elif defined(SLAVE)
constexpr uint8_t NBR_OF_US_SENSOR = 0;
constexpr uint8_t NBR_OF_DISTANCE_SENSOR = 3;
constexpr uint8_t NBR_SICK_MEASUREMENTS = 100;
#endif

// Divers headers de communication pour l'éthernet
constexpr uint8_t HEADER_LENGTH = 2;

constexpr uint8_t ANALOG_RESOLUTION = 10;

using Header = const char[HEADER_LENGTH];
Header STD_HEADER = {0x40,0x40};
Header DEBUG_HEADER = {0x40,0x43};
Header SENSOR_HEADER = {0x01,0x06};
Header POSITION_HEADER = {'@', 'P'};
Header EVENT_HEADER = {0x40,0x42};

Header SICK_HEADER = { 0x40,0x41 };
Header ATOM_COLOR_HEADER = { 0x20, 0x26 };


// Séparateurs des mots d'ordres
#define SEPARATOR  " "

constexpr uint16_t MAX_MESSAGE_LENGTH = 256;
constexpr uint16_t MAX_RETRY_ATTEMPTS = 1;

// Fréquence d'envoi de la position
constexpr uint8_t F_ENV_POS = 50;


/**
* Asservissement
*/

#define REDUCTION_COEFFICENT(translation_speed) \
            ((400 / (translation_speed))

constexpr int MIN_PWM_REACTION = 15;

constexpr int quadrature_state[]={0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
 
constexpr uint16_t  MCS_FREQ = 100; //100hz
constexpr double    MCS_PERIOD = 1000000.0 / MCS_FREQ; // Durée en µs entre deux mesures
constexpr uint32_t  STEPPER_FREQUENCY = 24000;//3000; // 625/2 Hz
constexpr double    STEPPER_PERIOD = 1000000.0 / STEPPER_FREQUENCY; // Durée en µs entre deux mesures
constexpr uint16_t  POSITION_UPDATE_FREQUENCY = 20; // 20 Hz
constexpr double    POSITION_UPDATE_PERIOD = 1000000.0 / POSITION_UPDATE_FREQUENCY; // Durée en µs entre deux mesures

constexpr uint16_t  TICKS_PER_TURN =            1024;   // Unité : ticks
constexpr float     COD_WHEEL_DIAMETER =        67.29;  // Unité : mm 63.57
constexpr float     MEAN_TICKS_PER_PERIOD =     3.6f;

//distance roue codeuse pneu = 14.36mm

constexpr uint8_t   DISTANCE_COD_GAUCHE_CENTRE = 160; // Unité : mm
constexpr uint8_t   DISTANCE_COD_DROITE_CENTRE = 160; // Unité : mm

constexpr float TICK_TO_MM = static_cast<float>(PI*COD_WHEEL_DIAMETER/TICKS_PER_TURN); // Unité : mm/ticks
constexpr float TICK_TO_RADIAN = TICK_TO_MM / DISTANCE_COD_GAUCHE_CENTRE; // Unité : rad/ticks

/**
*  Différents seuils des XL
*/
constexpr uint32_t VELOCITY_THRESHOLD = 1; // en unités de Dynamixel (1 -> ~0.2 rpm)
constexpr float POSITION_THRESHOLD = 3.5f; // en degrés
constexpr uint32_t ARM_POSITION_BUFFER_SIZE = 20; // nombre de positions stockées en même temps (pour pouvoir demander plusieurs positions aux XL à la suite)
constexpr uint32_t ARM_ATTEMPTS_BEFORE_MUTE = 3; // nombre d'essais de lecture de paquets à la suite avant de déclarer qu'un bras ne répondra jamais
constexpr uint32_t MUTE_ARM_DELAY = 50; // délai utilisé pour attendre qu'un bras qui ne répond pas ait bougé
constexpr uint32_t MUTE_ARM_CHECK_DELAY = 300; // délai entre deux vérifications qu'un bras est toujours muet

/**
 * Steppers
 */
const unsigned int STEPPER_DELAY = static_cast<const unsigned int>(0.1 * STEPPER_FREQUENCY); //temporistaion entre les commandes du pas à pas
const unsigned int STEP_COUNT = (1600+200-60)*3*4; //nombre de pas par palet
const unsigned int STEP_COUNT_OUST = (650)*3*4; //nombre de pas par palet

/**
 * Communication I2C
 *
 * IDs pour l'I2C
 */
 constexpr uint8_t ID_MAIN = 1;
 constexpr uint8_t ID_SLAVE_AVANT = 2;
 constexpr uint8_t ID_SLAVE_ARRIERE = 3;

 constexpr uint8_t ID_ORDER_GATE = 3;
 constexpr uint8_t ID_ORDER_VALVE = 1;
 constexpr uint8_t ID_ORDER_SUCK = 2;


#endif
