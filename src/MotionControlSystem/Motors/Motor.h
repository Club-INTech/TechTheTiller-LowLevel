/**
* Moteur.cpp
*
* Classe de gestion d'un moteur (PWM, direction...)
*
* Recapitulatif pins utilisees pour controler les deux moteurs :
*
* Gauche :
* 	-pins de sens : ??
* 	-pin de pwm : ??
* Droit :
* 	-pins de sens : ??
* 	-pin de pwm : ??
*
*/

#ifndef COMPLETE_LOW_LEVEL_MOTOR_H
#define COMPLETE_LOW_LEVEL_MOTOR_H

#include "Arduino.h"
#include "Utils/Utils.h"
#include "Config/PinMapping.h"

#include <stdint.h>

enum class Direction
{
		BACKWARD, FORWARD, NONE, BRAKE
};

enum class Side
{
    LEFT, RIGHT
};

class Motor {
private:
		Side side;
		Direction direction;
		uint8_t pin_fin;
		uint8_t pin_bin;

		void setPWM(uint8_t pin, int16_t pwm);

public:
		Motor(Side);
		int16_t pwm;
		void setDirection(Direction);
		void init();
		void run(int16_t);

		/**
	 	* Stoppe le moteur.
	 	*
	 	* C'est juste mettre le PWM à 0 mais c'est plus propre niveau code :)
	 	*/
		void stop();
};

#endif //COMPLETE_LOW_LEVEL_MOTOR_H
