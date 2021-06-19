#include "Motor.h"

void Motor::setDirection(Direction directionToSet)
{
		direction = directionToSet;
		if(directionToSet == Direction::NONE || directionToSet == Direction::BRAKE) {
      	setPWM(pin_fin, 0);
      	setPWM(pin_bin, 0);
		}
		// TODO: Brake?
}

Motor::Motor(Side definedSide):side(definedSide), direction(Direction::NONE)
{
		if (side == Side::LEFT) {
				pin_fin = FWD_LEFT;
				pin_bin = BKW_LEFT;
		}
		else if (side == Side::RIGHT) {
				pin_fin = FWD_RIGHT;
				pin_bin = BKW_RIGHT;
		}
		pinMode(pin_fin, OUTPUT);
		pinMode(pin_bin, OUTPUT);
		setDirection(Direction::NONE);
		pwm = 0;
}

// Initialise les pins, le pwm, bref tout ce dont le moteur a besoin
void Motor::init()
{
	//TODO: Initialiser les PWM
//	analogWriteResolution(8);
	analogWriteFrequency(20000); //FIXME: A CHANGER APRES NOUVEAU PONT EN H +pin_pwm ??
}

void Motor::run(int16_t newpwm)
{
		pwm = newpwm;
		if (pwm > 0) {
				setDirection(Direction::FORWARD);
				pwm = (int16_t)MIN(pwm, 255);
		    setPWM(pin_fin, pwm);
		    setPWM(pin_bin, 0);
		}
		else if (pwm < 0) {
				setDirection(Direction::BACKWARD);
				pwm = (int16_t)MIN(-pwm, 255);
	      setPWM(pin_fin, 0);
	      setPWM(pin_bin, pwm);
		}
		else
		{
				setDirection(Direction::NONE);
	      setPWM(pin_fin, 0);
	      setPWM(pin_bin, 0);
		}
}

void Motor::setPWM(uint8_t pin, int16_t pwm) {
    analogWrite(pin, pwm);
}

void Motor::stop() {
	run(0);
}
