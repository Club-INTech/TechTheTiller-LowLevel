﻿#include "MotionControlSystem.h"

MotionControlSystem::MotionControlSystem() :
											leftEncoder(PIN_B_LEFT_ENCODER, PIN_A_LEFT_ENCODER ),
											rightEncoder(PIN_A_RIGHT_ENCODER, PIN_B_RIGHT_ENCODER),
											leftMotor(Side::LEFT), rightMotor(Side::RIGHT), 
											rightSpeedPID(&currentRightSpeed, &rightPWM, &rightSpeedSetpoint),
											leftSpeedPID(&currentLeftSpeed, &leftPWM, &leftSpeedSetpoint),
											translationPID(&currentDistance, &translationSpeed, &translationSetpoint),
											rotationPID(&currentAngle, &rotationSpeed, &rotationSetpoint),
											averageLeftSpeed(), averageRightSpeed() {
	translationControlled = true;
	rotationControlled = true;
	leftSpeedControlled = true;
	rightSpeedControlled = true;
	controlled = true;
	originalAngle = 0.0;
	rotationSetpoint = 0;
	translationSetpoint = 0;
	leftSpeedSetpoint = 0;
	rightSpeedSetpoint = 0;
	
	speedTest = false;

	lastLeftPWM = 0;
	lastRightPWM = 0;
	x = 0;
	y = 0;
	moving = false;
	moveAbnormal = false;
	moveAbnormalSent = false;
	forcedMovement = false;
	translation = true;
	direction = NONE;

	leftSpeedPID.setOutputLimits(-255, 255);
	rightSpeedPID.setOutputLimits(-255, 255);

	maxSpeed = 3000; // Vitesse maximum, des moteurs (avec une marge au cas ou on s'amuse a faire forcer un peu la bestiole).
	maxSpeedTranslation = 2000; // Consigne max envoyee au PID
	maxSpeedRotation = 1400;

	delayToStop = 100; // temps a l'arret avant de considerer un blocage
	toleranceTranslation = 30;
	toleranceRotation = 50;
	toleranceSpeed = 40;
	toleranceSpeedEstablished = 50; // Doit etre la plus petite possible, sans bloquer les trajectoires courbes 50
	delayToEstablish = 100;
	toleranceDifferentielle = 500; // Pour les trajectoires "normales", verifie que les roues ne font pas nawak chacunes de leur cote.
	
	translationPID.setTunings(10, 0, 50);			//10,0,50
	rotationPID.setTunings(17, 0, 220);
	leftSpeedPID.setTunings(0.11, 0, 0.005); // ki 0.00001
	rightSpeedPID.setTunings(0.11, 0, 0.005);

	maxAcceleration = 4;

	leftMotor.init();
	rightMotor.init();
	enable(true);
}

int32_t MotionControlSystem::getLeftTick()
{
	return leftTicks;
}

int32_t MotionControlSystem::getRightTick()
{
	return rightTicks;
}

void MotionControlSystem::control() {
	if (controlled) {
		// Pour le calcul de la vitesse instantanee :
		static int32_t previousLeftTicks = 0;
		static int32_t previousRightTicks = 0;

		// Pour le calcul de l'acceleration intantanee :
		static int32_t previousLeftSpeedSetpoint = 0;
		static int32_t previousRightSpeedSetpoint = 0;

		leftTicks = leftEncoder.read();
		rightTicks = rightEncoder.read();

		currentLeftSpeed = (leftTicks - previousLeftTicks) * MC_FREQUENCY;
		currentRightSpeed = (rightTicks - previousRightTicks) * MC_FREQUENCY;

		averageLeftSpeed.add(currentLeftSpeed);										//Mise à jour des moyennes des vitesses
		averageRightSpeed.add(currentRightSpeed);

		previousLeftTicks = leftTicks;
		previousRightTicks = rightTicks;

		averageLeftDerivativeError.add(ABS(leftSpeedPID.getDerivativeError()));		// Mise à jour des moyennes de dérivées de l'erreur (pour les blocages)
		averageRightDerivativeError.add(ABS(rightSpeedPID.getDerivativeError()));

		currentLeftSpeed = averageLeftSpeed.value(); // On utilise pour l'asserv la valeur moyenne des dernieres current Speed
		currentRightSpeed = averageRightSpeed.value(); // sinon le robot il fait nawak.

		currentDistance = (leftTicks + rightTicks) / 2;
		currentAngle = ((rightTicks - currentDistance)*DISTANCE_COD_GAUCHE_CENTRE / DISTANCE_COD_DROITE_CENTRE - (leftTicks - currentDistance)) / 2;

		//Mise à jour des pids
		if (translationControlled) {
			translationPID.compute();
		}
		if (rotationControlled) {
			rotationPID.compute();
		}

		// Limitation de la consigne de vitesse en translation
		if (translationSpeed > maxSpeedTranslation)
			translationSpeed = maxSpeedTranslation;
		else if (translationSpeed < -maxSpeedTranslation)
			translationSpeed = -maxSpeedTranslation;

		// Limitation de la consigne de vitesse en rotation
		if (rotationSpeed > maxSpeedRotation)
			rotationSpeed = maxSpeedRotation;
		else if (rotationSpeed < -maxSpeedRotation)
			rotationSpeed = -maxSpeedRotation;

			leftSpeedSetpoint = (int32_t)(translationSpeed - rotationSpeed);
			rightSpeedSetpoint = (int32_t)(translationSpeed + rotationSpeed);

		// Limitation de la vitesse
		if (leftSpeedSetpoint > maxSpeed) {
			leftSpeedSetpoint = maxSpeed;
		}
		else if (leftSpeedSetpoint < -maxSpeed) {
			leftSpeedSetpoint = -maxSpeed;
		}
		if (rightSpeedSetpoint > maxSpeed) {
			rightSpeedSetpoint = maxSpeed;
		}
		else if (rightSpeedSetpoint < -maxSpeed) {
			rightSpeedSetpoint = -maxSpeed;
		}

		//Limiteurs d'accélération et décélération
		if (leftSpeedSetpoint - previousLeftSpeedSetpoint > maxAcceleration)
		{
			leftSpeedSetpoint = (int32_t)(previousLeftSpeedSetpoint + maxAcceleration);
		}
		else if (previousLeftSpeedSetpoint - leftSpeedSetpoint > maxAcceleration)
		{
			leftSpeedSetpoint = (int32_t)(previousLeftSpeedSetpoint - maxAcceleration);
		}
		if (rightSpeedSetpoint - previousRightSpeedSetpoint > maxAcceleration)
		{
			rightSpeedSetpoint = (previousRightSpeedSetpoint + maxAcceleration);
		}
		else if (previousRightSpeedSetpoint - rightSpeedSetpoint > maxAcceleration)
		{
			rightSpeedSetpoint = (previousRightSpeedSetpoint - maxAcceleration);
		}

		// Mise à jour des consignes de vitesse pour calculs postérieurs
		previousLeftSpeedSetpoint = leftSpeedSetpoint;
		previousRightSpeedSetpoint = rightSpeedSetpoint;

		//Mise à jour des PID en vitesse
		if (leftSpeedControlled) {
			leftSpeedPID.compute();
		}
		else {
			leftPWM = 0;
		}
		if (rightSpeedControlled) {
			rightSpeedPID.compute();
		}
		else {
			rightPWM = 0;
		}

		leftMotor.run(leftPWM);
		rightMotor.run(rightPWM);
	}
}

MOVING_DIRECTION MotionControlSystem::getMovingDirection() const
{
	return direction;
}

void MotionControlSystem::enable(bool enable) {
	controlled = enable;
}

void MotionControlSystem::enableTranslationControl(bool enabled) {
	translationControlled = enabled;
}
void MotionControlSystem::enableRotationControl(bool enabled) {
	rotationControlled = enabled;
}

void MotionControlSystem::enableSpeedControl(bool enabled) {
	leftSpeedControlled = enabled;
	rightSpeedControlled = enabled;
}

bool MotionControlSystem::isPhysicallyStopped() {
	return ((translationPID.getDerivativeError() == 0) && (rotationPID.getDerivativeError() == 0)) || (ABS(ABS(leftSpeedPID.getError()) - ABS(rightSpeedPID.getError()))>toleranceDifferentielle);
}

bool MotionControlSystem::isLeftWheelSpeedAbnormal() {
	return (ABS(leftSpeedPID.getError())>toleranceSpeedEstablished);
}

bool MotionControlSystem::isRightWheelSpeedAbnormal() {
	return (ABS(rightSpeedPID.getError())>toleranceSpeedEstablished);
}

void MotionControlSystem::enableForcedMovement() {
	forcedMovement = true;
}

void MotionControlSystem::disableForcedMovement() {
	forcedMovement = false;
}

void MotionControlSystem::manageStop()
{
	static uint32_t time = 0;
	static uint32_t time2 = 0;
	static int32_t timeToEstablish = 0;
	static bool isSpeedEstablished = false;

	if (moving&&
		averageLeftDerivativeError.value()<toleranceSpeedEstablished &&
		averageRightDerivativeError.value()<toleranceSpeedEstablished &&

		leftSpeedPID.getError()<toleranceSpeed &&
		rightSpeedPID.getError()<toleranceSpeed &&

		!forcedMovement) {

		if (timeToEstablish == 0) {
			timeToEstablish = millis();
		}

		else if ((timeToEstablish > delayToEstablish) && !isSpeedEstablished) {
			isSpeedEstablished = true;

		}
	}

	if (isPhysicallyStopped() && moving && !forcedMovement) // Pour un blocage classique
	{
		if (time == 0)
		{ //Debut du timer
			time = millis();
		}
		else
		{
			if ((millis() - time) >= delayToStop)
			{ //Si arreté plus de 'delayToStop' ms
				{ //Stoppe pour cause de fin de mouvement
					moveAbnormal = !(ABS((translationPID.getError()) <= toleranceTranslation) && ABS(rotationPID.getError()) <= toleranceRotation);
					stop();
				}
			}
		}
	}
	else if (forcedMovement && moving) {

		if (time2 == 0)
		{
			time2 = millis();
		}
		else
		{
			if ((millis() - time2) >= delayToStop) {
				if (ABS(translationPID.getError()) <= toleranceTranslation && ABS(rotationPID.getError()) <= toleranceRotation)
				{ //Stoppé pour cause de fin de mouvement
					stop();
					moveAbnormal = false;
				}
			}
		}
	}



	else
	{
		time = 0;
		time2 = 0;
		if (moving)
			moveAbnormal = false;
	}
}


void MotionControlSystem::updatePosition() {
	static volatile int32_t lastDistance = 0;

	float deltaDistanceMm = (currentDistance - lastDistance) * TICK_TO_MM;
	lastDistance = currentDistance;

	x += (deltaDistanceMm * cosf(getAngleRadian()));
	y += (deltaDistanceMm * sinf(getAngleRadian()));
}

void MotionControlSystem::resetPosition()
{
	this->setX(0);
	this->setY(0);
	this->setOriginalAngle(0);
	stop();
}

/**
* Ordres
*/


void MotionControlSystem::orderTranslation(int32_t mmDistance) {

	Serial.println(translationSetpoint);
	translationSetpoint += (int32_t)mmDistance / TICK_TO_MM;
	Serial.println(translationSetpoint);

	if (!moving)
	{
		translationPID.resetErrors();
		moving = true;
	}

	direction = (mmDistance < 0) ? BACKWARD : FORWARD;

	moveAbnormal = false;
}


void MotionControlSystem::orderRotation(float targetAngleRadian, RotationWay rotationWay) {

	static int32_t deuxPiTick = (int32_t)(2 * PI / TICK_TO_RADIAN);
	static int32_t piTick = (int32_t)(PI / TICK_TO_RADIAN);

	int32_t highLevelOffset = originalAngle / TICK_TO_RADIAN;

	int32_t targetAngleTick = targetAngleRadian / TICK_TO_RADIAN;
	Serial.println(targetAngleTick);
	int32_t currentAngleTick = currentAngle + highLevelOffset;

    targetAngleTick = modulo(targetAngleTick, deuxPiTick);
	currentAngleTick = modulo(currentAngleTick, deuxPiTick);

	int32_t rotationTick = targetAngleTick - currentAngleTick;

	if (rotationWay == FREE)
	{
		if (rotationTick >= piTick)
		{
			rotationTick -= deuxPiTick;
		}
		else if (rotationTick < -piTick)
		{
			rotationTick += deuxPiTick;
		}
	}
	else if (rotationWay == TRIGO)
	{
		if (rotationTick < 0)
		{
			rotationTick += deuxPiTick;
		}
	}
	else if (rotationWay == ANTITRIGO)
	{
		if (rotationTick > 0)
		{
			rotationTick -= deuxPiTick;
		}
	}

	rotationSetpoint = currentAngle + rotationTick;

	if (!moving)
	{
		rotationPID.resetErrors();
		moving = true;
	}
	direction = NONE;
	moveAbnormal = false;
}


void MotionControlSystem::orderRawPwm(Side side, int16_t pwm) {
	if (side == Side::LEFT)
		leftMotor.run(pwm);
	else
		rightMotor.run(pwm);
}


void MotionControlSystem::stop() {

	moving = false;
	translationSetpoint = currentDistance;
	rotationSetpoint = currentAngle;

	leftSpeedSetpoint = 0;
	rightSpeedSetpoint = 0;

	//Arrete les moteurs
	leftMotor.run(0);
	rightMotor.run(0);
	//Remet à zéro les erreurs
	translationPID.resetErrors();
	rotationPID.resetErrors();
	leftSpeedPID.resetErrors();
	rightSpeedPID.resetErrors();

	direction = NONE;
}

bool MotionControlSystem::isMoving() const
{
	return moving;
}

bool MotionControlSystem::isMoveAbnormal() const
{
	return moveAbnormal;
}

bool MotionControlSystem::sentMoveAbnormal() const 
{
	return moveAbnormalSent;
}

void MotionControlSystem::setMoveAbnormalSent(bool status) {
	moveAbnormalSent = status;
}

void MotionControlSystem::setRawPositiveTranslationSpeed() {
	translationSpeed = maxSpeedTranslation;
}

void MotionControlSystem::setRawPositiveRotationSpeed() {
	rotationSpeed = maxSpeedRotation;
}

void MotionControlSystem::setRawNegativeTranslationSpeed() {
	translationSpeed = -maxSpeedTranslation;
}

void MotionControlSystem::setRawNegativeRotationSpeed() {
	rotationSpeed = -maxSpeedRotation;
}

void MotionControlSystem::setRawNullSpeed() {
	rotationSpeed = 0;
	translationSpeed = 0;
}

/**
* Getters & Setters
*/

float MotionControlSystem::getAngleRadian() const
{
	return (float)(currentAngle * TICK_TO_RADIAN + originalAngle);
}

void MotionControlSystem::setOriginalAngle(float newAngle)
{
	originalAngle = newAngle - (getAngleRadian() - originalAngle);
}

float MotionControlSystem::getX() const
{
	return x;
}

void MotionControlSystem::setX(float newX)
{
	x = newX;
}

float MotionControlSystem::getY() const
{
	return y;
}

void MotionControlSystem::setY(float newY)
{
	y = newY;
}

void MotionControlSystem::setTranslationSpeed(float raw_speed)
{
	// Conversion de raw_speed de mm / s en ticks / s
	double speed = raw_speed / TICK_TO_MM;

	if (speed < 0) { // SINGEPROOF
		maxSpeedTranslation = 0;
	}
	else {
		maxSpeedTranslation = (int32_t)speed;
	}
}

void MotionControlSystem::setRotationSpeed(float raw_speed)
{
	// Conversion de raw_speed de rad/s en ticks/s
	int speed = raw_speed / TICK_TO_RADIAN;

	if (speed < 0) {
		maxSpeedRotation = 0;
	}
	else {
		maxSpeedRotation = speed;
	}
}

void MotionControlSystem::getTranslationTunings(float &kp, float &ki, float &kd) const {
	kp = translationPID.getKp();
	ki = translationPID.getKi();
	kd = translationPID.getKd();
}
void MotionControlSystem::getRotationTunings(float &kp, float &ki, float &kd) const {
	kp = rotationPID.getKp();
	ki = rotationPID.getKi();
	kd = rotationPID.getKd();
}
void MotionControlSystem::getLeftSpeedTunings(float &kp, float &ki, float &kd) const {
	kp = leftSpeedPID.getKp();
	ki = leftSpeedPID.getKi();
	kd = leftSpeedPID.getKd();
}
void MotionControlSystem::getRightSpeedTunings(float &kp, float &ki, float &kd) const {
	kp = rightSpeedPID.getKp();
	ki = rightSpeedPID.getKi();
	kd = rightSpeedPID.getKd();
}
void MotionControlSystem::setTranslationTunings(float kp, float ki, float kd) {
	translationPID.setTunings(kp, ki, kd);
}
void MotionControlSystem::setRotationTunings(float kp, float ki, float kd) {
	rotationPID.setTunings(kp, ki, kd);
}
void MotionControlSystem::setLeftSpeedTunings(float kp, float ki, float kd) {
	leftSpeedPID.setTunings(kp, ki, kd);
}
void MotionControlSystem::setRightSpeedTunings(float kp, float ki, float kd) {
	rightSpeedPID.setTunings(kp, ki, kd);
}

void MotionControlSystem::getPWMS(uint16_t& left, uint16_t& right) {
	left = leftPWM;
	right = rightPWM;
}

void MotionControlSystem::getSpeedErrors(uint16_t& leftProp, uint16_t& leftIntegral, uint16_t& leftDerivative, uint16_t& rightProp, uint16_t& rightIntegral, uint16_t& rightDerivative) {
	leftProp = leftSpeedPID.getError();
	leftIntegral = leftSpeedPID.getIntegralErrol();
	leftDerivative = leftSpeedPID.getDerivativeError();

	rightProp = rightSpeedPID.getError();
	rightIntegral = rightSpeedPID.getIntegralErrol();
	rightDerivative = rightSpeedPID.getDerivativeError();
}

void MotionControlSystem::rawWheelSpeed(uint16_t speed, uint16_t& leftOut,uint16_t& rightOut) {
	controlled = false;
	translationControlled = false;
	rotationControlled = false;

	leftSpeedSetpoint = speed;
	rightSpeedSetpoint = speed;
	while (ABS(currentLeftSpeed - leftSpeedSetpoint) > 100 && ABS(currentRightSpeed - rightSpeedSetpoint) > 100) {
		delayMicroseconds(MC_FREQUENCY);


		static int32_t previousLeftTicks = 0;
		static int32_t previousRightTicks = 0;

		leftTicks = leftEncoder.read();
		rightTicks = rightEncoder.read();

		currentLeftSpeed = (leftTicks - previousLeftTicks) * MC_FREQUENCY;
		currentRightSpeed = (rightTicks - previousRightTicks) * MC_FREQUENCY;

		averageLeftSpeed.add(currentLeftSpeed);										//Mise à jour des moyennes des vitesses
		averageRightSpeed.add(currentRightSpeed);

		previousLeftTicks = leftTicks;
		previousRightTicks = rightTicks;

		averageLeftDerivativeError.add(ABS(leftSpeedPID.getDerivativeError()));		// Mise à jour des moyennes de dérivées de l'erreur (pour les blocages)
		averageRightDerivativeError.add(ABS(rightSpeedPID.getDerivativeError()));

		currentLeftSpeed = averageLeftSpeed.value(); // On utilise pour l'asserv la valeur moyenne des dernieres current Speed
		currentRightSpeed = averageRightSpeed.value(); // sinon le robot il fait nawak.


		leftSpeedPID.compute();
		rightSpeedPID.compute();

		leftMotor.run(leftPWM);
		rightMotor.run(rightPWM);
	}

	leftOut = currentLeftSpeed;
	rightOut = currentRightSpeed;

	translationControlled = true;
	rotationControlled = true;
	controlled = true;
}

void MotionControlSystem::getSpeedSetpoints(uint16_t& left, uint16_t& right) {
	left = leftSpeedSetpoint;
	right = rightSpeedSetpoint;
}

void MotionControlSystem::printValues() {

	Serial.print("LeftAvg  - ");Serial.println(this->averageLeftSpeed.value());
	Serial.print("RightAvg - "); Serial.println(this->averageRightSpeed.value());
	Serial.print("LeftPwm  - "); Serial.println(this->leftPWM);
	Serial.print("RightPwm - "); Serial.println(this->rightPWM);
}