//
// Created by jglrxavpok aka Coin-Coin Ier <3 (27/02) on 20/12/18.
//

#include "MCS.h"


MCS::MCS() 
    : leftMotor(Side::LEFT), rightMotor(Side::RIGHT), 
      encoderLeft(ENCODER_LEFT_A, ENCODER_LEFT_B), encoderRight(ENCODER_RIGHT_A, ENCODER_RIGHT_B), 
      leftTicks(0), previousLeftTicks(0),
      rightTicks(0), previousRightTicks(0)
    {

    initSettings();
    initStatus();
  
#if defined(MAIN)

    leftSpeedPID.setTunings(0.45, 4*1e-4, 1e-2, 0); //0.45, 4*1e-4, 1e-2, 0
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(0.35, 2.9*1e-4, 4*1e-2, 0); //0.35, 2.9*1e-4, 4*1e-2, 0
    rightSpeedPID.enableAWU(false);

    translationPID.setTunings(2,0,0,0);
    translationPID.enableAWU(false);
    rotationPID.setTunings(2.2,0,0,0);
    rotationPID.enableAWU(false);

#elif defined(SLAVE)

/* asserv en vitesse */
    leftSpeedPID.setTunings(0.53, 0.00105, 30, 0);//0.0015
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(0.53, 0.00105, 30, 0);//0.0015
    rightSpeedPID.enableAWU(false);
/* asserv en translation */
    translationPID.setTunings(2.75,0,5,0);//2.75  0  5
    translationPID.enableAWU(false);
/* asserv en rotation */
    rotationPID.setTunings(3.2,0.0000,0,0);  //3.2  0  0
    rotationPID.enableAWU(false);

#endif

    leftMotor.init();
    rightMotor.init();

    encoderLeft.begin();
    encoderRight.begin();

    encoderLeft.initInterrupt();
    encoderRight.initInterrupt();

}

void MCS::initSettings() {
    robotStatus.inRotationInGoto = false;
    robotStatus.movement = MOVEMENT::NONE;


    /* mm/s^2/MCS_PERIOD */
    controlSettings.maxAcceleration = 2;//2;
    controlSettings.maxDeceleration = 2;//2;


    /* rad/s */
    controlSettings.maxRotationSpeed = 0.04 * PI;


    /* mm/s */
    controlSettings.maxTranslationSpeed = 400; 
    controlSettings.tolerancySpeed = 5;

    /* rad */
    controlSettings.tolerancyAngle = 0.0005;

    controlSettings.tolerancyTranslation = 1;
    controlSettings.tolerancyX=10;
    controlSettings.tolerancyY=10;


    /* ms */
    controlSettings.stopDelay = 25;

    /* mm/s */
    controlSettings.tolerancyDerivative = 7;

    /* patate */
    controlSettings.tolerancyDifferenceSpeed = 500*2;

    //filter.set_decade_attenuation(0.04f);
}

void MCS::initStatus() {

    robotStatus.movement = MOVEMENT::NONE;
    robotStatus.moving = false;
    robotStatus.inRotationInGoto = false;
    robotStatus.inGoto = false;
    robotStatus.controlled = true;
    robotStatus.controlledRotation = true;
    robotStatus.controlledTranslation = true;
    previousLeftSpeedGoal = 0;
    previousRightSpeedGoal = 0;
    previousLeftTicks = 0;
    previousRightTicks = 0;
    robotStatus.sentMoveAbnormal = false;
    expectedWallImpact = false;
    rotationPID.active = false;
    translationPID.active = false;
    robotStatus.translation = false;
    leftSpeedPID.active = true;
    rightSpeedPID.active = true;
    robotStatus.forcedMovement = false;

}

void MCS::updatePositionOrientation() {

    leftDistance = (float) (encoderLeft.get_ticks()) * TICK_TO_MM / MEAN_TICKS_PER_PERIOD;
    rightDistance = (float) (encoderRight.get_ticks()) * TICK_TO_MM / MEAN_TICKS_PER_PERIOD; 

    //DEBUG("l: %i, r: %i\n", encoderLeft.get_ticks(), encoderRight.get_ticks());

    float current_angle = getAngle();

    float dxLeft = leftDistance * sinf(current_angle);
    float dyLeft = leftDistance * cosf(current_angle);

    float dxRight = rightDistance * sinf(current_angle);
    float dyRight = rightDistance * cosf(current_angle);

    robotStatus.leftWheelX -= dxLeft;
    robotStatus.leftWheelY += dyLeft;

    robotStatus.rightWheelX -= dxRight;
    robotStatus.rightWheelY += dyRight; 

    // Serial.printf("l: %f, r: %f\n", robotStatus.leftWheelY, robotStatus.rightWheelY);
    // DEBUG("l: %f, r: %f\n", robotStatus.leftWheelY, robotStatus.rightWheelY);

    robotStatus.orientation = atan((robotStatus.rightWheelY - robotStatus.leftWheelY) / (robotStatus.rightWheelX - robotStatus.leftWheelX)) + angleOffset;

    /* somme des résultantes */
    robotStatus.x = (robotStatus.leftWheelX + robotStatus.rightWheelX) / 2.0f;
    robotStatus.y = (robotStatus.leftWheelY + robotStatus.rightWheelY) / 2.0f;

    

}

void MCS::updateSpeed()
{
    /* le robot calcul sa vitesse */
    robotStatus.speedLeftWheel = leftDistance * (float) MCS_FREQ;
    robotStatus.speedRightWheel = rightDistance * (float) MCS_FREQ;

    if(robotStatus.speedLeftWheel > 1500 || robotStatus.speedRightWheel > 1500 || robotStatus.speedLeftWheel < -1500 || robotStatus.speedRightWheel < -1500) {
        robotStatus.speedLeftWheel = 0;
        robotStatus.speedRightWheel = 0;
    }

    // if(robotStatus.speedLeftWheel > 2*robotStatus.speedTranslation || robotStatus.speedLeftWheel < -2*robotStatus.speedTranslation) {
    //     robotStatus.speedLeftWheel = robotStatus.speedTranslation;
    // }

    // if(robotStatus.speedRightWheel > 2*robotStatus.speedTranslation || robotStatus.speedRightWheel < -2*robotStatus.speedTranslation) {
    //     robotStatus.speedRightWheel = robotStatus.speedTranslation;
    // }

    // if(robotStatus.speedLeftWheel != 0 || robotStatus.speedRightWheel != 0) {
    //     DEBUG("l: %f, r: %f\n", robotStatus.speedLeftWheel, robotStatus.speedRightWheel);
    // }

    encoderLeft.reset_ticks();
    encoderRight.reset_ticks();

    //previousLeftTicks = leftTicks;
    //previousRightTicks = rightTicks;

    if(robotStatus.controlledTranslation)
    {
        /* retourne la sortie du PID en translation */
        robotStatus.speedTranslation = translationPID.compute(currentDistance);
    }
    else if(!robotStatus.forcedMovement)
    {
        robotStatus.speedTranslation = 0.0f;
    }

    if(robotStatus.controlledRotation && !expectedWallImpact)
    {
        /* retourne la sortie du PID en rotation */
        robotStatus.speedRotation = rotationPID.compute(robotStatus.orientation);
    }
    else if(!robotStatus.forcedMovement)
    {
        robotStatus.speedRotation = 0.0f;
    }

    robotStatus.speedTranslation = MAX(-controlSettings.maxTranslationSpeed, MIN(controlSettings.maxTranslationSpeed, robotStatus.speedTranslation));
    robotStatus.speedRotation = MAX(-controlSettings.maxRotationSpeed, MIN(controlSettings.maxRotationSpeed, robotStatus.speedRotation)) * DISTANCE_COD_GAUCHE_CENTRE;

    if (leftSpeedPID.active) {
        /* défini la consigne en vitesse gauche */
        leftSpeedPID.setGoal(robotStatus.speedTranslation - robotStatus.speedRotation * 36.325f); //on travaille en vitesse, ptdr les vieux vous etes en vacances ou quoi?
    }
    if (rightSpeedPID.active) {
        /* défini la consigne en vitesse droite */
        rightSpeedPID.setGoal(robotStatus.speedTranslation + robotStatus.speedRotation * 36.325f);
    }


    if( leftSpeedPID.getCurrentGoal() - previousLeftSpeedGoal > controlSettings.maxAcceleration ) {
        leftSpeedPID.setGoal( previousLeftSpeedGoal + controlSettings.maxAcceleration );
    }
    if( previousLeftSpeedGoal - leftSpeedPID.getCurrentGoal() > controlSettings.maxDeceleration && !robotStatus.stuck) {
        leftSpeedPID.setGoal( previousLeftSpeedGoal - controlSettings.maxDeceleration );
    }

    if( rightSpeedPID.getCurrentGoal() - previousRightSpeedGoal > controlSettings.maxAcceleration ) {
        rightSpeedPID.setGoal( previousRightSpeedGoal + controlSettings.maxAcceleration );
    }
    if( previousRightSpeedGoal - rightSpeedPID.getCurrentGoal() > controlSettings.maxDeceleration && !robotStatus.stuck) {
        rightSpeedPID.setGoal( previousRightSpeedGoal - controlSettings.maxDeceleration );
    }

    previousLeftSpeedGoal = leftSpeedPID.getCurrentGoal();
    previousRightSpeedGoal = rightSpeedPID.getCurrentGoal();

    //DEBUG("l: %f, r: %f\n", previousLeftSpeedGoal, previousRightSpeedGoal);
}

/**
 * Rafraîchit les consignes de vitesses enregistrées aux moteurs
 * D'autres méthodes de MCS se chargent d'enregistrer les consignes. Les consignes sont maintenues jusqu'à un nouvel
 * appel de 'control'
 */
void MCS::control()
{
    /* Si l'asserv est désactivé */
    time_points_criteria = millis();
    if(!robotStatus.controlled) return;

    //leftTicks = encoderLeft.get_ticks();
    //rightTicks = encoderRight.get_ticks();

    updatePositionOrientation();
    updateSpeed();

    int32_t leftPWM = ((255 / (ABS(robotStatus.speedTranslation) + ABS(robotStatus.speedRotation) * 36.325f)) * leftSpeedPID.compute(robotStatus.speedLeftWheel));
    int32_t rightPWM = ((255 / (ABS(robotStatus.speedTranslation) + ABS(robotStatus.speedRotation) * 36.325f)) * rightSpeedPID.compute(robotStatus.speedRightWheel));
    // if (robotStatus.speedTranslation != 0 || robotStatus.speedRotation != 0)  {
    //     leftPWM = ((255 / (ABS(leftSpeedPID.getCurrentGoal()))) * leftSpeedPID.compute(robotStatus.speedLeftWheel) + SIGN(leftSpeedPID.getCurrentGoal() - robotStatus.speedLeftWheel)* MIN_PWM_REACTION); 
    //     rightPWM = ((255 / (ABS(rightSpeedPID.getCurrentGoal()))) * rightSpeedPID.compute(robotStatus.speedRightWheel) + SIGN(rightSpeedPID.getCurrentGoal() - robotStatus.speedRightWheel)* MIN_PWM_REACTION);
    // } else {
    //     leftPWM = ((255 / (controlSettings.maxTranslationSpeed)) * leftSpeedPID.compute(robotStatus.speedLeftWheel) + SIGN(leftSpeedPID.getCurrentGoal() - robotStatus.speedLeftWheel)* MIN_PWM_REACTION); 
    //     rightPWM = ((255 / (controlSettings.maxTranslationSpeed)) * rightSpeedPID.compute(robotStatus.speedRightWheel) + SIGN(rightSpeedPID.getCurrentGoal() - robotStatus.speedRightWheel)* MIN_PWM_REACTION); 
    // }
    leftMotor.run(leftPWM);
    rightMotor.run(rightPWM);
    // if(robotStatus.speedLeftWheel != 0 ) {
    //     DEBUG("lpid: %f, rpid: %f \n", leftSpeedPID.compute(robotStatus.speedLeftWheel), rightSpeedPID.compute(robotStatus.speedRightWheel));
    // }
    
}

void MCS::manageStop() { //TODO :  a modifier pour moins de tolerance
    static int timeCounter =0;

    if(robotStatus.controlledTranslation || robotStatus.controlledRotation) {
        averageRotationDerivativeError.add(rotationPID.getDerivativeError());
        averageTranslationDerivativeError.add(translationPID.getDerivativeError());
        /* tolérance en translation et rotation */
        if (robotStatus.moving && !robotStatus.inGoto &&
            ABS(averageTranslationDerivativeError.value()) <= controlSettings.tolerancyDerivative &&
            ABS(translationPID.getCurrentState() - translationPID.getCurrentGoal()) <=controlSettings.tolerancyTranslation && (
            (ABS(averageRotationDerivativeError.value()) <= controlSettings.tolerancyDerivative &&
            ABS(rotationPID.getCurrentState() - rotationPID.getCurrentGoal()) <= controlSettings.tolerancyAngle) || expectedWallImpact)) {
            leftMotor.setDirection(Direction::NONE);
            rightMotor.setDirection(Direction::NONE);
            bool ElBooly = robotStatus.inRotationInGoto;
            if (robotStatus.inRotationInGoto) {
                gotoTimer = MIN_TIME_BETWEEN_GOTO_TR_ROT;
            }


            stop();
            robotStatus.inRotationInGoto = ElBooly;

        }
        /* tolérance en vitesse */
        if (ABS(robotStatus.speedLeftWheel) <= controlSettings.tolerancySpeed &&
            ABS(robotStatus.speedRightWheel) <= controlSettings.tolerancySpeed &&
            ABS(robotStatus.speedTranslation - robotStatus.speedRotation) <= 1 &&
            ABS(robotStatus.speedTranslation + robotStatus.speedRotation) <= 1 &&
            ABS(leftSpeedPID.getDerivativeError()) <= controlSettings.tolerancyDerivative &&
            ABS(rightSpeedPID.getDerivativeError()) <= controlSettings.tolerancyDerivative &&
            leftSpeedPID.active && rightSpeedPID.active) {
            robotStatus.controlled = false;
            robotStatus.moving = false;
            robotStatus.inGoto = false;
        }
    }
}

void MCS::stop() {
#if defined(MAIN)
    digitalWrite(LED2,HIGH);
#elif defined(SLAVE)
    digitalWrite(LED2_1,LOW);
#endif
    /* on arrête les moteurs */
    leftMotor.stop();
    rightMotor.stop();

    expectedWallImpact = false;

    /* on remet les consignes en translation et rotation à 0 */
    translationPID.setGoal(currentDistance);
    rotationPID.setGoal(robotStatus.orientation);

    /* reset des erreurs */
    translationPID.resetOutput(0);
    rotationPID.resetOutput(0);
    if (robotStatus.stuck)
    {
        robotStatus.inGoto = false;
        robotStatus.inRotationInGoto = false;
        robotStatus.moving = false;

        InterruptStackPrint::Instance().push(EVENT_HEADER,"unableToMove");
    }

    trajectory.clear();
    translationPID.resetErrors();
    rotationPID.resetErrors();
    leftSpeedPID.resetErrors();
    rightSpeedPID.resetErrors();

    robotStatus.stuck=false;
    robotStatus.translation = false;
    robotStatus.inGoto=false;
    robotStatus.moving = false;
}

void MCS::translate(int16_t amount) {
    robotStatus.controlled = true;
    if(!robotStatus.controlledTranslation)
        return;
    targetDistance = amount;
    robotStatus.translation = true;
    leftSpeedPID.active = true;
    rightSpeedPID.active = true;


    if (!translationPID.active){
        translationPID.active = true;
        translationPID.fullReset();
    }

    if(amount == 0) {
        translationPID.setGoal(currentDistance);
        robotStatus.moving = true;
        return;
    }
    robotStatus.movement = amount > 0 ? MOVEMENT::FORWARD : MOVEMENT::BACKWARD;
    translationPID.setGoal(amount + currentDistance);
    robotStatus.moving = true;
#if defined(MAIN)
    digitalWrite(LED2,LOW);
#elif defined(SLAVE)
    digitalWrite(LED2_1,HIGH);
#endif
}

void MCS::rotate(float angle) {
    robotStatus.controlled = true;
    if(!robotStatus.controlledRotation){
        return;
    }
    targetAngle = angle;

    leftSpeedPID.active = true;
    rightSpeedPID.active = true;

    float differenceAngle = robotStatus.orientation-targetAngle;
    while(ABS(differenceAngle) > PI)
    {
        float signe = ABS(differenceAngle)/differenceAngle;
        float ratio = floor(ABS(differenceAngle)/PI);
        targetAngle += signe*2*PI*ratio;


        differenceAngle = robotStatus.orientation-targetAngle;
    }
    if (!rotationPID.active){
        rotationPID.active = true;
        rotationPID.fullReset();
    }

    robotStatus.movement = (differenceAngle < PI && differenceAngle > - PI) ? MOVEMENT::TRIGO : MOVEMENT::ANTITRIGO;

    rotationPID.setGoal(targetAngle);
    robotStatus.moving = true;
#if defined(MAIN)
    digitalWrite(LED2,LOW);
#elif defined(SLAVE)
    digitalWrite(LED2_1,LOW);
#endif
}


void MCS::gotoPoint2(int16_t x, int16_t y) {

    robotStatus.controlled = true;

    robotStatus.translation=false;
    robotStatus.inGoto=true;
    targetX = x;
    targetY = y;

    robotStatus.moving = true;
    robotStatus.inRotationInGoto = true;
    leftSpeedPID.active = true;
    rightSpeedPID.active = true;
}


void MCS::followTrajectory(const double* xTable, const double* yTable, int count) {
    trajectory.set(xTable, yTable, count);
    if(count > 0) { // s'il y a bien des points dans cette trajectoire
        std::pair<double, double> point = trajectory.query();
        gotoPoint(point.first, point.second, false);
    }
}

void MCS::stopTranslation() {
    robotStatus.speedTranslation = 0.0f;
}

void MCS::stopRotation() {
    robotStatus.speedRotation = 0.0f;
}

void MCS::speedBasedMovement(MOVEMENT movement) {
    if(!robotStatus.controlled)
    {
        return;
    }

    robotStatus.moving = true;

    switch(movement)
    {
        case MOVEMENT::FORWARD:
            robotStatus.speedTranslation = controlSettings.maxTranslationSpeed;
            break;

        case MOVEMENT::BACKWARD:
            robotStatus.speedTranslation = -controlSettings.maxTranslationSpeed;
            break;

        case MOVEMENT::TRIGO:
            robotStatus.speedRotation = controlSettings.maxRotationSpeed;
            break;

        case MOVEMENT::ANTITRIGO:
            robotStatus.speedRotation = -controlSettings.maxRotationSpeed;
            break;

        case MOVEMENT::NONE:
        default:
            leftSpeedPID.setGoal(0);
            rightSpeedPID.setGoal(0);
            leftSpeedPID.resetErrors();
            rightSpeedPID.resetErrors();
            robotStatus.speedRotation = 0;
            robotStatus.speedTranslation = 0;
            robotStatus.movement = MOVEMENT::NONE;
            encoderLeft.reset_ticks();
            encoderRight.reset_ticks();
            return;
    }
    robotStatus.movement = movement;
}

void MCS::sendPositionUpdate() {
    // FIXME : Does not seem to work properly
    ComMgr::Instance().printfln(POSITION_HEADER, "%f %f %f %li", robotStatus.x, robotStatus.y, robotStatus.orientation, millis());
}

void MCS::resetEncoders() {
    leftTicks = 0;
    rightTicks = 0;
    encoderLeft.reset_ticks();
    encoderRight.reset_ticks();
    previousLeftTicks = 0;
    previousRightTicks = 0;
    currentDistance = 0;
    translationPID.setGoal(currentDistance);
    rotationPID.setGoal(robotStatus.orientation);
}

void MCS::disableP2P() {
    trajectory.clear();
    robotStatus.inRotationInGoto = false;
}

void MCS::setControl(bool b) {
    robotStatus.controlled = b;
}

void MCS::controlledTranslation(bool b) {
    robotStatus.controlledTranslation = b;
}

void MCS::controlledRotation(bool b) {
    robotStatus.controlledRotation = b;
}

void MCS::setForcedMovement(bool newState) {
    robotStatus.forcedMovement = newState;
}

void MCS::setTranslationSpeed(float speed) {
    robotStatus.speedTranslation = speed;
}

void MCS::setRotationSpeed(float speed) {
    robotStatus.speedRotation = speed;
}

void MCS::setMaxTranslationSpeed(float speed) {
    controlSettings.maxTranslationSpeed = speed;
}

void MCS::setMaxRotationSpeed(float speed) {
    controlSettings.maxRotationSpeed = speed;
}

int16_t MCS::getX() {
    return (int16_t) robotStatus.x;
}

int16_t MCS::getY() {
    return (int16_t) robotStatus.y;
}

float MCS::getAngle() {
    return robotStatus.orientation;
}

void MCS::setX(int16_t x) {
    robotStatus.x = x;
}

void MCS::setY(int16_t y) {
    robotStatus.y = y;
}

void MCS::setAngle(float angle) {
    robotStatus.orientation = angle;
}

void MCS::setAngleOffset(float offset) {
    angleOffset = offset;
}

int32_t MCS::getLeftTicks() {
    return leftTicks;
}

int32_t MCS::getRightTicks() {
    return rightTicks;
}

float MCS::getLeftSpeed() {
    return robotStatus.speedLeftWheel;
}

float MCS::getRightSpeed() {
    return robotStatus.speedRightWheel;
}

float MCS::getXLeftWheel() {
    return robotStatus.leftWheelX;
}

float MCS::getYLeftWheel() {
    return robotStatus.leftWheelY;
}

float MCS::getXRightWheel() {
    return robotStatus.rightWheelX;
}

float MCS::getYRightWheel() {
    return robotStatus.rightWheelY;
}

void MCS::getSpeedGoals(float &leftGoal, float &rightGoal) {
    leftGoal = leftSpeedPID.getCurrentGoal();
    rightGoal = rightSpeedPID.getCurrentGoal();
}

void MCS::expectWallImpact()
{
    expectedWallImpact = true;
}

bool MCS::sentMoveAbnormal() {
    return robotStatus.sentMoveAbnormal;
}

bool MCS::isMoveAbnormal() {
    return robotStatus.stuck;
}

void MCS::setMoveAbnormalSent(bool val) {
    robotStatus.sentMoveAbnormal = val;
}
