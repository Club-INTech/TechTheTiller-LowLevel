//
// Created by jglrxavpok aka Coin-Coin Ier <3 (27/02) on 20/12/18.
//

#include "MCS.h"


MCS::MCS(): leftMotor(Side::LEFT), rightMotor(Side::RIGHT), 
                encoderInterruptManager(EncoderInterruptManager::Instance())  {


    encoderLeft = new Encoder(ENCODER_LEFT_B,ENCODER_LEFT_A);
    encoderRight = new Encoder(ENCODER_RIGHT_B,ENCODER_RIGHT_A);


    initSettings();
    initStatus();
  
#if defined(MAIN)

    leftSpeedPID.setTunings(0.92, 3.73*1e-6, 1.15*1e-4, 0); //0.92 3.73*1e-6 1.15e-4
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(0.89, 3.65*1e-6, 1.15*1e-4, 0); //0.89 3.65*1e-6 1.15e-4
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
}

void MCS::initSettings() {
    robotStatus.inRotationInGoto = false;
    robotStatus.movement = MOVEMENT::NONE;


    /* mm/s^2/MCS_PERIOD */
    controlSettings.maxAcceleration = 2;//2;
    controlSettings.maxDeceleration = 2;//2;


    /* rad/s */
    controlSettings.maxRotationSpeed = 0.5 *PI;


    /* mm/s */
    controlSettings.maxTranslationSpeed = 1000; 
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

    filter.set_decade_attenuation(0.04f);
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

    clock::init();

}

void MCS::updatePositionOrientation() {

//    digitalWrite(LED2,LOW);
    if (robotStatus.inGoto) {
        float dx = targetX-robotStatus.x;
        float dy = targetY-robotStatus.y;
        /* calcul de la translation à effectuer */
        float translation = sqrtf(dx * dx + dy * dy);
        /* calcul de la rotation à effectuer */
        float rotation = atan2f(dy, dx);

        float currentAngle = getAngle();
        if (ABS(currentAngle - rotation) > (float) PI) { /* évite de faire un tour de plus de PI rad */
            if (rotation < 0) {
                rotation += 2 * PI;
            } else {
                rotation -= 2 * PI;
            }
        }
        if (translation < 100) {
            robotStatus.inGoto = false;
            translate(translation);
            InterruptStackPrint::Instance().push(EVENT_HEADER, "stoppedMoving");
        }
        /* défini la consigne en rotation */
        rotate(rotation);

        if (ABS(currentAngle - rotation) < 0.01) //0.654 /* si on est à moins de 0.01 rad de la consigne */
        {
            /* active la translation */
            robotStatus.translation = true;
        }
        if (robotStatus.translation) {
            /* défini la consigne en translation */
            translate(translation);
        }
    }

    int32_t leftDistance = leftTicks * TICK_TO_MM;
    int32_t rightDistance = rightTicks * TICK_TO_MM;

    robotStatus.orientation = (encoderInterruptManager.get_all_ticks<LEFT>() - encoderInterruptManager.get_all_ticks<RIGHT>()) / 2.0 * TICK_TO_RADIAN + angleOffset;

    /* somme des résultantes */
    int32_t distance = (leftDistance+rightDistance)/2;

    float distanceTravelled = (encoderInterruptManager.get_ticks<RIGHT>() + encoderInterruptManager.get_ticks<LEFT>())*TICK_TO_MM/2.0f;
    /* le robot calcul sa position */
    robotStatus.x += distanceTravelled * cosf(getAngle());
    robotStatus.y += distanceTravelled * sinf(getAngle());

    currentDistance = distance;

}

void MCS::updateSpeed()
{
    /* le robot calcul sa vitesse */
    if(encoderInterruptManager.get_ticks<LEFT>() != 0) {
        robotStatus.speedLeftWheel = (float) (encoderInterruptManager.get_ticks<LEFT>()) * 1e6 * (float) TICK_TO_MM / encoderInterruptManager.get_delta<LEFT>();
        encoderInterruptManager.reset_ticks<LEFT>();
    }
    if(encoderInterruptManager.get_ticks<RIGHT>() != 0) {
        robotStatus.speedRightWheel = (float) (encoderInterruptManager.get_ticks<RIGHT>()) * 1e6 * (float) TICK_TO_MM / encoderInterruptManager.get_delta<RIGHT>();
        encoderInterruptManager.reset_ticks<RIGHT>();
    }

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
        leftSpeedPID.setGoal(robotStatus.speedTranslation - robotStatus.speedRotation);
    }
    if (rightSpeedPID.active) {
        /* défini la consigne en vitesse droite */
        rightSpeedPID.setGoal(robotStatus.speedTranslation + robotStatus.speedRotation);
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
}

/**
 * Rafraîchit les consignes de vitesses enregistrées aux moteurs
 * D'autres méthodes de MCS se chargent d'enregistrer les consignes. Les consignes sont maintenues jusqu'à un nouvel
 * appel de 'control'
 */
void MCS::control()
{
    //filter.reset_filter(robotStatus.speedTranslation);
    /* Si l'asserv est désactivé */
    time_points_criteria = millis();
    if(!robotStatus.controlled) return;

    if (encoderInterruptManager.get_ticks<LEFT>() != 0) {
        leftTicks = encoderInterruptManager.get_ticks<LEFT>();
    }
    if (encoderInterruptManager.get_ticks<RIGHT>() != 0) {
        rightTicks = encoderInterruptManager.get_ticks<RIGHT>();
    }

    updatePositionOrientation();
    updateSpeed();

    int32_t leftPWM = (0.25 * leftSpeedPID.compute(robotStatus.speedLeftWheel) + 15); 
    int32_t rightPWM = (0.25 * rightSpeedPID.compute(robotStatus.speedRightWheel) + 15);
    leftMotor.run(leftPWM);
    rightMotor.run(rightPWM);
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
            robotStatus.speedRotation = 0;
            robotStatus.speedTranslation = 0;
            robotStatus.movement = MOVEMENT::NONE;
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
    encoderLeft->write(0);
    encoderRight->write(0);
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
