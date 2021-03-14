//
// Created by jglrxavpok aka Coin-Coin Ier <3 (27/02) on 20/12/18.
//

#include "MCS.h"


MCS::MCS(): leftMotor(Side::LEFT), rightMotor(Side::RIGHT)  {

    encoderLeft = new Encoder(ENCODER_LEFT_B,ENCODER_LEFT_A);
    encoderRight = new Encoder(ENCODER_RIGHT_B,ENCODER_RIGHT_A);


    initSettings();
    initStatus();
  
#if defined(MAIN)

    leftSpeedPID.setTunings(0.87, 1e-6, 0, 0); //0.5   0.000755   21.5 ; 0.87 1e-6 0
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(0.86, 1.6*1e-6, 0, 0); //0.85 0.000755 0 ; 0.86 1.6*1e-6 0
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
    controlSettings.maxAcceleration = 0.5;//2;
    controlSettings.maxDeceleration = 0.5;//2;


    /* rad/s */
    controlSettings.maxRotationSpeed = 0.5 *PI;


    /* mm/s */
    controlSettings.maxTranslationSpeed = 0; // probablement des cm/s ptdr
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

    robotStatus.orientation = (rightTicks - leftTicks) / 2.0 * TICK_TO_RADIAN + angleOffset;

    /* somme des résultantes */
    int32_t distance = (leftDistance+rightDistance)/2;

    float distanceTravelled = ((rightTicks-previousRightTicks) + (leftTicks-previousLeftTicks))*TICK_TO_MM/2.0f;
    /* le robot calcul sa position */
    robotStatus.x += distanceTravelled * cosf(getAngle());
    robotStatus.y += distanceTravelled * sinf(getAngle());

    currentDistance = distance;

}

void MCS::updateSpeed()
{
    /* le robot calcul sa vitesse */
    averageLeftSpeed.add((leftTicks - previousLeftTicks) * TICK_TO_MM * MCS_FREQ);
    averageRightSpeed.add((rightTicks - previousRightTicks) * TICK_TO_MM * MCS_FREQ);
    robotStatus.speedLeftWheel = averageLeftSpeed.value();
    robotStatus.speedRightWheel = averageRightSpeed.value();

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
    /* Si l'asserv est désactivé */
    time_points_criteria = millis();
    if(!robotStatus.controlled) return;

    encoderLeft->tick();
    encoderRight->tick();

    leftTicks = encoderLeft->read();
    rightTicks = encoderRight->read();

    updatePositionOrientation();
    updateSpeed();

    int32_t leftPWM = leftSpeedPID.compute(robotStatus.speedLeftWheel);
    int32_t rightPWM = rightSpeedPID.compute(robotStatus.speedRightWheel);
    leftMotor.run(leftPWM);
    rightMotor.run(rightPWM);
    previousLeftTicks = leftTicks;
    previousRightTicks = rightTicks;
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
//            digitalWrite(LED3_3, LOW);

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
//            digitalWrite(LED3_2, LOW);
            robotStatus.controlled = false;
            robotStatus.moving = false;
            robotStatus.inGoto = false;
//          leftSpeedPID.fullReset();
//          leftSpeedPID.resetOutput(0);
//          leftMotor.brake();
        }

//    if((ABS(leftSpeedPID.getCurrentState())<0.4*ABS(leftSpeedPID.getCurrentGoal())) && ABS((rightSpeedPID.getCurrentState())<0.4*ABS(rightSpeedPID.getCurrentGoal())) && robotStatus.moving && expectedWallImpact){          //si robot a les deux roues bloquées
//        if (timeCounter==100) {
//            robotStatus.controlledRotation = true;
//
//            leftMotor.setDirection(Direction::NONE);
//            rightMotor.setDirection(Direction::NONE);
//            expectedWallImpact = false;
//            timeCounter = 0;
//            robotStatus.stuck = true;
////            InterruptStackPrint::Instance().push("blocage symétrique");
//#if defined(SLAVE)
//            digitalWrite(LED3_1, LOW);
//            digitalWrite(LED3_2, HIGH);
//            digitalWrite(LED3_3, HIGH);
//#endif
//            stop();
//        }
//        timeCounter++;
//    }
//    else {
//        timeCounter=0;
//    }
//
//    digitalWrite(LED3,robotStatus.moving);
//    if(ABS(ABS(leftSpeedPID.getCurrentState())-ABS(rightSpeedPID.getCurrentState()))>controlSettings.tolerancyDifferenceSpeed && robotStatus.moving){          //si le robot a une seule roue bloquée
//        leftMotor.setDirection(Direction::NONE);
//        rightMotor.setDirection(Direction::NONE);
//        stop();
//        robotStatus.stuck=true;
//#if defined(MAIN)
//        digitalWrite(LED4,HIGH);
//#elif defined(SLAVE)
//        digitalWrite(LED3_1,LOW);
//#endif
//
//    }
        /*if(translationPID.getDerivativeError()==0 && ABS(translationPID.getCurrentOutput()-translationPID.getCurrentGoal())<=controlSettings.tolerancyTranslation && rotationPID.getDerivativeError()==0 && ABS(rotationPID.getCurrentOutput()-rotationPID.getCurrentGoal())<=controlSettings.tolerancyAngle){
            leftMotor.setDirection(Direction::NONE);
            rightMotor.setDirection(Direction::NONE);
            digitalWrite(LED1,HIGH);
        }*/
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


//    bool shouldResetP2P = true;
//    if(!robotStatus.inRotationInGoto) {
//        if(!robotStatus.moving && robotStatus.inGoto && ABS(targetX-robotStatus.x)>=controlSettings.tolerancyX && ABS(targetY-robotStatus.y)>=controlSettings.tolerancyY && !robotStatus.stuck){
//            translationPID.resetErrors();
//            rotationPID.resetErrors();
//            leftSpeedPID.resetErrors();
//            rightSpeedPID.resetErrors();
//
//            gotoPoint2(targetX,targetY);
//            InterruptStackPrint::Instance().push(EVENT_HEADER, "renvoie un goto");
//            shouldResetP2P = false;
//
//        }
//        else {
//            InterruptStackPrint::Instance().push(EVENT_HEADER, "stoppedMoving");
//            robotStatus.inGoto=false;
//            leftSpeedPID.setGoal(0);
//            rightSpeedPID.setGoal(0);
//            rotationPID.setGoal(robotStatus.orientation);
//        }
//    }
//
//    if(shouldResetP2P) {
//        robotStatus.moving = false;
//        robotStatus.inRotationInGoto = false;
//        robotStatus.movement = MOVEMENT::NONE;
//    }

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
//    rotationPID.active = false;
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

//#if defined(MAIN)
//
//    if(1.57<ABS(differenceAngle)) {
//        rotationPID.setTunings(3.5,0.000001,0,0);
//    }
//    else if (0.75<ABS(differenceAngle) and ABS(differenceAngle)<=1.57) {
//        rotationPID.setTunings(5.1,0.000001,0,0);
//    }
//    else {
//        rotationPID.setTunings(9,0.000001,0,0);
//    }
//
//#elif defined(SLAVE)
//
//    if((1<=ABS(differenceAngle) and ABS(differenceAngle)<1.5)){
//        //rotationPID.setTunings(7.74,0.000001,0,0);
//        rotationPID.setTunings(-5.4*ABS(differenceAngle)+13.07,0.000001,0,0);
//    }
//    else if(ABS(differenceAngle)>=1.5){
//        rotationPID.setTunings(-1.15*ABS(differenceAngle)+7.23,0.000001,0,0);
//    }
//    else{
//        rotationPID.setTunings(-13.54*ABS(differenceAngle)+20.53,0.000001,10,0);
//    }
//
//#endif

    robotStatus.movement = (differenceAngle < PI && differenceAngle > - PI) ? MOVEMENT::TRIGO : MOVEMENT::ANTITRIGO;

    rotationPID.setGoal(targetAngle);
    robotStatus.moving = true;
#if defined(MAIN)
    digitalWrite(LED2,LOW);
#elif defined(SLAVE)
    digitalWrite(LED2_1,LOW);
#endif
}

/*void MCS::gotoPoint(int16_t x, int16_t y, bool sequential) {
    targetX = x;
    targetY = y;
    robotStatus.controlledP2P = true;
    sequentialMovement = sequential;
    robotStatus.moving = true;
}*/

void MCS::gotoPoint2(int16_t x, int16_t y) {

//    robotStatus.inGoto=true;
//    targetX = x;
//    targetY = y;
//    digitalWrite(LED2,LOW);
//    float dx = x-robotStatus.x;
//    float dy = y-robotStatus.y;
//    ComMgr::Instance().printfln(DEBUG_HEADER, "goto %i %i (diff is %f %f) x= %f; y= %f", x, y, dx, dy, robotStatus.x, robotStatus.y);
//    float rotation = atan2f(dy, dx);
//    ComMgr::Instance().printfln(DEBUG_HEADER, "Required angle: %f", rotation);
//
//    rotate(rotation);
//    robotStatus.inRotationInGoto = true;
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
