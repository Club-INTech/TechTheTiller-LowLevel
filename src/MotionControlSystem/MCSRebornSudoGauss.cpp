//
// Created by jglrxavpok aka Coin-Coin Ier <3 (27/02) on 20/12/18.
//

/*/////////////////////////////////////////////////////////////////
//===============================================================//

            Patched a lot by SUDOGAUSS. You must verify
            all steps of calculating and updating speed 
            before you start working on MCS. If you are
            not sure about existing code and how to change
            it, please contact me for, so I can explain

            mail: tsimafei.liashkevich@telecom-sudparis.eu
            TECH_THE_TILLER 2020 - 2021. Thank you!!!!!!!!

//===============================================================//
////////////////////////////////////////////////////////////////*/

#include "MCS.h"


MCS::MCS() 
    : leftMotor(Side::LEFT), rightMotor(Side::RIGHT), 
      encoderLeft(ENCODER_LEFT_A, ENCODER_LEFT_B), encoderRight(ENCODER_RIGHT_A, ENCODER_RIGHT_B), 
      leftTicks(0), rightTicks(0)
    {

    initSettings();
    initStatus();
  
#if defined(MAIN)

    leftSpeedPID.setTunings(0.92, 1*1e-4, 3.1*1e-3, 0); //1, 6.75*1e-5, 3*1e-3, 0; new_method 0.92, 0, 2*1e-3
    leftSpeedPID.enableAWU(false);
    rightSpeedPID.setTunings(0.35, 1.8*1e-4, 8*1e-4, 0); //0.63, 6*1e-5, 7.1*1e-3, 0; new_method 0.26, 2.8*1e-4, 0
    rightSpeedPID.enableAWU(false);

    translationPID.setTunings(1,1e-5,0,0); // 1, 1e-5, 0, 0
    translationPID.enableAWU(false);
    rotationPID.setTunings(2.2,0,41,0); // 2.2, 0, 41, 0
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
    controlSettings.maxAcceleration = 10;
    controlSettings.maxDeceleration = 10;


    /* rad/s */
    controlSettings.maxRotationSpeed = PI;


    /* mm/s */
    controlSettings.maxTranslationSpeed = 900; 
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
    robotStatus.currentLeftSpeedGoal = 0;
    robotStatus.currentRightSpeedGoal = 0;
    robotStatus.previousOrientation = 0;
    robotStatus.baseX = 0;
    robotStatus.baseY =0;
    robotStatus.turnPeriod = 0;
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


/* 
    Updates position and orientation using encoders ticks.
    The idea is that between two updates(~10ms) robot has not
    changed a lot his angle with horizontal axe. In other terms
    the angle between two updates is constant.

    Knowing that we measure a distance travelled by each encoder wheel,
    it is enough to project this distance on xAxis and yAxis.

    Than we use acos to determine a new angle. We don't use a measured 
    distance between encoders wheels, but we calculate it, because
    incertitude can cause a acos nan output.
*/

void MCS::updatePositionOrientation() {

    // We use on change interruptions, so we must divide by ticks per period to know the real travalled distance

    leftDistance = (float) (encoderLeft.get_ticks()) * TICK_TO_MM / MEAN_TICKS_PER_PERIOD; 
    rightDistance = (float) (encoderRight.get_ticks()) * TICK_TO_MM / MEAN_TICKS_PER_PERIOD; 


    //run `pio run -e main_debug_ticks -t upload` in order to see ticks in real time in Serial
    #ifdef SERIAL_DEBUG_TICKS 
    SERIAL_DEBUG("l: %i, r: %i\n", encoderLeft.get_ticks(), encoderRight.get_ticks());
    #endif

    float current_angle = getAngle();

    // angle is considered as constant, so we can calculate xAxis and yAxis projections

    float dxLeft = leftDistance * sinf(current_angle);
    float dyLeft = leftDistance * cosf(current_angle);

    float dxRight = rightDistance * sinf(current_angle);
    float dyRight = rightDistance * cosf(current_angle);

    // update positions

    robotStatus.leftWheelX -= dxLeft;
    robotStatus.leftWheelY += dyLeft;

    robotStatus.rightWheelX -= dxRight;
    robotStatus.rightWheelY += dyRight;

    // robot length calculated only with encoders data, we don't use the real one, because of incertitude

    float robotTheoricLength = sqrtf(powf((robotStatus.rightWheelY - robotStatus.leftWheelY),2.0f) + powf((robotStatus.rightWheelX - robotStatus.leftWheelX),2.0f)); 

    //run `pio run -e main_debug_position -t upload` in order to see ticks in real time in Serial
    #ifdef SERIAL_DEBUG_POSITION
    SERIAL_DEBUG("l: %f, r: %f, a: %f\n", robotStatus.leftWheelY, robotStatus.rightWheelY, current_angle);
    #endif

    
    // we calculate a new angle (it is not a real value because acos is defined only in [0, PI])
    float orientationCosinusMeasure = acosf((robotStatus.rightWheelX - robotStatus.leftWheelX) / robotTheoricLength) + angleOffset; 

    // raw angle value, because we don't know number of periods, so it is an angle with 2*pi*k precision
    float rawAngle = robotStatus.rightWheelY < robotStatus.leftWheelY ? 2*PI-orientationCosinusMeasure : orientationCosinusMeasure;

    /* 
        Gaps between current and previous orientation in the case where we move in trigonometric or
        non-trigonometric direction and angle s positif or negatif

        It helps us to determine the real angle.

        For example, if we move in trigonometric direction the angle grows, so the positive gap value
        will be bigger than negative gap within all possible periods.

        Than if positive gap is less than tolerancy(-pi), it does mean that we incremented the period, etc...
        
    */
    float positiveTurnGap = (rawAngle + robotStatus.turnPeriod * 2*PI) - robotStatus.previousOrientation;
    float negativeTurnGap = (rawAngle + (robotStatus.turnPeriod - 1) * 2*PI) - robotStatus.previousOrientation;

    if(ABS(positiveTurnGap) < ABS(negativeTurnGap)) {
        if(positiveTurnGap < -PI) {
            robotStatus.turnPeriod += 1;
        }
        robotStatus.orientation = rawAngle + robotStatus.turnPeriod*2*PI;
    }
    else {
        if(negativeTurnGap > PI) {
            robotStatus.turnPeriod -= 1;
        }
        robotStatus.orientation = rawAngle + (robotStatus.turnPeriod - 1)*2*PI;
    }

    // x and y of robot's center
    robotStatus.x = (robotStatus.leftWheelX + robotStatus.rightWheelX) / 2.0f;
    robotStatus.y = (robotStatus.leftWheelY + robotStatus.rightWheelY) / 2.0f;

    // changing previous orientation for next iteration
    robotStatus.previousOrientation = robotStatus.orientation;

    // current distance
    currentDistance = sqrtf(powf((robotStatus.x - robotStatus.baseX),2) + powf((robotStatus.y - robotStatus.baseY),2));
}

void MCS::updateSpeed()
{

    // in order to know the speed we multiply measured distance by a mcs.control call frequency

    robotStatus.speedLeftWheel = leftDistance * (float) MCS_FREQ;
    robotStatus.speedRightWheel = rightDistance * (float) MCS_FREQ;

    //run `pio run -e main_debug_speed -t upload` in order to see ticks in real time in Serial
    #ifdef SERIAL_DEBUG_SPEED
    SERIAL_DEBUG("l: %f, r: %f\n", robotStatus.speedLeftWheel, robotStatus.speedRightWheel);
    #endif


    if(robotStatus.controlledTranslation)
    {
        robotStatus.speedTranslation = translationPID.compute(currentDistance);
    }
    else if(!robotStatus.forcedMovement) // forced movement is used to activate a speed asservisement
    {
        robotStatus.speedTranslation = 0.0f;
    }

    if(robotStatus.controlledRotation && !expectedWallImpact) // robot can't turn when he is close to the wall
    {
        robotStatus.speedRotation = rotationPID.compute(robotStatus.orientation);
    }
    else if(!robotStatus.forcedMovement)
    {
        robotStatus.speedRotation = 0.0f;
    }

    // order speed must be in segment [-max_speed, max_speed]

    robotStatus.speedTranslation = MAX(-controlSettings.maxTranslationSpeed, MIN(controlSettings.maxTranslationSpeed, robotStatus.speedTranslation));
    robotStatus.speedRotation = MAX(-controlSettings.maxRotationSpeed, MIN(controlSettings.maxRotationSpeed, robotStatus.speedRotation));

    // final goals for each wheel are used because of acceleration
    // we multiply rotation speed by a radius of circle which surround robot's base

    robotStatus.finalLeftSpeedGoal = robotStatus.speedTranslation - robotStatus.speedRotation * DISTANCE_COD_GAUCHE_CENTRE;
    robotStatus.finalRightSpeedGoal = robotStatus.speedTranslation + robotStatus.speedRotation * DISTANCE_COD_DROITE_CENTRE;

    // if speed pids are active, we set a new goal; goal can change because of acceleration or translation goal or rotation goal

    if (leftSpeedPID.active) {
        
        if( robotStatus.currentLeftSpeedGoal + controlSettings.maxAcceleration <= robotStatus.finalLeftSpeedGoal) {
            robotStatus.currentLeftSpeedGoal += controlSettings.maxAcceleration;
        } 
        else if(robotStatus.currentLeftSpeedGoal - controlSettings.maxDeceleration >= robotStatus.finalLeftSpeedGoal && !robotStatus.stuck) {
            robotStatus.currentLeftSpeedGoal -= controlSettings.maxDeceleration;
        } else  {
            robotStatus.currentLeftSpeedGoal = robotStatus.finalLeftSpeedGoal;
        }


        leftSpeedPID.setGoal(robotStatus.currentLeftSpeedGoal);
    
    }

    if (rightSpeedPID.active) {
        
        if( robotStatus.currentRightSpeedGoal + controlSettings.maxAcceleration <= robotStatus.finalRightSpeedGoal) {
            robotStatus.currentRightSpeedGoal += controlSettings.maxAcceleration;
        }

        else if(robotStatus.currentRightSpeedGoal - controlSettings.maxDeceleration >= robotStatus.finalRightSpeedGoal && !robotStatus.stuck) {
            robotStatus.currentRightSpeedGoal -= controlSettings.maxDeceleration;
        } else {
            robotStatus.currentRightSpeedGoal = robotStatus.finalRightSpeedGoal;
        }


        rightSpeedPID.setGoal(robotStatus.currentRightSpeedGoal);
    
    }

}

/* 
    mcs control is the principal function that orchestrate robot movement
    it is called with MCS_FREQ frequency in main loop
*/

void MCS::control()
{
    time_points_criteria = millis();
    if(!robotStatus.controlled) return;


    updatePositionOrientation();
    updateSpeed();

    //calculating a pwm for each wheel

    int32_t leftPWM =  leftSpeedPID.compute(robotStatus.speedLeftWheel);
    int32_t rightPWM = rightSpeedPID.compute(robotStatus.speedRightWheel);

    // if(robotStatus.orientation > 0.1) {
    //     SERIAL_DEBUG("a: %f\n", rotationPID.getDerivativeError());
    // }

    // transmitting a pwm to motors
    
    leftMotor.run(leftPWM);
    rightMotor.run(rightPWM);

    // after entire mcs period, we reset all encoders ticks

    encoderLeft.reset_ticks();
    encoderRight.reset_ticks();
    
}


void MCS::stop() {
    /* on arrête les moteurs */
    leftMotor.stop();
    rightMotor.stop();

    expectedWallImpact = false;

    /* on remet les consignes en translation et rotation à 0 */
    translationPID.setGoal(currentDistance);
    rotationPID.setGoal(robotStatus.orientation);


    /* reset des erreurs */
    if (robotStatus.stuck)
    {
        robotStatus.inGoto = false;
        robotStatus.inRotationInGoto = false;
        robotStatus.moving = false;

        InterruptStackPrint::Instance().push(EVENT_HEADER,"unableToMove");
    }

    trajectory.clear();
    translationPID.fullReset();
    rotationPID.fullReset();
    leftSpeedPID.fullReset();
    rightSpeedPID.fullReset();

    translationPID.resetOutput(0);
    rotationPID.resetOutput(0);
    leftSpeedPID.resetOutput(0);
    rightSpeedPID.resetOutput(0);

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

    if (!rotationPID.active){
        rotationPID.active = true;
        rotationPID.fullReset();
    }

    

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
            leftSpeedPID.fullReset();
            rightSpeedPID.fullReset();

            leftSpeedPID.setGoal(0);
            rightSpeedPID.setGoal(0);

            robotStatus.speedRotation = 0;
            robotStatus.speedTranslation = 0;

            robotStatus.movement = MOVEMENT::NONE;

            //encoderLeft.reset_ticks();
            //encoderRight.reset_ticks();
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
