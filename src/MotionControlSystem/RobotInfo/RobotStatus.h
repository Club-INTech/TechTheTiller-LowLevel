//
// Created by trotfunky on 26/11/18.
//

#ifndef LL_ROBOTSTATUS_H
#define LL_ROBOTSTATUS_H

#include <cstdint>

enum class MOVEMENT { FORWARD, BACKWARD, TRIGO, ANTITRIGO, CURVE, NONE };

struct RobotStatus
{
    bool translation;
    bool controlled;
    bool controlledTranslation;
    bool controlledRotation;
    bool inRotationInGoto;
    bool inGoto;
    bool forcedMovement;
    bool moving;
    bool stuck;
    bool termination;

    float x;
    float y;
    float baseX;
    float baseY;
    float orientation;
    float previousOrientation;
    int turnPeriod;

    float leftWheelX;
    float leftWheelY;

    float rightWheelX;
    float rightWheelY;

    MOVEMENT movement;

    float speedTranslation;
    float speedRotation;
    float speedLeftWheel;
    float speedRightWheel;

    float currentLeftSpeedGoal;
    float currentRightSpeedGoal;

    float finalLeftSpeedGoal;
    float finalRightSpeedGoal;

    bool sentMoveAbnormal;

    RobotStatus();
    void updateStatus();
};

#endif //LL_ROBOTSTATUS_H
