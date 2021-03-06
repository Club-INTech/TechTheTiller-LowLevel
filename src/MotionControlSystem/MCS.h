//
// Created by trotfunky on 26/11/18.
//

#ifndef LL_MCSREBORN_H
#define LL_MCSREBORN_H

#include "Utils/Singleton.hpp"
#include "Utils/Average.hpp"
#include "Utils/Utils.h"
#include "Utils/Debug.h"
#include "Config/Defines.h"
#include "Config/PinMapping.h"
#include "COM/ComMgr.h"
#include "COM/InterruptStackPrint.h"

#include "RobotInfo/ControlSettings.h"
#include "RobotInfo/RobotStatus.h"
#include "Motors/Motor.h"
#include "PID/PID.hpp"
#include "PID/SelfContainedPID.hpp"
#include "PointToPointTrajectory.h"
#include "Utils/Filter.hpp"
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "Encoders/Encoder.hpp"

#include <cmath>

// TODO : Tout docu
// TODO : P'tet passer les config dans un fichier dans src/Config ?
class MCS : public Singleton<MCS>
{
public:
    MCS();

    void initEncoders();

    void manageStop();
    void updatePositionOrientation();
    void updateSpeed();
    void control();
    void stop();
    void smoothReset();
    void stopTranslation();
    void stopRotation();

    void translate(float);
    void rotate(float);
    void gotoPoint(int16_t,int16_t,bool);
    void gotoPoint2(int16_t,int16_t);
    void goto1(int16_t,int16_t);
    void followTrajectory(const double* xTable, const double* yTable, int count);

    void speedBasedMovement(MOVEMENT);

    void setControl(bool);
    void controlledTranslation(bool);
    void controlledRotation(bool);
    void setForcedMovement(bool);
    void setTranslationSpeed(float);
    void setRotationSpeed(float);
    void setMaxTranslationSpeed(float);
    void setMaxRotationSpeed(float);

    void initSettings();
    void initStatus();

    /**
     * Méthode appelée par un InterruptTimer afin d'envoyer au HL la position du robot
     */
    void sendPositionUpdate();

    /**
     * Reset des codeuses, utilisé quand le HL reset la position du robot (grâce aux SICK par exemple)
     */
    void resetEncoders();

    float getX();
    float getY();
    float getAngle();
    int32_t getLeftTicks();
    int32_t getRightTicks();
    float getLeftSpeed();
    float getRightSpeed();

    float getXLeftWheel();
    float getYLeftWheel();

    float getXRightWheel();
    float getYRightWheel();

    long time_points_criteria;

    void getSpeedGoals(float&,float&);

    float getCurrentDistance();

    /**
     * Permet de définir une rotation à la fin d'un mouvement (au lieu de devoir attendre la fin du mouvement et de donner un ordre de rotation)
     * /!\\ Cette valeur est réinitialisée dès la fin du mouvement!!! (Histoire de pas se décaler avec les mouvements suivants)
     * @param offset l'angle, en radians, duquel le robot doit tourner à la fin du mouvement
     */
    void setAngleOffset(float offset);

    /**
     * Annule le suivi de trajectoire courant
     */
    void disableP2P();

    SelfContainedPID<float> translationPID;
    SelfContainedPID<float> rotationPID;

    void setX(float);
    void setY(float);
    void setAngle(float);

    void expectWallImpact();

    bool sentMoveAbnormal();
    bool isMoveAbnormal();
    void setMoveAbnormalSent(bool);

    inline void setLeftTunings(float kp, float ki, float kd) { leftSpeedPID.setTunings(kp, ki, kd); }
    inline void setRightTunings(float kp, float ki, float kd) { rightSpeedPID.setTunings(kp, ki, kd); }

    Motor leftMotor;
    Motor rightMotor;

    RobotStatus robotStatus;
    float currentDistance;

private:
    Encoder<RIGHT> encoderRight;
    Encoder<LEFT> encoderLeft;
    ControlSettings controlSettings;


    SelfContainedPID<float> leftSpeedPID;
    SelfContainedPID<float> rightSpeedPID;

    int16_t targetX;
    int16_t targetY;

    int32_t leftTicks;
    int32_t rightTicks;
    int32_t previousLeftTicks;
    int32_t previousRightTicks;
    float leftDistance;
    float rightDistance;
    float previousLeftSpeedGoal;
    float previousRightSpeedGoal;
    int16_t targetDistance;
    float targetAngle;
    float angleOffset;
    bool expectedWallImpact;

    //Average<float, 100> averageLeftSpeed;
    //Average<float, 100> averageRightSpeed;

    //Filter<float> filter;


#if defined(MAIN)
    Average<float, 25> averageRotationDerivativeError;
    Average<float, 25> averageTranslationDerivativeError;
#elif defined(SLAVE)
    Average<float, 10> averageRotationDerivativeError;
    Average<float, 10> averageTranslationDerivativeError;
#endif

    bool sequentialMovement;
    PointToPointTrajectory trajectory;

    // Timer entre translation et rotation pour les goto
    uint32_t gotoTimer;
};

#endif //LL_MCSREBORN_H
