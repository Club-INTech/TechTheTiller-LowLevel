// SensorMgr.h

#ifndef _SENSORMGR_h
#define _SENSORMGR_h

#include <Arduino.h>
#include <array>
#include <algorithm>
//#include <Wire.h>

#include "Utils/Median.h"
#include "Utils/Singleton.hpp"
#include "Config/PinMapping.h"
#include "MotionControlSystem/MCS.h"
#include "MotionControlSystem/RobotStatus.h"
#include "COM/ComMgr.h"
#include "AbstractSensorUS.h"
#include "SICKDT35_B15851.h"


class SensorMgr : public Singleton<SensorMgr>
{
public:

	SensorMgr();
    void init() override;

    bool isJumperEngaged();
	bool isReadyToGo();

	/**
	 * Récupère le capteur à l'indice donné
	 * @param index l'indice du capteur
	 * @return
	 */
	SICKDT35_B15851& getDistanceSensor(size_t index);


private:

    ComMgr& highLevel;

    SICKDT35_B15851 distanceSensors[NBR_OF_DISTANCE_SENSOR];
    MOVEMENT measure_direction;

    bool jumperPlugged;
    bool basicBlocked;

};

#endif
