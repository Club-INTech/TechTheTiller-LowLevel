#include "SensorMgr.h"

SensorMgr::SensorMgr()
	:highLevel(ComMgr::Instance())
{
}

void SensorMgr::init() {
	pinMode(PIN_JMPR, INPUT_PULLUP);
	analogReadResolution(ANALOG_RESOLUTION);

	//Wire.begin(); //TODO

	for(int i = 0 ; i < NBR_OF_DISTANCE_SENSOR; i++) {
		// TODO: tester chacun des SICK
		distanceSensors[i] = SICKOD2_N250W150I2(SICK_PINS[i], 50, 1074);
	}

	// TODO: changer les valeurs pour chaque SICK (chaque résistance a une précision de ~1%)
	distanceSensors[0].setResistorValue(165);
	distanceSensors[1].setResistorValue(165);
	distanceSensors[2].setResistorValue(165);
#if defined(MAIN)
	distanceSensors[3].setResistorValue(165);
	distanceSensors[4].setResistorValue(165);
	distanceSensors[5].setResistorValue(165);
#endif

	jumperPlugged = isJumperEngaged();
	basicBlocked = false;
}

//Contacteurs et Jumper

bool SensorMgr::isJumperEngaged()
/**
 * Check l'état du jumper
 * @return : Vrai si le jumper est présent
 */
{
	return !digitalRead(PIN_JMPR);				// Inversé car le switch descend à faux quand il est inséré
}

bool SensorMgr::isReadyToGo()
/**
 * Vérifie si on doit lancer le match
 * @return : Vrai si on lance le match
 */
{
	if(jumperPlugged)							// Si le jumper était présent au test précédent
	{
		jumperPlugged = isJumperEngaged();
		return(!jumperPlugged);					// Alors on part si il ne l'est plus à ce test
	}
	jumperPlugged = isJumperEngaged();
	return(false);								// Sinon on ne part pas de toutes façons
}

SICKOD2_N250W150I2& SensorMgr::getDistanceSensor(size_t index) {
	return distanceSensors[index];
}
