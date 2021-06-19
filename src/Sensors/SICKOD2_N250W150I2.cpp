//
// Created by jglrxavpok on 14/01/19.
// Modifié par gaetan le 15/05/19 
//

#include "Arduino.h"
#include "SICKOD2_N250W150I2.h"

SICKOD2_N250W150I2::SICKOD2_N250W150I2(uint8_t pin, uint16_t rangeMin, uint16_t rangeMax): pin(pin), rangeMin(rangeMin), rangeMax(rangeMax) {
    pinMode(pin, INPUT);
}

float SICKOD2_N250W150I2::readRawDistance() {
    uint16_t valueRead = (uint16_t) analogRead(pin);
    double alpha = ((double)valueRead)/(1<<ANALOG_RESOLUTION);
    double tension = alpha * 3.3;
    // le courant est entre 4 mA et 20 mA donc une tension minimale de 0.6V
    const double minCurrent = 0.004;
    const double maxCurrent = 0.020;
    const double minVoltage = (minCurrent * resistorValue);
    const double maxVoltage = (maxCurrent * resistorValue);
    const double maxDistance = 400; // Distance en mm qui correspond à 20mA
    const double minDistance = 100; // Distance en mm
    //tension -= minVoltage;
    //double t = tension/(maxVoltage-minVoltage);
//    Serial.printf("[DEBUG] >> %f (%i - %i)\n", t, rangeMin, rangeMax);
    //return (uint16_t ) (t*rangeMax + (1.0-t) * rangeMin);

    //return (float) (11.36 * tension + 2.52);

    return (float) ((maxDistance - minDistance) / (maxVoltage-minVoltage) * tension + (maxVoltage*minDistance-minVoltage*maxDistance)/(maxVoltage-minVoltage));
}

float SICKOD2_N250W150I2::readDistance() {
    double sum = 0.0;
    for(int i = 0; i < NBR_SICK_MEASUREMENTS; i++) {
        sum += readRawDistance();
        delayMicroseconds(100);
    }
    //return static_cast<uint16_t>(sum / NBR_SICK_MEASUREMENTS);
    return static_cast<float>(sum / NBR_SICK_MEASUREMENTS);
}

void SICKOD2_N250W150I2::setRange(uint16_t min, uint16_t max) {
    rangeMin = min;
    rangeMax = max;
}

void SICKOD2_N250W150I2::setResistorValue(double value) {
    resistorValue = value;
}