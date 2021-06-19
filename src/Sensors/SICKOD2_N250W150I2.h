//
// Created by jglrxavpok on 14/01/19.
// Modifié par gaetan le 15/05/19
//


#ifndef TECHTHETACHYON_LOWLEVEL_SICKOD2_N250W150I2
#define TECHTHETACHYON_LOWLEVEL_SICKOD2_N250W150I2

#include <stdint.h>
#include <Utils/Average.hpp>
#include "Config/Defines.h"

class SICKOD2_N250W150I2 {

public:
    
    SICKOD2_N250W150I2() = default;

    /**
     * Constructeur de la représentation des capteurs SICK. Le câble gris (MF) n'est pas pris en compte.
     * @param pin pin connectée au câble blanc (QA/Q2 -> sortie)
     * @param bord bas de l'intervalle de détection du SICK (en mm)
     * @param bord haut de l'intervalle de détection du SICK (en mm)
     */
    
    explicit SICKOD2_N250W150I2(uint8_t pin, uint16_t rangeMin = 100, uint16_t rangeMax = 400);

    /**
     * Lis la distance avec le capteur (prend en compte le range configuré) en faisant une moyenne sur NBR_SICK_MEASUREMENTS
     * @return
     */
    float readDistance();

    /**
     * Lis la distance avec le capteur (prend en compte le range configuré)
     * @return
     */
    float readRawDistance();

    /**
     * Configure l'intervalle de détection du capteur
@@ -49,10 +50,10 @@ class SICKDT35_B15851 {

    */
   void setRange(uint16_t min, uint16_t max);

   void setResistorValue(double value);
   
private:
    uint8_t pin = 255;
    double resistorValue = 165.0;
    uint16_t rangeMin;
    uint16_t rangeMax;
};



#endif TECHTHETACHYON_LOWLEVEL_SICKOD2_N250W150I2_H