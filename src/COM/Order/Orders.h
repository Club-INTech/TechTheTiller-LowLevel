//
// Created by asphox on 29/04/18.
//

#ifndef TECHTHETOWN_LOWLEVEL_ORDERS_H
#define TECHTHETOWN_LOWLEVEL_ORDERS_H

#include "AbstractOrder.h"
#include "OrderManager.h"
#include "Utils/Utils.h"
#include "Utils/DelayingBuffer.hpp"
#include "Config/PinMapping.h"
#include "MotionControlSystem/RobotInfo/RobotStatus.h"
#include "COM/dxl.hpp"
//
#include "Servo.h"
#include "Stepper.h"
#include "I2CC.h"
#include <Arduino.h>
#include <Wire.h>

#include "external.hpp"
#include "dxl/2.0/packet.hpp"

/*******************************************************************************
  ORDER pour 2020-2021
*******************************************************************************/

// HL crew requests
ORDER(hammers, 1);

// Actionators
ORDER(set_hammer_angle, 2);
ORDER(raise_hammer, 1);
ORDER(lower_hammer, 1);
ORDER(raise_dxl, 0);
ORDER(lower_dxl, 0);
ORDER(toggle_valve, 2);
ORDER(suck, 2);

// Motion data acquisition
constexpr auto motion_datum_string_size = 50;

ORDER(start_mda, 1);
ORDER(send_md, 0);
ORDER(set_pid, 3);

/**
 * @Description: Répond pong, utile pour tester la com.
 */
ORDER(ping,0);
/**
 * @Description: Active l'attente de l'activation du jumper.
 */
ORDER(j,0);
/**
 * @Description: Retourne la position (x,y) et en angle.
 */
ORDER(xyo,0);
/**
 * @Description: Lance une translation.
 * @Arguments: Distance en mm, [excpectedWallImpact: booléen]
 */
ORDER(d,1);
/**
 * @Description: Lance une rotation.
 * @Arguments: Angle en radians ou 'pi'
 */
ORDER(t,1);
/**
 * @Description: Goto a position by first rotating then translating.
 * @Arguments: x cible en mm, y cible en mm, [séquentiel: booléen]
 */
ORDER(goto,2);
/**
 * @Description: Essaye d'arrêter le robot à la position actuelle.
 */
ORDER(stop,0);

/*			 __________________
* 		 *|                  |*
*		   *|   POS & VITESSE  |*
*		   *|__________________|*
*/

/**
 * @Description: Change la position x actuelle.
 * @Arguments: Position x en mm
 */
ORDER(cx,1);
/**
 * @Description: Change la position y actuelle.
 * @Arguments: Position y en mm
 */
ORDER(cy,1);
/**
 * @Description: Change l'orientation actuelle.
 * @Arguments: Angle en radians
 */
ORDER(co,1);
/**
 * @Description: Change la position (x,y) et l'angle.
 * @Arguments: x,y,angle (mm,mm,rad)
 */
ORDER(cxyo,3);
/**
 * @Description: Change la vitesse de translation.
 * @Arguments: Vitesse en mm
 */
ORDER(ctv,1);
/**
 * @Description: Change la vitesse de rotation.
 * @Arguments: Vitesse en rad/s
 */
ORDER(crv,1);
/**
 * @Description: Change les vitesses de translation et de rotation.
 * @Arguments: Vitesse en mm/s, vitesse en rad/s
 */
ORDER(ctrv,2);
/**
 * @Description: Active les mouvements forcés.
 */
ORDER(efm,0);
/**
 * @Description: Désactive les mouvements forcés.
 */
ORDER(dfm,0);

/*		 	 ___________________
 * 		 *|                   |*
 *		 *|  ASSERVISSEMENTS  |*
 *		 *|___________________|*
 */

/**
 * @Description: Désactive l'asservissement en translation.
 */
ORDER(ct0,0);
/**
 * @Description: Active l'asservissement en translation.
 */
ORDER(ct1,0);
/**
 * @Description: Désactive l'asservissement en rotation.
 */
ORDER(cr0,0);
/**
 * @Description: Active l'asservissement en rotation.
 */
ORDER(cr1,0);
/**
 * @Description: Désactive l'asservissement en vitesse (/!\ : plus de déplacement)
 */
ORDER(cv0,0);
/**
 * @Description: Active l'asservissement en vitesse.
 */
ORDER(cv1,0);

/*			 ___________________________
* 		 *|                           |*
*		   *|					                  |*
*		   *|			DEBUG			            |*
*		   *|						                |*
*		   *|___________________________|*
*/

/**
 * @Description: Affiche les ticks de codeuse.
 */
ORDER(cod,0);

/**
 * @Description: Retourne, dans l'ordre: x,y,angle, v_g, cible v_g, v_d, cible v_d.
 */
ORDER(rawposdata,0);

/*			 ___________________________
* 		 *|                           |*
*		   *|         MONTLHERY         |*
*		   *|   DEPLACEMENT ET ROTATION |*
*		   *|    SANS ASSERVISSEMENT    |*
*		   *|___________________________|*
*/

/**
 * @Description: Asservissement en vitesse seulement, mouvements forcés.
 */
ORDER(montlhery,0);
/**
 * @Description: Impose une consigne de vitesse en translation vers l'avant.
 */
ORDER(av,0);
/**
 * @Description: Impose une consigne de vitesse en translation vers l'arrière.
 */
ORDER(rc,0);
/**
 * @Description: Impose une consigne de vitesse afin de tourner en sens horaire.
 */
ORDER(td,0);
/**
 * @Description: Impose une consigne de vitesse afin de tourner en sens trigo.
 */
ORDER(tg,0);
/**
 * @Description: Impose une consigne de vitesse et un PWM nul.
 */
ORDER(sstop,0);
/**
 * @Description: Change la vitesse maximale en translation.
 * @Arguments: Vitesse de translation en mm/s
 */
ORDER(maxtr,1);
/**
 * @Description: Change la vitesse maximale en rotation.
 * @Arguments: Vitesse de rotaion en rad/s
 */
ORDER(maxro,1);
/**
 * @Description: Change les vitesses maximales en translation et rotation.
 * @Arguments: Vitesse de translation en mm/s, vitesse de rotation en rad/s
 */
ORDER(maxtrro,2);
ORDER(trstop,0);
ORDER(rostop,0);

/*			   _________________________________
 * 		   *|                                 |*
 *		   *|CONSTANTES D'ASSERV (pour le PID)|*
 *    	 *|_________________________________|*
*/

/*			   _________________________________
 * 		   *|                                 |*
 *		   *|			   HOOKS	                  |*
 *    	 *|_________________________________|*
*/

/**
 * @Description: Créé un nouvel hook et les conditions d'activation.
 * @Arguments: ID, x, y, rayon, angle, écart max à l'angle, ordre à exécuter
 */
ORDER(nh,7);
/**
 * @Description: Active un hook.
 * @Arguments: ID du hook à activer
 */
ORDER(eh,1);
/**
 * @Description: Désactive un hook.
 * @Arguments: ID du hook à désactiver.
 */
ORDER(dh,1);

/*			     _________________________________
 * 		     *|                                 |*
 *		     *|	       ACKNOWLEDGEMENT          |*
 *    	   *|_________________________________|*
*/

ORDER(ptpdemo,0);
ORDER(ptpdemoseq,0);

/*			   _________________________________
 * 		   *|                                 |*
 *		   *|	      NOUVEAUX ORDRES           |*
 *    	 *|_________________________________|*
 */

/* SICK */
/**
 * @Description: Renvoie les distances lues par les SICK (sens trigo).
 */
ORDER(lectureSICK, 0);
/**
 * @Description: Renvoie la valeur lue par un SICK.
 * @Arguments: Indice du SICK.
 */
ORDER(testSICK, 1);
/**
 * @Description: Règle la fenêtre de mesure d'un SICK.
 * @Arguments: Indice, valeur min, valeur max.
 */
ORDER(rangeSICK, 3);

/**
 * @Description: Met le bas niveau en attente du jumper.
 */
ORDER(waitJumper, 0);
/**
 * @Description: Arrête le robot en fin de match.
 */
ORDER(endMatch, 0);



#endif //TECHTHETOWN_LOWLEVEL_ORDERS_H
