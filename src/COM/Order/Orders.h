//
// Created by asphox on 29/04/18.
//

#ifndef TECHTHETOWN_LOWLEVEL_ORDERS_H
#define TECHTHETOWN_LOWLEVEL_ORDERS_H

#include "AbstractOrder.h"
#include "OrderManager.h"
#include "Utils/Utils.h"
#include "Config/PinMapping.h"
#include "MotionControlSystem/RobotStatus.h"
#include "Actuators/ActuatorValues.h"
#include "Actuators/ActuatorsMgr.h"
#include "Servo.h"
#include "Stepper.h"
#include "I2CC.h"
#include <Arduino.h>
#include <Wire.h>



/**
 * @Description: Répond pong, utile pour tester la com.
 */
ORDER(ping,0);
/**
 * @Description: Active l'attente de l'activation du jumper.
 */
ORDER(j,0);
ORDER(f,0);
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
ORDER(followTrajectory,1);
/**
 * @Description: Essaye d'arrêter le robot à la position actuelle.
 */
ORDER(stop,0);

/*			 __________________
* 		   *|                  |*
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

/*			 ___________________
* 		   *|                   |*
*		   *|  ASSERVISSEMENTS  |*
*		   *|___________________|*
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
* 		   *|                           |*
*		   *|					        |*
*		   *|			DEBUG			|*
*		   *|						    |*
*		   *|___________________________|*
*/

/**
 * @Description: Affiche les ticks de codeuse.
 */
ORDER(cod,0);
ORDER(pfdebug,0);
ORDER(rawpwm,1);
ORDER(getpwm,0);
ORDER(errors,0);
ORDER(rawspeed,0);
/**
 * @Description: Retourne, dans l'ordre: x,y,angle, v_g, cible v_g, v_d, cible v_d.
 */
ORDER(rawposdata,0);
/**
 * @Description: Force un reset du module ethernet.
 */
ORDER(reseteth,0);

/**
 * @Description: Désactive le couple du bras sélectionné.
 * @Arguments: Côté du bras ("right"/"left")
 */
ORDER(disableTorque, 1);
/**
 * @Description: Active le couple du bras sélectionné.
 * @Arguments: Côté du bras ("right"/"left")
 */
ORDER(enableTorque, 1);

/*			 ___________________________
* 		   *|                           |*
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

/*			 _________________________________
* 		   *|                                 |*
*		   *|CONSTANTES D'ASSERV (pour le PID)|*
*    	   *|_________________________________|*
*/

ORDER(toggle,0);
ORDER(displayAsserv,0);
ORDER(kpt,1);
ORDER(kdt,1);
ORDER(kit,1);
ORDER(kpr,1);
ORDER(kir,1);
ORDER(kdr,1);
ORDER(kpg,1);
ORDER(kig,1);
ORDER(kdg,1);
ORDER(kpd,1);
ORDER(kid,1);
ORDER(kdd,1);

/*			 _________________________________
* 		   *|                                 |*
*		   *|			   HOOKS	          |*
*    	   *|_________________________________|*
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

/*			 _________________________________
* 		   *|                                 |*
 *		   *|	       ACKNOWLEDGEMENT        |*
 *    	   *|_________________________________|*
*/

ORDER(demo,0);
ORDER(ptpdemo,0);
ORDER(ptpdemoseq,0);

/*			 _________________________________
* 		   *|                                 |*
 *		   *|	      NOUVEAUX ORDRES         |*
 *    	   *|_________________________________|*
*/


/* Bras */
/**
 * @Description: Bouge un XL à une position donnée.
 * @Arguments: ID du XL, position en °
 */
ORDER(XLm,2);
/**
 * @Description: Change la vitesse d'un XL.
 * @Arguments: ID du XL, vitesse en unités de vitesse XL
 */
ORDER(XLs,2);
/**
 * @Description: Renvoie la position des XL d'un bras.
 * @Arguments: Côté du bras ("right"/"left")
 */
ORDER(posBras,1);

/**
 * @Description: Déploie le bras pour manches à air/phare.
 * @Arguments: Côté du bras ("right"/"left")
 */
ORDER(BrasOut,1);

/**
 * @Description: Rentre le bras pour manches à air/phare.
 * @Arguments: Côté du bras ("right"/"left")
 */
ORDER(BrasIn,1);


/**
 * @Description: Renvoie le couple mesuré par les XLs d'un bras.
 * @Arguments: Côté du bras ("right"/"left")
 */
ORDER(torqueBras,1);
/**
 * @Description: Renvoie le couple d'un XL.
 * @Arguments: ID du XL
 */
ORDER(torqueXL,1);

// This is horrible and should probably burn

/** Pour les pompes et électrovannes */

/**
 * @Description: Ouvre ou ferme l'électrovanne.
 * @Arguments: ID de la vanne / on ou off
 */
ORDER(Valve, 2);

/**
 * @Description: Allume ou éteint la pompe.
 * @Arguments: ID de la pompe / on ou off
 */
ORDER(Suck,2);

/** Pour le drapeau des deux robots */

/**
* @Description: Lève de drapeau
* @Arguments: ID du drapeau
*/
ORDER(FlagUp,0);

/**
 * @Description: Baisse le drapeau
 * @Arguments: ID du drapeau
 */
ORDER(FlagDown,0);


/**
 * @Description: Allume une diode via le protocole I2C
 */
ORDER(DiodeOn,1);

/**
 * @Description: Eteint une diode via le protocole I2C
 */
ORDER(DiodeOff,1);

#if defined(MAIN)

/**
 * @Description: Monte l'ascenseur
 */
ORDER(LiftUp, 0);

/**
 * @Description: Baisse l'ascenseur
 */
ORDER(LiftDown, 0);

/**
 * @Description: Bouge les portes du robot principal
 * @Arguments: angle voulu
 */
ORDER(Gate,1);




#elif defined(SLAVE)
/**
 * @Description: Bouge un bras du robot secondaire en position haute
 * @Arguments: ID du bras
 */
ORDER(BrasStock, 1);

/**
 * @Description: Bouge un bras du robot secondaire au niveau des écueils
 * @Arguments: ID du bras
 */
ORDER(BrasEcueil, 1);

/**
 * @Description: Bouge un bras du robot secondaire au niveau de la table
 * @Arguments: ID du bras
 */
ORDER(BrasDepot, 1);


ORDER(grnd,1);

/*
 * @Description: Pousse palet en dehors des ascenceurs
 */
ORDER(oust,0);

#endif



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

// Permet de bouger un bras sans se soucier du type
#if defined(MAIN)
#define MOVE_ARM(side, actions)             \
if(!strcmp(args[0], "right")) {             \
    Arm<XL430>* arm = manager.rightArm;      \
    actions;                                \
} else {                                    \
    Arm<XL430>* arm = manager.leftArm;      \
    actions;                                \
}
#elif defined(SLAVE)
#define MOVE_ARM(side, actions)             \
if(!strcmp(args[0], "right")) {             \
    Arm<XL430>* arm = manager.rightArm;      \
    actions;                                \
}
#endif


#endif //TECHTHETOWN_LOWLEVEL_ORDERS_H
