# TechTheTide-LowLevel
## SETUP
> NB: Plusieurs aspects de configuration du projet sont pensés pour utiliser platformio avec CLion et ne sont ni testés ni prévus pour d'autres IDE.

Pour configurer le projet, deux possibilités:
1. Utiliser le script `setup.sh`. Se charge d'installer platformio avec apt ou pacman et d'initialiser le projet.
2. À la main, en suivant les étapes suivantes
   1. Installer Platformio ***version > 4.1*** ([Guide d'installation](https://club-intech.minet.net/images/9/97/Guide_PIO.pdf))
   2. **Initialisez le sous-module DynamixelCom avec la commande**: `git submodule update --init`
   3. Ouvrez un terminal dans le dossier du dépot, et effectuez la commande \
     ```pio init --ide IDE --board teensy35```\
     avec IDE=clion si vous utilisez CLion, ou vscode si vous utilisez Visual Studio Code


Une fois que le projet est configuré, vous pouvez l'ouvrir avec l'IDE de votre choix.

## UTILISATION

Afin de fusionner les branches des deux robots et d'éviter les divergences, un système à base de define et de `#if defined` \
a été mit en place.

 Pour choisir un robot en particulier:
 1. Choisir le nom du robot dans la liste au dessus des cibles (`PLATFORMIO_BUILD` etc)
 2. Vous devriez avoir `PLATFORMIO_** | main` ou `PLATFORMIO_** | slave` 
 3. Exécuter la cible désirée (`PLATFORMIO_BUILD` pour build, `PLATFORMIO_UPLOAD` pour upload...)
 
 La configuration `All` permet d'exécuter la commande pour les deux robots
 
## TODO

- [x] Asservissement de test
- [ ] Test des ordres
- [ ] Asservissement sur robot définitif
- [ ] Refaire la détection de blocage (+recalage mécanique)
- [ ] Refaire le goto
- [ ] Suivi de trajectoire (De jolies courbes !)
- [ ] Gagner la Coupe

### Choses en plus

- [ ] Bien documenter et commenter tout le code
- [ ] Nettoyer les warnings
- [ ] Refaire au propre le watchdog des XL
- [ ] Ordre propre pour la récupération de données d'asservissement + scripts en conséquence
- [ ] Mise en place de cas d'erreur explicites et plus nombreux pour le HL
- [ ] Meilleure façon de séparer les robots dans le code

## TABLE DES ORDRES

> Text starting from here is generated automatically. **ANY MODIFICATION WILL BE OVERWRITTEN**.

|      Ordre       |                            Description                             |                           Arguments                           |
|:----------------:|:------------------------------------------------------------------:|:-------------------------------------------------------------:|
|       ping       |               Répond pong, utile pour tester la com.               |                                                               |
|        j         |            Active l'attente de l'activation du jumper.             |                                                               |
|        f         |                                                                    |                                                               |
|       xyo        |              Retourne la position (x,y) et en angle.               |                                                               |
|        d         |                       Lance une translation.                       |        Distance en mm, [excpectedWallImpact: booléen]         |
|        t         |                        Lance une rotation.                         |                   Angle en radians ou 'pi'                    |
|       goto       |        Goto a position by first rotating then translating.         |      x cible en mm, y cible en mm, [séquentiel: booléen]      |
| followTrajectory |                                                                    |                                                               |
|       stop       |         Essaye d'arrêter le robot à la position actuelle.          |                                                               |
|        cx        |                   Change la position x actuelle.                   |                       Position x en mm                        |
|        cy        |                   Change la position y actuelle.                   |                       Position y en mm                        |
|        co        |                   Change l'orientation actuelle.                   |                       Angle en radians                        |
|       cxyo       |                Change la position (x,y) et l'angle.                |                     x,y,angle (mm,mm,rad)                     |
|       ctv        |                 Change la vitesse de translation.                  |                         Vitesse en mm                         |
|       crv        |                   Change la vitesse de rotation.                   |                       Vitesse en rad/s                        |
|       ctrv       |         Change les vitesses de translation et de rotation.         |               Vitesse en mm/s, vitesse en rad/s               |
|       efm        |                   Active les mouvements forcés.                    |                                                               |
|       dfm        |                  Désactive les mouvements forcés.                  |                                                               |
|       ct0        |             Désactive l'asservissement en translation.             |                                                               |
|       ct1        |              Active l'asservissement en translation.               |                                                               |
|       cr0        |              Désactive l'asservissement en rotation.               |                                                               |
|       cr1        |                Active l'asservissement en rotation.                |                                                               |
|       cv0        | Désactive l'asservissement en vitesse (/!\ : plus de déplacement)  |                                                               |
|       cv1        |                Active l'asservissement en vitesse.                 |                                                               |
|       cod        |                   Affiche les ticks de codeuse.                    |                                                               |
|     pfdebug      |                                                                    |                                                               |
|      rawpwm      |                                                                    |                                                               |
|      getpwm      |                                                                    |                                                               |
|      errors      |                                                                    |                                                               |
|     rawspeed     |                                                                    |                                                               |
|    rawposdata    | Retourne, dans l'ordre: x,y,angle, v_g, cible v_g, v_d, cible v_d. |                                                               |
|     reseteth     |                 Force un reset du module ethernet.                 |                                                               |
|  disableTorque   |              Désactive le couple du bras sélectionné.              |                 Côté du bras ("right"/"left")                 |
|   enableTorque   |               Active le couple du bras sélectionné.                |                 Côté du bras ("right"/"left")                 |
|    montlhery     |      Asservissement en vitesse seulement, mouvements forcés.       |                                                               |
|        av        |    Impose une consigne de vitesse en translation vers l'avant.     |                                                               |
|        rc        |   Impose une consigne de vitesse en translation vers l'arrière.    |                                                               |
|        td        |  Impose une consigne de vitesse afin de tourner en sens horaire.   |                                                               |
|        tg        |   Impose une consigne de vitesse afin de tourner en sens trigo.    |                                                               |
|      sstop       |           Impose une consigne de vitesse et un PWM nul.            |                                                               |
|      maxtr       |             Change la vitesse maximale en translation.             |                Vitesse de translation en mm/s                 |
|      maxro       |              Change la vitesse maximale en rotation.               |                  Vitesse de rotaion en rad/s                  |
|     maxtrro      |     Change les vitesses maximales en translation et rotation.      | Vitesse de translation en mm/s, vitesse de rotation en rad/s  |
|      trstop      |                                                                    |                                                               |
|      rostop      |                                                                    |                                                               |
|      toggle      |                                                                    |                                                               |
|  displayAsserv   |                                                                    |                                                               |
|       kpt        |                                                                    |                                                               |
|       kdt        |                                                                    |                                                               |
|       kit        |                                                                    |                                                               |
|       kpr        |                                                                    |                                                               |
|       kir        |                                                                    |                                                               |
|       kdr        |                                                                    |                                                               |
|       kpg        |                                                                    |                                                               |
|       kig        |                                                                    |                                                               |
|       kdg        |                                                                    |                                                               |
|       kpd        |                                                                    |                                                               |
|       kid        |                                                                    |                                                               |
|       kdd        |                                                                    |                                                               |
|        nh        |        Créé un nouvel hook et les conditions d'activation.         | ID, x, y, rayon, angle, écart max à l'angle, ordre à exécuter |
|        eh        |                          Active un hook.                           |                     ID du hook à activer                      |
|        dh        |                         Désactive un hook.                         |                   ID du hook à désactiver.                    |
|       demo       |                                                                    |                                                               |
|     ptpdemo      |                                                                    |                                                               |
|    ptpdemoseq    |                                                                    |                                                               |
|       XLm        |                 Bouge un XL à une position donnée.                 |                    ID du XL, position en °                    |
|       XLs        |                     Change la vitesse d'un XL.                     |           ID du XL, vitesse en unités de vitesse XL           |
|     posBras      |               Renvoie la position des XL d'un bras.                |                 Côté du bras ("right"/"left")                 |
|     BrasOut      |                                                                    |                                                               |
|      BrasIn      |                                                                    |                                                               |
|    torqueBras    |          Renvoie le couple mesuré par les XLs d'un bras.           |                 Côté du bras ("right"/"left")                 |
|     torqueXL     |                     Renvoie le couple d'un XL.                     |                           ID du XL                            |
|      Valve       |                                                                    |                                                               |
|       Suck       |                   Ouvre ou ferme l'électrovanne.                   |                  ID de la vanne / on ou off                   |
|      FlagUp      |                                                                    |                                                               |
|     FlagDown     |                          Lève de drapeau                           |                         ID du drapeau                         |
|      LiftUp      |                                                                    |                                                               |
|     LiftDown     |                         Monte l'ascenseur                          |                                                               |
|       Gate       |                         Baisse l'ascenseur                         |                                                               |
|    BrasStock     |                                                                    |                                                               |
|    BrasEcueil    |        Bouge un bras du robot secondaire en position haute         |                          ID du bras                           |
|    BrasDepot     |      Bouge un bras du robot secondaire au niveau des écueils       |                          ID du bras                           |
|       grnd       |                                                                    |                                                               |
|       oust       |               Pousse palet en dehors des ascenceurs                |                                                               |
|   lectureSICK    |       Renvoie les distances lues par les SICK (sens trigo).        |                                                               |
|     testSICK     |                 Renvoie la valeur lue par un SICK.                 |                        Indice du SICK.                        |
|    rangeSICK     |               Règle la fenêtre de mesure d'un SICK.                |                Indice, valeur min, valeur max.                |
|    waitJumper    |              Met le bas niveau en attente du jumper.               |                                                               |
|     endMatch     |                  Arrête le robot en fin de match.                  |                                                               |
