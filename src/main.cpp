/**
*Initialisation et boucle principale du programme
*
* @author caillou, sylvain, rémi, melanie, Ug
*
**/
#include "Utils/Monitoring.h"
#include "Config/PinMapping.h"
#include "COM/InterruptStackPrint.h"
#include "COM/Order/OrderManager.h"

//#include "MotionControlSystem/HardwareEncoder_ISRDEF.h"

/* Interruptions d'asservissements */
void motionControlInterrupt(HardwareTimer* hardwareTimer) {
	static MCS &motionControlSystem = MCS::Instance();
    motionControlSystem.control();
	motionControlSystem.manageStop();
}

void positionInterrupt() {
	static MCS &motionControlSystem = MCS::Instance();
	motionControlSystem.sendPositionUpdate();
}

void setup(){}

void __attribute__((noreturn)) loop() {
	/*************************
	 * Initialisation du LL, gère:
	 * Les pins
	 * La série
	 * Les actionneurs
	 * L'asservissement
	 *************************/

    InitAllPins();

    // TODO : Automate init
    ComMgr::Instance().init();
    Serial.println("Com OK");
    ActuatorsMgr::Instance().init();
    Serial.println("Actuateurs OK");
    SensorMgr::Instance().init();
    Serial.println("Capteurs OK");
    MCS::Instance().init();
    Serial.println("MCS OK");
    OrderManager& orderMgr = OrderManager::Instance();
    orderMgr.init();
    Serial.println("Ordres OK");

	Serial.println("Init OK");
	delay(250);

	/* Actuators */
	// Par sécurité on met tout les actuators à LOW quand on les initialise
	/* Pompe */
	pinMode(RIGHT_PUMP_PIN,OUTPUT);
	digitalWrite(RIGHT_PUMP_PIN,LOW);

	/* Electrovanne */
	pinMode(LEFT_VALVE_PIN,OUTPUT);
	digitalWrite(LEFT_VALVE_PIN,LOW);
	pinMode(RIGHT_VALVE_PIN,OUTPUT);
	digitalWrite(RIGHT_VALVE_PIN,LOW);

    /* InterruotStackPrint */
    InterruptStackPrint& interruptStackPrint = InterruptStackPrint::Instance();

    // MotionControlSystem interrupt on timer
    // FIXME: Pour le débug
//    HardwareTimer motionControlInterruptTimer(TIM6);
//    motionControlInterruptTimer.setMode(1, TIMER_OUTPUT_COMPARE);
//    motionControlInterruptTimer.setOverflow(MCS_FREQ,HERTZ_FORMAT);
//    motionControlInterruptTimer.attachInterrupt(motionControlInterrupt);
    // FIXME: Wait for upstream update
    //    motionControlInterruptTimer.setInterruptPriority(0,0);
//    motionControlInterruptTimer.resume();


    // Timer pour steppers
    // FIXME: Pour le débug
//    HardwareTimer stepperTimer(TIM2);   // Check needed timers
//    stepperTimer.setMode(1,TIMER_OUTPUT_COMPARE);
//    stepperTimer.setOverflow(STEPPER_FREQUENCY, HERTZ_FORMAT);
//    stepperTimer.attachInterrupt(stepperInterrupt);
    // FIXME: Wait for upstream update
    //    stepperTimer.setInterruptPriority(10,0);
//    stepperTimer.resume();

    Serial.println("Interrupt Timers OK");
    // FIXME: Pour le débug
    ActuatorsMgr::Instance().initTorques();
    Serial.println("Dynamixels OK");
    Serial.println("Setup DONE");

	Serial.println("Starting...");

	/**
	 * Boucle principale, y est géré:
	 * La communication HL
	 * L'execution des ordres de ce dernier
	 * Les capteurs
	 */

	Wire.setSCL(D1);
	Wire.setSDA(D0);
	Wire.begin();

//    I2CC::executeRPC(1, 4, nullptr);

//    Wire.beginTransmission(2);
//    Wire.write(4);
//    orderMgr.execute("DiodeOn 2");
//    Wire.endTransmission();


    while (true) {
        interruptStackPrint.print();
        orderMgr.communicate();

    }

}

                   /*``.           `-:--.`
                  `.-::::/.        .:::::::y
                 `::::::::o``````.-::::::/h-
                 `::::::::::::::::::::::/h.
                  -::::::://++++//::::::::.
               `-::::/oso+/:-....--://:::::-.``.---:`
      ````   .-:::/so/-      ``````  `.:::::::::::::+ ``````````````````````````````````````````````````````````````````````
    `-:::::--:::os+.       `/sssssss`   ./syyyyyys+/y+sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssy- `-:::://
   `-:::::::::+y/`        .shhhhhdd:   `/hhhhhhhhhdo:/dddddddddddddddddhhhhhhhdddddddddddddddddddddddddddddddddddddddddddddo`.:////sy.
   ::::::::::so`         :yhhhhhmy`   .ohhhhhhhhhhd  `//oooooo+//////+hhhhhhdd+///////+++++++/////////////////+++++++/////-`-////+h+` `
   `://:::::y/         `+hhhhhdm+    -yhhhhhdhhhhhm`  `oyyyyyhs     .shhhhhdh-  ``.-:::////:::-.       ``.-::::////::://  `://///o+:::::::-.
      `::::y/         .shhhhhdh-   `+hhhhhdd+shhhhd- -yhhhhhmo`    /yhhhhhms` .-://+ssso+++/////o   `.:://+ossoo+++o+oy- -/////+ssoo+//////h
      .:::/y         :yhhhhhms`   .shhhhhdy::hhhhhho+hhhhhdd:    `ohhhhhdd/ .:///oyo::-----////oy `-:///oyo:.`      `-``:////oy+-` `:////+h:
      -:::o:       `+hhhhhdm/    :yhhhhddo:::+hhhhhhhhhhhms.    -yhhhhhdh. -////yo.`ooooooooooss``:////yo.            -/////yo`   .:////ys.
   ``.::::+-      .shhhhhdh-   `+hhhhhdd/::::/hhhhhhhhhdd/     /hhhhhhmo` `/////s```..----:+:..  -/////o``````..:.  `:////oh:   `-////+h/
`.-::::::::+     :yhhhhhms`   .shhhhhmy:::::::hhhhhhhhdh.    `ohhhhhdd:    :++////:::///+oys`    `/++/////://++yo` .:////ys`   .:////ys.
-::::::::::/-    /oooooo/     -sssyyyo:+o/++//hyssssss+`     .sssssss.      `-/++oooo++//:.        .:/+oooo++/:-   /ooooo:     :ooooo/
.:::::/o:::::-`               `.--:::os+```.---
 :+ooo/.`::::::-..`````````.--::::+ss:`
  ``     `-/::::::::::::::::::::::s.
          `:::::::::::://+::::::::o-
         `:::::::/h////::.-:::::::y-
         :::::::ss`        -:/+sso:
         .:/++sy:          `//*/


/*
 *   Dead Pingu in the Main !
 *      	  . --- .
		    /        \
		   |  X  _  X |
		   |  ./   \. |
		   /  `-._.-'  \
		.' /         \ `.
	.-~.-~/    o   o  \~-.~-.
.-~ ~    |    o  o     |    ~ ~-.
`- .     |      o  o   |     . -'
	 ~ - |      o      | - ~
	 	 \             /
	 	__\           /___
	 	~;_  >- . . -<  _i~
	 	  `'         `'
*/
