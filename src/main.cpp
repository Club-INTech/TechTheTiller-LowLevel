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

auto getMotionDatum() {
  float leftSpeedGoal, rightSpeedGoal;
	auto& orderManager = OrderManager::Instance();
  orderManager.motionControlSystem.getSpeedGoals(leftSpeedGoal, rightSpeedGoal);

  int16_t xPos = orderManager.motionControlSystem.getX();
  int16_t yPos = orderManager.motionControlSystem.getY();
  float angle = orderManager.motionControlSystem.getAngle();
  float leftSpeed = orderManager.motionControlSystem.getLeftSpeed();
  float rightSpeed = orderManager.motionControlSystem.getRightSpeed();

  char s[50];
  snprintf(s,50,"%f,%f,%f,%f\n", leftSpeed, leftSpeedGoal,rightSpeed,rightSpeedGoal);
  return String(s);
}

void setup(){
	InitAllPins();

	ComMgr::Instance().init();
	Serial.println("Com OK");

	SensorMgr::Instance().init();
	Serial.println("Capteurs OK");

	MCS::Instance().init();
	Serial.println("MCS OK");

	OrderManager::Instance().init();
	Serial.println("Ordres OK");

	Serial.println("Init OK");

	Wire.setSDA(D0);
	Wire.setSCL(D1);
	Wire.begin();
}

void loop() {
	auto& mcs = MCS::Instance();
	auto& orderManager = OrderManager::Instance();
    mcs.controlledTranslation(false);
    mcs.setTranslationSpeed(50.0);

	while (true) {
		mcs.control();
		orderManager.communicate();
        orderManager.execute("rawposdata");
		//if (dbuf::buffer.length() + motion_datum_string_size < dbuf::capacity) dbuf::buffer.concat(getMotionDatum());
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
