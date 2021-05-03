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
#include "Config/Defines.h"



auto getMotionDatum() {
  float leftSpeedGoal, rightSpeedGoal;
	auto& orderManager = OrderManager::Instance();
  orderManager.motionControlSystem.getSpeedGoals(leftSpeedGoal, rightSpeedGoal);

  int16_t xPos = orderManager.motionControlSystem.getX();
  int16_t yPos = orderManager.motionControlSystem.getY();
  float angle = orderManager.motionControlSystem.getAngle();
  float leftSpeed = orderManager.motionControlSystem.getLeftSpeed();
  float rightSpeed = orderManager.motionControlSystem.getRightSpeed();
  long time = clock::time_left;

  char s[50];
  snprintf(s,50,"%f,%f,%f,%f\n", leftSpeed, leftSpeedGoal,rightSpeed,rightSpeedGoal);
  return String(s);
}

long time_now = 0;
long prev_time = 0;

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
	pinMode(LED_BUILTIN, OUTPUT);


	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_A), clock::inc_left_ticks, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_B), clock::dec_left_ticks, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_A), clock::inc_right_ticks, RISING);
	attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_B), clock::dec_right_ticks, RISING);

	time_now = millis();
	prev_time = millis();
}


void loop() {
	auto& mcs = MCS::Instance();
	auto& orderManager = OrderManager::Instance();
	orderManager.execute("montlhery");
	delay(2000);
	orderManager.execute("av");
	delay(10);
	orderManager.execute("start_mda 4096");


	while (true) {
		if(time_now - prev_time >= 5) {
			mcs.control();
			prev_time = millis();
			if (dbuf::buffer.length() + motion_datum_string_size < dbuf::capacity && dbuf::init_buff ) dbuf::buffer.concat(getMotionDatum()); //2105 offset mais c'est bizzare
		}
		orderManager.communicate();
		time_now = millis();
        
		//orderManager.execute("rawposdata");
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
