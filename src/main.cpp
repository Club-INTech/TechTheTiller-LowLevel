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
#include <SimpleTimer.h>


SimpleTimer mcsTimer;
SimpleTimer samplingTimer;


auto getMotionDatum() {
  float leftSpeedGoal, rightSpeedGoal;
	auto& orderManager = OrderManager::Instance();
  orderManager.motionControlSystem.getSpeedGoals(leftSpeedGoal, rightSpeedGoal);

  int16_t xPos = orderManager.motionControlSystem.getX();
  int16_t yPos = orderManager.motionControlSystem.getY();
  float angle = orderManager.motionControlSystem.getAngle();
  float leftSpeed = orderManager.motionControlSystem.getLeftSpeed();
  float rightSpeed = orderManager.motionControlSystem.getRightSpeed();

  float xlw = orderManager.motionControlSystem.getXLeftWheel();
  float ylw = orderManager.motionControlSystem.getYLeftWheel();
  float xrw = orderManager.motionControlSystem.getXRightWheel();
  float yrw = orderManager.motionControlSystem.getYRightWheel();

  char s[50];
  //snprintf(s,50,"%f,%f,%f,%f\n", xlw, ylw, xrw, yrw);
  snprintf(s,50,"%f,%f,%f,%f\n", leftSpeed, leftSpeedGoal, rightSpeed, rightSpeedGoal);
  return String(s);
}


void setup(){
	//noInterrupts();
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
	//interrupts();
}


void loop() {
	//noInterrupts();
	auto& mcs = MCS::Instance();
	auto& orderManager = OrderManager::Instance();

	//mcs.leftMotor.run(40);
	//mcs.rightMotor.run(40);

	mcsTimer.setInterval(1000 / MCS_FREQ, [&](){
		mcs.control();
	});

	samplingTimer.setInterval(1000 / SAMPLING_FREQUENCY, [&](){
		if (dbuf::buffer.length() + motion_datum_string_size < dbuf::capacity && dbuf::init_buff ) dbuf::buffer.concat(getMotionDatum());
	});
	//orderManager.execute("montlhery");

	// // orderManager.execute("ct0");
	// // orderManager.execute("cr1");
	//delay(2000);
	//orderManager.execute("av");
	//orderManager.execute("td");
	
	// // mcs.rotate(PI);
	//orderManager.execute("start_mda 4096");
	
	// //interrupts();
	//delay(4000);
	while (true) {
		mcsTimer.run();
		samplingTimer.run();
		orderManager.communicate();
        
	}

	// int stateR = static_cast<int>(((GPIOA->IDR & 0x0020) >> 5) | ((GPIOB->IDR & 0x0001) << 1));
	// int stateL = static_cast<int>(((GPIOA->IDR & 0x0040) >> 5) | ((GPIOA->IDR & 0x1000) >> 12));
	// Serial.printf("%i, %i\n", stateL, stateR);
	// delay(100);
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
