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
#include "COM/dxl.hpp"
#include "Config/Defines.h"
#include <SimpleTimer.h>

SimpleTimer mcsTimer;
SimpleTimer samplingTimer;

auto getMotionDatum() {
  float leftSpeedGoal, rightSpeedGoal;
	auto& orderManager = OrderManager::Instance();
  orderManager.motionControlSystem.getSpeedGoals(leftSpeedGoal, rightSpeedGoal);

  float xPos = orderManager.motionControlSystem.getX();
  float yPos = orderManager.motionControlSystem.getY();
  float angle = orderManager.motionControlSystem.getAngle();
  float leftSpeed = orderManager.motionControlSystem.getLeftSpeed();
  float rightSpeed = orderManager.motionControlSystem.getRightSpeed();

  float xlw = orderManager.motionControlSystem.getXLeftWheel();
  float ylw = orderManager.motionControlSystem.getYLeftWheel();
  float xrw = orderManager.motionControlSystem.getXRightWheel();
  float yrw = orderManager.motionControlSystem.getYRightWheel();

  float d = orderManager.motionControlSystem.getCurrentDistance();

  char s[80];
  //snprintf(s,50,"%f,%f,%f,%f\n", xlw, ylw, xrw, yrw);
  snprintf(s,80,"%f,%f,%f,%f,%f,%f\n", d, angle, leftSpeed, leftSpeedGoal, rightSpeed, rightSpeedGoal);
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

  // I2C
	Wire.setSDA(D0);
	Wire.setSCL(D1);
	Wire.begin();

  // DXL
  hammer_dxl_stream.begin(57600);
  dxl::send_packet(hammer_dxl_stream, 0xfe, dxl::Instruction::sync_write, uint16_t{64}, uint16_t{1},
    uint8_t{hammer_dxl_ids[0]}, uint8_t{1},
    uint8_t{hammer_dxl_ids[1]}, uint8_t{1});

  arm_dxl_stream.begin(57600);
  dxl::send_packet(arm_dxl_stream, 0xfe, dxl::Instruction::sync_write, uint16_t{64}, uint16_t{1},
    uint8_t{arm_dxl_ids[0]}, uint8_t{1},
    uint8_t{arm_dxl_ids[1]}, uint8_t{1});

}

void loop() {
	//noInterrupts();
	auto& mcs = MCS::Instance();
	auto& orderManager = OrderManager::Instance();

	//mcs.leftMotor.run(20);
	//mcs.rightMotor.run(60);

	mcsTimer.setInterval(1000 / MCS_FREQ, [&](){
		mcs.control();
	});

	samplingTimer.setInterval(1000 / SAMPLING_FREQUENCY, [&](){
		if (dbuf::buffer.length() + motion_datum_string_size < dbuf::capacity && dbuf::init_buff ) dbuf::buffer.concat(getMotionDatum());
	});
	orderManager.execute("montlhery");
	//orderManager.execute("cr1");
	//orderManager.execute("ct1");
	
	orderManager.execute("start_mda 4096");

	while (true) {
		mcsTimer.run();
		samplingTimer.run();
		orderManager.communicate();
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
