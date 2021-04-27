#include <Arduino.h>
#include <unity.h>
#include "MotionControlSystem/MCS.h"


void test_null_output() {

    auto& mcs = MCS::Instance();
    mcs.robotStatus.speedLeftWheel = 40.0;
    mcs.leftSpeedPID.setGoal(40.0);
    TEST_ASSERT_EQUAL_FLOAT(0.0, mcs.leftSpeedPID.compute(mcs.robotStatus.speedLeftWheel));
}

void setup() {
    // NOTE!!! Wait for >2 secs
    // if board doesn't support software reset via Serial.DTR/RTS
    delay(2000);

	MCS::Instance().init();
	Serial.println("MCS OK");

    UNITY_BEGIN();    // IMPORTANT LINE!
    RUN_TEST(test_null_output);
	UNITY_END();

}
