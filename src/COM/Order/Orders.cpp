//
// Created by asphox on 29/04/18.
//

#include "Orders.h"


// TODO : Nettoyer


using namespace I2CC;

void ORDER_hammer::impl(Args args) {
  using namespace external;

  for (int i = 0; i < 5; i++) {
    BufferedData data(sizeof(int));
    putData(i, &data);

    if (OrderManager::parseInt(args[i]) == 0)
      executeRPC(hammers_id, lower_hammer_id, &data);
    else
      executeRPC(hammers_id, raise_hammer_id, &data);
  }
}

void ORDER_hammers::impl(Args args) {
  using namespace external;

  for (int i = 0; i < 5; i++) {
    BufferedData data(sizeof(int));
    putData(i, &data);

    if (OrderManager::parseInt(args[0]))
      executeRPC(hammers_id, raise_hammer_id, &data);
    else
      executeRPC(hammers_id, lower_hammer_id, &data);
  }
}

void ORDER_suckall::impl(Args args) {
  using namespace external;

  for (int i = 0; i < 5; i++) {
    BufferedData data(2 * sizeof(int));

    putData(i, &data);
    putData(OrderManager::parseInt(args[0]), &data);
    executeRPC(pumps_id, suck_id, &data);
  }
}

void ORDER_set_hammer_angle::impl(Args args) {
  using namespace external;
  static BufferedData data(sizeof(int) + sizeof(float));

  putData(OrderManager::parseInt(args[0]), &data);
  putData(OrderManager::parseFloat(args[1]), &data);
  executeRPC(hammers_id, set_hammer_angle_id, &data);
}

void ORDER_raise_hammer::impl(Args args) {
  using namespace external;
  static BufferedData data(sizeof(int));

  putData(OrderManager::parseInt(args[0]), &data);
  executeRPC(hammers_id, raise_hammer_id, &data);
}

void ORDER_lower_hammer::impl(Args args) {
  using namespace external;
  static BufferedData data(sizeof(int));

  putData(OrderManager::parseInt(args[0]), &data);
  executeRPC(hammers_id, lower_hammer_id, &data);
}

void ORDER_raise_dxl::impl(Args args) {
  using namespace dxl;

  delayMicroseconds(1750);
  dxl::send_packet(hammer_dxl_stream, 0xfe, dxl::Instruction::sync_write, uint16_t{116}, uint16_t{4},
    uint8_t{hammer_dxl_ids[0]}, uint32_t{hammer_dxl_home_positions[0] + hammer_dxl_raised_offsets[0]},
    uint8_t{hammer_dxl_ids[1]}, uint32_t{hammer_dxl_home_positions[1] + hammer_dxl_raised_offsets[1]});
}

void ORDER_lower_dxl::impl(Args args) {
  using namespace dxl;

  delayMicroseconds(1750);
  dxl::send_packet(hammer_dxl_stream, 0xfe, dxl::Instruction::sync_write, uint16_t{116}, uint16_t{4},
    uint8_t{hammer_dxl_ids[0]}, uint32_t{hammer_dxl_home_positions[0] + hammer_dxl_lowered_offsets[0]},
    uint8_t{hammer_dxl_ids[1]}, uint32_t{hammer_dxl_home_positions[1] + hammer_dxl_lowered_offsets[1]});
}

void ORDER_arm::impl(Args args) {
  using namespace dxl;

  Serial.println(OrderManager::parseInt(args[0]));
  Serial.println(OrderManager::parseInt(args[1]));

  uint8_t dxl_index = OrderManager::parseInt(args[0]);
  dxl::send_packet(arm_dxl_stream, arm_dxl_ids[dxl_index], dxl::Instruction::write,
    uint16_t{116},
    OrderManager::parseInt(args[1]) == 0 ? arm_dxl_lowered_positions[dxl_index] : arm_dxl_raised_positions[dxl_index]);
}

void ORDER_toggle_valve::impl(Args args) {
  using namespace external;
  static BufferedData data(2 * sizeof(int));

  putData(OrderManager::parseInt(args[0]), &data);
  putData(OrderManager::parseInt(args[1]), &data);
  executeRPC(pumps_id, toggle_valve_id, &data);
}

void ORDER_suck::impl(Args args) {
  using namespace external;
  static BufferedData data(2 * sizeof(int));

  putData(OrderManager::parseInt(args[0]), &data);
  putData(OrderManager::parseInt(args[1]), &data);
  executeRPC(pumps_id, suck_id, &data);
}

// TODO : C'est déprécié, mais je vois pas d'autre solution.
// Si vous voulez réimplementer ça, effacez les deux ORDERs et 'motion_datum_string_size' ici et dans le .cpp et
// supprimez 'DelayingBuffer.hpp'
void ORDER_start_mda::impl(Args args) {
  using namespace dbuf;
  buffer = "";
  capacity = motion_datum_string_size * OrderManager::parseInt(args[0]);
  buffer.reserve(capacity);
  init_buff = true;

}

void ORDER_send_md::impl(Args args) {
  Serial.print(dbuf::buffer);
}

void ORDER_set_pid::impl(Args args) {
  auto left_kp = OrderManager::parseFloat(args[0]);
  auto left_ki = OrderManager::parseFloat(args[1]);
  auto left_kd = OrderManager::parseFloat(args[2]);
  auto right_kp = OrderManager::parseFloat(args[3]);
  auto right_ki = OrderManager::parseFloat(args[4]);
  auto right_kd = OrderManager::parseFloat(args[5]);
  auto& mcs = MCS::Instance();
  mcs.setLeftTunings(left_kp, left_ki, left_kd);
  mcs.setRightTunings(right_kp, right_ki, right_kd);
}

void ORDER_ping::impl(Args args)
{
    orderManager.highLevel.printfln(EVENT_HEADER,"pong");
}

void ORDER_j::impl(Args args)
{
    orderManager.HLWaiting = true;
}

void ORDER_xyo::impl(Args args)
{
    orderManager.highLevel.printfln(STD_HEADER,"%i",orderManager.motionControlSystem.getX());
    orderManager.highLevel.printfln(STD_HEADER,"%i",orderManager.motionControlSystem.getY());
    orderManager.highLevel.printfln(STD_HEADER,"%f",orderManager.motionControlSystem.getAngle());
}

void ORDER_d::impl(Args args)
{
    int16_t deplacement = OrderManager::parseInt(args[0]);
    bool expectedWallImpact = false;
    if(args.nbrParams() == 2) {
        expectedWallImpact = ! strcmp(args[1], "true");
    }
//    orderManager.highLevel.printfln(DEBUG_HEADER,"distance : %d %i",deplacement, expectedWallImpact);
    orderManager.motionControlSystem.disableP2P();
    if(expectedWallImpact) {
        orderManager.motionControlSystem.expectWallImpact();
    }
    orderManager.execute("ct1");
    orderManager.execute("sstop");
    orderManager.execute("stop");
    orderManager.execute("tw");
    orderManager.motionControlSystem.translate(deplacement);
}

void ORDER_t::impl(Args args)
{
    float angle;
    if (!strcmp(args[0], "pi")) {
        angle = (float) PI;
    } else {
        angle = OrderManager::parseFloat(args[0]);
    }
    orderManager.highLevel.printfln(DEBUG_HEADER,"angle : %f", angle);

    orderManager.motionControlSystem.disableP2P();
    orderManager.execute("ct0");
    orderManager.execute("sstop");
    orderManager.execute("stop");
    orderManager.execute("tw");
    orderManager.motionControlSystem.rotate(angle);
}

void ORDER_tw::impl(Args args) {
    orderManager.motionControlSystem.robotStatus.termination = false;
}

void ORDER_goto::impl(Args args)
{
    float targetX = OrderManager::parseFloat(args[0]);
    float targetY = OrderManager::parseFloat(args[1]);
    bool isSequential = false;

    if(args.nbrParams() == 3)
    {
        isSequential = !strcmp(args[2],"true") || !strcmp(args[2],"1");
        Serial.print("On séquentialise : ");
        Serial.println(isSequential);
    }
    orderManager.motionControlSystem.gotoPoint2(targetX,targetY);
    orderManager.highLevel.printfln(DEBUG_HEADER, "goto %f %f %i", targetX, targetY, isSequential);
}

void ORDER_stop::impl(Args args)
{
    orderManager.motionControlSystem.stop();
    orderManager.highLevel.printfln(DEBUG_HEADER,"A priori, je m'arrête");
}

void ORDER_cx::impl(Args args)
{
    orderManager.motionControlSystem.setX(OrderManager::parseFloat(args[0]));
}

void ORDER_cy::impl(Args args)
{
    orderManager.motionControlSystem.setY(OrderManager::parseFloat(args[0]));
}

void ORDER_co::impl(Args args)
{
    orderManager.motionControlSystem.setAngle(OrderManager::parseFloat(args[0]));
}

void ORDER_cxyo::impl(Args args)
{
    orderManager.motionControlSystem.setX(OrderManager::parseFloat(args[0]));
    orderManager.motionControlSystem.setY(OrderManager::parseFloat(args[1]));
    orderManager.motionControlSystem.setAngle(OrderManager::parseFloat(args[2]));

    // Mise à jour de l'offset et du target des codeuses. Faut pas tourner parce que le HL te dit où t'es. Je sais il est pas gentil mais faut l'accepter
    orderManager.motionControlSystem.setAngleOffset(OrderManager::parseFloat(args[2]));
    orderManager.motionControlSystem.resetEncoders();
    orderManager.highLevel.printfln(DEBUG_HEADER, "X,Y,O set");
}

void ORDER_ctv::impl(Args args)
{
    orderManager.motionControlSystem.setTranslationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_crv::impl(Args args)
{
    orderManager.motionControlSystem.setRotationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_ctrv::impl(Args args)
{
    orderManager.motionControlSystem.setTranslationSpeed(OrderManager::parseFloat(args[0]));
    orderManager.motionControlSystem.setRotationSpeed(OrderManager::parseFloat(args[1]));
}

void ORDER_efm::impl(Args args)
{
    orderManager.motionControlSystem.setForcedMovement(true);
}

void ORDER_dfm::impl(Args args)
{
    orderManager.motionControlSystem.setForcedMovement(false);
}

void ORDER_ct0::impl(Args args)
{
    orderManager.motionControlSystem.controlledTranslation(false);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Non asservi en translation");
}

void ORDER_ct1::impl(Args args)
{
    orderManager.motionControlSystem.controlledTranslation(true);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Asservi en translation");
}

void ORDER_cr0::impl(Args args)
{
    orderManager.motionControlSystem.controlledRotation(false);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Non asservi en rotation");
}

void ORDER_cr1::impl(Args args)
{
    orderManager.motionControlSystem.controlledRotation(true);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Asservi en rotation");
}

void ORDER_cv0::impl(Args args)
{
    orderManager.motionControlSystem.setControl(false);
    orderManager.highLevel.printfln(DEBUG_HEADER,"Non asservi en vitesse");
}

void ORDER_cv1::impl(Args args)
{
    orderManager.motionControlSystem.setControl(true);
    orderManager.highLevel.printfln(DEBUG_HEADER,"asservi en vitesse");
}

void ORDER_cod::impl(Args args)
{
    orderManager.highLevel.printfln(DEBUG_HEADER,"Gauche:");
    orderManager.highLevel.printfln(DEBUG_HEADER,"%ld", orderManager.motionControlSystem.getLeftTicks());
    orderManager.highLevel.printfln(DEBUG_HEADER,"Droite:");
    orderManager.highLevel.printfln(DEBUG_HEADER,"%ld", orderManager.motionControlSystem.getRightTicks());
}

void ORDER_rawposdata::impl(Args args)
{
    float leftSpeedGoal, rightSpeedGoal;
    orderManager.motionControlSystem.getSpeedGoals(leftSpeedGoal, rightSpeedGoal);

    int16_t xPos = orderManager.motionControlSystem.getX();
    int16_t yPos = orderManager.motionControlSystem.getY();
    float angle = orderManager.motionControlSystem.getAngle();
    float leftSpeed = orderManager.motionControlSystem.getLeftSpeed();
    float rightSpeed = orderManager.motionControlSystem.getRightSpeed();

    char s[50];

    snprintf(s,50,"%d,%d,%f,%f,%f,%f,%f\n", xPos,yPos,angle,leftSpeed, leftSpeedGoal,rightSpeed,rightSpeedGoal);
    Serial.print(s);
}

void ORDER_montlhery::impl(Args args)
{
    orderManager.motionControlSystem.controlledRotation(false);
    orderManager.motionControlSystem.controlledTranslation(false);
    orderManager.motionControlSystem.setForcedMovement(true);
    orderManager.highLevel.printfln(DEBUG_HEADER, "monthlery received");
}

void ORDER_maxtr::impl(Args args) {
    orderManager.motionControlSystem.setMaxTranslationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_maxro::impl(Args args) {
    orderManager.motionControlSystem.setMaxRotationSpeed(OrderManager::parseFloat(args[0]));
}

void ORDER_maxtrro::impl(Args args){
    orderManager.motionControlSystem.setMaxTranslationSpeed(OrderManager::parseFloat(args[0]));
    orderManager.motionControlSystem.setMaxRotationSpeed(OrderManager::parseFloat(args[1]));
}

void ORDER_av::impl(Args args)
{
    orderManager.execute("sstop");
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::FORWARD);
    orderManager.highLevel.printfln(DEBUG_HEADER, "av received");
}

void ORDER_rc::impl(Args args)
{
    orderManager.execute("stop");
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::BACKWARD);
    orderManager.highLevel.printfln(DEBUG_HEADER, "rc received");
}

void ORDER_td::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::ANTITRIGO);
    orderManager.highLevel.printfln(DEBUG_HEADER, "td received");
}

void ORDER_tg::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::TRIGO);
    orderManager.highLevel.printfln(DEBUG_HEADER, "tg received");
}

void ORDER_trstop::impl(Args args)
{
    orderManager.motionControlSystem.stopTranslation();
}

void ORDER_rostop::impl(Args args)
{
    orderManager.motionControlSystem.stopRotation();
}

void ORDER_sstop::impl(Args args)
{
    orderManager.motionControlSystem.speedBasedMovement(MOVEMENT::NONE);
    orderManager.highLevel.printfln(DEBUG_HEADER, "sstop received");
}

void ORDER_nh::impl(Args args)
{
    uint8_t id;
    int32_t x;
    uint32_t y, r;
    float angleHook, angleTolerance;
        id = (uint8_t) OrderManager::parseInt(args[0]);
        x = (int32_t) OrderManager::parseInt(args[1]);
        y = (uint32_t) OrderManager::parseInt(args[2]);
        r = (uint32_t) OrderManager::parseInt(args[3]);
        angleHook = OrderManager::parseFloat(args[4]);
        angleTolerance = OrderManager::parseFloat(args[5]);

        char hookOrder[RX_BUFFER_SIZE] = "";

        for (int i = 6; i < nbr_args; i++) {
            strcat(hookOrder, args[i]);
            strcat(hookOrder, " ");
        }
        hookOrder[RX_BUFFER_SIZE - 1] = '\0';

        orderManager.hookList.addHook(id, x, y, r, angleHook, angleTolerance, hookOrder);

        Serial.print("Ordre du hook: ");
        Serial.println(hookOrder);

        //TEST:
        Serial.println(orderManager.hookList.getHook(id).getOrder());
}

void ORDER_eh::impl(Args args)
{
    int hookId = OrderManager::parseInt(args[0]);
    if(orderManager.hookList.hookWithId(hookId))
    {
        orderManager.hookList.enableHook((uint8_t)hookId); //Singe proof ?
    }
    else
    {
        orderManager.highLevel.printfln(DEBUG_HEADER,"ERREUR::Activation d'un hook inexistant");
    }
}

void ORDER_dh::impl(Args args) {
    int hookId = OrderManager::parseInt(args[0]);

    if (orderManager.hookList.hookWithId(hookId)) {
        orderManager.hookList.disableHook((uint8_t) hookId); //Singe proof ?
    } else {
        orderManager.highLevel.printfln(DEBUG_HEADER, "ERREUR::Désactivation d'un hook inexistant");
    }
}

void ORDER_ptpdemo::impl(Args args)
{
    orderManager.execute("goto 500 -700");
    delay(5000);
    orderManager.execute("goto 1000 -400");
    delay(5000);
    orderManager.execute("goto 750 100");
    delay(5000);
    orderManager.execute("goto 0 0");
}


void ORDER_ptpdemoseq::impl(Args args)
{
    orderManager.execute("goto 500 -700 true");
    delay(5000);
    orderManager.execute("goto 1000 -400 true");
    delay(5000);
    orderManager.execute("goto 750 100 true");
    delay(5000);
    orderManager.execute("goto 0 0 true");
}

void ORDER_rangeSICK::impl(Args args) {
    uint8_t index = (uint8_t) OrderManager::parseInt(args[0]);
    uint16_t min = (uint16_t) OrderManager::parseInt(args[1]);
    uint16_t max = (uint16_t) OrderManager::parseInt(args[2]);
    if(index < NBR_OF_DISTANCE_SENSOR) {
        SensorMgr::Instance().getDistanceSensor(index).setRange(min, max);
    } else {
        orderManager.highLevel.printf(DEBUG_HEADER, "Aucun SICK à l'indice %i!\n", index);
    }
    orderManager.highLevel.printf(DEBUG_HEADER, "Le SICK %i est maintenant dans l'intervalle [%i; %i]\n", index, min, max);
}

void ORDER_testSICK::impl(Args args) {
    if(args.size() > 0) {
        uint8_t index = (uint8_t) OrderManager::parseInt(args[0]);
        if(index < NBR_OF_DISTANCE_SENSOR) {
            orderManager.highLevel.printf(SICK_HEADER, "%i\n", SensorMgr::Instance().getDistanceSensor(index).readDistance());
        } else {
            orderManager.highLevel.printf(DEBUG_HEADER, "Aucun SICK à l'indice %i!\n", index);
        }
    }
}

void ORDER_lectureSICK::impl(Args args) {
    SensorMgr mgr = SensorMgr::Instance();
    if(NBR_OF_DISTANCE_SENSOR == 3) {
        orderManager.highLevel.printfln(SICK_HEADER, "%d %d %d",
                                        mgr.getDistanceSensor(0).readDistance(),
                                        mgr.getDistanceSensor(1).readDistance(),
                                        mgr.getDistanceSensor(2).readDistance());
    } else {
        orderManager.highLevel.printfln(SICK_HEADER, "%d %d %d %d %d %d",
                                        mgr.getDistanceSensor(0).readDistance(),
                                        mgr.getDistanceSensor(1).readDistance(),
                                        mgr.getDistanceSensor(2).readDistance(),
                                        mgr.getDistanceSensor(3).readDistance(),
                                        mgr.getDistanceSensor(4).readDistance(),
                                        mgr.getDistanceSensor(5).readDistance());
    }
}

void ORDER_waitJumper::impl(Args args) {
    Serial.println("Waiting for jumper...");

#if defined(MAIN)
    digitalWrite(LED1, HIGH);
#endif
    // attente de front
    while(digitalRead(PIN_JMPR) == HIGH) {
        InterruptStackPrint::Instance().print();
    }
    while(digitalRead(PIN_JMPR) == LOW) {
        InterruptStackPrint::Instance().print();
    }
    ComMgr::Instance().printfln(EVENT_HEADER, "gogogofast");
#if defined(MAIN)
    digitalWrite(LED1, LOW);
#endif
}

void ORDER_endMatch::impl(Args args) {
    orderManager.execute("stop");
    orderManager.execute("sstop");

    while(true) {
        digitalWrite(LED1, LOW);
        digitalWrite(LED2, LOW);
        delay(100);
        digitalWrite(LED1, HIGH);
        digitalWrite(LED2, HIGH);
        delay(100);
    }
}
