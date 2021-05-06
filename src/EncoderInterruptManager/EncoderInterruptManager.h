#ifndef ENCODER_INTERRUPT_MANAGER
#define ENCODER_INTERRUPT_MANAGER

#include "Utils/Singleton.hpp"
#include "Controllers/InterruptController.h"

enum RobotSide {LEFT, RIGHT};

class EncoderInterruptManager : public Singleton<EncoderInterruptManager> {

public:
    EncoderInterruptManager();
    template<RobotSide S>int get_ticks();
    template<RobotSide S>long get_delta();
    template<RobotSide S>void reset_ticks();
private:
    InterruptController leftController;
    InterruptController rightController;
};

#endif