#ifndef CONFIG_INTERRUPTS
#define CONFIG_INTERRUPTS

#include "stm32l4xx_hal_exti.h"


extern "C" void enableOnlyOneLine(short line_nbr) {
    if (line_nbr == 6) {
        NVIC_DisableIRQ(EXTI9_5_IRQn);
        NVIC_DisableIRQ(EXTI0_IRQn);
        NVIC_DisableIRQ(EXTI9_5_IRQn);
    }
}

#endif