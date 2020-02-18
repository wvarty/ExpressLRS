#pragma once

#include "../../src/targets.h"
#include <Arduino.h>
#include <stdio.h>

#define TimerIntervalUSDefault 20000

//HardwareTimer *MyTim;

class hwTimer
{
public:
    //static TIM_TypeDef *Instance;
    static HardwareTimer *MyTim;

    static volatile uint32_t HWtimerInterval;
    static volatile bool TickTock;
    static volatile int16_t PhaseShift;
    static volatile bool ResetNextLoop;
    static volatile uint32_t LastCallbackMicrosTick;
    static volatile uint32_t LastCallbackMicrosTock;

    static void init();
    static void ICACHE_RAM_ATTR pause();
    static void ICACHE_RAM_ATTR stop();
    static void ICACHE_RAM_ATTR callback(HardwareTimer *);
    static void ICACHE_RAM_ATTR updateInterval(uint32_t newTimerInterval);
    static void ICACHE_RAM_ATTR phaseShift(int32_t newPhaseShift);

    static void inline nullCallback();
    static void (*callbackTick)();
    static void (*callbackTock)();
};