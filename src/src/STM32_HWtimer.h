#pragma once
#include "Arduino.h"
#include <HardwareTimer.h>
#include "targets.h"



class HWtimer
{
public:
    static volatile uint32_t TimerInterval;
    static uint32_t LastCallbackMicros;
    static uint32_t LastCallbackMicros_180;
    static volatile bool TickTock;
    static int16_t PhaseShift;
    static bool ResetNextLoop;

    static void inline nullCallback(void);
    static void (*CallBack)();     // inphase callback
    static void (*CallBack_180)(); //180 out of phase callback

    static void ICACHE_RAM_ATTR UpdateInterval(uint32_t TimerInterval_);
    static void ICACHE_RAM_ATTR UpdatePhaseShift(int16_t Offset_);
    static void ICACHE_RAM_ATTR TimerCallback();
    static void ICACHE_RAM_ATTR Init();
    static void ICACHE_RAM_ATTR Stop();
};