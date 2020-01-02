#include "Arduino.h"
#include "HardwareTimer.h"

volatile uint32_t HWtimerInterval = TimerIntervalUSDefault;
volatile bool TickTock = false;
uint32_t HWtimerLastCallbackMicros = 0;
uint32_t HWtimerLastCallbackMicros90 = 0;
int16_t PhaseShift = 0;
bool ResetNextLoop = false;
void (*HWtimerCallBack)(void) = NULL;
void (*HWtimerCallBack90)(void) = NULL;

#ifdef PLATFORM_STM32
    #if defined(TIM1)
    TIM_TypeDef *Instance = TIM1;
    #else
    TIM_TypeDef *Instance = TIM2;
    #endif
    
    HardwareTimer *MyTim = new HardwareTimer(Instance);
#endif

void ICACHE_RAM_ATTR InitHarwareTimer()
{
    noInterrupts();
    #ifdef PLATFORM_STM32
    MyTim->setMode(2, TIMER_OUTPUT_COMPARE);                     // In our case, channekFalling is configured but not really used. Nevertheless it would be possible to attach a callback to channel compare match.
    MyTim->setOverflow(HWtimerInterval / 2, MICROSEC_FORMAT); // 10 Hz
    MyTim->attachInterrupt(Timer0Callback);
    MyTim->resume();
    #else
    timer1_attachInterrupt(Timer0Callback);
    timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP); //5MHz ticks
    timer1_write(HWtimerInterval);                //120000 us
    #endif
    interrupts();
}

uint32_t ICACHE_RAM_ATTR HWtimerGetlastCallbackMicros()
{
    return HWtimerLastCallbackMicros;
}

uint32_t ICACHE_RAM_ATTR HWtimerGetlastCallbackMicros90()
{
    return HWtimerLastCallbackMicros90;
}

uint32_t ICACHE_RAM_ATTR HWtimerGetIntervalMicros()
{
    return HWtimerInterval;
}

void ICACHE_RAM_ATTR HWtimerUpdateInterval(uint32_t _TimerInterval)
{
    #ifdef PLATFORM_STM32
    HWtimerInterval = _TimerInterval
    MyTim->setOverflow(HWtimerInterval / 2, MICROSEC_FORMAT);
    MyTim->refreash();
    #else
    HWtimerInterval = _TimerInterval * 5;
    timer1_write(HWtimerInterval / 2);
    #endif
}

void ICACHE_RAM_ATTR HWtimerPhaseShift(int16_t Offset)
{
    PhaseShift = Offset * 5;
}

void ICACHE_RAM_ATTR Timer0Callback()
{
    if (TickTock)
    {

        if (ResetNextLoop)
        {
            #ifdef PLATFORM_STM32
            MyTim->setOverflow(HWtimerInterval / 2, MICROSEC_FORMAT);
            MyTim->refreash();
            #else
            timer1_write(HWtimerInterval / 2);
            #endif
            ResetNextLoop = false;
        }

        if (PhaseShift > 0 || PhaseShift < 0)
        {
            #ifdef PLATFORM_STM32
            MyTim->setOverflow((HWtimerInterval + PhaseShift) / 2, MICROSEC_FORMAT);
            MyTim->refreash();
            #else
            timer1_write((HWtimerInterval + PhaseShift) / 2);
            #endif
            ResetNextLoop = true;
            PhaseShift = 0;
        }

        //uint32_t next = ESP.getCycleCount() + HWtimerInterval * 5;
        //timer1_write(next + PhaseShift); // apply phase shift to next cycle
        //PhaseShift = 0; //then reset the phase shift variable
        HWtimerCallBack();
        HWtimerLastCallbackMicros = micros();
    }
    else
    {
        HWtimerCallBack90();
        HWtimerLastCallbackMicros90 = micros();
    }
    TickTock = !TickTock;
}

void StopHWtimer()
{
    #ifdef PLATFORM_STM32
    MyTim->detachInterrupt();
    #else
    timer1_detachInterrupt();
    #endif
}

void HWtimerSetCallback(void (*CallbackFunc)(void))
{
    HWtimerCallBack = CallbackFunc;
}

void HWtimerSetCallback90(void (*CallbackFunc)(void))
{
    HWtimerCallBack90 = CallbackFunc;
}