#include "STM32_HWtimer.h"

void inline HWtimer::nullCallback(void){};

void (*HWtimer::CallBack)(void) = &nullCallback;     // inphase callback
void (*HWtimer::CallBack_180)(void) = &nullCallback; //180 out of phase callback

volatile uint32_t HWtimer::TimerInterval = 20000;

volatile bool HWtimer::TickTock = false;
bool HWtimer::ResetNextLoop = false;
int16_t HWtimer::PhaseShift = 0;
uint32_t HWtimer::LastCallbackMicros = 0;
uint32_t HWtimer::LastCallbackMicros_180 = 0;

void ICACHE_RAM_ATTR HWtimer::Init()
{

// Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
#if defined(TIM1)
    TIM_TypeDef *Instance = TIM1;
#else
    TIM_TypeDef *Instance = TIM2;
#endif

    HardwareTimer *MyTim = new HardwareTimer(Instance);

    noInterrupts();
    //timer1_attachInterrupt(HWtimer::TimerCallback);
    //timer1_enable(TIM_DIV16, TIM_EDGE, TIM_LOOP); //5MHz ticks
    //timer1_write(HWtimer::TimerInterval);

    MyTim->setMode(2, TIMER_OUTPUT_COMPARE);                     // In our case, channekFalling is configured but not really used. Nevertheless it would be possible to attach a callback to channel compare match.
    MyTim->setOverflow(HWtimer::TimerInterval, MICROSEC_FORMAT); // 10 Hz
    MyTim->attachInterrupt(HWtimer::TimerCallback);
    MyTim->resume();
    interrupts();
}

void ICACHE_RAM_ATTR HWtimer::Stop()
{
    //timer1_detachInterrupt();
}

void ICACHE_RAM_ATTR HWtimer::UpdateInterval(uint32_t TimerInterval_)
{
    HWtimer::TimerInterval = TimerInterval;
    //timer1_write(TimerInterval / 2);
    MyTim->setOverflow(HWtimer::TimerInterval / 2, MICROSEC_FORMAT);
}

void ICACHE_RAM_ATTR HWtimer::UpdatePhaseShift(int16_t Offset_)
{
    //HWtimer::PhaseShift = Offset_ * 5;
    HWtimer::PhaseShift = Offset_;
}

void ICACHE_RAM_ATTR HWtimer::TimerCallback(HardwareTimer*)
{
    if (TickTock)
    {

        if (ResetNextLoop)
        {
            //timer1_write(HWtimer::TimerInterval / 2);
            MyTim->setOverflow(HWtimer::TimerInterval / 2, MICROSEC_FORMAT);
            HWtimer::ResetNextLoop = false;
        }

        if (HWtimer::PhaseShift > 0 || HWtimer::PhaseShift < 0)
        {
            //timer1_write((HWtimer::TimerInterval + HWtimer::PhaseShift) / 2);
            MyTim->setOverflow(((HWtimer::TimerInterval + HWtimer::PhaseShift) / 2), MICROSEC_FORMAT);
            HWtimer::ResetNextLoop = true;
            HWtimer::PhaseShift = 0;
        }

        //uint32_t next = ESP.getCycleCount() + HWtimerInterval * 5;
        //timer1_write(next + PhaseShift); // apply phase shift to next cycle
        //PhaseShift = 0; //then reset the phase shift variable
        (CallBack)();
        HWtimer::LastCallbackMicros = micros();
    }
    else
    {
        (CallBack_180)();
        HWtimer::LastCallbackMicros_180 = micros();
    }
    HWtimer::TickTock = !HWtimer::TickTock;
}