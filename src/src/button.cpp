#include "button.h"

Button::Button()
    : m_buttonLatchedTime(0)
{
}

void Button::setActiveLevel(button_active_level_e activeLevel)
{
    m_activeLevel = activeLevel;
}

void Button::sample(bool buttonState)
{
    if (m_activeLevel == BUTTON_ACTIVE_LOW)
    {
        // Invert the button state if it's active low
        buttonState = !buttonState;
    }

    if (!buttonState)
    {
        // Reset the latched time to now
        m_buttonLatchedTime = millis();
    }
    
    for (uint8_t i = 0; i < m_numCallbacks; ++i)
    {
        
    }
}

void Button::addCallback(uint16_t holdIntervalMs, void (*callback)())
{
}
