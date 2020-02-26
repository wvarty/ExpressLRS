#pragma once

#include <Arduino.h>

typedef enum ActiveLevel
{
    BUTTON_ACTIVE_HIGH,
    BUTTON_ACTIVE_LOW,
} button_active_level_e;

class Button
{
    void setActiveLevel(button_active_level_e activeLevel);
    void sample(bool buttonState);
    void addCallback(uint16_t holdIntervalMs, void (*callback)());

private:
    button_active_level_e   m_activeLevel;
    uint32_t                m_buttonLatchedTime;
    uint16_t                m_intervals[];
    void                    (*m_callbacks[])();
};