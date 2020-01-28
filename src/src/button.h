#include "Arduino.h"

class Button
{
// Class API
public:
    Button();
    void Sample();
    bool IsPressed() const;
    void Reset();
    void SetPressedInterval(uint16_t intervalMs); // Interval in ms that the button must be pressed for to be HIGH

private:
    bool        m_buttonState;
    bool        m_lastButtonState;
    uint16_t    m_lastDebounceTime;
    uint16_t    m_pressedIntervalMs;
};
