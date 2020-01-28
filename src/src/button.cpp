#include "button.h"
#include "targets.h"

#define DEFAULT_PRESSED_INTERVAL 1000 // Default to 1000ms before triggering

Button::Button()
    : m_buttonState(false)
    , m_lastButtonState(false)
    , m_lastDebounceTime(0)
    , m_pressedIntervalMs(DEFAULT_PRESSED_INTERVAL)
{}

void
Button::Sample()
{
  bool currentState = digitalRead(GPIO_PIN_BUTTON);

  if (!BUTTON_ACTIVE_HIGH) {
    currentState = !currentState;
  }

  if (currentState != m_lastButtonState) {
    // reset the debouncing timer
    m_lastDebounceTime = millis();
  }

  m_lastButtonState = currentState;

  if (!currentState) {
    // Don't latch un-pressed state
    m_buttonState = false;
    return;
  }

  // Wait till button is pressed for the pressed interval in ms
  if ((millis() - m_lastDebounceTime) > m_pressedIntervalMs) {
    if (currentState) {
      m_buttonState = true;
    }
  }
}

bool
Button::IsPressed() const
{
  return m_buttonState;
}

void
Button::Reset()
{
  m_buttonState = m_lastButtonState = false;
  m_lastDebounceTime = millis();
}

void
Button::SetPressedInterval(uint16_t intervalMs)
{
    m_pressedIntervalMs = intervalMs;
}
