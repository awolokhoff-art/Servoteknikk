#include "LED.h"
#include "Control.h" 

const int LED::ledPins[LED::NUM_LEDS] = {49, 48, 47, 46, 45, 44, 43, 42}; // LEDs 0-7

void LED::LED_init() {
  for (int i = 0; i < NUM_LEDS; ++i) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
}

