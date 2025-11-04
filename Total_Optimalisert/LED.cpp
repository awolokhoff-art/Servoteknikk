#include "LED.h"
#include "Button.h"



const int ledPins[] = {49, 48, 47, 46, 45, 44, 43, 42}; // LEDs 0-7

void LED_init() {
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW); // All LEDs off


  }
}

