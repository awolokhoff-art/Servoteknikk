#ifndef LED_h
#define LED_h

#include <Arduino.h>
#include "Control.h"

class LED{
private:
  static const int NUM_LEDS = 8;

public:
  static const int ledPins[NUM_LEDS];
  static void LED_init();
};
#endif