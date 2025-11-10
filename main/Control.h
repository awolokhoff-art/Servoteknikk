#ifndef Control_h
#define Control_h

#include <Arduino.h>
#include "Elevator.h"  // For å bruke elev.addCarRequest()

class Control{

private:
  static const int NUM_BUTTONS = 8;
  static const int buttonPins[NUM_BUTTONS];
  const int ledPins[NUM_BUTTONS];

  // --- Button Debounce State ---
  static unsigned long lastDebounceTime[NUM_BUTTONS];
  static bool lastStableState[NUM_BUTTONS];
  static bool lastReading[NUM_BUTTONS];
  static const unsigned long debounceDelay;


public:
  Control();
  void Button_init();       // Initialiser pins og debounce state
  void checkButtons();      // Sjekk de fysiiske knappene  og håndter presses
  void checkSerialInput();  // Får Serial Monitor inputs, konverter dem til "taste trykk"

};
#endif // Button_h
