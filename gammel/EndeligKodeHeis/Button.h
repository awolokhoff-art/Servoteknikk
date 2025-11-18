#ifndef Button_h
#define Button_h

#include <Arduino.h>
#include "Elevator.h"  // For å bruke elev.addCarRequest()

// --- Buttons & LEDs ---
extern const int NUM_BUTTONS;
extern const int buttonPins[];
extern const int ledPins[];

// --- Button Debounce State ---
extern unsigned long lastDebounceTime[];
extern bool lastStableState[];
extern bool lastReading[];
extern const unsigned long debounceDelay;

// --- Funksjoner ---
void checkButtons();       // Sjekk knappene og håndter presses
void Button_init();        // Initialiser pins og debounce state

#endif
