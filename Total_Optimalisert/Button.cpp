#include "Button.h"
#include "Elevator.h"
extern Elevator elev;  // Viktig: fortell compiler at "elev" finnes et annet sted

// --- Buttons & LEDs (from 'vise det på skjerm') ---
// 8 buttons for floors 0-7
const int NUM_BUTTONS = 8;
const int buttonPins[] = {22, 23, 24, 25, 26, 27, 28, 29};


// --- Button Debounce State ---
unsigned long lastDebounceTime[NUM_BUTTONS] = {0};
bool lastStableState[NUM_BUTTONS] = {HIGH};
bool lastReading[NUM_BUTTONS] = {HIGH};
const unsigned long debounceDelay = 50;

void checkButtons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    bool reading = digitalRead(buttonPins[i]);
    
    // If the switch changed, due to noise or pressing:
    if (reading != lastReading[i]) {
      lastDebounceTime[i] = millis();
    }
    
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
      if (reading != lastStableState[i]) {
        lastStableState[i] = reading;
        
        // If the button was just PRESSED (went from HIGH to LOW)
        if (lastStableState[i] == LOW) {
          
          // *** MODIFISERT LOGIKK ***
          // Prøv å legg til forespørsel. Funksjonen returnerer true
          // hvis den ble akseptert, false hvis den ble ignorert.
          bool request_accepted = elev.addCarRequest(i);
          
          if (request_accepted) {
            Serial.print("UI: Button pressed for floor ");
            Serial.println(i);
            digitalWrite(ledPins[i], HIGH); // Turn on LED
          } else {
            Serial.print("UI: Ignored button press for floor ");
            Serial.println(i);
            // Ikke slå på lysdioden
          }
        }
      }
    }
    lastReading[i] = reading;
  }
}

void Button_init(){
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP); // Use internal pull-up resistors
    // Read initial state for debounce
    lastStableState[i] = digitalRead(buttonPins[i]);
    lastReading[i] = lastStableState[i];
  }
  }