#include "Button.h"
#include "Elevator.h" 
#include "LED.h" // LED::ledPins

extern Elevator elev; 

const int Button::buttonPins[] = {22, 23, 24, 25, 26, 27, 28, 29};

unsigned long Button::lastDebounceTime[Button::NUM_BUTTONS] = {0};
bool Button::lastStableState[Button::NUM_BUTTONS] = {HIGH};
bool Button::lastReading[Button::NUM_BUTTONS] = {HIGH};
const unsigned long Button::debounceDelay = 50;

Button::Button(){
    for (int i = 0; i < NUM_BUTTONS; ++i) {
        lastStableState[i] = HIGH;
        lastReading[i] = HIGH;
        lastDebounceTime[i] = 0;
    }
}

void Button::Button_init(){

  for (int i = 0; i < NUM_BUTTONS; ++i) 
  {
    pinMode(buttonPins[i], INPUT_PULLUP);
    lastStableState[i] = digitalRead(buttonPins[i]);
    lastReading[i] = lastStableState[i];
  }
}

void Button::checkSerialInput(){
  if (Serial.available() > 0) {
    // Les hele strengen til brukeren trykker Enter
    String input = Serial.readStringUntil('\n');
    input.trim(); // Fjern eventuelle mellomrom

    if (input.length() != 2) {
      Serial.println("Ugyldig input. Bruk format: [etasje][retning] (f.eks. '3u')");
      return;
    }

    // Konverter første tegn (etasje) til et tall
    // '3' (char) - '0' (char) = 3 (int)
    int floor = input.charAt(0) - '0';

    // Les andre tegn (retning)
    char direction = input.charAt(1);
    bool request_accepted = false; // Lag en variabel for å sjekke

    if (direction == 'u') {
      Serial.print("Simulerer eksternt kall: Etasje ");
      Serial.print(floor);
      Serial.println(" OPP");
      request_accepted = elev.addHallRequest(floor, Elevator::Request::Up); // Få returverdi
    } 
    else if (direction == 'd') {
      Serial.print("Simulerer eksternt kall: Etasje ");
      Serial.print(floor);
      Serial.println(" NED");
      request_accepted = elev.addHallRequest(floor, Elevator::Request::Down); // Få returverdi
    } 
    else {
      Serial.println("Ugyldig retning. Bruk 'u' for opp eller 'd' for ned.");
      return; // Avslutt hvis retning er feil
    }
  
    // Slå på LED hvis kallet ble akseptert 
    if (request_accepted) {
      if(floor >= 0 && floor < Button::NUM_BUTTONS) { // Dobbeltsjekk at etasjen er gyldig
        digitalWrite(LED::ledPins[floor], HIGH);
      }
    } else {
      Serial.println("... forespørsel ignorert (sannsynligvis allerede der).");
    }
  }
}

void Button::checkButtons(){
  for (int i = 0; i < NUM_BUTTONS; i++) 
  {
    bool reading = digitalRead(buttonPins[i]);
    
    // If the switch changed, due to noise or pressing:
    if (reading != lastReading[i])
    {
      lastDebounceTime[i] = millis();
    }
    
    if ((millis() - lastDebounceTime[i]) > debounceDelay) 
    {
      // whatever the reading is at, it's been there for longer
      // than the debounce delay, so take it as the actual current state:
      if (reading != lastStableState[i]) {
        lastStableState[i] = reading;
        
        // If the button was just PRESSED (went from HIGH to LOW)
        if (lastStableState[i] == LOW) {
      
          bool request_accepted = elev.addCarRequest(i);
          
          if (request_accepted) {
            Serial.print("UI: Button pressed for floor ");
            Serial.println(i);
            digitalWrite(LED::ledPins[i], HIGH); // Turn on LED
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



