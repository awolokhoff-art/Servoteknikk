/*

  For å bruke serial monitor må man ha baudrate på 115200 baud

*/


// -------------------- INCLUDES --------------------------------------

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <dac.h> 
#include "Elevator.h"
#include "DC_motor.h"
#include "Stepper.h"
#include "LCD.h"
#include "PhysicalState.h"
#include "PID.h"
#include "Button.h"
#include "LED.h"


// --- Global Queue Object ---
Elevator elev;

//-----Global variables-----
    float currentEncoderFloor = 0.0;
    int currentLogicalFloor = 0;
    long physicalTargetFloor = 0;

    PhysicalState physicalState = P_IDLE;
    unsigned long doorTimer = 0;
    const unsigned long doorOpenTime = 2000; // 2 sekunder



/*
 * Clears all requests for a given floor and turns off its LED.
 * Called upon arrival at a floor.
 */
void clearRequestsAt(int floor) {
  if (floor < 0 || floor > elev.MAX_FLOOR) return;
  
  // Check if any request exists before printing
  if (elev.stops[floor] || elev.upStops[floor] || elev.downStops[floor]) {
    Serial.print("Phys: Servicing floor ");
    Serial.println(floor);
    elev.stops[floor] = false;
    elev.upStops[floor] = false;
    elev.downStops[floor] = false;
    digitalWrite(ledPins[floor], LOW);
  }
}

/*
 * Finds the *next* physical floor to stop at based on the
 * current *logical* direction from the Elevator class.
 * The floor number to go to, or -1 if no target.
 */
int findNextTargetInDirection() {
  if (elev.state == Elevator::State::MovingUp) {
    // 1. Find closest stop (car or hall-up) in direction of travel
    for (int f = currentLogicalFloor + 1; f <= elev.upperFloor; f++) {
      if (elev.stops[f] || elev.upStops[f]) return f;
    }
    // 2. If none, find highest hall-down call (turnaround point)
    for (int f = elev.upperFloor; f > currentLogicalFloor; f--) {
      if (elev.downStops[f]) return f;
    }
  }
  
  else if (elev.state == Elevator::State::MovingDown) {
    // 1. Find closest stop (car or hall-down) in direction of travel
    for (int f = currentLogicalFloor - 1; f >= elev.lowerFloor; f--) {
      if (elev.stops[f] || elev.downStops[f]) return f;
    }
    // 2. If none, find lowest hall-up call (turnaround point)
    for (int f = elev.lowerFloor; f < currentLogicalFloor; f++) {
      if (elev.upStops[f]) return f;
    }
  }
  
  return -1; // No target found
}

/*
 * Sjekker for input fra Serial Monitor
 * Format: [etasje][retning] + Enter (f.eks. "3u" eller "2d")
 */
void checkSerialInput() {
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
      request_accepted = elev.addHallRequest(floor, Request::Up); // Få returverdi
    } 
    else if (direction == 'd') {
      Serial.print("Simulerer eksternt kall: Etasje ");
      Serial.print(floor);
      Serial.println(" NED");
      request_accepted = elev.addHallRequest(floor, Request::Down); // Få returverdi
    } 
    else {
      Serial.println("Ugyldig retning. Bruk 'u' for opp eller 'd' for ned.");
      return; // Avslutt hvis retning er feil
    }
    


    // Slå på LED hvis kallet ble akseptert 
    if (request_accepted) {
      if(floor >= 0 && floor < NUM_BUTTONS) { // Dobbeltsjekk at etasjen er gyldig
        digitalWrite(ledPins[floor], HIGH);
      }
    } else {
      Serial.println("... forespørsel ignorert (sannsynligvis allerede der).");
    }
  }
}




// -------------------- SETUP --------------------------------------

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial
  Serial.println("Initialiserer system...");

  // --- Initialize Stepper (Doors) ---
  Step_init();

  // --- Initialize DC Motor (Lift) ---
  DC_init();

  // --- Initialize Encoders ---
  Enc_init();
  
  // --- Initialize DAC ---
  dac_init();
  set_dac(3500, 3500);

  // --- Initialize Buttons (UI) ---
  Button_init();

  //--- Initialize LED ---

  LED_init();

  // --- Initialize LCD ---
  Lcd_init();


  // --- Initialize Timers ---
  lastTime = millis();
  delay(1000); // Wait 1 sec
  Serial.println("Klar!");

}


// -------------------- MAIN LOOP ------------------------

void loop() {

  // Les inputs
  GetEncoderPos();

  // Check for button presses
  checkButtons();
  
  // Sjekk for seriell input 
  checkSerialInput();


  // --- State machine ---
  switch (physicalState) {

    case P_IDLE: {
      // Elevator is stopped, doors are closed. Time to find a new job.
      
      // First, check if there's a request AT the current floor
      if (elev.stops[currentLogicalFloor] || elev.upStops[currentLogicalFloor] || elev.downStops[currentLogicalFloor]) {
        Serial.println("Phys: Request at current floor. Opening doors.");
        physicalState = P_OPENING_DOOR;
        // NOTE: We no longer set doorTimer here
      } 
      else {
        // If no job here, ask the logic class what to do
        elev.changeElevatorMoving(); // Updates elev.state
        
        // RETTET: Bruker "::State::" istedenfor ".State::"
        if (elev.state != Elevator::State::Idle) {
          // Logic class wants to move. Find the *next* stop.
          physicalTargetFloor = findNextTargetInDirection();
          
          if (physicalTargetFloor != -1) {
            // We have a target!
            Serial.print("Phys: New target acquired: ");
            Serial.println(physicalTargetFloor);
            physicalState = P_MOVING;
            // Reset PID
            integral = 0;
            lastError = 0;
            lastTime = millis();
          } else {
            // Logic state is moving, but no target found?
            // This can happen if only a request at current floor exists
            // which was missed. Force logic back to Idle.
            // RETTET: Bruker "::State::" istedenfor ".State::"
            elev.state = Elevator::State::Idle;
          }
        }
      }
      break;
    }

   case P_MOVING: {
    long targetPulses = physicalTargetFloor * PulsesPerRevolution;
    computePID(targetPulses, EncoderCount);
    analogWrite(PWMpin, PwmValue);
    digitalWrite(directionPin, MotorDirection ? HIGH : LOW);


    // Sjekk om vi er innenfor toleranse
    if (abs((float)(targetPulses - EncoderCount) / PulsesPerRevolution) < floorTolerance) {
        PwmValue = 0;
        analogWrite(PWMpin, 0);

        currentLogicalFloor = physicalTargetFloor;
        elev.currentFloor = currentLogicalFloor;

        Serial.print("Phys: Arrived at target floor ");
        Serial.println(currentLogicalFloor);

        physicalState = P_OPENING_DOOR;
    }
    break;
}

    
    case P_OPENING_DOOR:
      // Linjen under er flyttet for å oppfylle REQ 13
      // clearRequestsAt(currentLogicalFloor); 
      delay(200); // Small pause before door motor

      
      // Force the LCD to update to "HALF O" *before* we block the loop
      updateLcd(); 
      lastLcdUpdate = millis(); // Reset LCD timer

      OpenDoor(); // This is the blocking call

      // *** NY PLASSERING for REQ 13 ***
      // Forespørsler slettes *etter* at døren er helt åpen 
      clearRequestsAt(currentLogicalFloor); 

      // Now that the door is fully open, change state and start the timer
      physicalState = P_DOOR_OPEN;
      doorTimer = millis();
      break;

    
    case P_DOOR_OPEN:
      // This state is now non-blocking. It just waits for the timer.
      if (millis() - doorTimer > doorOpenTime) {
        // Timer expired, change state to *begin* closing.
        physicalState = P_CLOSING_DOOR;
        // We do *not* call CloseDoor() here anymore.
      }
      break;

    
    case P_CLOSING_DOOR:
      
      // Force the LCD to update to "HALF C" *before* we block the loop
      updateLcd();
      lastLcdUpdate = millis(); // Reset LCD timer

      CloseDoor(); // This is the blocking call

      // Now that the door is fully closed, go back to IDLE
      physicalState = P_IDLE;
      Serial.println("Phys: Ready for next job.");
      break;
  }

  // --- 3. UPDATE DISPLAY (Adaptive) ---
  
  // 1. Bestem riktig oppdateringsintervall
  long currentInterval;
  
  // RETTET: Bruker "::State::" istedenfor ".State::"
  if (physicalState == P_IDLE && elev.state == Elevator::State::Idle && !elev.hasAnyStops()) {
    // Systemet er helt i hvilemodus (ingen bevegelse, ingen jobber i kø)
    // Senk oppdateringshastigheten.
    currentInterval = lcdUpdateInterval_Idle;
  } else {
    // Systemet er aktivt (flytter seg, dører åpne, eller har jobber som venter)
    currentInterval = lcdUpdateInterval_Active;
  }

  // 2. Sjekk mot det valgte intervallet
  // Dette håndterer "OPEN" og "IDLE"/"MOVING" tilstandene.
  if (millis() - lastLcdUpdate > currentInterval) {
    updateLcd();
    lastLcdUpdate = millis();
  }

  // Small delay to keep loop stable
  delay(5);
}