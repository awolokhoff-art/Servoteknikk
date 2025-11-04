/*
For å bruke serial monitor må man ha baudrate på 115200 baud

 * ====================================================================
 * ==                     KOMBINERT HEISKONTROLLER                     ==
 * ====================================================================
 * * BESKRIVELSE:
 * Denne Arduino-skissen er en komplett kontroller for en heismodell.
 * Den kombinerer tre hoveddeler:
 * 1.  Logikk (Elevator.h): Et C++ klassebasert køsystem som bestemmer
 * hvilken etasje heisen skal til (logisk tilstand).
 * 2.  Motorstyring: PID-regulering for heismotoren (DC-motor) og
 * styring av en steppermotor for dørene.
 * 3.  Grensesnitt (UI): Håndtering av knapper (etasjevalg), lysdioder
 * og en 16x2 LCD-skjerm for statusvisning.
 *
 * --- HOVEDKOMPONENTER ---
 *
 * 1. ELEVATOR LOGIC CLASS (Elevator.h)
 * - Dette er "hjernen" i heisen.
 * - `Elevator elev;`: Hovedobjektet som holder styr på køen.
 * - `Elevator::State`: Logisk tilstand (Idle, MovingUp, MovingDown).
 * - `addCarRequest(floor)`: Legger til en forespørsel fra innsiden (knapp).
 * - `changeElevatorMoving()`: Oppdaterer den logiske tilstanden (f.eks.
 * bytter retning når den når toppen).
 *
 * 2. GLOBALE OBJEKTER OG PINS
 * - Definerer alle pins for LCD, motorer, knapper og lysdioder.
 *
 * 3. GLOBAL TILSTAND OG PARAMETERE
 * - `EncoderCount`: En 'volatile long' som telles opp/ned av interrupts
 * for å gi nøyaktig posisjon.
 * - `currentLogicalFloor`: Den etasjen heisen "er i", rundet av fra
 * encoder-verdien.
 * - `physicalTargetFloor`: Den *neste* etasjen heisen skal stoppe i.
 * - `Kp, Ki, Kd`: PID-parametere for å styre DC-motoren jevnt.
 * - `PhysicalState`: En state-machine som styrer den *fysiske*
 * tilstanden til heisen (P_IDLE, P_MOVING, P_OPENING_DOOR, etc.).
 *
 * 4. ENCODER INTERRUPTS
 * - `encoderAChange()` og `encoderBChange()`: Kjøres automatisk hver gang
 * encoder-signalene endres. Dette sikrer at vi aldri mister
 * posisjonen, uavhengig av hva som skjer i hovedloopen.
 *
 * 5. STEPPERFUNKSJONER
 * - `OpenDoor()` og `CloseDoor()`: Blokkerende funksjoner som kjører
 * steppermotoren et bestemt antall steg for å åpne/lukke døren.
 *
 * 6. HJELPEFUNKSJONER
 * - `clearRequestsAt(floor)`: Slukker lyset og fjerner alle forespørsler
 * for en etasje når heisen ankommer.
 * - `findNextTargetInDirection()`: Ser i kø-objektet (`elev`) for å finne
 * den neste logiske etasjen å stoppe på i nåværende retning.
 * - `checkButtons()`: Leser alle knappene (non-blocking) med debounce.
 * - `get...StateString()`: Konverterer 'enum'-tilstander til tekst
 * for LCD-skjermen.
 * - `updateLcd()`: Oppdaterer LCD-skjermen med ny info.
 *
 * 7. SETUP()
 * - Initialiserer alt: Serial, pins for motorer/UI, LCD-skjerm,
 * og kobler encoder-pins til interrupts.
 *
 * 8. LOOP()
 * Hovedloopen er delt i tre deler:
 *
 * 1. LES INPUTS:
 * - Henter den nåværende encoder-posisjonen (`EncoderCount`).
 * - Oppdaterer `currentLogicalFloor` basert på posisjonen.
 * - Kjører `checkButtons()` for å se etter nye forespørsler.
 *
 * 2. KJØR FYSISK STATE MACHINE:
 * - En stor `switch(physicalState)` som bestemmer hva heisen
 * fysisk skal gjøre.
 * - `P_IDLE`: Heisen står stille. Sjekker om den har en jobb
 * (enten på nåværende etasje eller en annen etasje).
 * - `P_MOVING`: Heisen er i bevegelse. Kjører PID-logikken for
 * å justere motor-PWM (`analogWrite`) basert på feilen
 * mellom `EncoderCount` og `targetPulses`.
 * - `P_OPENING_DOOR`: Kjører `updateLcd()` for å vise "HALF O",
 * kaller `OpenDoor()`, og går til `P_DOOR_OPEN`.
 * - `P_DOOR_OPEN`: Venter i `doorOpenTime` millisekunder.
 * - `P_CLOSING_DOOR`: Kjører `updateLcd()` for å vise "HALF C",
 * kaller `CloseDoor()`, og går til `P_IDLE`.
 *
 * 3. OPPDATER DISPLAY:
 * - Kjører `updateLcd()` med jevne mellomrom (`lcdUpdateInterval`)
 * for å vise status (etasje, mål, logisk tilstand, dørstatus).
 *
 * ====================================================================
 */

// --------------------------------------------------------------------
// -------------------- INCLUDES --------------------------------------
// --------------------------------------------------------------------
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <dac.h> // Assumes dac.h is available in your environment
#include "Elevator.h"
#include "DC_motor.h"
#include "Stepper.h"
#include "LCD.h"
#include "PhysicalState.h"
#include "PID.h"
#include "Button.h"
#include "LED.h"

// --------------------------------------------------------------------
// -------------------- 2. GLOBAL OBJECTS & PINS ----------------------
// --------------------------------------------------------------------

// --- Global Queue Object ---
Elevator elev;

// --------------------------------------------------------------------
// -------------------- 3. GLOBAL STATE & PARAMETERS ------------------
// --------------------------------------------------------------------
//-----Global variables-----
    float currentEncoderFloor = 0.0;
    int currentLogicalFloor = 0;
    int physicalTargetFloor = 0;

    PhysicalState physicalState = P_IDLE;
    unsigned long doorTimer = 0;
    const unsigned long doorOpenTime = 2000; // 2 sekunder

// --------------------------------------------------------------------
// -------------------- 6. HELPER FUNCTIONS (New) ---------------------
// --------------------------------------------------------------------

/**
 * @brief Clears all requests for a given floor and turns off its LED.
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

/**
 * @brief Finds the *next* physical floor to stop at based on the
 * current *logical* direction from the Elevator class.
 * @return The floor number to go to, or -1 if no target.
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

/**
 * @brief Non-blocking check of all 8 car buttons.
 * On press, adds request to queue and lights LED.
 */

/**
 * @brief Sjekker for input fra Serial Monitor for å simulere
 * eksterne etasjeknapper (REQ 9 & 10).
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
    
    // *** NY BLOKK: Slå på lyset hvis kallet ble akseptert ***
    if (request_accepted) {
      if(floor >= 0 && floor < NUM_BUTTONS) { // Dobbeltsjekk at etasjen er gyldig
        digitalWrite(ledPins[floor], HIGH);
      }
    } else {
      Serial.println("... forespørsel ignorert (sannsynligvis allerede der).");
    }
  }
}

/**
 * @brief Helper to convert states to printable strings for the LCD.
 */

/**
 * @brief Updates the 16x2 LCD display. Called periodically.
 */


// --------------------------------------------------------------------
// -------------------- 7. SETUP --------------------------------------
// --------------------------------------------------------------------

void setup() {
  // Start Serial monitor (115200 for faster debug)
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

// --------------------------------------------------------------------
// -------------------- 8. MAIN LOOP (REVISED) ------------------------
// --------------------------------------------------------------------

void loop() {

  // --- 1. READ INPUTS ---
  GetEncoderPos();

  // Check for button presses
  checkButtons();
  
  // *** NY LINJE: Sjekk for seriell input ***
  checkSerialInput();


  // --- 2. RUN PHYSICAL STATE MACHINE (MODIFIED LOGIC) ---
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

    // *** MODIFIED STATE (for REQ 13) ***
    case P_OPENING_DOOR:
      // Linjen under er flyttet for å oppfylle REQ 13
      // clearRequestsAt(currentLogicalFloor); 
      delay(200); // Small pause before door motor

      // ****** KEY FIX ******
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

    // *** MODIFIED STATE ***
    case P_DOOR_OPEN:
      // This state is now non-blocking. It just waits for the timer.
      if (millis() - doorTimer > doorOpenTime) {
        // Timer expired, change state to *begin* closing.
        physicalState = P_CLOSING_DOOR;
        // We do *not* call CloseDoor() here anymore.
      }
      break;

    // *** MODIFIED STATE ***
    case P_CLOSING_DOOR:
      // ****** KEY FIX ******
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
  // "HLF" (Half) tilstandene håndteres av den tvungne oppdateringen i state-maskinen.
  if (millis() - lastLcdUpdate > currentInterval) {
    updateLcd();
    lastLcdUpdate = millis();
  }

  // Small delay to keep loop stable
  delay(5);
}