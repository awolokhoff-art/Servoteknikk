/*
 * ====================================================================
 * ==                COMBINED ELEVATOR CONTROLLER                  ==
 * ====================================================================
 * * This sketch integrates three separate codebases:
 * 1. Elevator.h: A C++ class for the elevator's queuing logic (kø system).
 * 2. Motorstyring: PID control for the DC motor (lift), stepper for doors,
 * and encoder feedback.
 * 3. vise det på skjerm: LCD display and button/LED interface.
 *
 * Integration by: Gemini
 * Date: 2025-11-03
 * */

// --------------------------------------------------------------------
// -------------------- INCLUDES --------------------------------------
// --------------------------------------------------------------------
#include <LiquidCrystal.h>
#include <dac.h> // Assumes dac.h is available in your environment

// --------------------------------------------------------------------
// -------------------- 1. ELEVATOR LOGIC CLASS (from Elevator.h) -----
// --------------------------------------------------------------------
// NOTE: This is the full class definition, placed directly in the .ino
#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <Arduino.h>
//in .ino #include "Elevator.h", and call Serial.begin(115200) in setup;
//Use the class like Elevator elev; elev.addHallRequest(3, Request::Up);

enum class Request { Down, Up };

class Elevator {
public:
  enum class State { MovingDown = 0, MovingUp = 1, Idle = 2 };

  // Configure number of floors here (0..MAX_FLOOR)
  // *** MODIFIED: Changed from 8 to 7 to match the 8 UI buttons (0-7) ***
  static const int MAX_FLOOR = 7; 
  static const int NUM_FLOORS = MAX_FLOOR + 1; // Now 8 floors (0-7)

  // public members
  int currentFloor = 0;
  int lowerFloor = 0;
  int upperFloor = MAX_FLOOR;
  State state = State::Idle;

  // constructor (default)
  Elevator(State state_ = State::Idle, int current = 0)
    : currentFloor(current), state(state_) {
    clearStops();
  }

  // add a stop from inside the car
  void addCarRequest(int floor) {
    if (!acceptedFloorRange(floor)) return;
    if (floor == currentFloor && state == State::Idle) return; // Already here
    stops[floor] = true;
    startElevatorMoving(floor);
  }

  // add a hall request (outside), with direction
  void addHallRequest(int floor, Request request) {
    Serial.println(F("in addHallRequest"));
    if (!acceptedFloorRange(floor)) return;
    if (floor == currentFloor && state == State::Idle) {
      // would open doors in real system
      return;
    }

    if (request == Request::Up) {
      upStops[floor] = true;
    }
    // keep consistent with original: requests may be inserted also depending on currentFloor relation
    // *** NOTE: This original logic is a bit strange, but preserved ***
    if (currentFloor < floor) {
      upStops[floor] = true;
    }

    if (request == Request::Down) {
      downStops[floor] = true;
    }
    if (currentFloor > floor) {
      downStops[floor] = true;
    }

    startElevatorMoving(floor);
  }

  // *** NOTE: stopAtNextFloor() and nextFloor() are NOT used in the combined
  //     non-blocking sketch, as they rely on simulated movement. ***
  
  // call repeatedly - returns true if elevator stops at the new currentFloor
  bool stopAtNextFloor() {
    if (state == State::MovingUp) {
      Serial.print(F("Moving up "));
      Serial.println(currentFloor);
      // move one floor up
      if (currentFloor < upperFloor) ++currentFloor;

      if (upStops[currentFloor]) {
        upStops[currentFloor] = false;
        return true;
      }
    }

    if (state == State::MovingDown) {
      Serial.print(F("Moving down "));
      Serial.println(currentFloor);
      if (currentFloor > lowerFloor) --currentFloor;

      if (downStops[currentFloor]) {
        downStops[currentFloor] = false;
        return true;
      }
    }

    if (stops[currentFloor]) {
      stops[currentFloor] = false;
      return true;
    }

    return false;
  }

  // blocks until next stop and returns the floor number
  int nextFloor() {
    unsigned long loopStart = millis();
    const unsigned long TIMEOUT_MS = 20000UL; 
    while (!stopAtNextFloor()) {
      changeElevatorMoving();
      delay(10);
      if (!hasAnyStops()) {
        Serial.println(F("No stops left - breaking nextFloor loop"));
        break;
      }
      if (millis() - loopStart > TIMEOUT_MS) {
        Serial.println(F("nextFloor timeout - breaking loop"));
        break;
      }
    }
    Serial.print(F("Next Floor is "));
    Serial.println(currentFloor);
    return currentFloor;
  }


  // change direction/state when needed (based on remaining stops)
  // *** This IS used in the combined sketch ***
  void changeElevatorMoving() {
    if (!hasAnyStops()) {
      state = State::Idle;
      //Serial.println(F("Stopping elevator")); // Too noisy
      return;
    }

    if (state == State::MovingUp) {
      if (hasStopsUp(currentFloor)) return;
      if (hasStopsUpFromAny(currentFloor)) return;
      state = State::MovingDown;
      Serial.println(F("Logic: Start moving down"));
      return;
    }

    if (state == State::MovingDown) {
      if (hasStopsDown(currentFloor)) return;
      if (hasStopsDownFromAny(currentFloor)) return;
      state = State::MovingUp;
      Serial.println(F("Logic: Start moving up"));
      return;
    }

    // If Idle but there are stops, decide direction based on nearest/requests:
    if (state == State::Idle) {
      // simple policy: if there is any stop above => move up, else move down
      if (hasStopsUpFromAny(currentFloor) || hasStopsUp(currentFloor)) {
        state = State::MovingUp;
        Serial.println(F("Logic: Idle -> start moving up"));
      } else if (hasStopsDownFromAny(currentFloor) || hasStopsDown(currentFloor)) {
        state = State::MovingDown;
        Serial.println(F("Logic: Idle -> start moving down"));
      }
    }
  }

  // call to reset all stops
  void clearStops() {
    for (int i = 0; i < NUM_FLOORS; ++i) {
      upStops[i] = false;
      downStops[i] = false;
      stops[i] = false;
    }
  }

  // simple helpers
  bool hasAnyStops() const {
    for (int i = 0; i < NUM_FLOORS; ++i) {
      if (upStops[i] || downStops[i] || stops[i]) return true;
    }
    return false;
  }

  // For testing convenience: set a raw stop
  void setStop(int floor) {
    if (acceptedFloorRange(floor)) stops[floor] = true;
  }

  // internal stop storage (arrays indexed by floor number)
  bool upStops[NUM_FLOORS];
  bool downStops[NUM_FLOORS];
  bool stops[NUM_FLOORS];

  // start moving based on a requested floor
  void startElevatorMoving(int floor) {
    if (state == State::Idle) {
      if (currentFloor < floor) {
        state = State::MovingUp;
        Serial.print(F("Logic: Moving up to "));
        Serial.print(floor);
        Serial.print(F(" from "));
        Serial.println(currentFloor);
      } else if (currentFloor > floor) {
        state = State::MovingDown;
        Serial.print(F("Logic: Moving down to "));
        Serial.print(floor);
        Serial.print(F(" from "));
        Serial.println(currentFloor);
      } else {
        // same floor -> remain idle (doors would open)
      }
    }
  }

  // check if there are stops strictly above given floor in upStops array
  bool hasStopsUp(int floor) const {
    for (int i = floor + 1; i <= upperFloor; ++i) {
      if (upStops[i]) return true;
    }
    return false;
  }

  // check if any stops above in any stops array (used in changeElevatorMoving)
  bool hasStopsUpFromAny(int floor) const {
    for (int i = floor + 1; i <= upperFloor; ++i) {
      if (upStops[i] || stops[i] || downStops[i]) return true;
    }
    return false;
  }

  // check if there are stops at or below floor in downStops array
  bool hasStopsDown(int floor) const {
    for (int i = floor - 1; i >= lowerFloor; --i) {
      if (downStops[i]) return true;
    }
    return false;
  }

  // check if any stops below in any stops array
  bool hasStopsDownFromAny(int floor) const {
    for (int i = floor - 1; i >= lowerFloor; --i) {
      if (downStops[i] || stops[i] || upStops[i]) return true;
    }
    return false;
  }

  // check valid floor range
  bool acceptedFloorRange(int floor) const {
    if (floor >= lowerFloor && floor <= upperFloor) return true;
    Serial.println(F("You have entered a floor outside of the scope, try again"));
    return false;
  }
};

#endif // ELEVATOR_H


// --------------------------------------------------------------------
// -------------------- 2. GLOBAL OBJECTS & PINS ----------------------
// --------------------------------------------------------------------

// --- Global Queue Object ---
Elevator elev;

// --- LCD (from 'vise det på skjerm') ---
LiquidCrystal lcd(41, 40, 37, 36, 35, 34);
const int LCD_BACKLIGHT = 4;
unsigned long lastLcdUpdate = 0;
const long lcdUpdateInterval = 250; // Update LCD every 250ms

// --- Stepper Motor (Doors) (from 'Motorstyring') ---
const int A_phase  = 68;
const int B_phase  = 66;
const int A_enable = 69;
const int B_enable = 67;
const int stepsPerRevolution = 197; // For door motor

// --- DC Motor (Lift) (from 'Motorstyring') ---
const int directionPin = 6;
const int PWMpin = 7;
const int EncoderA = 20;
const int EncoderB = 21;

// --- Buttons & LEDs (from 'vise det på skjerm') ---
// 8 buttons for floors 0-7
const int NUM_BUTTONS = 8;
const int buttonPins[] = {22, 23, 24, 25, 26, 27, 28, 29};
const int ledPins[] = {49, 48, 47, 46, 45, 44, 43, 42}; // LEDs 0-7

// --- Button Debounce State ---
unsigned long lastDebounceTime[NUM_BUTTONS] = {0};
bool lastStableState[NUM_BUTTONS] = {HIGH};
bool lastReading[NUM_BUTTONS] = {HIGH};
const unsigned long debounceDelay = 50;


// --------------------------------------------------------------------
// -------------------- 3. GLOBAL STATE & PARAMETERS ------------------
// --------------------------------------------------------------------

// --- Encoder & Position ---
volatile long EncoderCount = 0;
const int PulsesPerRevolution = 8380;
float currentEncoderFloor = 0.0; // Precise float position
int currentLogicalFloor = 0;    // Rounded integer position
int physicalTargetFloor = 0;    // The *next* floor the motor will stop at
const float floorTolerance = 0.05; // Arrival tolerance

// --- DC Motor Control ---
bool MotorDirection = true; // true = UP, false = DOWN
int PwmValue = 0;

// --- PID-parametre ---
float Kp = 0.2;  // proporsjonal
float Ki = 0.1;  // integrasjon
float Kd = 0.05;  // derivasjon
long lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// --- Physical State Machine (from 'Motorstyring') ---
enum PhysicalState {
  P_IDLE,
  P_MOVING,
  P_OPENING_DOOR,
  P_DOOR_OPEN,
  P_CLOSING_DOOR
};
PhysicalState physicalState = P_IDLE;
unsigned long doorTimer = 0;
const unsigned long doorOpenTime = 2000; // 2 sekunder


// --------------------------------------------------------------------
// -------------------- 4. ENCODER INTERRUPTS (from 'Motorstyring') ---
// --------------------------------------------------------------------

void encoderAChange() {
  int a = digitalRead(EncoderA);
  int b = digitalRead(EncoderB);
  if (a == b) EncoderCount++;
  else EncoderCount--;
}

void encoderBChange() {
  int a = digitalRead(EncoderA);
  int b = digitalRead(EncoderB);
  if (a != b) EncoderCount++;
  else EncoderCount--;
}

// --------------------------------------------------------------------
// -------------------- 5. STEPPER FUNCTIONS (from 'Motorstyring') ----
// --------------------------------------------------------------------
int step = 0;
void fullStepForward() {
  digitalWrite(A_enable, HIGH);
  digitalWrite(B_enable, HIGH);
  switch (step) {
    case 0: digitalWrite(A_phase, HIGH); digitalWrite(B_phase, LOW); break;
    case 1: digitalWrite(A_phase, HIGH); digitalWrite(B_phase, HIGH); break;
    case 2: digitalWrite(A_phase, LOW);  digitalWrite(B_phase, HIGH); break;
    case 3: digitalWrite(A_phase, LOW);  digitalWrite(B_phase, LOW);  break;
  }
  step = (step + 1) % 4;
  delay(7);
}
void fullStepBackward() {
  digitalWrite(A_enable, HIGH);
  digitalWrite(B_enable, HIGH);
  switch (step) {
    case 0: digitalWrite(A_phase, LOW); digitalWrite(B_phase, LOW); break;
    case 1: digitalWrite(A_phase, LOW); digitalWrite(B_phase, HIGH); break;
    case 2: digitalWrite(A_phase, HIGH);  digitalWrite(B_phase, HIGH); break;
    case 3: digitalWrite(A_phase, HIGH);  digitalWrite(B_phase, LOW);  break;
  }
  step = (step + 1) % 4;
  delay(7);
}
void OpenDoor() {
  Serial.println("Phys: Opening door...");
  step = 0;
  digitalWrite(A_enable, HIGH);
  digitalWrite(B_enable, HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) fullStepForward();
  stopMotor(); // Disable stepper
  delay(50);
  Serial.println("Phys: Door open.");
}
void CloseDoor() {
  Serial.println("Phys: Closing door...");
  step = 0;
  digitalWrite(A_enable, HIGH);
  digitalWrite(B_enable, HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) fullStepBackward();
  stopMotor(); // Disable stepper
  delay(50);
  Serial.println("Phys: Door closed.");
}
void stopMotor() {
  digitalWrite(A_enable, LOW);
  digitalWrite(B_enable, LOW);
}


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
          Serial.print("UI: Button pressed for floor ");
          Serial.println(i);
          elev.addCarRequest(i);          // Add to queue
          digitalWrite(ledPins[i], HIGH); // Turn on LED
        }
      }
    }
    lastReading[i] = reading;
  }
}

/**
 * @brief Helper to convert states to printable strings for the LCD.
 */
const char* getPhysicalStateString(PhysicalState s) {
  switch (s) {
    case P_IDLE: return "IDLE  ";
    case P_MOVING: return "MOVING";
    case P_OPENING_DOOR: return "OPENIN";
    case P_DOOR_OPEN: return "OPEN  ";
    case P_CLOSING_DOOR: return "CLOSIN";
    default: return "???   ";
  }
}

const char* getLogicalStateString(Elevator::State s) {
  switch (s) {
    case Elevator::State::Idle: return "Idle";
    case Elevator::State::MovingUp: return "Up  ";
    case Elevator::State::MovingDown: return "Down";
    default: return "??? ";
  }
}


/**
 * @brief Updates the 16x2 LCD display. Called periodically.
 */
void updateLcd() {
  lcd.clear();
  
  // Line 0: Floor Status
  lcd.setCursor(0, 0);
  lcd.print("Flr:");
  lcd.print(currentLogicalFloor);
  lcd.print(" Tgt:");
  lcd.print(physicalTargetFloor);
  
  // Line 1: State Status
  lcd.setCursor(0, 1);
  lcd.print("Log:");
  lcd.print(getLogicalStateString(elev.state));
  lcd.print(" Phy:");
  lcd.print(getPhysicalStateString(physicalState));
}


// --------------------------------------------------------------------
// -------------------- 7. SETUP --------------------------------------
// --------------------------------------------------------------------

void setup() {
  // Start Serial monitor (115200 for faster debug)
  Serial.begin(115200);
  while (!Serial); // Wait for serial
  Serial.println("Initialiserer system...");

  // --- Initialize Stepper (Doors) ---
  pinMode(A_phase, OUTPUT);
  pinMode(B_phase, OUTPUT);
  pinMode(A_enable, OUTPUT);
  pinMode(B_enable, OUTPUT);
  stopMotor(); // Ensure doors are not drawing power

  // --- Initialize DC Motor (Lift) ---
  pinMode(directionPin, OUTPUT);
  pinMode(PWMpin, OUTPUT);
  analogWrite(PWMpin, 0);

  // --- Initialize Encoders ---
  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderA), encoderAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderB), encoderBChange, CHANGE);
  
  // --- Initialize DAC ---
  dac_init();
  set_dac(3500, 3500);

  // --- Initialize Buttons (UI) ---
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    pinMode(buttonPins[i], INPUT_PULLUP); // Use internal pull-up resistors
    // Read initial state for debounce
    lastStableState[i] = digitalRead(buttonPins[i]);
    lastReading[i] = lastStableState[i];
  }

  // --- Initialize LEDs (UI) ---
  for (int i = 0; i < NUM_BUTTONS; ++i) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW); // All LEDs off
  }

  // --- Initialize LCD (UI) ---
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, HIGH);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Elevator ready");
  lcd.setCursor(0, 1);
  lcd.print("Floors: 0-7");

  // --- Initialize Timers ---
  lastTime = millis();
  delay(1000); // Wait 1 sec
  Serial.println("Klar!");
}


// --------------------------------------------------------------------
// -------------------- 8. MAIN LOOP ----------------------------------
// --------------------------------------------------------------------

void loop() {

  // --- 1. READ INPUTS ---
  
  // Get current encoder position
  noInterrupts();
  long countCopy = EncoderCount;
  interrupts();
  
  currentEncoderFloor = (float)countCopy / (float)PulsesPerRevolution;
  int newLogicalFloor = round(currentEncoderFloor);
  
  if (newLogicalFloor != currentLogicalFloor) {
      currentLogicalFloor = newLogicalFloor;
      elev.currentFloor = currentLogicalFloor; // Sync the logic class
      Serial.print("Phys: Reached floor ");
      Serial.println(currentLogicalFloor);
  }

  // Check for button presses
  checkButtons();


  // --- 2. RUN PHYSICAL STATE MACHINE ---
  switch (physicalState) {

    case P_IDLE: {
      // Elevator is stopped, doors are closed. Time to find a new job.
      
      // First, check if there's a request AT the current floor
      if (elev.stops[currentLogicalFloor] || elev.upStops[currentLogicalFloor] || elev.downStops[currentLogicalFloor]) {
        Serial.println("Phys: Request at current floor. Opening doors.");
        physicalState = P_OPENING_DOOR;
        doorTimer = millis();
      } 
      else {
        // If no job here, ask the logic class what to do
        elev.changeElevatorMoving(); // Updates elev.state
        
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
            elev.state = Elevator::State::Idle;
          }
        }
      }
      break;
    }

    case P_MOVING: {
      // PID-regulering
      unsigned long now = millis();
      float dt = (now - lastTime) / 1000.0;
      lastTime = now;

      long targetPulses = (long)physicalTargetFloor * PulsesPerRevolution;
      long error = targetPulses - EncoderCount;
      
      // Don't let integral wind up if we are saturated
      if(abs(PwmValue) < 255) {
         integral += error * dt;
      }
      
      float derivative = (error - lastError) / dt;
      float output = Kp * error + Ki * integral + Kd * derivative;
      lastError = error;

      // Sett retning og PWM
      MotorDirection = (output >= 0); // true = UP
      PwmValue = constrain(abs(output), 0, 100); // Constrain PWM 0-100

      analogWrite(PWMpin, PwmValue);
      digitalWrite(directionPin, MotorDirection ? HIGH : LOW);

      // Sjekk om vi er innenfor toleranse (ARRIVED)
      if (abs((float)error / PulsesPerRevolution) < floorTolerance) {
        PwmValue = 0;
        analogWrite(PWMpin, 0); // Stop motor
        // stopMotor(); // This is for stepper
        
        currentLogicalFloor = physicalTargetFloor; // Lock in our floor
        elev.currentFloor = currentLogicalFloor;   // Sync logic
        
        Serial.print("Phys: Arrived at target floor ");
        Serial.println(currentLogicalFloor);
        
        physicalState = P_OPENING_DOOR;
        doorTimer = millis();
      }
      break;
    }

    case P_OPENING_DOOR:
      // Clear all requests for this floor *before* opening door
      clearRequestsAt(currentLogicalFloor); 
      delay(200); // Small pause before door motor
      OpenDoor();
      physicalState = P_DOOR_OPEN;
      doorTimer = millis();
      break;

    case P_DOOR_OPEN:
      if (millis() - doorTimer > doorOpenTime) {
        CloseDoor();
        physicalState = P_CLOSING_DOOR;
      }
      break;

    case P_CLOSING_DOOR:
      physicalState = P_IDLE;
      Serial.println("Phys: Ready for next job.");
      break;
  }

  // --- 3. UPDATE DISPLAY (Non-blocking) ---
  if (millis() - lastLcdUpdate > lcdUpdateInterval) {
    updateLcd();
    lastLcdUpdate = millis();
  }

  // Small delay to keep loop stable
  delay(5);
}