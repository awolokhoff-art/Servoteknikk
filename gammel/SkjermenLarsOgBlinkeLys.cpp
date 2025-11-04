#include <Arduino.h>
#include <LiquidCrystal.h>

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// LCD setup
LiquidCrystal lcd(41, 40, 37, 36, 35, 34);
const int LCD_BACKLIGHT = 4;
// Button pins 
const int BUTTON_PIN0 = 22;
const int BUTTON_PIN1 = 23;
const int BUTTON_PIN2 = 24;
const int BUTTON_PIN3 = 25;
const int BUTTON_PIN4 = 26;
const int BUTTON_PIN5 = 27;
const int BUTTON_PIN6 = 28;
const int BUTTON_PIN7 = 29;
//


// LED pins
const int LED_PIN0 = 49;    // LED on pin D49
const int LED_PIN1 = 48;    // LED on pin D48
const int LED_PIN2 = 47;    // LED on pin D47
const int LED_PIN3 = 46;    // LED on pin D46
const int LED_PIN4 = 45;    // LED on pin D45
const int LED_PIN5 = 44;    // LED on pin D44
const int LED_PIN6 = 43;    // LED on pin D43
const int LED_PIN7 = 42;    // LED on pin D42
//


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Elevator state machine
enum ElevatorState { IDLE, MOVING };
ElevatorState state = IDLE;
int currentFloor = 1;      // start at floor 1
int targetFloor = -1;      // no target
unsigned long moveStart = 0;
const unsigned long moveTimePerFloor = 800; // ms to move one floor (tweakable)

// Helper arrays (map index 0..7 to pins)
int buttonPins[8] = { BUTTON_PIN0, BUTTON_PIN1, BUTTON_PIN2, BUTTON_PIN3, BUTTON_PIN4, BUTTON_PIN5, BUTTON_PIN6, BUTTON_PIN7 };
int ledPins[8] = { LED_PIN0, LED_PIN1, LED_PIN2, LED_PIN3, LED_PIN4, LED_PIN5, LED_PIN6, LED_PIN7 };

// Debounce helpers
int lastButtonState[8] = { LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW };
unsigned long lastDebounceMillis[8] = { 0,0,0,0,0,0,0,0 };
const unsigned long debounceDelay = 30; // ms


// LCD display cache to avoid unnecessary updates
String prevLine1 = ""; 
String prevLine2 = "";
// Blink settings for requested floor LED
const unsigned long blinkInterval = 400; // ms
bool blinkState = false;
unsigned long lastBlinkMillis = 0;
int prevSimFloor = -1; // previous simulated floor index (1..8), -1 = none
// Target blink state (separate so both can blink)
bool targetBlinkState = false;
unsigned long lastTargetBlinkMillis = 0;

void setup() {

  // LCD Screen
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, HIGH);
  
  // Set up all button pins as inputs
  pinMode(BUTTON_PIN0, INPUT);
  pinMode(BUTTON_PIN1, INPUT);
  pinMode(BUTTON_PIN2, INPUT);
  pinMode(BUTTON_PIN3, INPUT);
  pinMode(BUTTON_PIN4, INPUT);
  pinMode(BUTTON_PIN5, INPUT);
  pinMode(BUTTON_PIN6, INPUT);
  pinMode(BUTTON_PIN7, INPUT);
  
  // Set all LED pins as outputs and ensure off
  for (int i = 0; i < 8; ++i) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Elevator ready");
}

// non-blocking loop with simple state machine
void loop() {
  unsigned long now = millis();

  // If we're idle, poll buttons for a request
  if (state == IDLE) {
    for (int i = 0; i < 8; ++i) {
      int reading = digitalRead(buttonPins[i]);

      // simple debounce: only act when stable for debounceDelay
      if (reading != lastButtonState[i]) {
        lastDebounceMillis[i] = now; // reset debounce timer
        lastButtonState[i] = reading;
      }

      if ((now - lastDebounceMillis[i]) > debounceDelay) {
        // treat HIGH as pressed (user wiring option)
        if (reading == HIGH) {
          targetFloor = i + 1; // floors numbered 1..8
          state = MOVING;
          moveStart = now;

          // show on LCD once
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Move to floor ");
          lcd.print(targetFloor);
          lcd.setCursor(0, 1);
          lcd.print("Current: ");
          lcd.print(currentFloor);

          // show target LED
          // clear LEDs, then start target blink
          for (int j = 0; j < 8; ++j) digitalWrite(ledPins[j], LOW);
          targetBlinkState = true;
          lastTargetBlinkMillis = now;
          digitalWrite(ledPins[targetFloor - 1], targetBlinkState ? HIGH : LOW);
          break; // only handle one request at a time
        }
      }
    }
  }

  // MOVING state: simulate travel using millis()
  if (state == MOVING) {
    int floorsToMove = abs(targetFloor - currentFloor);
    unsigned long totalMove = (unsigned long)floorsToMove * moveTimePerFloor;

    if (floorsToMove == 0) {
  // already there
  state = IDLE;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Arrived floor ");
  lcd.print(currentFloor);
  // turn on current floor LED steady, clear others
  for (int j = 0; j < 8; ++j) digitalWrite(ledPins[j], LOW);
  if (currentFloor >= 1 && currentFloor <= 8) digitalWrite(ledPins[currentFloor - 1], HIGH);
  // stop blinking
  blinkState = false;
  lastBlinkMillis = 0;
    } else if (now - moveStart >= totalMove) {
  // arrived
  currentFloor = targetFloor;
  state = IDLE;
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Arrived floor ");
  lcd.print(currentFloor);
  // turn on current floor LED steady, clear others
  for (int j = 0; j < 8; ++j) digitalWrite(ledPins[j], LOW);
  if (currentFloor >= 1 && currentFloor <= 8) digitalWrite(ledPins[currentFloor - 1], HIGH);
  // stop blinking
  blinkState = false;
  lastBlinkMillis = 0;
    } else {
      // still moving - show intermediate floor on line 2 and blink that floor's LED
      int progressed = (now - moveStart) / moveTimePerFloor;
      int simFloor = currentFloor + ((targetFloor > currentFloor) ? progressed : -progressed);
      lcd.setCursor(0, 1);
      lcd.print("At floor ");
      lcd.print(simFloor);

      // if simulated floor changed, reset blink state and clear LEDs
      if (simFloor != prevSimFloor) {
        if (prevSimFloor >= 1 && prevSimFloor <= 8) digitalWrite(ledPins[prevSimFloor - 1], LOW);
        prevSimFloor = simFloor;
        blinkState = false;
        lastBlinkMillis = now;
      }

      // Blink the simulated current floor LED
      if (simFloor >= 1 && simFloor <= 8) {
        if (now - lastBlinkMillis >= blinkInterval) {
          lastBlinkMillis = now;
          blinkState = !blinkState;
          digitalWrite(ledPins[simFloor - 1], blinkState ? HIGH : LOW);
        }
      }

      // Also blink the target LED independently
      if (targetFloor >= 1 && targetFloor <= 8) {
        if (now - lastTargetBlinkMillis >= blinkInterval) {
          lastTargetBlinkMillis = now;
          targetBlinkState = !targetBlinkState;
          digitalWrite(ledPins[targetFloor - 1], targetBlinkState ? HIGH : LOW);
        }
      }
    }
  }

  // keep loop fast and non-blocking
}


