#include <Arduino.h>
#include <dac.h>

// -------------------- Pin-konfigurasjon --------------------
const int A_phase  = 68;
const int B_phase  = 66;
const int A_enable = 69;
const int B_enable = 67;

const int directionPin = 6;
const int PWMpin = 7;
const int EncoderA = 20;
const int EncoderB = 21;

// -------------------- Motorparametre DC --------------------
volatile long EncoderCount = 0;
bool MotorDirection = true;
int PwmValue = 0;
const int PulsesPerRevolution = 8380;
int CurrentFloor = 0;
int TargetFloor = 0;
const float floorTolerance = 0.05;
bool FirstButtonPressed = false;

// -------------------- Motorparametre Stepper --------------------
const int stepsPerRevolution = 197;

// -------------------- PID-parametre --------------------
float Kp = 0.2;  // proporsjonal
float Ki = 0.1;  // integrasjon
float Kd = 0.05;  // derivasjon

long lastError = 0;
float integral = 0;
unsigned long lastTime = 0;

// -------------------- Tilstandsmaskin --------------------
enum ElevatorState {
  IDLE,
  MOVING,
  OPENING_DOOR,
  DOOR_OPEN,
  CLOSING_DOOR
};
ElevatorState elevatorState = IDLE;
unsigned long doorTimer = 0;
const unsigned long doorOpenTime = 2000; // 2 sekunder

// -------------------- Floor-klasse --------------------
class Floor {
  public:
    int floorNumber;
    int buttonPin;
    bool requested = false;
    unsigned long lastDebounceTime = 0;
    unsigned long debounceDelay = 50;
    bool lastStableState = HIGH;
    bool lastReading = HIGH;

    Floor(int number, int pin) {
      floorNumber = number;
      buttonPin = pin;
      pinMode(buttonPin, INPUT_PULLUP);
      lastStableState = digitalRead(buttonPin);
      lastReading = lastStableState;
    }

    void update() {
      bool reading = digitalRead(buttonPin);
      if (reading != lastReading) {
        lastDebounceTime = millis();
        lastReading = reading;
      }
      if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != lastStableState) {
          lastStableState = reading;
          if (lastStableState == LOW) requested = true;
        }
      }
    }

    bool consumeRequest() {
      if (requested) {
        requested = false;
        return true;
      }
      return false;
    }
};

// -------------------- Etasje-objekter --------------------
const int NUM_FLOORS = 3;
Floor floors[] = {
  Floor(1, 23),
  Floor(2, 24),
  Floor(3, 25)
};

// -------------------- Encoder-interrupts --------------------
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

// -------------------- Oppsett --------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Initialiserer system...");

  pinMode(A_phase, OUTPUT);
  pinMode(B_phase, OUTPUT);
  pinMode(A_enable, OUTPUT);
  pinMode(B_enable, OUTPUT);
  pinMode(directionPin, OUTPUT);
  pinMode(PWMpin, OUTPUT);
  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);

  attachInterrupt(digitalPinToInterrupt(EncoderA), encoderAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderB), encoderBChange, CHANGE);

  dac_init();
  set_dac(3500, 3500);

  lastTime = millis();

  Serial.println("Klar!");
}

// -------------------- Hovedløkke --------------------
void loop() {

  noInterrupts();
  long countCopy = EncoderCount;
  interrupts();

  float EncoderFloor = (float)countCopy / (float)PulsesPerRevolution;

  // Første knapp
  if (!FirstButtonPressed) {
    CurrentFloor = 0;
    for (int i = 0; i < NUM_FLOORS; i++) {
      if (digitalRead(floors[i].buttonPin) == LOW) {
        FirstButtonPressed = true;
        break;
      }
    }
    if (!FirstButtonPressed) CurrentFloor = 0;
  }

  if (FirstButtonPressed) {
    CurrentFloor = round(EncoderFloor);
  }

  // Oppdater knapper
  for (int i = 0; i < NUM_FLOORS; i++) floors[i].update();

  // Knappebehandling (kun når IDLE)
  if (elevatorState == IDLE) {
    for (int i = 0; i < NUM_FLOORS; i++) {
      if (floors[i].consumeRequest()) {
        TargetFloor = floors[i].floorNumber;
        if (TargetFloor != CurrentFloor) {
          elevatorState = MOVING;
          Serial.print("Heis starter til etasje ");
          Serial.println(TargetFloor);
        }
      }
    }
  }

  // -------------------- Tilstandslogikk --------------------
  switch (elevatorState) {

    case MOVING: {
      // PID-regulering
      unsigned long now = millis();
      float dt = (now - lastTime)/1000.0;
      lastTime = now;

      long targetPulses = TargetFloor * PulsesPerRevolution;
      long error = targetPulses - EncoderCount;
      integral += error * dt;
      float derivative = (error - lastError)/dt;
      float output = Kp*error + Ki*integral + Kd*derivative;
      lastError = error;

      // Sett retning og PWM
      MotorDirection = (output >= 0);
      PwmValue = constrain(abs(output), 0, 100);

      analogWrite(PWMpin, PwmValue);
      digitalWrite(directionPin, MotorDirection ? HIGH : LOW);

      // Sjekk om vi er innenfor toleranse
      if (abs((float)error / PulsesPerRevolution) < floorTolerance) {
        PwmValue = 0;
        analogWrite(PWMpin,0);
        stopMotor();
        elevatorState = OPENING_DOOR;
        doorTimer = millis();
        Serial.println("Heis har ankommet etasjen. Åpner dør...");
      }

      break;
    }

    case OPENING_DOOR:
      delay(500); // liten pause
      OpenDoor();
      elevatorState = DOOR_OPEN;
      doorTimer = millis();
      break;

    case DOOR_OPEN:
      if (millis() - doorTimer > doorOpenTime) {
        CloseDoor();
        elevatorState = CLOSING_DOOR;
        Serial.println("Lukker dør...");
      }
      break;

    case CLOSING_DOOR:
      elevatorState = IDLE;
      Serial.println("Dør lukket. Heis klar.");
      break;

    case IDLE:
      PwmValue = 0;
      break;
  }

  // Debug
  Serial.print("State: "); Serial.print(elevatorState);
  Serial.print(" | Curr: "); Serial.print(CurrentFloor);
  Serial.print(" | Target: "); Serial.print(TargetFloor);
  Serial.print(" | EncFloor: "); Serial.println(EncoderFloor);

  delay(10);
}

// -------------------- Stepperfunksjoner --------------------
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
  step = (step+1)%4;
  delay(7);
}
void fullStepBackward() {
  digitalWrite(A_enable, HIGH);
  digitalWrite(B_enable, HIGH);
  switch (step) {
    case 0: digitalWrite(A_phase, LOW); digitalWrite(B_phase,LOW); break;
    case 1: digitalWrite(A_phase, LOW); digitalWrite(B_phase, HIGH); break;
    case 2: digitalWrite(A_phase, HIGH);  digitalWrite(B_phase, HIGH); break;
    case 3: digitalWrite(A_phase, HIGH);  digitalWrite(B_phase, LOW);  break;
  }
  step = (step+1)%4;
  delay(7);
}
void OpenDoor() {
  step=0;
  digitalWrite(A_enable,HIGH);
  digitalWrite(B_enable,HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) fullStepForward();
  stopMotor();
  delay(50);
}
void CloseDoor() {
  step=0;
  digitalWrite(A_enable,HIGH);
  digitalWrite(B_enable,HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) fullStepBackward();
  stopMotor();
  delay(50);
}
void stopMotor() {
  digitalWrite(A_enable, LOW);
  digitalWrite(B_enable, LOW);
}
