#include "Stepper.h"
#include "Elevator.h"

// ---- Pin definitions ----
const int A_phase  = 68;
const int B_phase  = 66;
const int A_enable = 69;
const int B_enable = 67;
const int stepsPerRevolution = 197;

// ---- Step state ----
int step = 0;

// ---- Functions ----
void Step_init() {
  pinMode(A_phase, OUTPUT);
  pinMode(B_phase, OUTPUT);
  pinMode(A_enable, OUTPUT);
  pinMode(B_enable, OUTPUT);
  stopMotor();
}

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
    case 0: digitalWrite(A_phase, LOW);  digitalWrite(B_phase, LOW);  break;
    case 1: digitalWrite(A_phase, LOW);  digitalWrite(B_phase, HIGH); break;
    case 2: digitalWrite(A_phase, HIGH); digitalWrite(B_phase, HIGH); break;
    case 3: digitalWrite(A_phase, HIGH); digitalWrite(B_phase, LOW);  break;
  }
  step = (step + 1) % 4;
  delay(7);
}

void OpenDoor() {
  step = 0;
  digitalWrite(A_enable, HIGH);
  digitalWrite(B_enable, HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) fullStepForward();
  stopMotor();
}

void CloseDoor() {
  step = 0;
  digitalWrite(A_enable, HIGH);
  digitalWrite(B_enable, HIGH);
  for (int i = 0; i < stepsPerRevolution; i++) fullStepBackward();
  stopMotor();
}

void stopMotor() {
  digitalWrite(A_enable, LOW);
  digitalWrite(B_enable, LOW);
}
