#include "DC_motor.h"
#include "Elevator.h"
extern Elevator elev;  // Viktig: fortell compiler at "elev" finnes et annet sted


// --- PIN DEFINISJONER ---
const int directionPin = 6;
const int PWMpin = 7;
const int EncoderA = 20;
const int EncoderB = 21;

// --- GLOBAL VARIABLES ---
volatile long EncoderCount = 0;
float floorTolerance = 0.05;
bool MotorDirection = true;
int PwmValue = 0;
const int PulsesPerRevolution = 8380;
long physicalTargetFloor = 0;
long targetPulses = physicalTargetFloor * PulsesPerRevolution;

// --- FUNKSJONER ---
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

void DC_init() {
  pinMode(directionPin, OUTPUT);
  pinMode(PWMpin, OUTPUT);
  analogWrite(PWMpin, 0);
}

void Enc_init() {
  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderA), encoderAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderB), encoderBChange, CHANGE);
}

void GetEncoderPos() {  
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
  }}

  void floorReached(){
    if (abs((float)(targetPulses - EncoderCount) / PulsesPerRevolution) < floorTolerance) {
        PwmValue = 0;
        analogWrite(PWMpin, 0);}

    }
