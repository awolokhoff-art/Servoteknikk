#include "DC_motor.h"
#include "Elevator.h"

extern Elevator elev;

const int DCmotor::EncoderA = 20;
const int DCmotor::EncoderB = 21;
int DCmotor::currentLogicalFloor = 0;
float DCmotor::currentEncoderFloor = 0.0; 

const int DCmotor::PulsesPerRevolution = 8380;
const int DCmotor::PWMpin = 7;
const int DCmotor::directionPin = 6;

int DCmotor::PwmValue = 0;

volatile long DCmotor::EncoderCount = 0;

bool DCmotor::MotorDirection = true;

float DCmotor::floorTolerance = 0.05;
float DCmotor::velocity = 0.0;


DCmotor::DCmotor(){
}

void DCmotor::DC_init() {
  pinMode(directionPin, OUTPUT);
  pinMode(PWMpin, OUTPUT);
  analogWrite(PWMpin, 0);
  Serial.println("DCmotor initialized");
}

void DCmotor::Enc_init() {
  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderA), encoderAChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(EncoderB), encoderBChange, CHANGE);
}

void DCmotor::encoderAChange() {
  int a = digitalRead(EncoderA);
  int b = digitalRead(EncoderB);
  if (a == b) EncoderCount++;
  else EncoderCount--;
}

void DCmotor::encoderBChange() {
  int a = digitalRead(EncoderA);
  int b = digitalRead(EncoderB);
  if (a != b) EncoderCount++;
  else EncoderCount--;
}

void DCmotor::GetEncoderPos() {  
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


void DCmotor::UpdateVelocity() {
    static unsigned long lastTime = 0;
    static float lastPos = 0.0;

    unsigned long now = millis();
    if (now - lastTime >= 50) {  // 10 ms sampling interval
        float currentPos = currentEncoderFloor;

        float dt = (now - lastTime) / 1000.0;  // convert ms → seconds
        velocity = (currentPos - lastPos) / dt;

        lastPos = currentPos;
        lastTime = now;

        Serial.print("Position: ");
        Serial.print(currentPos);
        Serial.print(", Velocity: ");
        Serial.println(velocity);
    }
}

