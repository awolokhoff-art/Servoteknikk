#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include <Arduino.h>
#include "Elevator.h"

// --- PINS ---
extern const int directionPin;
extern const int PWMpin;
extern const int EncoderA;
extern const int EncoderB;

// --- GLOBAL VARIABLES ---
extern volatile long EncoderCount;
extern float floorTolerance;
extern bool MotorDirection;
extern int PwmValue;
extern const int PulsesPerRevolution;
extern long physicalTargetFloor;
extern long targetPulses;



// Global states fra .ino
extern float currentEncoderFloor;
extern int currentLogicalFloor;
extern Elevator elev;  // allerede brukt

// --- FUNKSJONER ---
void DC_init();
void Enc_init();
void encoderAChange();
void encoderBChange();
void GetEncoderPos();
void floorReached();


#endif
